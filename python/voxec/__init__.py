import contextlib
import datetime
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time

VOXEC = shutil.which("voxec")
assert VOXEC

def draw_points(arr, fn, colors, factor=1., label=None, vec=None):
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    xyz = np.hstack((arr[:, 0:3], np.ones((len(arr), 1))))
    clr = arr.T[3]
    if vec is not None:
        vec /= np.linalg.norm(vec)
        Z = np.array((0,0,1.0))
        up = np.cross(vec, np.cross(Z, vec))
        up /= np.linalg.norm(up)
        M = np.eye(4)
        M[0:3,0:3] = np.cross(up, vec), up, vec
        xyzw = (np.asmatrix(M) * np.asmatrix(xyz).T).T.A
        xyzw[:, 0:3] /= xyzw[:,3:4]
        xyz = xyzw
    order = xyz[:, 2].argsort()
    xy = xyz[order][:,0:2]
    plt.figure(figsize=np.ptp(xy, axis=0))
    norm = plt.Normalize(colors[0][0], colors[-1][0])
    colors = [(norm(p[0]), *p[1:]) for p in colors]
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", colors)
    plt.scatter(xy.T[0], xy.T[1], c=clr[order] * factor, s=4, label=label, cmap=cmap, norm=norm)
    plt.axis('off')
    if label:
        plt.colorbar(shrink=0.5)
    plt.savefig(fn, bbox_inches='tight')

class voxel_post_process:
    pass

class get_json(voxel_post_process):
    def __init__(self, filename):
        self.filename = filename

    def __call__(self, cwd):
        with open(os.path.join(cwd, self.filename)) as f:
            return json.load(f)

class get_csv(voxel_post_process):
    def __init__(self, filename):
        self.filename = filename

    def __call__(self, cwd):
        import pandas as pd
        return pd.read_csv(os.path.join(cwd, self.filename), delimiter=',').values

def run(command, ifc_files, *ops, keep_dir=False, **kwargs):
    def make_args(d):
        for kv in d.items():
            if kv[1] is True:
                yield "--%s" % kv[0]
            else:
                yield "--%s=%s" % kv

    if keep_dir:
        cwd = tempfile.mkdtemp()
        context = contextlib.nullcontext
    else:
        cwd = None
        context = tempfile.TemporaryDirectory

    with context() as c:
        cwd = cwd or c

        with open(os.path.join(cwd, "voxelfile.txt"), "w") as f:
            f.write(command)

        for i, fn in enumerate(ifc_files):
            if isinstance(fn, str):
                shutil.copyfile(fn, os.path.join(cwd, os.path.basename(fn)))
            else:
                fn.write(os.path.join(cwd, f'model{i:03d}.ifc'))

        rc = None
        log = []
        completed = False

        def run_process():
            nonlocal rc, completed
            rc = subprocess.call([
                VOXEC, 
                "-q", "--log-file", "log.json",
                "voxelfile.txt",
                *make_args(kwargs)
            ], cwd=cwd, stdout=subprocess.PIPE)
            completed = True

        thread = threading.Thread(target=run_process)
        thread.start()

        def check_log():
            def read_log():
                try:
                    with open(os.path.join(cwd, "log.json"), "r") as f:
                        for l in [l.strip() for l in f if l.strip()]:
                            yield json.loads(l)
                except: pass
                
            new_log = list(read_log())
            for x in new_log[len(log):]:
                print(x)
            log[:] = new_log

        while not completed:
            check_log()
            time.sleep(1.)
   
        if rc != 0:
            with open(os.path.join(cwd, "log.json"), "a") as f:
                json.dump({"severity": "fatal", "message": "internal error", "time": str(datetime.datetime.now())}, f)

        check_log()

        return [op(cwd) for op in ops]

def do_try(fn, default=None):
    try:
        return fn()
    except:
        return default

def exterior_elements(*file, threads=None, size=None):
    elements = [x['guid'] for x in run("""file = parse("*.ifc")
all_surfaces = create_geometry(file)
voxels = voxelize(all_surfaces)
external = exterior(voxels)
shell = offset(external)
export_json(file, shell, all_surfaces, "result.json", individual_faces=0, factor=4)
""", file, get_json("result.json"), keep_dir=True, threads=threads or 4, size=size or 0.1, chunk=16)[0]]
    print("Exterior:")
    print("=========")
    for el in elements:
        print(el)

def headroom(*file, threads=None, size=None):
    cmd = """function get_reachability(file)
    surfaces = create_geometry(file, exclude={"IfcOpeningElement", "IfcDoor", "IfcSpace"})
    surface_voxels = voxelize(surfaces)
    slabs = create_geometry(file, include={"IfcSlab"})
    slab_voxels = voxelize(slabs)
    doors = create_geometry(file, include={"IfcDoor"})
    door_voxels = voxelize(doors)
    walkable = shift(slab_voxels, dx=0, dy=0, dz=1)
    walkable_minus = subtract(walkable, slab_voxels)
    walkable_seed = intersect(door_voxels, walkable_minus)
    surfaces_sweep = sweep(surface_voxels, dx=0, dy=0, dz=0.5)
    surfaces_padded = offset_xy(surface_voxels, 0.1)
    surfaces_obstacle = sweep(surfaces_padded, dx=0, dy=0, dz=-0.5)
    walkable_region = subtract(surfaces_sweep, surfaces_obstacle)
    walkable_seed_real = subtract(walkable_seed, surfaces_padded)
    reachable = traverse(walkable_region, walkable_seed_real)
return reachable

file = parse("*.ifc")
all_surfaces = create_geometry(file, exclude={"IfcSpace", "IfcOpeningElement", "IfcDoor"})
voxels = voxelize(all_surfaces)
reachable = get_reachability(file)
space = sweep(reachable, dx=0, dy=0, dz=1, until=voxels, max=60)
height_count = collapse_count(space, dx=0, dy=0, dz=-1)
export_csv(height_count, "height_count.csv")
"""
    flow = run(cmd, file, get_csv("height_count.csv"), threads=threads or 4, size=size or 0.1, chunk=16)[0]
    colors = [(0.0, "red"), (1.5, "yellow"), (3, "green"), (5, "green"), (5.5, (0.9, 0.9, 0.9))]
    draw_points(flow, "heights.png", colors, label="height", factor=0.1, vec=(3,-1,1))

def evacuation_distance(*file, threads=None, size=None):
    cmd = """function get_reachability(file)
    surfaces = create_geometry(file, exclude={"IfcOpeningElement", "IfcDoor", "IfcSpace"})
    surface_voxels = voxelize(surfaces)
    slabs = create_geometry(file, include={"IfcSlab"})
    slab_voxels = voxelize(slabs)
    doors = create_geometry(file, include={"IfcDoor"})
    door_voxels = voxelize(doors)
    walkable = shift(slab_voxels, dx=0, dy=0, dz=1)
    walkable_minus = subtract(walkable, slab_voxels)
    walkable_seed = intersect(door_voxels, walkable_minus)
    surfaces_sweep = sweep(surface_voxels, dx=0, dy=0, dz=0.5)
    surfaces_padded = offset_xy(surface_voxels, 0.1)
    surfaces_obstacle = sweep(surfaces_padded, dx=0, dy=0, dz=-0.5)
    walkable_region = subtract(surfaces_sweep, surfaces_obstacle)
    walkable_seed_real = subtract(walkable_seed, surfaces_padded)
    reachable = traverse(walkable_region, walkable_seed_real)
return reachable

file = parse("*.ifc")
all_surfaces = create_geometry(file, exclude={"IfcSpace", "IfcOpeningElement"})
voxels = voxelize(all_surfaces)
reachable = get_reachability(file)
reachable_shifted = shift(reachable, dx=0, dy=0, dz=1)
reachable_bottom = subtract(reachable, reachable_shifted)
reachable_bottom_offset = offset_xy(reachable_bottom, 1)
reachable_bottom_incl = union(reachable_bottom_offset, reachable_bottom)
external = exterior(voxels)
seed = intersect(external, reachable_bottom_incl)
reachable = union(reachable, seed)
distance = traverse(reachable, seed, connectedness=26, type="uint")
distance_bottom = intersect(distance, reachable_bottom)
export_csv(distance_bottom, "distance.csv")
"""
    flow = run(cmd, file, get_csv("distance.csv"), threads=threads or 4, size=size or 0.1, chunk=16)[0]
    max = flow.T[3].max() * 0.01
    colors = [(0., (0.9,0.9,0.9)), (0.1, "green"), (max / 3., "orange"), (max / 3. * 2., "red"), (max, "purple")]
    draw_points(flow, "evacuation_distance.png", colors, label="evacuation distance", factor=0.01, vec=(3,-1,1))

COMMANDS = [exterior_elements, headroom, evacuation_distance]