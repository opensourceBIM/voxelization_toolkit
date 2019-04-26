echo "
import os
import sys
import time
import tempfile
import subprocess

from collections import defaultdict

S = float(sys.argv[1])

def run_voxelfile(fn, args=None):
    def make_args(d):
        for kv in d.items():
            yield \"--%s=%s\" % kv
        
    di = defaultdict(dict)
    proc = subprocess.Popen([r\"voxec\", fn, \"--threads=1\"] + list(make_args(args or {})), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while True:
        ln = proc.stderr.readline().decode('utf-8')
        if not ln:
            if proc.poll() is None:
                time.sleep(0.5)
                continue
            else: 
                break
        if ln.startswith('@'):
            attrs = list(map(str.strip, ln.split(';')))
            id = int(attrs[0][1:])
            for a in attrs[1:]:
                k, v = map(str.strip, a.split(':'))
                di[id][k] = v
                
    return di


tmp, fn = tempfile.mkstemp(suffix='voxelfile.txt', text=True)
os.write(tmp, '''file = parse(\"duplex.ifc\")
surfaces = create_geometry(file)
voxels = voxelize(surfaces)
offset_voxels = offset(voxels)
outmost_voxels = outmost(offset_voxels)
'''.encode('ascii'))
os.fsync(tmp)
os.close(tmp)

di = run_voxelfile(fn, {'size': S})
print(S, float(int(di.get(4).get('count'))) * S ** 2., sep=',')

os.remove(fn)
" > exec.py

cat <(seq 0.001 0.001 0.009) <(seq 0.01 0.01 0.2) | tac | xargs -P 4 -n 1 python3 exec.py
