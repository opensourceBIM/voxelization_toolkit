from .voxec import *

import multiprocessing

default_VOXELSIZE = 0.05
default_CHUNKSIZE = 16
default_THREADS = multiprocessing.cpu_count()
SILENT = False

def run(name, *args, **kwargs):
	ctx = context()
	ctx.set('VOXELSIZE', default_VOXELSIZE)
	ctx.set('CHUNKSIZE', default_CHUNKSIZE)
	ctx.set('THREADS', default_THREADS)
	return run_(name, args, kwargs, ctx, SILENT)
