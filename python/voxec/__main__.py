import glob
import sys

from . import COMMANDS

cmds = {c.__name__: c for c in COMMANDS}

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <command> <ifc> ...")
        exit(1)
    cmd = cmds.get(sys.argv[1])
    if not cmd:
        print(f"Unknown command {sys.argv[1]}, available: {', '.join(cmds)}")
        exit(1)
    files = sum(map(glob.glob, sys.argv[2:]), [])
    cmd(*files)