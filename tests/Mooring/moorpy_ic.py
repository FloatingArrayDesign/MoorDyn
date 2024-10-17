import os
ROOT = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(os.path.join(ROOT, '../../wrappers/python/moordyn/'))
from moorpyic import *
import argparse


parser = argparse.ArgumentParser(
    prog='moorpy_ic',
    description='Create an Initial Condition using MoorPy')
parser.add_argument('filename')
parser.add_argument('-t', '--tol', type=float, default=0.05,
                    help='Equilibrium solver absolute tolerance')

with open(os.path.join(ROOT, '../../CMakeLists.txt'), 'r') as f:
    txt = f.read()
    def read_version_line(txt, name):
        subtxt = "set(" + name + " "
        txt = txt[txt.find(subtxt) + len(subtxt):]
        txt = txt[:txt.find(')\n')]
        return int(txt.strip())
    version_major = read_version_line(txt, "MOORDYN_MAJOR_VERSION")
    version_minor = read_version_line(txt, "MOORDYN_MINOR_VERSION")

args = parser.parse_args()
if not moorpy_ic(args.filename,
                 tol=args.tol,
                 version_major=version_major,
                 version_minor=version_minor):
    print("The system did not converge!")
