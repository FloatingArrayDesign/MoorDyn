import argparse
import moorpy


parser = argparse.ArgumentParser(
    prog='moorpy_ic',
    description='Create an Initial Condition using MoorPy')
parser.add_argument('filename')
parser.add_argument('-t', '--tol', type=float, default=0.05,
                    help='Equilibrium solver absolute tolerance')

args = parser.parse_args()
system = moorpy.System(file=args.filename)
system.initialize()
if system.solveEquilibrium(tol=args.tol, display=1):
    print(f"Saving {args.filename + '.ic'}...")
    system.export_state(args.filename + '.ic')
else:
    print("The system did not converge!")
