from itertools import product
import os
from dataclasses import dataclass

"""Simple script for generating a bunch of model files for benchmarking purposes.

Just needs basic python (tested for version >=3.8).
"""


@dataclass
class BenchmarkParams:
    num_lines: int
    num_segments: int


def generate_benchmark(base_output_folder, params: BenchmarkParams):
    """Generate a single benchmark model file.

    The model has a user defined number of lines that each share a fixed top
    point, and have their own bottom point.
    They start off at a 10 degree angle from vertical.
    """
    points = "\n".join(
        f"{i + 2} free 34.20201433256687 0.0 -93.96926207859084 0 0 0 0 -"
        for i in range(params.num_lines)
    )
    line_length = 100.0
    lines = "\n".join(
        f"{i+1} chain 1 {i + 2} {line_length} {params.num_segments} p,t"
        for i in range(params.num_lines)
    )

    file_str = f"""--------------------- MoorDyn Input File -------------------------------------------------------
Output from generate_cases.py
----------------------- LINE TYPES --------------------------------------------------------------
TypeName   Diam    Mass/m     EA         BA/-zeta    EI         Cd     Ca     CdAx    CaAx
(name)     (m)     (kg/m)     (N)        (N-s/-)     (N-m^2)    (-)    (-)    (-)     (-)
chain      0.252   390        16000000.0 -0.3        0.0        1.37   0.64   1.0     0.0
----------------------- POINTS -------------------------------------------------------------------
Node      Type      X        Y         Z        M        V         CdA    CA
(-)       (-)       (m)      (m)       (m)      (kg)     (m^3)     (m^2)  (-)
1         free      0.0      0.0       0.0      0        0         0      0 -
{points}
-------------------------- LINES -----------------------------------------------------------------
Line     LineType NodeA     NodeB  UnstrLen  NumSegs     Flags/Outputs
(-)      (-)       (-)       (-)   (m)         (-)          (-)
{lines}
-------------------------- SOLVER OPTIONS---------------------------
0        writeLog     - Write a log file
0.001 dtM          - time step to use in mooring integration
rk4      tScheme
3000000.0    kb           - bottom stiffness
300000.0    cb           - bottom damping
200.0    WtrDpth      - water depth
4.0      ICDfac       - factor to scale drag coeff during IC
1.5e-06    threshIC     - threshold for IC convergence
0.0    TmaxIC       - threshold for IC convergence
1e-05     dtIC        - Time lapse between convergence tests (s)
9.81     g     - gravitational force constant
1025     rho     - water density
0     WaveKin     - waves mode
0     Currents     - current mode
------------------------- need this line --------------------------------------
"""

    folder_name = f"{params.num_lines}_lines_{params.num_segments}_segs"

    output_folder = os.path.join(base_output_folder, folder_name)
    try:
        os.mkdir(output_folder)
    except FileExistsError:
        pass

    out_path = os.path.join(output_folder, "lines.txt")

    with open(out_path, "w") as f:
        f.write(file_str)


def main():

    number_of_lines = [2**n for n in range(7)]
    number_of_segments = [2**n for n in range(7)]

    for vals in product(number_of_lines, number_of_segments):
        params = BenchmarkParams(*vals)
        generate_benchmark("./", params)


if __name__ == "__main__":
    main()
