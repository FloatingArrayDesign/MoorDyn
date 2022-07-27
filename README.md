MoorDyn v2
==========

MoorDyn v2 is a lumped-mass mooring dynamics model intended for coupling with
floating structure codes. As of 2022 it is available under the BSD 3-Clause
license.

More information about MoorDyn is now available at moordyn.readthedocs.io

## Roadmap

The version 2 is currently under development:

 [X] BSD-3 license
 [X] Rigid bodies
 [X] Rigid cylindrical Rod objects, with surface-piercing capabilities
 [ ] Wave kinematics
 [X] Bending stiffness for power cable simulation
 [X] pinned (3 DOF) and rigid (6 DOF) coupling options
 [ ] Replace the custom algebra code by [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
 [ ] New API (C & C++)
 [X] Standarize code styling with [clang-format](https://clang.llvm.org/docs/ClangFormat.html)
 [X] Replace the custom compilation scripts by [CMake](https://cmake.org/) autotools
 [ ] FORTRAN wrappers
 [X] Python wrappers
 [ ] Rust wrappers
 [ ] Documentation for users
 [ ] Documentation for developers

## Aknowledgments

Many thanks to all the team of the
[National Renewable Energy Laboratory (NREL)](https://www.nrel.gov/):

  - [Matt Hall](http://matt-hall.ca/moordyn.html)
  - Jason Jonkman

Thanks also to [CoreMarine](https://www.core-marine.com/) for the help with the
version 2 development:

  - Jose Luis Cercos-Pita
  - Aymeric Devulder
  - Elena Gridasova
