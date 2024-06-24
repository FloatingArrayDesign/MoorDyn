MoorDyn v2
==========

**This repository is for MoorDyn-C.** 

MoorDyn is a lumped-mass model for simulating the dynamics of mooring systems connected to floating offshore structures. As of 2022 it is available under the BSD 3-Clause
license.

Read the docs here: [moordyn.readthedocs.io](https://moordyn.readthedocs.io/en/latest/)

It accounts for internal axial stiffness and damping forces, weight and buoyancy forces, hydrodynamic forces from Morison's equation (assuming calm water so far), and vertical spring-damper forces from contact with the seabed. MoorDyn's input file format is based on that of [MAP](https://www.nrel.gov/wind/nwtc/map-plus-plus.html). The model supports arbitrary line interconnections, clump weights and floats, different line properties, and six degree of freedom rods and bodies.

MoorDyn is implemented both in Fortran and in C++. The Fortran version of MoorDyn (MoorDyn-F) is a core module in [OpenFAST](https://github.com/OpenFAST/openfast) and can be used as part of an OpenFAST or FAST.Farm simulation, or used in a standalone form. The C++ version of MoorDyn (MoorDyn-C) is more adaptable to different use cases and couplings. It can be compiled as a dynamically-linked library or wrapped for use in Python (as a module), Fortran, or Matlab. It features simpler functions for easy coupling with models or scripts coded in C/C++, Fortran, Matlab/Simulink, etc., including a coupling with [WEC-Sim](https://wec-sim.github.io/WEC-Sim/master/index.html). Users should take care to ensure their input file format matches the respective version of MoorDyn they are trying to use. Details on the input file differences can be found in the [documentation](https://moordyn.readthedocs.io/en/latest/inputs.html).

Both forms of MoorDyn feature the same underlying mooring model, use the same input and output conventions, and are being updated and improved in parallel. They follow the same version numbering, with a "C" or "F" suffix for differentiation.

Further information on MoorDyn can be found on the [documentation site](https://moordyn.readthedocs.io/en/latest/). MoorDyn-F is available in the [OpenFAST repository](https://github.com/OpenFAST/openfast/tree/main/modules/moordyn). MoorDyn-C is available in this repository with the following three maintained branches. The master branch represents the most recent release of 
MoorDyn-C. The dev branch contains new features currently in development. The v1 branch is the now deprecated version one of MoorDyn-C. 

## Acknowledgments

[National Renewable Energy Laboratory (NREL)](https://www.nrel.gov/):

  - [Matt Hall](http://matt-hall.ca/moordyn.html)
  - Ryan Davies
  - Andy Platt
  - Stein Housner
  - Lu Wang
  - Jason Jonkman

[CoreMarine](https://www.core-marine.com/) [MoorDyn-C v2]:

  - Jose Luis Cercos-Pita
  - Aymeric Devulder
  - Elena Gridasova

[Kelson Marine](https://kelsonmarine.com) [MoorDyn-C v2]:

  - [David Joseph Anderson](https://davidjosephanderson.com/)
  - [Alex Kinley](https://github.com/AlexWKinley)
