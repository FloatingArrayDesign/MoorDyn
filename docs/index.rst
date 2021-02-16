.. MoorDyn documentation master file, created by
   sphinx-quickstart on Mon Mar  2 13:35:40 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MoorDyn - Lumped-Mass Mooring Dynamics
======================================

.. toctree::
   :maxdepth: 2
   :hidden:

   starting
   structure
   usage
   theory
   coupling
   version2


Welcome to MoorDyn's (UNDER CONSTRUCTION) online documentation.
See the pages on this site for information about the operation, usage, and theory of MoorDyn. 

About MoorDyn
-------------

MoorDyn is a simple, efficient, and versatile mooring system dynamics model designed to work 
in concert with other simulation tools. It is based on a lumped-mass discretization of a 
mooring line's dynamics, and adds point-mass and rigid-body objects to enable simulation of a 
wide variety of mooring and cabling arrangements. Hydrodynamics are included using a version of
the Morison equation.

There are two main forms. The standalone form is coded in C++, is very easy to couple with
a variety of models and programming languages, and is released under the GPL. This is the
original form of the model, and it is used with WEC-Sim, DualSPHysics, and various custom 
scripting approaches. The other form is MoorDyn-F, a full rewrite of MoorDyn in FORTRAN that
follows the FAST Modularization Framework. This form is a core module in the OpenFAST floating
wind turbine simulator. Both forms of MoorDyn have the same underlying physics model, and nearly
identical input file formats.

A "version 2" of MoorDyn is currently in progress, and information about it will be posted :ref:`here <version-2>`.

This website serves as a public-facing guide for MoorDyn in both forms. It focuses on MoorDyn's
principles of operation, setup of the input files, common problems, etc. It will also have guidance
for coupling with MoorDyn in standalone form, as well as a guide for the API. For information
about use of MoorDyn-F in the larger context of OpenFAST simulations, refer also to the 
`OpenFAST documentation <https://openfast.readthedocs.io>`_.

The MoorDyn source code is available on GitHub. The standalone C++ code is `here <https://github.com/mattEhall/MoorDyn>`_ and 
code of MoorDyn-F in FORTRAN is available within the `OpenFAST repository <https://github.com/openfast/openfast>`_.
