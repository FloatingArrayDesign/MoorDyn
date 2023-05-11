.. MoorDyn documentation master file, created by
   sphinx-quickstart on Mon Mar  2 13:35:40 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MoorDyn v2 - Lumped-Mass Mooring Dynamics
=========================================

.. toctree::
   :maxdepth: 2
   :hidden:

   starting
   structure
   usage
   theory
   coupling
   api_c
   api_cpp
   wrappers
   troubleshooting
   waterkinematics


Welcome to MoorDyn's (UNDER CONSTRUCTION) online documentation.
See the pages on this site for information about the operation,
:ref:`usage <usage>`, and :ref:`theory <theory>` of MoorDyn.

Note: This documentation is meant for users, a
`Doxygen documentation <./doxygen/index.html>`_ has been deployed for the
developers.

About MoorDyn
-------------

MoorDyn is a simple, efficient, and versatile mooring system dynamics model
designed to work  in concert with other simulation tools. It is based on a
lumped-mass discretization of a  mooring line's dynamics, and adds point-mass
and rigid-body objects to enable simulation of a wide variety of mooring and
cabling arrangements. Hydrodynamics are included using a version of the Morison
equation.

There are two main forms. The standalone form is coded in C++, is very easy to
couple with a variety of models and programming languages, and is released under
the `3-clause BSD license <https://opensource.org/licenses/BSD-3-Clause>`_. This
is the original form of the model, and it is used with WEC-Sim, DualSPHysics,
and various :ref:`wrappers in different languages <wrappers>`.

The other form is MoorDyn-F, a full rewrite of MoorDyn in FORTRAN that follows
the FAST Modularization Framework. This form is a core module in the OpenFAST
floating wind turbine simulator.

Both forms of MoorDyn have the same underlying physics model, and nearly
identical input file formats.

This website focuses though on the standalone form.

For information about use of MoorDyn-F in the larger context of OpenFAST
simulations, refer also to the
`OpenFAST documentation <https://openfast.readthedocs.io>`_.

The MoorDyn source code is available on GitHub. The standalone C++ code is
`here <https://github.com/mattEhall/MoorDyn>`_ and
code of MoorDyn-F in FORTRAN is available within the
`OpenFAST repository <https://github.com/openfast/openfast>`_.
