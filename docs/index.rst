.. MoorDyn documentation master file, created by
   sphinx-quickstart on Mon Mar  2 13:35:40 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MoorDyn - Lumped-Mass Mooring Dynamics
======================================
.. _index:

Welcome to MoorDyn's online documentation. MoorDyn is a simple, efficient,
and versatile mooring system dynamics model designed to work in concert 
with other simulation tools. It is based on a
lumped-mass discretization of a mooring line's dynamics, and adds point-mass
and rigid-body objects to enable simulation of a wide variety of mooring and
cabling arrangements. Hydrodynamics are included using a version of the 
Morison equation, and there is support for wave loads on surface-piercing 
cylinders.

The pages on this site provide information 
about :ref:`compiling <compiling>`, MoorDyn's :ref:`input files <inputs>`, 
how to :ref:`drive a simulation <drivers>`, how MoorDyn is :ref:`structured <structure>`, 
and its :ref:`theory <theory>`. MoorDyn is available in multiple forms,
which are outlined below. The *latest* version of the docs is kept up-to-date with the 
dev branches of both MoorDyn-C and MoorDyn-F. The *master* version of the docs is up to date with 
the `latest MoorDyn-C release <https://github.com/FloatingArrayDesign/MoorDyn/releases>`_.

Note: This documentation is meant for users, a
`Doxygen documentation <./doxygen/html/index.html>`_ has been deployed for the
developers.

About MoorDyn
-------------

MoorDyn was originally developed in 2014 as a C++ library that could be 
easily coupled with other codes (such as WEC-Sim and DualSPHysics). Soon 
after, a second form was created as a module of the floating wind turbine 
simulator FAST, which is a FORTRAN code following the `FAST Modularization 
Framework <https://www.nrel.gov/docs/fy13osti/57228.pdf>`_. Both forms of 
MoorDyn have the same underlying physics modes but are different in how they
can be used with other codes. An effort is made to keep the physics modeling 
in both forms synchronized when new capabilities get added to one of them.
To distinguish, the two forms can be referred to as MoorDyn-C and
MoorDyn-F.

Beginning in 2019, a set of new features was added to MoorDyn including 
rigid bodies and support for wave loading at the free surface. This set of 
new capabilities is referred to as Version 2 (v2). Version 2 has been
realized in both C++ and FORTRAN forms of MoorDyn. The MoorDyn source code 
is available on GitHub. The MoorDyn-C v1 and v2 is
`here <https://github.com/FloatingArrayDesign/MoorDyn>`_ and
the MoorDyn-F code is included in the `OpenFAST repository 
<https://github.com/openfast/openfast>`_.

As of version 2, the input file formats of MoorDyn-C and MoorDyn-F are 
identical. We recommend using MoorDyn v2 for all applications unless
you have a specific reason to use MoorDyn v1.

MoorDyn-C v2 contains wrappers that make it available as a python library and allow
it to be easily packaged into other languages. It is released under the `3-clause BSD 
license <https://opensource.org/licenses/BSD-3-Clause>`_.

This website serves as a public-facing guide for MoorDyn in all forms. It 
focuses on MoorDyn’s principles of operation, setup of the input files, 
common problems, etc. It will also have guidance for coupling with 
MoorDyn in standalone form, as well as a guide for the API. For 
information about use of MoorDyn-F in the larger context of OpenFAST 
simulations, refer also to the `OpenFAST documentation <https://openfast.readthedocs.io>`_.

In this documentation, when MoorDyn is mentioned without specifying C or
F then it is applicable to both. If a version number is not stated, 
then version 2 is assumed.


Ways of Using MoorDyn:
----------------------

As a mooring dynamics library, MoorDyn needs some other program
to run it. This other “driver” program can be a simple script that moves
(or holds still) the ends of the mooring lines, or a full dynamics model
for other parts of a floating system. 

MoorDyn-F is usually run as part of the full OpenFAST simulation model,
driven by the OpenFAST or FAST.Farm “glue code”. It also comes with a
dedicated “MoorDyn driver ” code that allows MoorDyn-F to run in 
isolation with only an input file of floating platform motions. Instructions
for the use of the MoorDyn-F driver can be found :ref:`here <MDF_driver_in>`.

MoorDyn-C is designed for coupling with a wide number of codes. Some
couplings already exist and can be found :ref:`here <coupling>` (e.g. WEC-Sim 
and DualSPHysics). For coupling with other codes or more manual driving of 
MoorDyn from your own script, several APIs, wrappers, and example driver 
scripts are available :ref:`here <drivers>`. 

Additionally, an example directory contains instructions for common couplings
and basic input file set ups. A recording of the presentation that walks through
the directory is also available `here <https://www.youtube.com/watch?v=FqW7Xpl_VNk>`_.

Table of Contents:
------------------

.. toctree::
   :maxdepth: 1

   compiling
   drivers
   inputs
   structure
   api_c
   waterkinematics
   theory

