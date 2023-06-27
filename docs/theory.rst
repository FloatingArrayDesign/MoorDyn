.. _theory:

Features and References
=======================

Features
--------

Version 1 - v1
^^^^^^^^^^^^^^
MoorDyn is based on a lumped-mass discretization of a mooring line’s dynamics, and contains point-mass objects to enable simulation of a wide 
variety of mooring and cabling arrangements. Hydrodynamics are included using a version of the Morison equation.

Version 2 - v2
^^^^^^^^^^^^^^
MoorDynV2 contains all the features of v1 with the following additional features:
  - Simulation of 6 DOF rigid bodies
  - Non-linear tension
  - Wave kinematics
  - Bending stiffness
  - Bathymetry
  - Seabed friction

Fortran - F
^^^^^^^^^^^
MoorDynF is a module built into OpenFAST and shares all the same capabilities as MoorDynV2. In addition it can ....

References
----------

The theory behind MoorDyn is available in a collection of papers, listed below by which version they were implemented in.

Version 1 - v1
^^^^^^^^^^^^^^
The v1 lumped-mass formulation of MoorDyn as well as its validation against experiments:

  `M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with DeepCwind semisubmersible model test data,” 
  Ocean Engineering, vol. 104, pp. 590–603, Aug. 2015.' <http://www.sciencedirect.com/science/article/pii/S0029801815002279>`_

Coupling with WEC-Sim or any Simulink code for wave energy converter simulation:

  `S. Sirnivas, Y.-H. Yu, M. Hall, and B. Bosma, “Coupled Mooring Analysis for the WEC-Sim Wave Energy Converter Design Tool,” 
  in Proceedings of the 35th International Conference on Ocean, Offshore and Arctic Engineering, Busan, South Korea, 2016.
  <http://www.nrel.gov/docs/fy16osti/65918.pdf>`_

  `G. Vissio, B. Passione, and M. Hall, “Expanding ISWEC Modelling with a Lumped-Mass Mooring Line Model,” 
  presented at the European Wave and Tidal Energy Conference, Nantes, France, 2015. <http://matt-hall.ca/docs/vissio_2015_eim.pdf>`_

Version 2 - v2
^^^^^^^^^^^^^^

Version 2 builds upon all of the capabilities of Version 1. The theory behind the new features is described in the following references. 

Early work on seabed friction and independent fairlead points:

  `M. Hall, “Efficient Modelling of Seabed Friction and Multi-Floater Mooring Systems in MoorDyn,” 
  in Proceedings of the 12th European Wave and Tidal Energy Conference, Cork, Ireland, 2017. <http://matt-hall.ca/docs/hall_2017_ems.pdf>`_

Preliminary comparison of seabed friction formulations:

  `K. Devries, M. Hall, “Comparison of Seabed Friction Formulations in a LumpedMass Mooring Model”. in Proceedings of the ASME 
  International Offshore Wind Technical Conference, San Francisco, California, Nov. 2018. <http://matt-hall.ca/publications.html>`_

Seabed friction and bathymetry implementation and verification:

  `S. Housner, E. Lozon, B. Martin, D. Brefort, M. Hall, “Seabed bathymetry and friction modeling in MoorDyn”. in Journal of Physics: 
  Conference Series 2362(2022):012018, Nov. 2022. <https://www.nrel.gov/docs/fy23osti/82033.pdf>`_

Cable bending stiffness implementation and verification:

  `M. Hall, S. Sirnivas, Y. Yu, “Implementation and Verification of Cable Bending Stiffness in MoorDyn: Preprint”. 
  Golden, CO: National Renewable Energy Laboratory. 2020. NREL/CP-5000-76968. <https://www.nrel.gov/docs/fy21osti/76968.pdf>`_

Non-linear tensions:

  `E. Lozon, M. Hall, P. McEvoy, S. Kim, B. Ling, “Design and Analysis of a Floating-Wind Shallow-Water Mooring System Featuring 
  Polymer Springs: Preprint”. Golden, CO: National Renewable Energy Laboratory. 2022. NREL/CP-5000- 83342. <https://www.nrel.gov/docs/fy23osti/83342.pdf>`_

Overview of MoorDyn v2:

  https://www.nrel.gov/docs/fy20osti/76555.pdf

Implementation of bending stiffness modeling for power cables:

  https://www.nrel.gov/docs/fy21osti/76968.pdf

Fortran - F
^^^^^^^^^^^

The fortran version of MoorDyn is available as a module inside of OpenFAST:
  <https://openfast.readthedocs.io/en/main/>. 

Verification
<Some paper that talks about Moordyn and OpenFAST. Fortran verification>


