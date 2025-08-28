Features and References
=======================
.. _theory:

Most of MoorDyn’s theory is described in the following publications. This page 
gives a very high-level overview, highlights specific theory aspects that may 
be important to users, and lists the papers where more detail can be found.

Features
--------

Version 1
^^^^^^^^^
MoorDyn is based on a lumped-mass discretization of a mooring line’s dynamics, and adds point-mass and rigid-body objects to enable simulation of a wide 
variety of mooring and cabling arrangements. Hydrodynamics are included using a version of the Morison equation.

Version 2
^^^^^^^^^
MoorDyn v2 contains all the features of v1 with the following additional features:
  - Simulation of 6 degree of freedom objects
  - Non-linear tension
  - Wave kinematics
  - Bending stiffness
  - Bathymetry
  - Seabed friction

The main difference between MoorDyn-C and MoorDyn-F is that MoorDyn-C uses quaternions to describe the orientation of 6DOF objects, while F uses traditional Euler angles to handle 6DOF object rotations.

Orientation of 6 DOF objects:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoorDyn-C, MoorDyn-F, and `MoorPy <https://github.com/NREL/MoorPy>`_ share the
same Intrinsic Euler-XYZ (Tait-Bryan) angles criteria to compute orientations.
You can learn more about this on
`Hall M. Generalized Quasi-Static Mooring System Modeling with Analytic Jacobians. Energies. 2024; 17(13):3155. https://doi.org/10.3390/en17133155 <https://www.mdpi.com/1996-1073/17/13/3155>`_

However, while on MoorDyn-F this is handled by considering orientation
matrices, on MoorDyn-C quaternions are considered to describe the location and
orientation of 6 DOF objects.
Further description of quaternions can be found in PR #90 in the MoorDyn
repository, put together by Alex Kinley of Kelson Marine:
https://github.com/FloatingArrayDesign/MoorDyn/pull/90#issue-1777700494

References
----------

The theory behind MoorDyn is available in a collection of papers, listed below by which version they were implemented in.

Version 1
^^^^^^^^^
The v1 lumped-mass formulation of MoorDyn as well as its validation against experiments:

  `M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with DeepCwind semisubmersible model test data,” 
  Ocean Engineering, vol. 104, pp. 590–603, Aug. 2015. <http://www.sciencedirect.com/science/article/pii/S0029801815002279>`_

Coupling with WEC-Sim or any Simulink code for wave energy converter simulation:

  `S. Sirnivas, Y.-H. Yu, M. Hall, and B. Bosma, “Coupled Mooring Analysis for the WEC-Sim Wave Energy Converter Design Tool,” 
  in Proceedings of the 35th International Conference on Ocean, Offshore and Arctic Engineering, Busan, South Korea, 2016.
  <http://www.nrel.gov/docs/fy16osti/65918.pdf>`_

  `G. Vissio, B. Passione, and M. Hall, “Expanding ISWEC Modelling with a Lumped-Mass Mooring Line Model,” 
  presented at the European Wave and Tidal Energy Conference, Nantes, France, 2015. <http://matt-hall.ca/docs/vissio_2015_eim.pdf>`_

Version 2
^^^^^^^^^

Version 2 builds upon the capabilities of Version 1. The theory behind the new features is described in the following references. 

Early work on seabed friction and independent fairlead points:

  `M. Hall, “Efficient Modelling of Seabed Friction and Multi-Floater Mooring Systems in MoorDyn,” 
  in Proceedings of the 12th European Wave and Tidal Energy Conference, Cork, Ireland, 2017. <http://matt-hall.ca/docs/hall_2017_ems.pdf>`_

Preliminary comparison of seabed friction formulations:

  `K. Devries, M. Hall, “Comparison of Seabed Friction Formulations in a LumpedMass Mooring Model”. in Proceedings of the ASME 
  International Offshore Wind Technical Conference, San Francisco, California, Nov. 2018. <http://matt-hall.ca/publications.html>`_

Overview of MoorDyn v2 (bodies, rods, and line failures):

  `Hall, Matthew, “MoorDyn V2: New Capabilities in Mooring System Components and Load Cases.” In Proceedings of the ASME 2020 39th International 
  Conference on Ocean, Offshore and Arctic Engineering. virtual conference, 2020. <https://www.nrel.gov/docs/fy20osti/76555.pdf>`_

Seabed friction and bathymetry approach used in v2:

  `Housner, Stein, Ericka Lozon, Bruce Martin, Dorian Brefort, and Matthew Hall, “Seabed Bathymetry and Friction Modeling in MoorDyn.” Journal of 
  Physics: Conference Series 2362, no. 1, Nov 2022: 012018. <https://doi.org/10.1088/1742-6596/2362/1/012018>`_

Implementation of bending stiffness modeling for power cables:

  `Hall, Matthew, Senu Sirnivas, and Yi-Hsiang Yu, “Implementation and Verification of Cable Bending Stiffness in MoorDyn.” In ASME 2021 3rd International Offshore Wind 
  Technical Conference, V001T01A011. Virtual, Online: American Society of Mechanical Engineers, 2021. <https://doi.org/10.1115/IOWTC2021-3565>`_

Non-linear line stiffness:

 `Lozon, Ericka, Matthew Hall, Paul McEvoy, Seojin Kim, and Bradley Ling, “Design and Analysis of a Floating-Wind Shallow-Water Mooring System 
 Featuring Polymer Springs.” American Society of Mechanical Engineers Digital Collection, 2022. <https://doi.org/10.1115/IOWTC2022-98149>`_

Bladed-MoorDyn Coupling:

  `Alexandre, Armando, Francesc Fabregas Flavia, Jingyi Yu, Ali Bakhshandehrostami, and Steven Parkinson. "Coupling Bladed With External Finite-Element Mooring Libraries." 
  American Society of Mechanical Engineers Digital Collection, 2023. <https://doi.org/10.1115/IOWTC2023-119346>`_

Viscoelastic approach for non-linear rope behavior:

  `Hall, Matthew, Brian Duong, and Ericka Lozon, “Streamlined Loads Analysis of Floating Wind Turbines With Fiber Rope Mooring Lines.” In ASME 2023 
  5th International Offshore Wind Technical Conference, V001T01A029. Exeter, UK: American Society of Mechanical Engineers, 2023. <https://doi.org/10.1115/IOWTC2023-119524>`_

Updated MoorDyn-OpenFOAM Coupling:
  
  `Haifei Chen, Tanausú Almeida Medina, and Jose Luis Cercos-Pita, "CFD simulation of multiple moored floating structures using OpenFOAM: An open-access mooring restraints 
  library." Ocean Engineering, vol. 303, Jul. 2024. <https://doi.org/10.1016/j.oceaneng.2024.117697>`_

Reef3D-MoorDyn Coupling:

  `Soydan, Ahmet, Widar Weizhi Wang, and Hans Bihs. "An Improved Direct Forcing Immersed Boundary Method With Integrated Mooring Algorithm for Floating Offshore Wind 
  Turbines." American Society of Mechanical Engineers Digital Collection, 2024. <https://doi.org/10.1115/1.4067117>`_

Modeling of Bi-stable Nonlinear Energy Sinks in MoorDyn (most recent description of MoorDyn theory):

  `Anargyros Michaloliakos, Wei-Ying Wong, Ryan Davies, Malakonda Reddy Lekkala, Matthew Hall, Lei Zuo, Alexander F. Vakakis, "Stabilizing dynamic subsea power cables using 
  Bi-stable nonlinear energy sinks", Ocean Engineering, vol. 334, August 2025. <https://doi.org/10.1016/j.oceaneng.2025.121613>`_

The Fortran version of MoorDyn is available as a module inside of OpenFAST:
  
  https://openfast.readthedocs.io/en/main/

Hydrodynamics of 6DOF objects follows a similar approach to Hydrodyn:

  https://www.nrel.gov/wind/nwtc/assets/downloads/HydroDyn/HydroDyn_Manual.pdf

Quaternion references:

1. Fossen, Thor I. Handbook of marine craft hydrodynamics and motion control. 
   Page 25 John Wiley & Sons, 2011.
2. https://en.wikipedia.org/wiki/Gimbal_lock
3. https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
4. https://en.wikipedia.org/wiki/Quaternion#Hamilton_product

MoorDyn-C Packages used:
 - Eigen: https://eigen.tuxfamily.org 
 - Catch2: https://github.com/catchorg/Catch2
 - KISSFFT: https://github.com/mborgerding/kissfft
