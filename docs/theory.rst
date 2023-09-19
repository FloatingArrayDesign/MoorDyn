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

Euler angles – MoorDyn-F
""""""""""""""""""""""""

In the following figure the 6DOF object orientation angles convention is depicted:

.. figure:: angles.svg
   :alt: Angles criteria schematic view

The roll and yaw angles, :math:`\phi` and :math:`\psi`, follow the
right hand criteria, while the pitch angle, :math:`\theta`, follows the left
hand criteria.
This way the classic rotation matrices can be considered,

.. math::
   \begin{alignat}{1}
   R_x(\phi) &= \begin{bmatrix}
   1 &  0         &  0           \\
   0 &  \cos \phi & -\sin \phi \\[3pt]
   0 &  \sin \phi & \cos \phi \\[3pt]
   \end{bmatrix} \\[6pt]
   R_y(\theta) &= \begin{bmatrix}
   \cos \theta & 0 & \sin \theta \\[3pt]
   0           & 1 &  0           \\[3pt]
   -\sin \theta & 0 &  \cos \theta \\
   \end{bmatrix} \\[6pt]
   R_z(\psi) &= \begin{bmatrix}
   \cos \psi & -\sin \psi & 0 \\[3pt]
   \sin \psi &  \cos \psi & 0 \\[3pt]
   0         &  0         & 1 \\
   \end{bmatrix}
   \end{alignat}


Quaternions – MoorDyn-C
"""""""""""""""""""""""

The latest MoorDyn-C internally uses quaternions to describe the location and orientation of 6 DOF objects. Externally MoorDyn-C behaves the same as MoorDyn-F, using Euler angles for both inputs and outputs. Quaternions are a common alternative to Euler angles for describing orientations of 3D objects. 
Further description of quaternions can be found in PR #90 in the MoorDyn repository, put together by Alex Kinley of Kelson Marine: https://github.com/FloatingArrayDesign/MoorDyn/pull/90#issue-1777700494

References
----------

The theory behind MoorDyn is available in a collection of papers, listed below by which version they were implemented in.

Version 1
^^^^^^^^^
The v1 lumped-mass formulation of MoorDyn as well as its validation against experiments:

  `M. Hall and A. Goupee, “Validation of a lumped-mass mooring line model with DeepCwind semisubmersible model test data,” 
  Ocean Engineering, vol. 104, pp. 590–603, Aug. 2015.' <http://www.sciencedirect.com/science/article/pii/S0029801815002279>`_

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

Overview of MoorDyn v2:

  https://www.nrel.gov/docs/fy20osti/76555.pdf

Implementation of bending stiffness modeling for power cables:

  https://www.nrel.gov/docs/fy21osti/76968.pdf

The Fortran version of MoorDyn is available as a module inside of OpenFAST:
  
  https://openfast.readthedocs.io/en/main/

Dynamics of 6DOF objects follows a similar approach to Hydrodyn:

  https://www.nrel.gov/wind/nwtc/assets/downloads/HydroDyn/HydroDyn_Manual.pdf

Quaternion references:

1. Fossen, Thor I. Handbook of marine craft hydrodynamics and motion control. 
   Page 25 John Wiley & Sons, 2011.
2. https://en.wikipedia.org/wiki/Gimbal_lock
3. https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
4. https://en.wikipedia.org/wiki/Quaternion#Hamilton_product

MoorDyn-C Packages used:
	Eigen: https://eigen.tuxfamily.org 
	Catch2: https://github.com/catchorg/Catch2 
