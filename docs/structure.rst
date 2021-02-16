Model Structure
===============




MoorDyn Objects
---------------

Lines
^^^^^

MoorDyn uses a lumped-mass approach to discretize the dynamics over the length of the mooring line.  
A line is broken up into N evenly-sized line segments connecting N+1 node points.  The indexing starts at the anchor (or lower end), 
with the anchor node given a value of zero, and the cable segment between nodes 0 and 1 given an index of 1/2.
 
The model uses a right-handed inertial reference frame with the z axis being measured positive up from the water plane, 
consistent with NREL’s FAST simulator.  Each node’s position is defined by a vector r.  Each segment of the cable has 
identical properties of unstretched length, diameter, density, and Young's modulus.  Different cables can have different 
sets of properties, and cables can be connected together at the ends, enabling mooring systems with interconnected lines. 

Hydrodynamic loads are calculated directly at the node points rather than at the segment centers.  This ensures damping of 
transverse cable vibrations having a wavelength of twice the cable segment length.  To approximate the cable direction at 
the node points, the cable tangent at each node is assumed to be the average of the tangent directions of the two 
adjacent cable elements.  Aside from this detail, the formulation of the mooring model is fairly standard.  
Further technical details and some validation results are available in :ref:`some papers <theory>`.

Bending stiffness is a recent capability addition in MoorDyn v2 (it is not yet implemented in MoorDyn-F). 
In the explanations that follow, the word cable is used to refer to a Line object with nonzero bending stiffness.


Points
^^^^^^

The ends of each mooring line are defined by Connection objects, which can be considered a special type of node.  
Using the same terminology as MAP [2], there are three Connection node types:

- 1: Fixed nodes have a certain location and never move.  They can be used as anchor points. OR attached to a platform for platform-centric coupling
- -1: Vessel nodes can move under the control of an outside program.  They can be used as fairlead connections.
- 0: Connect nodes are not fixed in space but rather are moved according to the forces acting on them.  
They are what can be used to connect two or more mooring lines together.  The forces they experience can include the forces from the attached 
mooring lines (which Fixed and Vessel node types also experience) but also constant external forces, buoyancy forces, inertial and 
gravitational forces, and hydrodynamic drag and added mass forces.  


Rods 
^^^^

Rod objects provide an option for rigid cylindrical elements within a mooring system. They have similar modeling details as 
Lines except for their rigidity, which reduces their degrees of freedom to six. Like Lines, they are divided into a number 
of nodes at which weight, buoyancy, seabed contact, 
and Morison-based hydrodynamic forces are calculated. Unlike Lines, their internal forces are not calculated. 
The end nodes of a rod are available for attachment of lines (specified like "R2A" for end A of Rod 2).

Rods can have 6, 3, or 0 DOF. 

- "Free" Rods are unconstrained to move in all 6 DOF. 
- "Pinned" Rods are attached at end A to something else, whether that is a body, the ground, or a coupling point. 
  This type of Rod only has three rotational degrees of freedom, about end A.
- "Fixed" Rods are full constrained, and their movement is defined by that of a body, the ground, or a coupling point.

Pinned or Fixed Rods attached to a body (e.g. body 1) are labelled "Body1Pinned" or "Body1". 
Pinned or fixed rods that serve as a coupling point are labelled "CoupledPinned" or "Coupled"

A special case exists if a Rod is specified with zero elements: in that case it is given zero length, and
its end B input coordinates are instead interpreted as vector components to describe its direction vector. 
This case is meant for convenience when making cantilever connections of a line with bending stiffness. 
A fixed zero-length rod can be used to make a cantilever connection of a power cable to the ground, a body, or a coupling point.
A free zero-length rod can be used to join two different types of power cable segments, and it will pass moments 
between the cable segments without adding any mass or other characteristics.


Bodies
^^^^^^

Body objects provide a generic 6 DOF rigid-body representation based on a lumped-parameter model of translational 
and rotational properties (e.g. hydrodynamic drag and added mass coefficients). 
Rod elements can be added to bodies and mooring lines can be attached at any location, 
allowing a wide variety of submerged structures to be integrated into the mooring system. 
Aside from contributions which might come from incorporated Rod objects or attached Connection 
and Line objects, the core Body object properties are as follows:

- mass, and center of mass
- volumetric displacement (assumed to be at reference point)
- mass moment of inertia about each axis
- hydrodynamic drag coefficient in each direction
- rotational hydrodynamic drag coefficient about each axis
- added mass coefficient in each direction
- added mass moment of inertia coefficient about each axis




