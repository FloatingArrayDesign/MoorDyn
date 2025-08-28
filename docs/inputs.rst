Input Files
===========
.. _inputs:

..
  customize code highlight color through "hll" span css

.. raw:: html

    <style> .highlight .hll {color:#000080; background-color: #eeeeff} </style>
    <style> .fast {color:#000080; background-color: #eeeeff} </style>
    <style> .stnd {color:#008000; background-color: #eeffee} </style>

.. role:: fast
.. role:: stnd

The V1 Input File
-----------------
.. _v1_inputs:

MoorDyn v1 uses a plain-text input file for its description of the mooring system as well as 
simulation settings. This file is divided into sections identified by header lines. The exact 
whitespace and alignment of the lines of the file is not important, so long as values are 
separated by at least one space.

Most of the sections are set up to contain a table of input information. The table begins with two 
preset lines that contain the column names and the corresponding units. These lines are followed by 
any number of lines containing the entries in that section's table of inputs.

Front matter
^^^^^^^^^^^^

The first lines of the input file until a section heading is provided are reserved for free-form 
user input, for labeling the input file, writing notes, etc.

.. code-block:: none
 :emphasize-lines: 2

 MoorDyn input file...
 MoorDyn v1 sample input file
 True          Echo          echo the input file data (flag)

Line Types
^^^^^^^^^^

The Line Types section of the file contains one or more definitions of physical line properties and 
four hydrodynamic coefficients. 

.. code-block:: none
 :emphasize-lines: 2

 ------------------------- LINE TYPES --------------------------------------------------
 LineType  Diam    MassDen    EA       BA/-zeta   Can   Cat   Cdn   Cdt 
 (-)       (m)     (kg/m)     (N)      (Pa-s/-)   (-)   (-)   (-)   (-)  
 nylon    0.124    13.76    2515288.0    -0.8     1.0   0.0   1.6   0.05

The columns in order are as follows:

 - Name – an identifier word for the line type
 - Diam –  the volume-equivalent diameter of the line – the diameter of a cylinder having the same 
   displacement per unit length (m)
 - MassDen –  the mass per unit length of the line (kg/m)
 - EA – the line stiffness, product of elasticity modulus and cross-sectional area (N)
 - BA/-zeta –  the line internal damping (measured in N-s) or, if a negative value is entered, the 
   desired damping ratio (in fraction of critical) for the line type (and MoorDyn will set the BA 
   of each line accordingly – see Section 4.1 for more information)
 - Can –  transverse added mass coefficient (with respect to line displacement)
 - Cat – tangential added mass coefficient (with respect to line displacement)
 - Cdn –  transverse drag coefficient (with respect to frontal area, d*l)
 - Cdt –  tangential drag coefficient (with respect to surface area, π*d*l) 

Point Properties
^^^^^^^^^^^^^^^^^^^^^

The Point Properties section defines the point node points which mooring lines can be connected to.

.. code-block:: none
  :emphasize-lines: 2

  ----------------------- POINT PROPERTIES ----------------------------------------------
  Node      Type      X        Y         Z        M        V        FX       FY      FZ     CdA   CA
  (-)       (-)      (m)      (m)       (m)      (kg)     (m^3)    (kN)     (kN)    (kN)   (m^2)  (-)
  1         Vessel     0.0     0      -10.00       0        0        0        0       0       0     0
  2         Fixed    267.0     0      -70.00       0        0        0        0       0       0     0
  3         Connect    0.0     0      -10.00       0        0        0        0       0       0     0

The columns are as follows:

 - Node –  the ID number of the point (must be sequential starting with 1)
 - Type –  one of “Fixed”, “Vessel”, or “Connect”, as described :ref:`here <points>`
 - X, Y, Z –  Coordinates of the point (relative to global reference frame if “fixed” or “point”, 
   relative to platform reference frame if “vessel”).  In the case of “point” nodes, it is simply 
   an initial guess for position before MoorDyn calculates the equilibrium initial position.(m)
 - M – node mass in the case of clump weights (kg)
 - V –  node displacement in the case of floats (m^3)
 - FX, FY, FZ –  any steady external forces applied to the node (N)
 - CdA –  product of drag coefficient and projected area (assumed constant in all directions) to 
   calculate a drag force for the node (m^2)
 - Ca –  added mass coefficient used along with V to calculate added mass on node

Lines list
^^^^^^^^^^

The Line Properties section defines each uniform-property section of mooring line to be simulated.

.. code-block:: none
  :emphasize-lines: 2
 
  -------------------------- LINE PROPERTIES -------------------------------------------------
  Line     LineType  UnstrLen  NumSegs    NodeAnch  NodeFair  Flags/Outputs
  (-)      (-)       (m)         (-)       (-)       (-)      (-)
  1        nylon     300.0        50        2         1        p
  2        nylon     300.0        50        4         3        p
  3        nylon     300.0        50        6         5        p

The columns are as follows:

 - Line - the ID number of the line (must be sequential starting with 1)
 - LineType - a string matching a Line Dictionary entry, specifying which physical properties it 
   uses
 - UnstrLen - the unstretched length of the line
 - NumSegs - how many segments the line is discretized into (it will have NumSegs+1 nodes total, 
   including its two end nodes)
 - NodeAnch - the ID number of the point that the first (anchor) end of the line is attached to
 - NodeFair - the ID number of the point that the final (fairlead) end of the line is attached to
 - flags/outputs - any data to be output in a dedicated output file for that line. 
   
This last entry expects a string of one or more characters without spaces, each character 
activating a given output property. A placeholder character such as “-” should be used if no 
outputs are wanted.  Eight output properties are currently possible:

 - p – node positions
 - v – node velocities
 - U – wave velocities at each node
 - D – hydrodynamic drag force at each node
 - t – tension force at each segment 
 - c – internal damping force at each segment
 - s – strain of each segment
 - d – rate of strain of each segment

For example, outputting node positions and segment tensions could be achieved by writing “pt” for 
this last column.  These outputs will go to a dedicated output file for each line only.  For 
sending values to the global output file, use the Outputs section instead.


Options
^^^^^^^

The Solver Options section can contain any number of optional settings for the overall model, 
including seabed properties, initial condition (IC) generation settings, and the time step size. 

.. code-block:: none

 -------------------------- SOLVER OPTIONS---------------------------------------------------
 0.001    dtM          - time step to use in mooring integration
 3.0e6    kb           - bottom stiffness
 3.0e5    cb           - bottom damping
 70       WtrDpth      - water depth
 5.0      ICDfac       - factor by which to scale drag coefficients during dynamic relaxation IC gen
 0.001    ICthresh     - threshold for IC convergence
 0        ICTmax       - threshold for IC convergence (set to zero for debugging)

Any of these lines can be omitted, in which case default values will be used (shown in 
parenthesis). As such, they are all optional settings, although some of them (such as time step 
size) often need to be set by the user for proper operation. The list of possible options (with any 
default value provided in parentheses) is:

 - dtM (0.001) – desired mooring model time step (s)
 - g (9.80665) – gravitational constant (m/s^2)
 - rhoW (1025.0)– water density (kg/m^3)
 - WtrDpth (0.0) – water depth (m)
 - SeafloorFile (none) – Relative path of a 3D Seafloor file
 - kBot (3.0e6) – bottom stiffness constant (Pa/m) 
 - cBot (3.0e5)– bottom damping constant (Pa-s/m)
 - dtIC (1.0)– period for analyzing convergence of dynamic relaxation IC generation (s)
 - TmaxIC (120.0) – maximum simulation time to allow for IC generation without convergence (s)
 - CdScaleIC (5.0) – factor by which to scale drag coefficients to accelerate convergence of IC 
   generation (-)
 - ThreshIC (0.001) – convergence threshold for IC generation, acceptable relative difference 
   between three successive fairlead tension measurements (-)

The bottom contact parameters, kBot and cBot, result in a pressure which is then applied to the 
cross-sectional area (d*l) of each contacting line segment to give a resulting vertical contact 
force for each segment.
 
Outputs
^^^^^^^

The Outputs section is used to specify general outputs, which are written to the main MoorDyn 
output file. 

.. code-block:: none
 :emphasize-lines: 8
 
 ---------------------- OUTPUTS -----------------------------------------
 FairTen1
 FairTen2
 AnchTen1
 Con2px
 Con2py
 Con2Fz
 END
 ------------------------- need this line -------------------------------------

Each output channel name should have its own line.  There are keywords for fairlead and anchor 
tensions of a given line: fairten# and anchten#, where # is the line number. There is also a 
flexible naming system for outputting other quantities. There are currently five supported types of 
output quantities:

 - pX, pY , pZ  – x/y/z coordinate (m)
 - vX, vY, vZ – velocity (m/s)
 - aX, aY, aZ – acceleration (m/s^2)
 - T or Ten – tension (N)
 - fX, fY, fZ – net force in x/y/z direction (N)

These can be produced at a point object, denoted by the prefix Con#, where # is the point number.  
Or, they can be produced at a node along a line, denoted by the prefix L#N@, where # is the line 
number and @ is the number of the node along that line.  For example,

 - Con3vY outputs the point 3 y velocity,
 - L2N4pX outputs the line 2, node 4 x position.

The V2 Input File
-----------------
.. _v2_inputs:

MoorDyn v2 uses a standardized plain-text input file for its description of the
mooring system and simulation settings that has some important additions and
changes from V1.

Most helpfully, this new format is identical between C++ and FORTRAN versions of
MoorDyn, and it is designed to support future capability enhancements without
requiring format changes.

This file is divided into sections, some of which are optional. Each section is
identified (and detected) by a header line consisting of a key phrase (e.g. Line
Types) surrounded by dashes. While a couple sections are optional, the order of
the sections cannot be changed. The exact whitespace and alignment of the lines of the file is not 
important, as long as values are separated by at least one space. However, every column must have 
a value. MoorDyn only reads the values of each column, not the column headers or units. The column 
identifiers and units can be changed by the user but should use similar names to retain readability. 

To successfully run a simulation, MoorDyn requires at least one line. If you are aiming to simulate 
a system with no lines, the best approach is to create a short taut vertical line stretched between 
two fixed points located far from where your system is located. 

Most of the sections are set up to contain a table of input information. These
tables begin with two preset lines that contain the column names and the
corresponding units. These lines are followed by any number of lines containing
the entries in that section's table of inputs. # is the general comment character. If you are adding notes 
to self after any of the lines, # will prevent MoorDyn from reading them. 

Examples of input files for MoorDyn-C can be found in the `test directory <https://github.com/FloatingArrayDesign/MoorDyn/tree/master/tests/Mooring>`_ (note that these do not include outputs because they are for tests).

Examples for MoorDyn-F can be found in the `OpenFAST tests <https://github.com/OpenFAST/r-test/tree/main/modules/moordyn>`_. 

Front matter
^^^^^^^^^^^^

The first lines of the input file are reserved for free-form user input, for
labeling the input file, writing notes, etc.
There is not a limit on the number of lines you can write here.

.. code-block:: none

 --------------------- MoorDyn Input File ------------------------------------
 MoorDyn v2 sample input file

Line Types
^^^^^^^^^^

This section is required and describes the list of mooring line properties
that will be used in the simulation 

.. code-block:: none

 ---------------------- LINE TYPES ----------------------------------------------------------------------
 TypeName   Diam    Mass/m     EA     BA/-zeta    EI         Cd     Ca     CdAx    CaAx    Cl    dF    cF        
 (name)     (m)     (kg/m)     (N)    (N-s/-)     (N-m^2)    (-)    (-)    (-)     (-)     (-)   (-)   (-) 
 Chain      0.1      150.0     1e8    -1          0          2.3     1     1.0     0.5     0.8   0.08  0.18

The columns in order are as follows:

 - TypeName – an identifier word for the line type
 - Diam –  the volume-equivalent diameter of the line – the diameter of a cylinder having the same 
   displacement per unit length (m)
 - MassDen –  the mass per unit length of the line (kg/m)
 - EA – the line stiffness, product of elasticity modulus and cross-sectional area (N)
 - BA/-zeta –  the line internal damping (measured in N-s) or, if a negative value is entered, the 
   desired damping ratio (in fraction of critical) for the line type (and MoorDyn will set the BA 
   of each line accordingly)
 - EI – the line bent stiffness, product of elasticity modulus and inertia of the cross-sectional 
   area (N-m^2)
 - Cd –  transverse drag coefficient (with respect to frontal area, d*l)
 - Ca –  transverse added mass coefficient (with respect to line displacement)
 - CdAx –  tangential drag coefficient (with respect to surface area, π*d*l)
 - CaAx – tangential added mass coefficient (with respect to line displacement)
 - Cl – OPTIONAL - the crossflow VIV lift coefficient. If set to 0, then VIV calculations are disabled for the
   line. This coefficient has been made backwards compatible. If it is not provided, then it is 
   assumed to be 0.0. The theory of vortex induced vibrations can be found :ref:`here <version2>`. Note that VIV is disabled
   for end nodes (and thus end half-segments), so if simulating VIV users should ensure to include a higher number of segments. 
   Also note that VIV has only been tested with explicit time schemes (specifically rk2 and rk4). There may be unexpected behavior 
   if used with an implicit time scheme. 
 - dF - OPTIONAL - the cF +- range of non-dimensional frequencies for the CF VIV synchronization model. If it is not
   provided and VIV is enabled (Cl > 0) then it is default to 0.08 to align with the the theory found :ref:`here <version2>`
 - cF - OPTIONAL - the center of the range of non-dimensional frequencies for the CF VIV synchronization model. If it is not
   provided and VIV is enabled (Cl > 0) then it is default to 0.18 to align with the the theory found :ref:`here <version2>`

Note: Non-linear values for the stiffness (EA) are an option in MoorDyn. For this, a file name can be provided instead of a number. This file 
must be located in the same folder as the main MoorDyn input file for MoorDyn-C or for MoorDyn-F 
in the same folder as the executable calling MoorDyn-F, unless a path is specified. Such file is a 
tabulated file with 3 header lines and then a strain column and a tension column separated by a blank space:

.. code-block:: none

  ----Polyester----
  Strain    Tension
  (-)       (N)
  0.0       0.0
  ...       ...

Note: MoorDyn has the ability to model the viscoelastic properties of synthetic lines in two ways. The first method, from work linked in the 
:ref:`theory section <theory>`, allows a user to specify a bar-separated constant dynamic and static stiffness. The second method allows the user 
to provide a constant static stiffness and two terms to determine the dynamic stiffness as a linear function of mean load. The equation is:
`EA_d = EA_Dc + EA_D_Lm * mean_load` where `EA_D_Lm` is the slope of the load-stiffness curve. Both of these methods allow users to provide static 
and dynamic damping coefficients as values separated by |. While the static damping can be described as a fraction of critical, the dynamic damping 
needs to be input as a value. Example inputs are below: 

.. code-block:: none

  TypeName   Diam    Mass/m     EA                   BA
  (name)     (m)     (kg/m)     (N)                 (N-s)
  Polyester  ...      ...    EA_s|EA_d            BA_s|BA_d <-- Constant dynamic stiffness method with static and dynamic damping
  Polyester  ...      ...    EA_s|EA_Dc|EA_D_Lm   BA_s|BA_d <-- Load dependent dynamic stiffness method with static and dynamic damping

Rod Types
^^^^^^^^^

This section (required if there are any rod objects) describes the list of rod property sets that 
will be used in the simulation. 

.. code-block:: none

 ---------------------- ROD TYPES ------------------------------------
 TypeName      Diam     Mass/m    Cd     Ca      CdEnd    CaEnd       
 (name)        (m)      (kg/m)    (-)    (-)     (-)      (-)         
 Buoy          10       1.0e3     0.6    1.0     1.2      1.0      

The columns are as follows:
 - TypeName – an identifier word for the rod type
 - Diam – the cylinder diameter (m)
 - Mass/m – the mass per unit length of the rod (kg/m)
 - Cd – the normal rod drag coefficient (with respect to the central axis of the rod)
 - Ca – the normal rod added mass coefficient (with respect to the central axis of the rod)
 - CdEnd – the axial drag coefficient of the rod
 - CaEnd – the axial added mass coefficient of the rod 

Bodies list
^^^^^^^^^^^

This section (optional) describes the 6DOF body objects to be simulated. 

.. code-block:: none

 ---------------------- BODIES ---------------------------------------
 ID   Attachment  X0     Y0    Z0     r0      p0     y0     Mass  CG*   I*      Volume   CdA*   Ca*
 (#)   (word)     (m)    (m)   (m)   (deg)   (deg)  (deg)   (kg)  (m)  (kg-m^2)  (m^3)   (m^2)  (-)
 1     coupled     0     0      0     0       0      0       0     0     0        0       0      0
 
The columns are as follows:
 - ID –  the ID number of the Body (must be sequential starting with 1)
 - Attachment –  one of “Fixed”, “Vessel”, “Free”, etc, as described :ref:`here <bodies>`
 - X0/Y0/Z0 – Coordinates of the body relative to the global reference frame. Note that bodies 
   must have Z0 <= 0 (m)
 - r0/p0/y0 – Orientation of the body relative to the global reference frame in Euler angles 
   (deg)
 - Mass – Body mass not including attached rods and points. Typically used to account for above 
   surface mass such as a turbine (kg)
 - CG – Body center of gravity. If one value given, it is along the Z axis. To specify a coordinate 
   point, the XYZ values are listed separated by | with no spaces (m)
 - I – Body moment of inertia diagonals for the 3x3 inertia matrix. If one value given, it is used 
   for all three values. To specify three different values, the inputs are listed separated by | with no 
   spaces (kg-m^2)
 - Volume – The body displaced volume used in buoyancy calculations excluding attached rod and 
   point volume contributions (m^3)
 - CdA – The product of drag coefficient and frontal area of body. If one value given, it is used 
   for all six values. If two values are given, the first is used for the translational drag and 
   the second for the rotational drag. If three values are given, they are used as both the 
   translational and rotational drag coefficicents for the respective axis. If six values are given, 
   they are used as the drag coefficicnets in the respective degree of freedom. To specify different 
   values, the inputs are listed separated by | with no spaces (m^2)
 - Ca – The body added mass coefficient. If one value given, it is used for all three values. To 
   specify three different values, the inputs are listed separated by | with no spaces

Rods list
^^^^^^^^^

This section (optional) describes the rigid Rod objects 

.. code-block:: none

 ---------------------- RODS ----------------------------------------
 ID   RodType  Attachment  Xa    Ya    Za    Xb    Yb    Zb   NumSegs  RodOutputs
 (#)  (name)   (word/ID)   (m)   (m)   (m)   (m)   (m)   (m)  (-)       (-)
 1      Can      Body1      0     0     2     0     0     15   8         p
 2      Can   Body1Pinned   2     0     2     5     0     15   8         p
 
The columns are as follows:
 - ID –  the ID number of the Rod (must be sequential starting with 1)
 - RodType
 - Attachment –  one of “Fixed”, “Vessel”, “Pinned”, etc, as described :ref:`here <rods>`
 - Xa/Ya/Za – Coordinates of the A end (relative to global reference frame if “fixed/point/free”,
   or relative to platform/body reference frame if "body#"" or “body#pinned”) (m)
 - Xb/Yb/Zb – Coordinates of the B end (relative to global reference frame if “fixed/free”,
   or relative to platform/body reference frame if "body#"" or “body#pinned”) (m)
 - NumSegs - how many segments the rod is discretized into (it will have NumSegs+1 nodes total, 
   including its two end nodes)
 - RodOutputs - any data to be output in a dedicated output file for the rod.

This last entry expects a string of one or more characters without spaces, each character 
activating a given output property. A placeholder character such as “-” should be used if no 
outputs are wanted. Eight output properties are currently possible:

 - p – node positions
 - v – node velocities
 - U – wave/current velocities at each node
 - D – hydrodynamic drag force at each node
 - t – tension force at each segment 
 - c – internal damping force at each segment
 - s – strain of each segment
 - d – rate of strain of each segment
 - f – net node force 
 - W – weight at each node
 - B – buoyancy force at each node
 - P – dynamic pressure at each node
 - X – transverse drag force at each node
 - Y – tangential drag force at each node
 - A – transverse fluid inertia force at each node
 - a – tangential fluid inertia force at each node
 - b – bottom contact force
For example, outputting node positions and segment tensions could be achieved by writing “pt” for 
this last column.  These outputs will go to a dedicated output file for each rod.  For sending 
values to the global output file, use the Outputs section instead.

Points list
^^^^^^^^^^^

This section (optional) describes the Point objects 
 
.. code-block:: none

 ---------------------- POINTS ---------------------------------------
 ID   Attachment  X       Y     Z      Mass   Volume  CdA    Ca
 (#)   (word/ID) (m)     (m)   (m)    (kg)   (mˆ3)   (m^2)  (-)
 1     Fixed      -500    0     -150    0      0       0      0
 2     Coupled    0       0     -9      0      0       0      0
 3     Body2      0       0     1.0     0      0       0      0
 
The columns are as follows:

 - ID –  the ID number of the point (must be sequential starting with 1)
 - Attachment –  one of “Fixed”, “Coupled”, “Free”, etc, as described :ref:`here <points>`
 - X, Y, Z –  Coordinates of the point (relative to global reference frame if “fixed/point/free/coupled”,
   or relative to body reference frame if “body#”).  In the case of 
   “point/free” nodes, it is simply an initial guess for position before MoorDyn calculates the 
   equilibrium initial position.  (m)
 - Mass – node mass in the case of clump weights (kg)
 - Volume –  node displacement in the case of floats (m^3)
 - CdA –  product of drag coefficient and projected area (assumed constant in all directions) to 
   calculate a drag force for the node (m^2)
 - Ca –  added mass coefficient used along with V to calculate added mass on node

Lines list
^^^^^^^^^^

This section (required) describes the Line objects, typically used for mooring lines or dynamic 
power cables.

.. code-block:: none

 ---------------------- LINES ----------------------------------------
 ID   LineType   AttachA  AttachB  UnstrLen  NumSegs  LineOutputs
 (#)   (name)     (ID)     (ID)      (m)       (-)      (-)
 1     Chain       1        2        300        20       p
                    
The columns are as follows:

 - ID - the ID number of the line (must be sequential starting with 1)
 - LineType - a string matching a Line Dictionary entry, specifying which physical properties it 
   uses
 - AttachA - the ID number of the point (or Rod end) that the first (anchor) end of the line is 
   attached to. For lines connected to rod ends, the value should be R#A or R#B where # is the rod 
   number and A/B refer to which end of the rod the line is connected to.
 - AttachB - the ID number of the point (or Rod end) that the final (fairlead) end of the line is 
   attached to. For lines connected to rod ends, the value should be R#A or R#B where # is the rod 
   number and A/B refer to which end of the rod the line is connected to.
 - UnstrLen - the unstretched length of the line
 - NumSegs - how many segments the line is discretized into (it will have NumSegs+1 nodes total, 
   including its two end nodes)
 - LineOutputs - any data to be output in a dedicated output file for that line. 
   
This last entry expects a string of one or more characters without spaces, each character 
activating a given output property. A placeholder character such as “-” should be used if no 
outputs are wanted. Ten output properties are currently possible:

 - p – node positions
 - v – node velocities
 - U – wave/current velocities at each node
 - D – hydrodynamic drag force at each node
 - t – tension force at each segment 
 - c – internal damping force at each segment
 - V - the cross-flow VIV lift force at each node
 - K - the curvature at each node
 - s – strain of each segment
 - d – rate of strain of each segment

For example, outputting node positions and segment tensions could be achieved by writing “pt” for 
this last column.  These outputs will go to a dedicated output file for each line only.  For 
sending values to the global output file, use the Outputs section instead.

Failure (MoorDyn-F only)
^^^^^^^^^^^^^^^^^^^^^^^^

This section (optional) describes the failure conditions of the system. Line failures can only be
triggered at an existing point or rod end where a line/lines are attached. Failures can be 
triggered by a time or attachment tension threshold, which ever comes first. Users can specify
multiple failures for a given point, but duplicate failure configurations will be ignored.
If two lines attached to a point are listed to fail (failure 1 for example), then after the failure
the lines will remain attached to each other by a free point. In this multi line case, if any line
reaches the tension threshold then the failure will be triggered.

.. code-block:: none

  ---------------------- FAILURE ----------------------
  FailureID  Point    Line(s)   FailTime   FailTen
  ()           ()       (,)      (s or 0)   (N or 0)
  1          5        1,2,3,4     0         1200e3
  2          3        1           0         1200e3
  3         R1a       1,2,3       12          0

Control (MoorDyn-F only)
^^^^^^^^^^^^^^^^^^^^^^^^

This section (optional) is only available for MoorDyn-F and describes which lines are assigned to 
which control channel in the ServoDyn input file. Setting up active tension controls involves 
modifying the MoorDyn and ServoDyn input file and passing deltaL and deltaLdot control command values 
into the appropriate channel of the OpenFAST S function (``FAST_SFunc``).

.. code-block:: none

  ---------------------- CONTROL ----------------------
  ChannelID  Line(s)
  ()       (,)     
  1        1,2,3,4 
  2        5

In the example above, channel 1 is used to control lines 1-4 and channel 2 is used to control line 5.
These channel numbers must correspond to control channels in the ServoDyn input file. The ServoDyn 
summary file (enabled with ``SumPrint`` option) will contain a list of all line control channels in 
use and what they are assigned to.

An example set up using controls in Simulink would look like this:
 
For MoorDyn, add the control section below and specify which lines should be actuated using the tension 
control. In the below snippet, 5 lines are being independently controlled.

.. code-block:: none

  ---------------------- CONTROL ----------------------
  ChannelID Line(s)
  () (,)
  1 1
  2 2
  3 3
  4 4
  5 5
 
There are a total of 20 available channels for line deltaL command in the S function, and 20 available for 
the line deltaLdot command in the S function. So when passing in an array of values, pass in 0 for any unused 
channels. With all the other inputs (see example file described below for other controllers), the S function 
will take in a total of 51 channels.
 
In ServoDyn, change the cable control channel to 4 for Simulink control.

.. code-block:: none

  ---------------------- CABLE CONTROL -------------------------------------------
            4   CCmode       - Cable control mode {0: none, 4: user-defined from Simulink/Labview, 5: user-defined from Bladed-style DLL} (switch)
 
To run the active tension control using Simulink, an example .mdl Simulink file and a .m matlab script 
to run the .mdl file are available in the OpenFAST GitHub repo under `glue-codes/simulink/examples <https://github.com/OpenFAST/openfast/tree/main/glue-codes/simulink/examples>`_
to show how to pass in values. A .dll OpenFAST shared library file and .mex OpenFAST S function file need to be in 
the same directory the controller is being run from.

External Loads (MoorDyn-F only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section (optional) allows users to specify external forces (constant) and translational linear and quadratic 
damping coefficients to MoorDyn point, rod, and body objects.

.. code-block:: none

  ---------------------- EXTERNAL LOADS --------------------------------
  ID    Object          Fext             Blin          Bquad         CSys
  (#)   (name)           (N)            (Ns/m)       (Ns^2/m^2)      (-)
  1    Body1        0.0|0.0|0.0     0.0|0.0|0.0     0.0|0.0|0.0      G
  2    Body1        0.0|0.0|0.0     0.0|0.0|0.0     0.0|0.0|0.0      L
  3    Point1       0.0|0.0|0.0     0.0|0.0|0.0     0.0|0.0|0.0      -
  4    Rod1         0.0|0.0|0.0     0.0|0.0         0.0|0.0          -
  
For bodies, the force and damping are applied at the 
body reference point in the global earth-fixed coordinate system if CSys is G for global or in the local body-fixed 
coordinate system if CSys is L for local. CSys can only be G or L for bodies. For points, the force and damping are 
applied at the point location in the global earth-fixed coordinate system always. CSys should be -. Otherwise, a 
warning message with explanation will be shown. For rods, the force is applied at the rod end A in the earth-fixed 
coordinate system always. Only two linear and two quadratic damping coefficients can be specified for rods. The first 
one is for the transverse direction, and the second one is for the axial/tangential direction. The damping force is 
always evaluated in the body-fixed system. CSys should be -. Otherwise, a warning message with explanation will be shown. 

Options
^^^^^^^

This section (required) describes the simulation options

.. code-block:: none

 ---------------------- OPTIONS -----------------------------------------
 1             writeLog      Write log file
 0.002         dtM           time step to use in mooring integration (s)
 3000000       kbot          bottom stiffness (Pa/m)
 300000        cbot          bottom damping (Pa-s/m)
 0.5           dtIC          time interval for analyzing convergence during IC gen (s)
 10            TmaxIC        max time for ic gen (s)
 0.001         threshIC      threshold for IC convergence (-)
 
The options list differs from the other sections in that it uses a value + key approach.
MoorDyn reads in the value, and then assigns it to the corresponding key. This means the 
order of the options list does not matter, however any options listed above the ``writeLog`` 
flag will not be included in the log file. Any of these option lines can be omitted, in which 
case default values will be used (shown in parentheses). Default values with a C or a F 
indicate which version has that as the default. As such, they are all optional settings, although 
some of them (such as time step size) often needs to be set by the user for proper operation. 
The list of possible options is:

 - writeLog (0 C, -1 F): If >0 a log file is written recording information. The
   bigger the number the more verbose. Please, be mindful that big values would
   critically reduce the performance!
 - dtM (3.402823e+38 C, coupling timestep size F) – desired mooring model maximum time step (s). In
   MoorDyn-F if this is left blank it defaults to the
   :ref:`driver file <MDF_driver_in>` dtC value or the OpenFAST time step.
 - CFL (0.5) – Desired mooring model maximum Courant-Friedich-Lewy factor. CFL is the ratio 
   between the time step and the natural period, computed considering the math described in
   :ref:`the troubleshooting section <troubleshooting>`.
 - tScheme (RK2): The time integrator. It should be one of
   Euler, LEuler, Heun, RK2, RK4, AB2, AB3, AB4, LAB2, LAB3, LAB4, 
   BEuler\ *N*, Midpoint\ *N*, ACA\ *N*, Wilson\ *N*. Look at the
   :ref:`time schemes documentation <tschemes>` to learn more about this.
 - g (9.81): The gravity acceleration (m/s^2)
 - rho (1025): The water density (kg/m^3)
 - WtrDpth (0.0): The water depth (m). In MoorDyn-F the bathymetry file path can be inputted here.
 - kBot (3.0e6): The bottom stiffness (Pa/m)
 - cBot (3.0e5): The bottom damping (Pa-s/m)
 - dtIC (1.0 C, 2.0 F): The threshold amount of time the system must be converged for to be 
   considered stationary (s)
 - TmaxIC (120.0 C, 60.0 F): The maximum simulation time to run in order to find a stationary 
   initial condition (s)
 - CdScaleIC (5.0 C, 4.0 F): The damping scale factor during the initial condition computation
 - threshIC (0.001 C, 0.01 F): The lines tension maximum relative error to consider that the 
   initial condition have converged
 - WaveKin (0): The waves model to use. 0 = none, 1 = waves externally driven, 2 = FFT in a regular 
   grid, 3 = kinematics in a regular grid, 7 = Wave Component Summing. Details on these flags can
   be found :ref:`here <waterkinematics>`.
 - dtWave (0.25): The time step to evaluate the waves, only for wave grid (WaveKin = 3) (s)
 - Currents (0): The currents model to use. 0 = none, 1 = steady in a regular grid, 2 = dynamic in 
   a regular grid, 3 = WIP, 4 = WIP, 5 = 4D Current Grid. Details on these flags can
   be found :ref:`here <waterkinematics>`.
 - UnifyCurrentGrid (1): When both waves and currents are defined using a grid method, you may want 
   to pre-combine those grids into a single grid that stores the summed wave and current kinematics. 
   When this option is 1 the wave grid points get the interpolated current grid values added to 
   them. When this option is 0 the wave grid and current grid are kept separate
 - WriteUnits (1): 0 to do not write the units header on the output files, 1 otherwise
 - FrictionCoefficient (0.0): The seabed friction coefficient
 - FricDamp (200.0): The seabed friction damping, to scale from no friction at null velocity to 
   full friction when the velocity is large
 - StatDynFricScale (1.0): Ratio between Static and Dynamic friction coefficients
 - dtOut (0.0): Time step size to be written to output files. A value of zero will use the coupling 
   timestep as a step size (s)
 - SeafloorFile: A path to the :ref:`bathymetry file <seafloor_in>`
 - ICgenDynamic (0): MoorDyn-C switch for using older dynamic relaxation method (same as MoorDyn-F).
   If this is enabled initial conditions are calculated with scaled drag according to CdScaleIC. 
   The new stationary solver in MoorDyn-C is more stable and more precise than the dynamic solver, 
   but it can take longer to reach equilibrium.
 - disableOutput (0): Disables some console and file outputs to improve runtime. 
 - disableOutTime (0): Disables the printing of the current timestep to the console, useful for running with MATLAB

A note about time steps in MoorDyn-C: The internal time step is first taken from the dtM option. If
no CFL factor is provided, then the user provided time step is used to calculate CFL and MoorDyn-C 
runs using the user time step. If no time step is provided, then the user provided CFL factor is 
used to calculate the time step and MoorDyn-C uses this calculated time step. If both the time step
and CFL are provided, MoorDyn-C uses the smaller time step between user provided and CFL 
calculated.

In MoorDyn-F, the default values for g, rhoW, and WtrDpth are the values
provided by FAST, so it is  recommended to not use custom values for the sake
of consistency.

The following MoorDyn-C options are not supported by MoorDyn-F: 

 - CFL: In MoorDyn-F the time step is governed by the
   :ref:`driver file <MDF_driver_in>` dtC value or the OpenFAST time step. To
   override it just the option dtM is available.
 - WaveKin & Currents: In MoorDyn-F waves and currents are combined into a single option called 
   WaterKin which takes a file path as a value and defaults to an empty string (i.e. no WaterKin). 
   The file provided should be formatted as described in the additional MoorDyn inputs 
   :ref:`section <MDF_wtrkin>`. Further details on its implementation can be found in the 
   :ref:`water kinematics section <waterkinematics>`.
 - tScheme: MoorDyn-F only uses the Runge-Kutta 2 method for time integration. 
 - dtWave: MoorDyn-F uses the dtWave value from the :ref:`water kinematics file <MDF_wtrkin>`.
 - unifyCurrentGrid: Not available in MoorDyn-F because currents and waves are handled in the same 
   input file.
 - writeUnits: Units are always written to output file headers
 - SeafloorFile: MoorDyn-F accepts a bathymetry file path as an alternative to
   a number in the WtrDpth option
 - FrictionCoefficient: MoorDyn-F contains friction coefficients for lines in both the axial and 
   transverse directions while MoorDyn-C only has a general seafloor contact coefficient of friction
 - FricDamp: Same as CV in MoorDyn-F.
 - StatDynFricScale: Same as MC in MoorDyn-F.
 - ICgenDynamic: MoorDyn-F does not have a stationary solver for initial conditions
 - disableOutput: MoorDyn-F output verbosity is controlled by OpenFAST

The following options from MoorDyn-F are not supported by MoorDyn-C: 

 - WaterKin (Null): Path to the water kinematics file or the SEASTATE Keyword. The formatting of the 
   water kinematics file can be found :ref:`here <MDF_wtrkin>`. Details on the different MoorDyn-F 
   water kinematics options can be found in the :ref:`MoorDyn-F water kinematics section <waterkinematics-F>`.
 - MU_KT (0.0): Transverse line coefficient of friction.
 - MU_KA (0.0): Axial line coefficient of friction.
 - MC (1.0): Same as StatDynFricScale in MoorDyn-C.
 - CV (200.0): Same as FricDamp in MoorDyn-C.
 - inertialF (0): Toggle to include inertial components in the returned forces from coupled 
   bodies and rods. Transients in the acceleration passed into MoorDy-F by OpenFAST can result 
   in large non-physical forces and moments which can cause instability in the model [0: no, 
   1: yes, 2: yes with ramp to inertialF_rampT]
 - inertialF_rampT (30.0): Ramp time for inertial forces to reduce coupled object instability (s). 
   This is ignored unless inertialF = 2
 - OutSwitch (1): Switch to disable outputs when running with full OpenFAST simulations, where the 
   MoorDyn-F output channels are written to the main FAST output file. 
   0: no MD main outfile, 1: write MD main outfile

Outputs
^^^^^^^

This section (optional) lists any specific output channels to be written in the main output file. 
All output flags need to be all caps. The section needs to end with another header-style line (as 
shown below) for MoorDyn to know when to stop reading inputs.

.. code-block:: none

 ---------------------- OUTPUTS -----------------------------------------
 BODY1PX
 BODY1PY
 BODY1PZ
 BODY1RX
 BODY1RY
 LINE1N15FX
 LINE1NAPZ
 FAIRTEN1
 FAIRTEN2
 FAIRTEN3
 ANCHTEN1
 ANCHTEN2
 ANCHTEN3
 ------------------------- need this line -------------------------------------

The avaible output flags are decribed in the table below:

========= ============================ =========  ===========  ==============  ===========  ===========
Suffix    Description                  Units      Line         Rod             Body         Point
========= ============================ =========  ===========  ==============  ===========  ===========
PX/PY/PZ  Position                     [m]        Node         Object/Node	   Object       Object
RX/RY     Roll, Pitch                  [deg]                   Object          Object
RZ        Yaw                          [deg]                                   Object	
VX/VY/VZ  Velocity                     [m/s]      Node         Object/Node     Object       Object
RVX/RVY   Rotational Velocity X/Y      [deg/s]                 Object          Object	
RVZ       Rotational Velocity Z        [deg/s]                                 Object	
AX/AY/AZ  Acceleration                 [m/s^2]                 Object          Object       Object
RAX/RAY   Rotational acceleration X/Y  [deg/s^2]               Object          Object	
RAZ       Rotational acceleration Z    [deg/s^2]                               Object	
FX/FY/FZ  Force                        [N]        Node(1)      Object/Node(2)  Object       Object
MX/MY/MZ  Moments                      [Nm]                    Object          Object	
TEN/T     Net force                    [N]        Node(3)                      Object       Object
TENA/B    Net force on ends            [N]        Object       Object		
SUB       Submergence (frac of length) [frac]                  Object		
========= ============================ =========  ===========  ==============  ===========  ===========

When a node number is specified, the output pertains to that node and its kinematics or associated 
loads. In the output flag this node numer is given as [OBJECT#]N#[SUFFIX], i.e LINE1N15PX. When no 
node number is specified, the output pertains to the object as a whole and the values are of the 
object’s reference point (about the reference point for rotations). The output flag without a node 
number looks like [OBJECT#][SUFFIX], i.e. ROD1SUB.

Reference Points:

- Rods: End A (Node 0)
- No z rotations for rods (rotations along axis of rod negligible)
- A vertical rod with end A below end B is defined as a rod with zero rotation. ROD#RX and ROD#RY 
  will be zero for this case. 
- Bodies: Center of Mass
- Points: Center of Mass
- Lines: End A (Node 0)

Footnotes:

- The tension on the Line n fairlead can be output with the FAIRTEN[n] flag (see examples above)
- The tension on the Line n anchor can be output with the ANCHTEN[n] flag (see examples above)
- Object indicates output is for whole object, Node indicates output is for node of object
- Coupled/fixed bodies and points will output acceleration 0 because no forces are calculated
- When looking at the rotational outputs of coupled pinned rods that are hanging near vertical, 
  the best approach is to attach a rod to a zero-mass, zero-volume pinned body and output the body 
  rotations. Hanging pinned rods are inverted (end A over end B) in MoorDyn and the output range 
  for roll/pitch of rods is +/- 180 degrees. 
- There are a couple additional outputs left over from OpenFAST conventions that don’t follow the 
  same format: FairTen and AnchTen. FairTen[n] is the same as Line[n]TenB. For example, the 
  fairlead tension of line 1 would be FAIRTEN1 or LINE1TENB.
- The output list is not case sensitive, however all MoorDyn-F outputs will be printed to the output
  file in all caps. When searching OpenFAST output channels, users will need to search for MoorDyn
  channels in all caps. Example: the channel fairten1 would appear in the output file as FAIRTEN1.

1. Line node forces: Line node forces output the net force on the node, which includes the tension
   vectors of the adjacent segments plus the weight, buoyancy, seabed-contact, and hydrodynamic 
   forces on the node.
2. Rod node forces: The rod node forces contain weight, buoyancy (from pressure integration over 
   the surface), and hydrodynamics. No internal structural forces are accounted for in rod force 
   outputs.
3. Line node tension: Node tensions for lines output different values depending on whether it is an 
   end node or an internal node. End nodes output the net force on the end node, i.e. the magnitude 
   of the Fnet vector. Internal nodes output the average tension from the segments on either side 
   of the node. 

Additional MoorDyn Files
------------------------

MoorDyn-F Driver Input File
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. _MDF_driver_in:

The MoorDyn-F driver that compiles with Openfast requires its own input file (located in the same 
with the following format in addition to the MoorDyn input file). The exact whitespace and 
alignment of the lines of the file is not important, so long as values are separated by at least 
one space.

.. code-block:: none

 MoorDyn driver input file 
 another comment line
 ---------------------- ENVIRONMENTAL CONDITIONS ------------------------------- 
 9.80665                 Gravity          - Gravity (m/s^2) 
 1025.0                  rhoW             - Water density (kg/m^3) 
 75.0                    WtrDpth          - Water depth (m) 
 ---------------------- MOORDYN ------------------------------------------------ 
 "<input file path>"     MDInputFile      - Primary MoorDyn input file name (quoted string) 
 "Mooring/F"             OutRootName      - The name which prefixes all MoorDyn generated files (quoted string) 
 10.0                    TMax             - Number of time steps in the simulations (-) 
 0.001                   dtC              - TimeInterval for the simulation (sec) 
 0                       InputsMode       - MoorDyn coupled object inputs (0: all inputs are zero for every timestep, 1: time-series inputs) (switch) 
 "PtfmMotions.dat"       InputsFile       - Filename for the MoorDyn inputs file for when InputsMod = 1 (quoted string) 
 0                       NumTurbines      - Number of wind turbines (-) [>=1 to use FAST.Farm mode. 0 to use OpenFAST mode] 
 ---------------------- Initial Positions -------------------------------------- 
 ref_X    ref_Y    surge_init   sway_init  heave_init  roll_init  pitch_init  yaw_init 
 (m)      (m)        (m)          (m)        (m)       (rad)       (rad)        (rad)
 0         0         0.0          0.0        0.0        0.0         0.0          0.0  
 <followed by MAX(1,NumTurbines) rows of data>  
 END of driver input file

If InputsMode is set to 1, the MoorDyn-F driver will require a platform motions time series dataset of the 
coupled object movements. The time units are seconds, the translational position units are meters, 
and the orentation units are radians. For a single coupled body, the order of the data columns 
would look like the following (lines beginning with # are treated as comments by MoorDyn):

.. code-block:: none

 # Time(s)    PtfmSurge(m)    PtfmSway(m)    PtfmHeave(m)    PtfmRoll(rad)    PtfmPitch(rad)    PtfmYaw(rad)

If there are multiple coupled objects then the general order of columns beyond the time column 
follows the order of the state vector: Body degrees of freedom, rod degrees of freedom, and points 
degrees of freedom. For coupled pinned bodies and rods the full 6DOF need to be provided, however the rotational 
values will be ignored by by the MoorDyn-F driver (they can be set to zero).

When using the MoorDyn driver in OpenFAST mode, the initial positions represents the offsets to the 
global frame. When using OpenFAST mode with the positions set to 0's, then MoorDyn objects will be 
simulated based on the positions defined in the MoorDyn input file. If a non-zero value is provided,
it will be incorporated into the initial positions of coupled objects. For example, if the following 
initial positions are given:

.. code-block:: none
  
 ---------------------- Initial Positions -------------------------------------- 
 ref_X    ref_Y    surge_init   sway_init  heave_init  roll_init  pitch_init  yaw_init 
 (m)      (m)        (m)          (m)        (m)       (rad)       (rad)        (rad)
 0         0         10.0         0.0        0.0        0.0        20.0          0.0  

Then a coupled body with a inital state defined in the input file as [0, 0, 0, 0, 0, 0]
will have an inital state of [10, 0, 0, 20, 0, 0]. It is advised that for using the MoorDyn driver
in OpenFAST mode that the Inital Positions are set to 0 unless the user has a reason to do otherwise.

Seafloor/Bathymetry File 
^^^^^^^^^^^^^^^^^^^^^^^^
.. _seafloor_in:

For bathymetry inputs, MoorDyn-C takes a Seafloor file.
This file allows you to define a square grid of points and define depths at each of these points.

.. code-block:: none

  num_x_points num_y_points
  x_1 x_2 ... x_num_x_points
  y_1 y_2 ... y_num_y_points
  x_pos y_pos depth
  x_pos y_pos depth
  x_pos y_pos depth
  etc, etc

The two values on the first line define the number of points in each axis of the grid.
The second line defines the actual locations along the x axis for the x grid points.
The third line defines the locations along the y axis for the y grid points.
The remaining lines are (x, y, z) coordinates for the seafloor on grid points.
It is important that the x_pos be a value found in line 2 and y_pos be a value found in line 3.

The bathymetry file in MoorDyn-F looks slightly different but functions the same.

.. code-block:: none

 ----- MoorDyn Bathymetry Input File ----
 nGridX 2 
 nGridY 2 
        -1500 900
 -1200   1000 700
  1200   1000 700

In this the -1500, 900 are x location, the -1200, and 1200 are y location
while the 1000 and 700 are the depths at the corresponding x and y.

For both MoorDyn-C and MoorDyn-F what happens if one of these points does not fall on the grid is 
not defined and may overwrite other depth values.

If some part of the simulation falls outside of the defined grid area, it will use the depth of the 
nearest grid edge.

The V2 snapshot file
^^^^^^^^^^^^^^^^^^^^

In MoorDyn-C v2 two new functions have been added:

.. doxygenfunction:: MoorDyn_Save
.. doxygenfunction:: MoorDyn_Load

With the former a snapshot of the simulation can be saved, so that it can
be resumed in a different session using the latter function.
It is required to create the system using the same input file in both
sessions.
However, the initial equilibrium condition computation can be skipped in the second session by 
calling

.. doxygenfunction:: MoorDyn_Init_NoIC

Wave Kinematics file (MoorDyn-C)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the WaveKin option is nonzero then wave kinematics inputs need to be provided by a file with the 
formats described in the :ref:`water kinematics section <waterkinematics>`.

Water Kinematics file (MoorDyn-F)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. _MDF_wtrkin:

This file is used if simulating water kinematics in MoorDyn-F with a user defined grid (Old Method and Hybrid Method).
More details on the different MoorDyn-F water kinematics methods can be found in the :ref:`MoorDyn-F water kinematics section <waterkinematics-F>`.

The file provided to MoorDyn-F for water kinematics should have the following format, which 
specifies the inputted waves and current. MoorDyn-F can handle a maximum of 100 lines of current
data.

.. code-block:: none

 MoorDyn Waves and Currents input file
 ...any notes here...
  --------------------------- WAVES -------------------------------------
  2                    WaveKinMod  - type of wave input {0 no waves; 1 use the old method; 2 use the hybrid method}
  ""                   WaveKinFile - file containing wave elevation time series at 0,0,0 # Ignored if WaveKinMod = 2
  0                    dtWave      - time step to use in setting up wave kinematics grid (s) # Ignored if WaveKinMod = 2
  0                    WaveDir     - wave heading (deg) # Ignored if WaveKinMod = 2
  2                                - X wave input type (0: not used; 1: list values in ascending order; 2: uniform specified by -xlim, xlim, num) 
  -800, 10, 101                    - X wave grid point data
  2                                - Y wave input type (0: not used; 1: list values in ascending order; 2: uniform specified by -xlim, xlim, num)
  -5, 5, 3                         - Y wave grid point data
  2                                - Z wave input type (0: not used; 1: list values in ascending order; 2: uniform specified by -xlim, xlim, num)
  -600, 0, 61                      - Z wave grid point data
  --------------------------- CURRENT -------------------------------------
  2                    CurrentMod  - type of current input {0 no current; 1 steady current profile described below; 2 hybrid method}
  2                                - Z wave input type (0: not used; 1: list values in ascending order; 2: uniform specified by -xlim, xlim, num) # Ignored if CurrentMod = 1
  -600, 0, 50                      - Z wave grid point data # Ignored if CurrentMod = 1
 z-depth     x-current      y-current # Table ignored if CurrentMod = 2
 (m)           (m/s)         (m/s)
 0.0             0.9          0.0
 150             0.5          0.0
 1000            0.25         0.0
 1500            0.2          0.0
 5000            0.15         0.0
 --------------------- need this line ------------------

MoorDyn-F with FAST.Farm - Inputs
-------------------------------

MoorDyn is available at an array level in FAST.Farm using the MoorDyn-F v2 input file format.
A nice description of FAST.Farm is found on the OpenFAST repository README: 

"FAST.Farm extends the capabilities of OpenFAST to provide physics-based engineering simulation 
of multi-turbine land-based, fixed-bottom offshore, and floating offshore wind farms. With 
FAST.Farm, you can simulate each wind turbine in the farm with an OpenFAST model and capture 
the relevant physics for prediction of wind farm power performance and structural loads, including 
wind farm-wide ambient wind, super controller, and wake advection, meandering, and merging. 
FAST.Farm maintains computational efficiency through parallelization to enable loads analysis for 
predicting the ultimate and fatigue loads of each wind turbine in the farm."

FAST.Farm can be obtained from the `OpenFAST repository <https://github.com/OpenFAST/openfast/tree/main>`_.

General Organization
^^^^^^^^^^^^^^^^^^^^

The regular ability for each OpenFAST instance to have its own MoorDyn simulation is unchanged in 
FAST.Farm. This ability can be used for any non-shared mooring lines in all cases. To enable 
simulation of shared mooring lines, which are coupled with multiple turbines, an additional 
farm-level MoorDyn instance has been added. This MoorDyn instance is not associated with any 
turbine but instead is called at a higher level by FAST.Farm. Attachments to different turbines 
within this farm-level MoorDyn instance are handled by specifying "TurbineN" as the attachment for any 
points/bodies/rods that are attached to a turbine, where "N" is the specific turbine number as listed in the 
FAST.Farm input file. Pinned bodies and rods are not yet supported in FAST.Farm. 

MoorDyn Input File
^^^^^^^^^^^^^^^^^^

The following input file excerpt shows how points can be specified as attached to specific turbines 
(turbines 3 and 4 in this example). When a point has "TurbineN" as its attachment, it acts similarly to a 
"BodyN" attachment, where the X/Y/Z inputs specify the relative location of the fairlead on the platform. 
ex. For a turbine located at (200, 0, 0), a vertical line attached to it's center would have
a fixed point at end A at (200, 0, 0) and a turbineN point at (0, 0, 0). In the farm-level MoorDyn input 
file, "Coupled" point types cannot be used because it is ambiguous which turbine they attach to.

.. code-block:: none
 :emphasize-lines: 5,6,11
 
 ----------------------- POINTS ----------------------------------------------
 Node    Attachment    X       Y         Z        M        V       CdA   CA
 (-)       (-)        (m)     (m)       (m)      (kg)     (m^3)   (m^2)  (-)
 1         Turbine3   10.0     0      -10.00      0        0        0     0
 3         Turbine4  -10.0     0      -10.00      0        0        0     0
 2         Fixed     267.0    80      -70.00      0        0        0     0
 -------------------------- LINE PROPERTIES ----------------------------------
 ID    LineType      AttachA  AttachB  UnstrLen  NumSegs  LineOutputs
 (-)     (-)         (-)        (-)       (m)       (-)      (-)
 1     sharedchain    1         2        300.0      20        p
 2     anchorchain    1         3        300.0      20        p
 
In this example, Line 1 is a shared mooring line and Line 2 is an anchored mooring line that has a 
fairlead point in common with the shared line. Individual mooring systems can be modeled in the 
farm-level MoorDyn instance as well.

The same approach is used for bodies and rods, where the attachment is defined as "TurbineN".
The body and rod positions and rotations are defined relative to the turbines position and rotation. 
The following code snippet shows rods using the turbine convention.

.. code-block:: none
  :emphasize-lines: 12,13,17

  ---------------------- LINE TYPES --------------------------------------------------
  TypeName    Diam     Mass/m     EA      BA/-zeta     EI        Cd      Ca      CdAx    CaAx
  (name)      (m)      (kg/m)     (N)     (N-s/-)    (N-m^2)     (-)     (-)     (-)     (-)
  0           0.1410    35.78  2.030e+08 -1.000e+00  8.410e+03   1.200   1.000   0.20    0.00 
  --------------------- ROD TYPES -----------------------------------------------------
  TypeName      Diam     Mass/m    Cd     Ca      CdEnd    CaEnd
  (name)        (m)      (kg/m)    (-)    (-)     (-)      (-)
  connector    0.2000    0.00      0.000  0.000   0.000    0.000  
  ---------------------- RODS ---------------------------------------------------------
  ID   RodType    Attachment   Xa      Ya    Za      Xb      Yb    Zb      NumSegs  RodOutputs
  (#)  (name)     (#/key)      (m)     (m)   (m)     (m)     (m)   (m)     (-)       (-)
  1    connector  Turbine1     -0.62   0.00  -13.22  0.62    0.00  -14.78   0         - 
  2    connector  Free         -947.81 0.00  -150.60 -945.82 0.00  -150.75  0         -
  ---------------------- LINES --------------------------------------------------------
  ID    LineType      AttachA  AttachB  UnstrLen  NumSegs  LineOutputs
  (#)    (name)        (#)      (#)       (m)       (-)     (-)
  1        0           R1B      R2A     299.429     10      pt

In this example 0-length rods are used as bend-stiffeners for a suspended cable attached to 
Turbine1.

FAST.Farm Input File
^^^^^^^^^^^^^^^^^^^^

Several additional lines have been added to the FAST.Farm primary input file. These are highlighted 
in the example input file excerpt below:

.. code-block:: none
 :emphasize-lines: 9,10,13,14,15
 
 FAST.Farm v1.10.* INPUT FILE
 Sample FAST.Farm input file
 --- SIMULATION CONTROL ---
 False              Echo               Echo input data to <RootName>.ech? (flag)
 FATAL              AbortLevel         Error level when simulation should abort (string) {"WARNING", "SEVERE", "FATAL"}
 2000.0             TMax               Total run time (s) [>=0.0]
 False              UseSC              Use a super controller? (flag)
 1                  Mod_AmbWind        Ambient wind model (-) (switch) {1: high-fidelity precursor in VTK format, 2: one InflowWind module, 3: multiple instances of InflowWind module}
 2                  Mod_WaveField      Wave field handling (-) (switch) {1: use individual HydroDyn inputs without adjustment, 2: adjust wave phases based on turbine offsets from farm origin}
 3                  Mod_SharedMooring  Shared mooring system model (-) (switch) {0: None, 3: MoorDyn}
 --- SUPER CONTROLLER --- [used only for UseSC=True]
 "SC_DLL.dll"       SC_FileName        Name/location of the dynamic library {.dll [Windows] or .so [Linux]} containing the Super Controller algorithms (quoated string)
 --- SHARED MOORING SYSTEM --- [used only for Mod_SharedMooring > 0]
 "FarmMoorDyn.dat"  FarmMoorDyn-File    Name of file containing shared mooring system input parameters (quoted string) [used only when Mod_SharedMooring > 0]
 0.01               DT_Mooring         Time step for farm-level mooring coupling with each turbine (s) [used only when Mod_SharedMooring > 0]
 --- AMBIENT WIND: PRECURSOR IN VTK FORMAT --- [used only for Mod_AmbWind=1]
 2.0                DT_Low-VTK         Time step for low -resolution wind data input files  ; will be used as the global FAST.Farm time step (s) [>0.0]
 0.3333333          DT_High-VTK        Time step for high-resolution wind data input files   (s) [>0.0]
 "Y:\Wind\Public\Projects\Projects F\FAST.Farm\AmbWind\steady"          WindFilePath       Path name to VTK wind data files from precursor (string)
 False              ChkWndFiles        Check all the ambient wind files for data consistency? (flag)
 --- AMBIENT WIND: INFLOWWIND MODULE --- [used only for Mod_AmbWind=2 or 3]
 2.0                DT_Low             Time step for low -resolution wind data interpolation; will be used as the global FAST.Farm time step (s) [>0.0]
