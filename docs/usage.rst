MoorDyn Usage
=====================================================

..
  customize code highlight color through "hll" span css

.. raw:: html

    <style> .highlight .hll {color:#000080; background-color: #eeeeff} </style>
    <style> .fast {color:#000080; background-color: #eeeeff} </style>
    <style> .stnd {color:#008000; background-color: #eeffee} </style>

.. role:: fast
.. role:: stnd


The v1 Input File
-----------------

MoorDyn v1 uses a plain-text input file for its description of the mooring system as well as simulation settings.
This file is divided into sections identified by header lines. 

Most of the sections are set up to contain a table of input information. The FORTRAN/OpenFAST form of MoorDyn v1
requires these section to start with a declaration of how many data rows will be in the table (e.g. "3  nLines").
The C/standalone form of MoorDyn skips this. Lines that should only be the in the Fortran/OpenFAST for are 
:fast:`shown in blue`.
Next, the table of each section begins with two preset lines that contain the column names and the corresponding units. 
These lines are followed by any number of lines containing the entries in that section's
table of inputs.


Front matter
^^^^^^^^^^^^

The first 1-n lines of the input file are reserved for free-form user input, for labeling the input file, 
writing notes, etc.

.. code-block:: none
 :emphasize-lines: 2

 MoorDyn input file...
 MoorDyn-F v2 sample input file
 True          Echo          echo the input file data (flag)

Line Types
^^^^^^^^^^

The Line Types section of the file contains one or more definitions of physical line properties and 
four hydrodynamic coefficients. 

.. code-block:: none
 :emphasize-lines: 2

 ------------------------- LINE TYPES --------------------------------------------------
 1  NTypes - the number of line types
 LineType  Diam    MassDen    EA       BA/-zeta   Can   Cat   Cdn   Cdt 
 (-)       (m)     (kg/m)     (N)      (Pa-s/-)   (-)   (-)   (-)   (-)  
 nylon    0.124    13.76    2515288.0    -0.8     1.0   0.0   1.6   0.05

The columns in order are as follows:

 - Name – an identifier word for the line type
 - Diam –  the volume-equivalent diameter of the line – the diameter of a cylinder having the same displacement per unit length (m)
 - MassDen –  the mass per unit length of the line (kg/m)
 - EA – the line stiffness, product of elasticity modulus and cross-sectional area (N)
 - BA/-zeta –  the line internal damping (measured in N-s) or, if a negative value is entered, the desired damping ratio (in fraction of critical) for the line type (and MoorDyn will set the BA of each line accordingly – see Section 4.1 for more information)
 - Can –  transverse added mass coefficient (with respect to line displacement)
 - Cat – tangential added mass coefficient (with respect to line displacement)
 - Cdn –  transverse drag coefficient (with respect to frontal area, d*l)
 - Cdt –  tangential drag coefficient (with respect to surface area, π*d*l) 


Connection Properties
^^^^^^^^^^^^^^^^^^^^^

The Connection Properties section defines the connection node points which mooring lines can be connected to.

.. code-block:: none
 :emphasize-lines: 2
 
 ----------------------- CONNECTION PROPERTIES ----------------------------------------------
 3     NConnections - the number of connections
 Node      Type      X        Y         Z        M        V        FX       FY      FZ     CdA   CA
 (-)       (-)      (m)      (m)       (m)      (kg)     (m^3)    (kN)     (kN)    (kN)   (m^2)  (-)
 1         Vessel     0.0     0      -10.00       0        0        0        0       0       0     0
 2         Fixed    267.0     0      -70.00       0        0        0        0       0       0     0
 3         Connect    0.0     0      -10.00       0        0        0        0       0       0     0

The columns are as follows:

 - Node –  the ID number of the connection (must be sequential starting with 1)
 - Type –  one of “Fixed”, “Vessel”, or “Connect”, as described :ref:`here <points>`
 - X, Y, Z –  Coordinates of the connection (relative to inertial reference frame if “fixed” or “connect”, 
   relative to platform reference frame if “vessel”).  In the case of “connect” nodes, it is simply an 
   initial guess for position before MoorDyn calculates the equilibrium initial position.  (m)
 - M – node mass in the case of clump weights (kg)
 - V –  node displacement in the case of floats (m^3)
 - FX, FY, FZ –  any steady external forces applied to the node (N)
 - CdA –  product of drag coefficient and projected area (assumed constant in all directions) to calculate a drag force for the node (m^2)
 - Ca –  added mass coefficient used along with V to calculate added mass on node


Lines list
^^^^^^^^^^

The Line Properties section defines each uniform-property section of mooring line to be simulated.

.. code-block:: none
 :emphasize-lines: 2
 
 -------------------------- LINE PROPERTIES -------------------------------------------------
 3     NLines - the number of lines
 Line     LineType  UnstrLen  NumSegs    NodeA     NodeB  Flags/Outputs
 (-)      (-)       (m)         (-)       (-)       (-)      (-)
 1        nylon     300.0        50        2         1        p
 2        nylon     300.0        50        4         3        p
 3        nylon     300.0        50        6         5        p

The columns are as follows:

 - Line - the ID number of the line (must be sequential starting with 1)
 - LineType - a string matching a Line Dictionary entry, specifying which physical properties it uses
 - UnstrLen - the unstretched length of the line
 - NumSegs - how many segments the line is discretized into (it will have NumSegs+1 nodes total, including its two end nodes)
 - NodeA - the ID number of the connection that the first (anchor) end of the line is attached to
 - NodeB - the ID number of the connection that the final (fairlead) end of the line is attached to
 - flags/outputs - any data to be output in a dedicated output file for that line. 
   
This last entry expects a string of one or more characters without spaces, each character activating a given output property.  
A placeholder character such as “-” should be used if no outputs are wanted.  Eight output properties are currently possible:

 - p – node positions
 - v – node velocities
 - U – wave velocities at each node
 - D – hydrodynamic drag force at each node
 - t – tension force at each segment 
 - c – internal damping force at each segment
 - s – strain of each segment
 - d – rate of strain of each segment

For example, outputting node positions and segment tensions could be achieved by writing “pt” for this last column.  These outputs will go to a dedicated output file for each line only.  For sending values to the global output file, use the Outputs section instead.



Options
^^^^^^^

The Solver Options section can contain any number of optional settings for the overall model, including seabed properties, 
initial condition (IC) generation settings, and the time step size. 

.. code-block:: none

 -------------------------- SOLVER OPTIONS---------------------------------------------------
 0.001    dtM           - time step to use in mooring integration
 3.0e6    kb           - bottom stiffness
 3.0e5    cb           - bottom damping
 70       WtrDpth      - water depth
 5.0      ICDfac       - factor by which to scale drag coefficients during dynamic relaxation IC gen
 0.001    ICthresh     - threshold for IC convergence
 0        ICTmax       - threshold for IC convergence (set to zero for debugging)

Any of these lines can be omitted, in which case default values will be used.   As such, they are all optional settings, 
although some of them (such as time step size) often need to be set by the user for proper operation. 
The list of possible options is:

 - dtM – desired mooring model time step (s)
 - g – gravitational constant (m/s^2)*
 - rhoW – water density (kg/m^3)*
 - WtrDpth – water depth (m)*
 - kBot – bottom stiffness constant (Pa/m) 
 - cBot – bottom damping constant (Pa-s/m)
 - dtIC – period for analyzing convergence of dynamic relaxation IC generation (s)
 - TmaxIC – maximum simulation time to allow for IC generation without convergence (s)
 - CdScaleIC – factor by which to scale drag coefficients to accelerate convergence of IC generation (-)
 - ThreshIC – convergence threshold for IC generation, acceptable relative difference between three successive fairlead tension measurements (-)

:fast:`In MoorDyn-F, the default values for g, rhoW, and WtrDpth are the values provided by FAST, so it is recommended to not use 
custom values for the sake of consistency.`

The bottom contact parameters, kBot and cBot, result in a pressure which is then applied to the cross-sectional area (d*l) 
of each contacting line segment to give a resulting vertical contact force for each segment.

 
Outputs
^^^^^^^

The Outputs section is used to specify general outputs, which are written to the main MoorDyn output file 
:fast:`and also sent to the driver program for inclusion in the global output file.`  

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


Each output channel name should have its own line.  There are intuitive keywords for fairlead and anchor tensions 
of a given line: fairten# and anchten#, where # is the line number.  
There is also a flexible naming system for outputting other quantities.
There are currently five supported types of output quantities:

 - pX, pY , pZ  – x/y/z coordinate (m)
 - vX, vY, vZ – velocity (m/s)
 - aX, aY, aZ – acceleration (m/s^2)
 - T or Ten – tension (N)
 - fX, fY, fZ – net force in x/y/z direction (N)

These can be produced at a connection object, denoted by the prefix Con#, where # is the connect number.  
Or, they can be produced at a node along a line, denoted by the prefix L#N@, where # is the line number 
and @ is the number of the node along that line.  For example,

 - Con3vY outputs the connection 3 y velocity,
 - L2N4pX outputs the line 2, node 4 x position.




The v2 Input File (in progress)
-------------------------------

MoorDyn v2 uses a plain-text input file for its description of the mooring system and simulation settings that is 
expanded from that of v1.
This file is divided into sections, some of which are optional. Each section is identified (and detected) by
a header line consisting of a key phrase (e.g. Line Types) surrounded by dashes. While a couple sections are optional,
the order of the sections should never be changed.

Most of the sections are set 
up to contain a table of input information. These tables begin with two preset lines that contain the column names
and the corresponding units. These lines are followed by any number of lines containing the entries in that section's
table of inputs.


Front matter
^^^^^^^^^^^^

The first 1-n lines of the input file are reserved for free-form user input, for labeling the input file, 
writing notes, etc. ::

 MoorDyn-F v2 sample input file
 True          Echo          echo the input file data (flag)

Line Types
^^^^^^^^^^

This section (required if there are any mooring lines) describes the list of mooring line property sets
that will be used in the simulation ::

 ---------------------- LINE TYPES -----------------------------------
 TypeName      Diam          Mass/m        EA            BA/-zeta      EI            Cd            Ca            CdAx          CaAx          
 (-)           (m)           (kg/m)        (N)           (N-s/-)       (N-m^2)       (-)           (-)           (-)           (-)           
 Chain         0.1            150.0        1e8           -1            0             2.3            1            1.0           0.5           


Rod Types
^^^^^^^^^

This section (required if there are any rod objects) describes the list of rod property sets
that will be used in the simulation ::

 ---------------------- ROD TYPES ------------------------------------
 TypeName      Diam          Mass/m        Cd            Ca            CdEnd         CaEnd       
 (-)           (m)           (kg/m)        (-)           (-)           (-)           (-)         
 Buoy          10            1.0e3         0.6           1.0           1.2           1.0        


Bodies list
^^^^^^^^^^^

This section (optional) describes the 6DOF body objects to be simulated. ::

 ---------------------- BODIES ---------------------------------------
 BodyID        X0            Y0            Z0            r0            p0            y0            Xcg           Ycg           Zcg           Mass          Volume       Ix,y,z        CdA-x,y,z     Ca-x,y,z
 (-)           (m)           (m)           (m)           (deg)         (deg)         (deg)         (m)           (m)           (m)           (kg)          (m^3)        (kg-m^2)      (m^2)         (-)
 1coupled       0            0              0            0             0             0             0             0             0             0             0            0             0             0
 

Rods list
^^^^^^^^^

This section (optional) describes the rigid Rod objects ::

 ---------------------- RODS ----------------------------------------
 RodID         Type/BodyID   RodType       Xa            Ya            Za            Xb            Yb            Zb            NumSegs       RodOutputs
 (-)           (-)           (-)           (m)           (m)           (m)           (m)           (m)           (m)           (-)           (-)
 1             Body1fixed    Can           0             0             2             0             0             15            8             p
 2             Body1fixed    Can           2             0             2             5             0             15            8             p
 
 
Points list
^^^^^^^^^^^

This section (optional) describes the Point objects ::

 
 ---------------------- POINTS ---------------------------------------
 PointID       Type          X             Y             Z             Mass          Volume        CdA           Ca
 (-)           (-)           (m)           (m)           (m)           (kg)          (mˆ3)         (m^2)         (-)
 1             Fixed         -500          0             -150          0             0             0             0
 4             Coupled       0             0             -9            0             0             0             0
 11            Body2         0             0             1.0           0             0             0             0
 
 
Lines list
^^^^^^^^^^

This section (optional) describes the Line objects, typically used for mooring lines or dynamic power cables ::

 ---------------------- LINES ----------------------------------------
 LineID        LineType      UnstrLen      NumSegs       AttachA       AttachB       LineOutputs
 (-)           (-)           (m)           (-)           (point#)      (point#)      (-)
 1             Chain         300           20            1             2             p


Options
^^^^^^^

This section (required) describes the simulation options. ::

 ---------------------- OPTIONS -----------------------------------------
 0.002         dtM           time step to use in mooring integration (s)
 3000000       kbot          bottom stiffness (Pa/m)
 300000        cbot          bottom damping (Pa-s/m)
 0.5           dtIC          time interval for analyzing convergence during IC gen (s)
 10            TmaxIC        max time for ic gen (s)
 0.001         threshIC      threshold for IC convergence (-)
 
 
Outputs
^^^^^^^

This section (optional) lists any specific output channels to be written in the main output file. The section
needs to end with another header-style line (as shown below) for the program to know when to end. ::

 ---------------------- OUTPUTS -----------------------------------------
 Body1Px
 Body1Py
 Body1Pz
 Body1Roll
 Body1Pitch
 FairTen1
 FairTen2
 FairTen3
 AnchTen1
 AnchTen2
 AnchTen3
 END
 ------------------------- need this line -------------------------------------


Advice and Frequent Problems
----------------------------


Trying math

.. math::

   (a + b)^2 = a^2 + 2ab + b^2

   (a - b)^2 = a^2 - 2ab + b^2
   
   
   
Model Stability and Segment Damping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Two of the trickier input parameters are the internal damping (BA) for each line type, and the mooring simulation time step (dtM).  
Both relate to the discretization of the lines.
The highest axial vibration mode of the lumped-mass cable representation would be when adjacent nodes 
oscillate out of phase with each other, as depicted below.
 
In this mode, the midpoint of each segment would not move.  The motion of each node can then be 
characterized by mass-spring-damper values of

.. math::

  m=w L/N \; c=4NBA/L \; k=4NEA/L.

The natural frequency of this mode is then

.. math::

  \omega_n=\sqrt{k/m}=2/l \sqrt{EA/w}=2N/L \sqrt{EA/w}

and the damping ratio, ζ, is related to the internal damping coefficient, BA, by

.. math::

  \zeta =c/c_{crit} = B/l \sqrt{A/Ew} = NBA/L \sqrt{(1/EAw}  \;\;  BA=\zeta \frac{L}{N}\sqrt{EAw}.

The line dynamics frequencies of interest should be lower than ω_n in order to be resolved by the model.  
Accordingly, line dynamics at ω_n, which are likely to be dominated by the artificial resonance created 
by the discretization, can be damped out without necessarily impacting the line dynamics of interest.  
This is advisable because the resonances at ω_n can have a large impact on the results.  
To damp out the segment vibrations, a damping ratio approaching the critical value (ζ=1) is recommended.  
Care should be taken to ensure that the line dynamics of interest are not affected.

To simplify things, a desired line segment damping ratio can be specified in the input file.  This is done 
by entering the negative of the desired damping ratio in the BA/-zeta field of the Line Types section.  
A negative value here signals MoorDyn to interpret it as a desired damping ratio and then calculate the 
damping coefficient (BA) for each mooring line that will give every line segment that damping ratio 
(accounting for possible differences in segment length between lines).  

Note that the damping ratio is with respect to the critical damping of each segment along a mooring 
line, not with respect to the line as a whole or the floating platform as a whole.  It is just a way
of letting MoorDyn calculate the damping coefficient automatically from the perspective of damping 
non-physical segment resonances.    If the model is set up right, this damping can have a negligible 
contribution to the overall damping provided by the moorings on the floating platform.  However, if 
the damping contribution of the mooring lines on the floating platform is supposed to be significant, 
it is best to (1) set the BA value directly to ensure that the expected damping is provided and then 
(2) adjust the number of segments per line to whatever provides adequate numerical stability.



(THIS PAGE IN PROGRESS)