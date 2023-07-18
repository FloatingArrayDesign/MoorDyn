.. _usage:

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


Point Properties
^^^^^^^^^^^^^^^^^^^^^

The Point Properties section defines the point node points which mooring lines can be connected to.

.. code-block:: none
 :emphasize-lines: 2
 
 ----------------------- POINT PROPERTIES ----------------------------------------------
 3     NPoints - the number of points
 Node      Type      X        Y         Z        M        V        FX       FY      FZ     CdA   CA
 (-)       (-)      (m)      (m)       (m)      (kg)     (m^3)    (kN)     (kN)    (kN)   (m^2)  (-)
 1         Vessel     0.0     0      -10.00       0        0        0        0       0       0     0
 2         Fixed    267.0     0      -70.00       0        0        0        0       0       0     0
 3         Connect    0.0     0      -10.00       0        0        0        0       0       0     0

The columns are as follows:

 - Node –  the ID number of the point (must be sequential starting with 1)
 - Type –  one of “Fixed”, “Vessel”, or “Connect”, as described :ref:`here <points>`
 - X, Y, Z –  Coordinates of the point (relative to inertial reference frame if “fixed” or “point”, 
   relative to platform reference frame if “vessel”).  In the case of “point” nodes, it is simply an 
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
 - NodeA - the ID number of the point that the first (anchor) end of the line is attached to
 - NodeB - the ID number of the point that the final (fairlead) end of the line is attached to
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
 - SeafloorFile – Relative path of a 3D Seafloor file
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
 
Seafloor File
^^^^^^^^^^^^^

If you need the seafloor have different depths at different locations, it is possible to create and use a 3D Seafloor file.
This file allows you to define a square grid of points and define depths at each of these points.

code-block:: none
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
The order of these points is not important.
It is also important that the x_pos be a value found in line 2 and y_pos be a value found in line 3.
What happens if one of these points does not fall on the grid is not defined and may overwrite other depth values.

If some part of the simulation fall outside of the defined grid area, it will use the depth of the nearest grid edge.


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

These can be produced at a point object, denoted by the prefix Con#, where # is the point number.  
Or, they can be produced at a node along a line, denoted by the prefix L#N@, where # is the line number 
and @ is the number of the node along that line.  For example,

 - Con3vY outputs the point 3 y velocity,
 - L2N4pX outputs the line 2, node 4 x position.




The v2 Input File
-----------------

MoorDyn v2 uses a standardized plain-text input file for its description of the
mooring system and simulation settings  that has some important additions and
changes from v1.

Most helpfully, this new format is identical between C++ and FORTRAN versions of
MoorDyn, and it is designed to be support future capability enhancements without
requiring changes.

This file is divided into sections, some of which are optional. Each section is
identified (and detected) by a header line consisting of a key phrase (e.g. Line
Types) surrounded by dashes. While a couple sections are optional, the order of
the sections should never be changed.

Most of the sections are set up to contain a table of input information. These
tables begin with two preset lines that contain the column names and the
corresponding units. These lines are followed by any number of lines containing
the entries in that section's table of inputs.

Front matter
^^^^^^^^^^^^

The first lines of the input file are reserved for free-form user input, for
labeling the input file,  writing notes, etc.
There is not a limit on the number of lines you can write here.

.. code-block:: none

 --------------------- MoorDyn Input File ------------------------------------
 MoorDyn-F v2 sample input file


Line Types
^^^^^^^^^^

This section (required if there are any mooring lines) describes the list of mooring line property sets
that will be used in the simulation 

.. code-block:: none

 ---------------------- LINE TYPES -----------------------------------
 TypeName   Diam    Mass/m     EA     BA/-zeta    EI         Cd     Ca     CdAx    CaAx          
 (name)     (m)     (kg/m)     (N)    (N-s/-)     (N-m^2)    (-)    (-)    (-)     (-)           
 Chain      0.1      150.0     1e8    -1          0          2.3     1     1.0     0.5           

The columns in order are as follows:

 - Name – an identifier word for the line type
 - Diam –  the volume-equivalent diameter of the line – the diameter of a cylinder having the same displacement per unit length (m)
 - MassDen –  the mass per unit length of the line (kg/m)
 - EA – the line stiffness, product of elasticity modulus and cross-sectional area (N)
 - BA/-zeta –  the line internal damping (measured in N-s) or, if a negative value is entered, the desired damping ratio (in fraction of critical) for the line type (and MoorDyn will set the BA of each line accordingly – see Section 4.1 for more information)
 - EI – the line bent stiffness, product of elasticity modulus and inertia of the cross-sectional area (N)
 - Cd –  transverse drag coefficient (with respect to frontal area, d*l)
 - Ca –  transverse added mass coefficient (with respect to line displacement)
 - CdAx –  tangential drag coefficient (with respect to surface area, π*d*l)
 - CaAx – tangential added mass coefficient (with respect to line displacement)

Non-linear values for the stiffness (EA), internal damping (BA/-zeta) and bent
stiffness (EI) are accepted.
To this end, a file can be provided (to be located in the same folder than the
main MoorDyn input file) instead of a number.
Such file is simply a tabulated file with 2 columns, separated by a blank space.
The columns to be provided for each non-linear magnitude are the followings:

 - Stiffness: Strain rate - EA/Stretching rate (N)
 - Internal damping: Curvature - EI/Curvature (N-m^2)
 - Bent stiffness: Stretching rate - Damping coefficient/Stretching rate (N-s^2/s)


Rod Types
^^^^^^^^^

This section (required if there are any rod objects) describes the list of rod property sets
that will be used in the simulation 

.. code-block:: none

 ---------------------- ROD TYPES ------------------------------------
 TypeName      Diam     Mass/m    Cd     Ca      CdEnd    CaEnd       
 (name)        (m)      (kg/m)    (-)    (-)     (-)      (-)         
 Buoy          10       1.0e3     0.6    1.0     1.2      1.0        


Bodies list
^^^^^^^^^^^

This section (optional) describes the 6DOF body objects to be simulated. 

.. code-block:: none

 ---------------------- BODIES ---------------------------------------
 ID   Attachment  X0     Y0    Z0     r0      p0     y0     Mass  CG*   I*      Volume   CdA*   Ca*
 (#)   (word)     (m)    (m)   (m)   (deg)   (deg)  (deg)   (kg)  (m)  (kg-m^2)  (m^3)   (m^2)  (-)
 1     coupled     0     0      0     0       0      0       0     0     0        0       0      0
 

Rods list
^^^^^^^^^

This section (optional) describes the rigid Rod objects 

.. code-block:: none

 ---------------------- RODS ----------------------------------------
 ID   RodType  Attachment  Xa    Ya    Za    Xb    Yb    Zb   NumSegs  RodOutputs
 (#)  (name)   (word/ID)   (m)   (m)   (m)   (m)   (m)   (m)  (-)       (-)
 1      Can      Body1      0     0     2     0     0     15   8         p
 2      Can   Body1Pinned   2     0     2     5     0     15   8         p
 
 
Points list
^^^^^^^^^^^

This section (optional) describes the Point objects 
 
.. code-block:: none

 ---------------------- POINTS ---------------------------------------
 ID   Attachment  X       Y     Z      Mass   Volume  CdA    Ca
 (#)   (word/ID) (m)     (m)   (m)    (kg)   (mˆ3)   (m^2)  (-)
 1     Fixed      -500    0     -150    0      0       0      0
 4     Coupled    0       0     -9      0      0       0      0
 11    Body2      0       0     1.0     0      0       0      0
 
 
Lines list
^^^^^^^^^^

This section (optional) describes the Line objects, typically used for mooring lines or dynamic power cables

.. code-block:: none

 ---------------------- LINES ----------------------------------------
 ID   LineType   AttachA  AttachB  UnstrLen  NumSegs  LineOutputs
 (#)   (name)     (ID)     (ID)      (m)       (-)      (-)
 1     Chain       1        2        300        20       p
									  

Options
^^^^^^^

This section (required) describes the simulation options

.. code-block:: none

 ---------------------- OPTIONS -----------------------------------------
 0.002         dtM           time step to use in mooring integration (s)
 3000000       kbot          bottom stiffness (Pa/m)
 300000        cbot          bottom damping (Pa-s/m)
 0.5           dtIC          time interval for analyzing convergence during IC gen (s)
 10            TmaxIC        max time for ic gen (s)
 0.001         threshIC      threshold for IC convergence (-)
 
Any of these lines can be omitted, in which case default values will be used.
As such, they are all optional settings, although some of them (such as time
step size) often need to be set by the user for proper operation.
The list of possible options is:

 - writeLog (0): If >0 a log file is written recording information. The bigger the number the more verbose. Please, be mindful that big values would critically reduce the performance!
 - DT (0.001): The time step (s)
 - tScheme (RK2): The time integrator. It should be one of Euler, Heun, RK2, RK4, AB2, AB3, AB4, BEuler2, BEuler3, BEuler4, BEuler5, Midpoint2, Midpoint3, Midpoint4, Midpoint5. RK stands for Runge-Kutta while AB stands for Adams-Bashforth
 - g (9.81): The gravity acceleration (m/s^2)
 - rho (1025): The water density (kg/m^3)
 - WtrDpth (0.0): The water depth (m)
 - kBot (3.0e6): The bottom stiffness (Pa/m)
 - cBot (3.0e5): The bottom damping (Pa-s/m)
 - dtIC (1.0): The time lapse between convergency checks during the initial condition computation (s)
 - TmaxIC (120.0): The maximum simulation time to run in order to find a stationary initial condition (s)
 - CdScaleIC (5.0): The damping scale factor during the initial condition computation
 - threshIC (0.001): The lines tension maximum relative error to consider that the initial condition have converged
 - WaveKin (0): The waves model to use. 0 = none, 1 = waves externally driven, 2 = FFT in a regular grid, 3 = kinematics in a regular grid, 4 = WIP, 5 = WIP, 7 = Wave Component Summing
 - dtWave (0.25): The time step to evaluate the waves, only for wave grid (WaveKin = 3) (s)
 - Currents (0): The currents model to use. 0 = none, 1 = steady in a regular grid, 2 = dynamic in a regular grid, 3 = WIP, 4 = WIP, 5 = 4D Current Grid
 - UnifyCurrentGrid (1): When both waves and currents are defined using a grid method, you may want to pre-combine those grids into a single grid that stores the summed wave and current kinematics. When this option is 1 the wave grid points get the interpolated current grid values added to them. When this option is 0 the wave grid and current grid are kept separate.
 - WriteUnits (1): 0 to do not write the units header on the output files, 1 otherwise
 - FrictionCoefficient (0.0): The seabed friction coefficient
 - FricDamp (200.0): The seabed friction damping, to scale from no friction at null velocity to full friction when the velocity is large
 - StatDynFricScale (1.0): Rate between Static and Dynamic friction coefficients
 - dtOut (0.0): Time lapse between the ouput files printing (s)

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

General output suffixes

============== ======================== =======  =====  =====  =====  =====
Suffix         Description              Units    Node   Point  Rod    Body
============== ======================== =======  =====  =====  =====  =====
PX/PY/PZ       Position coordinates     [m]      X      X      X      X
VX/VY/VZ       Velocity components      [m/s]    X      X      X      X
Ax/Ay/AZ       Acceleration components  [m/s^2]  X      X      X      X
T              Tension or net force     [N]      X      X      X      X
Fx/Fy/Fz       Force components         [N]      X      X      X      X
Roll/Pitch/Yaw Orientation angles       [deg]                  X      X
Sub            Fraction of submergence  [0-1]                  X      
============== ======================== =======  =====  =====  =====  =====

The v2 snapshot file
--------------------

In MoorDyn v2 two new functions have been added:

.. doxygenfunction:: MoorDyn_Save
.. doxygenfunction:: MoorDyn_Load

With the former a snapshot of the simulation can be saved, in such a way it can
be resumed in a different session using the latter function.
It is anyway required to create the system using the same input file in both
sessions.
But the initial condition computation could be skip in the second session
calling

.. doxygenfunction:: MoorDyn_Init_NoIC





MoorDyn with FAST.Farm
----------------------

In FAST.Farm, a new ability to use MoorDyn at the array level to simulate shared mooring systems has been develop.
Until the main branch of OpenFAST, the FAST.Farm capability, and the MoorDyn v2 capability are merged, 
the shared moorings capability in FAST.Farm uses the MoorDyn-F v1 input file format, with a 
small adjustment to reference attachments to multiple turbines.

https://github.com/mattEhall/openfast/tree/f/fast-farm

General Organization
^^^^^^^^^^^^^^^^^^^^

The regular ability for each OpenFAST instance to have its own MoorDyn simulation is unchanged in FAST.Farm. This 
ability can be used for any non-shared mooring lines in all cases. To enable simulation of shared mooring lines, 
which are coupled with multiple turbines, an additional farm-level MoorDyn instance has been added. This MoorDyn
instance is not associated with any turbine but instead is called at a higher level by FAST.Farm. Attachments
to different turbines within this farm-level MoorDyn instance are handled by specifying "TurbineN" as the type
for any points that are attached to a turbine, where "N" is the specific turbine number as listed in the 
FAST.Farm input file.


MoorDyn Input File
^^^^^^^^^^^^^^^^^^

The following input file excerpt shows how points can be specified as attached to specific turbines (turbines 
3 and 4 in this example). When a point has "TurbineN" as its type, it acts similarly to a "Vessel" type, where
the X/Y/Z inputs specify the relative location of the fairlead on the platform. In the farm-level MoorDyn input 
file, "Vessel" point types cannot be used because it is ambiguous which turbine they attach to.

.. code-block:: none
 :emphasize-lines: 5,6,12
 
 ----------------------- POINTS ----------------------------------------------
 Node      Type        X       Y         Z        M        V       CdA   CA
 (-)       (-)        (m)     (m)       (m)      (kg)     (m^3)   (m^2)  (-)
 1         Turbine3   10.0     0      -10.00      0        0        0     0
 3         Turbine4  -10.0     0      -10.00      0        0        0     0
 2         Fixed     267.0    80      -70.00      0        0        0     0
 -------------------------- LINE PROPERTIES -------------------------------------------------
 2     NLines - the number of lines
 Line     LineType  UnstrLen   NumSegs    NodeA     NodeB  Flags/Outputs
 (-)      (-)        (m)        (-)       (-)       (-)      (-)
 1     sharedchain  300.0        20        1         2        p
 2     anchorchain  300.0        20        1         3        p
 
In this example, Line 1 is a shared mooring line and Line 2 is an anchored mooring line that has a fairlead point
in common with the shared line. Individual mooring systems can be modeled in the farm-level MoorDyn instance as well.



FAST.Farm Input File
^^^^^^^^^^^^^^^^^^^^

In the branch of FAST.Farm the supports shared mooring capabilities, several additional lines have been added to the
FAST.Farm primary input file. These are highlighted in the example input file excerpt below


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
 "FarmMoorDyn.dat"  FarmMoorDynFile    Name of file containing shared mooring system input parameters (quoted string) [used only when Mod_SharedMooring > 0]
 0.01               DT_Mooring         Time step for farm-level mooring coupling with each turbine (s) [used only when Mod_SharedMooring > 0]
 --- AMBIENT WIND: PRECURSOR IN VTK FORMAT --- [used only for Mod_AmbWind=1]
 2.0                DT_Low-VTK         Time step for low -resolution wind data input files  ; will be used as the global FAST.Farm time step (s) [>0.0]
 0.3333333          DT_High-VTK        Time step for high-resolution wind data input files   (s) [>0.0]
 "Y:\Wind\Public\Projects\Projects F\FAST.Farm\AmbWind\steady"          WindFilePath       Path name to VTK wind data files from precursor (string)
 False              ChkWndFiles        Check all the ambient wind files for data consistency? (flag)
 --- AMBIENT WIND: INFLOWWIND MODULE --- [used only for Mod_AmbWind=2 or 3]
 2.0                DT_Low             Time step for low -resolution wind data interpolation; will be used as the global FAST.Farm time step (s) [>0.0]






Advice and Frequent Problems
----------------------------
   
   
Model Stability and Segment Damping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Two of the trickier input parameters are the internal damping (BA) for each line type, and the mooring simulation time step (dtM).  
Both relate to the discretization of the lines.
The highest axial vibration mode of the lumped-mass cable representation would be when adjacent nodes 
oscillate out of phase with each other, as depicted below.
 
In this mode, the midpoint of each segment would not move.  The motion of each node can then be 
characterized by mass-spring-damper values of

.. math::

  m = w \frac{L}{N}, \\ c = 4 B A \frac{N}{L}, \\ k = 4 E A \frac{N}{L}.

The natural frequency of this mode is then

.. math::

  \omega_n = \sqrt{\frac{k}{m}} = \frac{2}{l} \sqrt{\frac{E A}{w}}=2 \frac{N}{L} \sqrt{\frac{E A}{w}}

and the damping ratio, ζ, is related to the internal damping coefficient, BA, by

.. math::

  \zeta =\frac{c}{c_{crit}} = \frac{B}{l} \sqrt{\frac{A}{E w}} = B A \frac{N}{L} \sqrt{\frac{1}{E A w}}, \\ B A= \zeta \frac{L}{N} \sqrt{E A w}.

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
non-physical segment resonances. If the model is set up right, this damping can have a negligible 
contribution to the overall damping provided by the moorings on the floating platform.  However, if 
the damping contribution of the mooring lines on the floating platform is supposed to be significant, 
it is best to (1) set the BA value directly to ensure that the expected damping is provided and then 
(2) adjust the number of segments per line to whatever provides adequate numerical stability.

Finally, to ensure stability the time step should be significantly smaller than
the natural period,

.. math::

  \Delta t < \frac{2 \pi}{\omega_n}.

However, in contrast to the damping, which can be selected line by line, the
time step is a constant of the whole system, and thus should be selected
considering the minimum natural period of all lines.
