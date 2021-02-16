MoorDyn Usage
=====================================================



The Input File
--------------

MoorDyn uses a plain-text input file for its description of the mooring system as well as simulation settings.
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

(THIS PAGE IN PROGRESS)