MoorDyn Wrappers and Drivers
============================

.. _drivers:

The following discussion explains how to drive MoorDyn-C using a driver function. 
MoorDyn-F contains a driver script that has a :ref:`separate input file <MDF_driver_in>`
and MoorDyn-F compiles in OpenFAST as moordyn_driver. The MoorDyn-F driver follows all 
the same principles as the examples below. See :ref:`compiling <compiling>` and 
:ref:`inputs <inputs>` sections for instructions on how to use the MoorDyn-F driver. 
Additionally, MoorDyn-F has a c-bindings interface, which allows it (along with the rest 
of OpenFAST) to be coupled with other languages. The MoorDyn-F C interface is set up using 
the MoorDyn V1 approach (single 6 DOF coupling), thus it requires a single coupled body to 
be used in the MoorDyn input file.

Currently MoorDyn-C v2 can be used in Python, C/C++, Fortran, and Matlab. You can
read more on how to install MoorDyn for each different language in
the :ref:`compiling documentation <compiling>`. 

All the wrappers are built off :ref:`the C API <api_c>` which can be found in the 
MoorDyn2.h file, so it is the best place to find a list of available functions.

For MoorDyn to be run, it needs a driver script written in one of the compatible 
languages using the corresponding wrapper. MoorDyn-C v2 allows for multiple instances of 
MoorDyn to be run simultaneously, while MoorDyn-F v2 and MoorDyn-C/F v1 only allow once 
instance at a time. Any driver function will call at least the following MoorDyn 
functions:

* Create - MoorDyn-C v2 only
* Initialize (r, rd)
* Step (r, rd, t ,dt)
* Close

The initialize function takes the state vectors at time 0 and sets up MoorDyn for a time 
series simulation. It will also find the steady state of the system if the input file calls for it.
The r and rd vectors in theinitalize function represent the inital status of the system and should 
match what is defined in the input file. 

The step function takes the state vectors (r - positions and rd - time dervative of r) at a given 
time, the time, and the time step size. The step function will integrate MoorDyn the ammount you 
provide with dt. If dt > dtM, then MoorDyn will substep using dtM as the internal timestep to 
integrate one dt timestep. If dt < dtM then MoorDyn will use dt as the internal timestep to 
integrate one dt timestep. The value of rd is assumed constant for the dt timestep, as well as the 
derived acceleration (relevant for the inertial response of coupled objects). For that reason, dt 
should be set as small as possible to achieve a reasonable runtime.

The close function clears up memory and safely destroys the MoorDyn system. 

For both the step and the initialize functions, the input state vector size needs 
to correspond to the DOF of the coupled object or objects. The input vector is 1D with a 
length of: degree of freedom of coupled object * number of coupled objects. If you have 
multiple different types of coupled objects then the order in the state vector is 
body (6 DOF), rod (6 DOF), and then points (3 DOF). The same order applies for the state 
derivative input vector, with each value being the time derivative of the respective value. 
The degrees of freedom are as follows (all relative to the global reference frame):

 - Bodies and cantilevered/coupled Rods: cartesian positions followed by the Euler angles 
   relative to the global reference frame. 
 - Pinned bodies and rods: cartesian positions relative to the global reference frame.
 - Coupled points: cartesian positions relative to the global reference frame.  

For example, the state vector for a coupled body and coupled point would be:

   [ x1, y1, z1, roll1, pitch1, yaw1, x2, y2, z2 ]

The state derivative vector represents the time derivate of the values in the state vector. It is multiplied by the 
internal timestep to get the new positions of the system. For rotational degrees of freedom the state derivative is
time derivatives of the angles. The rotation matrix in both MD-F and MD-C used to describe the 
objects orientation is given by R = X(roll)Y(pitch)Z(yaw). In the examples below, the state vector is referred to 
as x and the state derivative as xd.

Driving MoorDyn-C v1 is a similar process as MoorDyn v2. MoorDyn-C v1 has no built in 
couplings and needs to be driven based on the C API in the MoorDyn.h file. An example on 
how to do this in python is provided at the end of the 
:ref:`python section <python_wrapper>`. 

Note: For coupled pinned bodies and rods the state vector still needs to be size 6, and MoorDyn will just 
ignore the rotational values provided. 

MoorDyn-C Coupling
------------------

MoorDyn-C can be compiled as a dynamically linked library with C bindings or a more 
sophisticated API functions and wrappers, making it accessible from a wide range of 
programming languages.

Further examples of MoorDyn-C drivers with input files can be found in the `examples folder <https://github.com/FloatingArrayDesign/MoorDyn/tree/dev/example>`_.

Python
^^^^^^
.. _python_wrapper:

If you have pip installed MoorDyn then you are good to go. If not, see the 
:ref:`compiling section <compiling>` for instructions on how to pip install the python 
module, how to compile the python module locally (CMake compile method), or how to 
compile a library that is called from python (simple library compile method). 

We can start considering the following example, consisting of 3 lines
anchored at the seabed connected to 3 coupled fairleads that we 
control:

.. code-block:: python

    import moordyn

    system = moordyn.Create("Mooring/lines.txt")

    # 3 coupled points x 3 components per point = 9 DoF
    xd = [0] * 9
    # Get the initial positions from the system itself
    x = []
    for i in range(3):
        # 4 = first fairlead id
        point = moordyn.GetPoint(system, i + 4)
        x = x + list(moordyn.GetPointPos(point))

    # Setup the initial condition
    moordyn.Init(system, x, xd)

    # Make the points move at 0.5 m/s to the positive x direction
    for i in range(3):
        xd[3 * i] = 0.5
    t, dt = 0.0, 0.5
    f = moordyn.Step(system, x, xd, t, dt)

    # Print the position and tension of the line nodes
    n_lines = moordyn.GetNumberLines(system)
    for line_id in range(1, n_lines + 1):
        print("Line {}".format(line_id))
        print("=======")
        line = moordyn.GetLine(system, line_id)
        n_segs = moordyn.GetLineN(line)
        for node_id in range(n_segs+1):
            print("  node {}:".format(node_id))
            pos = moordyn.GetLineNodePos(line, node_id)
            print("  pos = {}".format(pos))
            ten = moordyn.GetLineNodeTen(line, node_id)
            print("  ten = {}".format(ten))

    # Alright, time to finish!
    moordyn.Close(system)

In Python the functions trigger exceptions if errors are detected. Python can stop 
execution when an error is detected using a try:

.. code-block:: python

    import moordyn

    system = moordyn.Create("Mooring/lines.txt")
    try:
        your_coupling_code(system)
    except Exception:
        raise
    finally:
        moordyn.Close(system)

So you can assert that the resources are always correctly released, no matter
if the code worked properly or exceptions were triggered. 

MoorDyn-C v1 and v2 can also be run in python using the C API with the use of the ctypes 
library. Below is an example of this on MacOS with MoorDyn compiled as a 
:ref:`simple library <compile_simple>`, assuming a stationary coupled body:

.. code-block:: python

   import ctypes
   import numpy as np

   rootname = 'lines'
   extension = '.txt'
   path = 'Mooring/'
   tMax = 25.0
   dtM = 0.001
   time = np.arange(0, tMax, dtM)
   vector_size = 6 # 6DOF coupled object
   size = (len(time), vector_size)

   #specifying correct dtypes for conversion to C types
   x = np.zeros(size, dtype = float)
   xd = np.zeros(size, dtype = float)

   dylib_path = 'MoorDyn/compile/DYLIB/libmoordyn2.dylib'
   filename = path+rootname+extension

   # Double vector pointer data type
   double_p = ctypes.POINTER(ctypes.c_double)

   # -------------------- load the MoorDyn DYLIB ---------------------
   # Make MoorDyn function prototypes and parameter lists (remember, first entry is return type, rest are args)
   MDInitProto = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), ctypes.c_char_p) #need to add filename option here, maybe this c_char works? #need to determine char size 
   MDStepProto = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), double_p, double_p)
   MDClosProto = ctypes.CFUNCTYPE(ctypes.c_int)

   MDInitParams = (1, "x"), (1, "xd"), (1, "infilename") # 1 flag is input, 2 flag is output
   MDStepParams = (1, "x"), (1, "xd"), (2, "f"), (1, "t"), (1, "dtC") 

   MDdylib = ctypes.CDLL(dylib_path) #load moordyn dylib

   MDInit = MDInitProto(("MoorDynInit", MDdylib), MDInitParams)
   MDStep = MDStepProto(("MoorDynStep", MDdylib), MDStepParams)
   MDClose= MDClosProto(("MoorDynClose", MDdylib))  
   
   # ------------------------ run MoorDyn ---------------------------
   # initialize some arrays for communicating with MoorDyn
   t  = double_p()    # pointer to t

   # parameters
   dtC = ctypes.pointer(ctypes.c_double(dtM))
   infile = ctypes.c_char_p(bytes(filename, encoding='utf8'))

   # initialize MoorDyn at origin
   MDInit((x[0,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)),(xd[0,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)),infile)
   print("MoorDyn initialized - now performing calls to MoorDynStep...")

   # loop through coupling time steps
   for i in range(len(time)):
      t = ctypes.pointer(ctypes.c_double(time[i]))
      MDStep((x[i,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)), (xd[i,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)), t, dtC)    
   print("Succesffuly simulated for {} seconds - now closing MoorDyn...".format(tMax))  

   # close MoorDyn simulation (clean up the internal memory, hopefully) when finished
   MDClose() 

Notes on the Python C API:

- The C API includes support for the v1 and v2 API. This example uses the v1 API 
  (MoorDyn.h in v1 and v2). A similar approach could be taken for the v2 API found in the 
  :ref:`C API section <api_c>` and also in the MoorDyn2.h file.
- The available functions can be found in the MoorDyn.h files.
	- These functions are declared in the following way:

  .. code-block:: python

   	 functionPROTO = ctypes.CFUNCTYPE(ctypes.c_int, <function inputs>)
	   functionParams = (1, "<input>"), (2, "<output>") # a tuple of tuples where each item in the function inputs list is given an input (1) or output (2) flag 
  	 function = functionPROTO(("<function name from C API>", MDdylib), functionParams)
   	
- Using this method does not call the create function because the v1 API does not allow 
  for simultaneous MoorDyn instances. 
- The initialize function is MDInit.   
- MoorDyn functions require C data types as inputs.

C/C++
^^^^^^

The easiest way to link MoorDyn to your C/C++ project is using CMake. The following
Is a code snippet where MoorDyn is included in a project with only a C source
code file named example.c:

.. code-block:: cmake

   CMake_minimum_required (VERSION 3.10)
   project (myproject)

   find_package (MoorDyn REQUIRED)

   add_executable (example example.c)
   target_link_libraries (example MoorDyn::moordyn)

CMake itself will take care of everything. In the example.c file you only
need to include the MoorDyn2.h header and start using the :ref:`C API <api_c>`,
as it is discussed in the :ref:`coupling documentation <coupling>`.

.. code-block:: c

   #include <moordyn/MoorDyn2.h>

   int main(int, char**)
   {
      MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
      MoorDyn_Close(system);
   }

The same CMake code for C is equally valid for C++. In your C++
code you must remember to start by including the MoorDyn configuration header and then
the main header, i.e.

.. code-block:: cpp

   #include <moordyn/Config.h>
   #include <moordyn/MoorDyn2.hpp>

   int main(int, char**)
   {
      auto system = new moordyn::MoorDyn("Mooring/lines.txt");
      delete system;
   }

It is recommended to use CMake to link
MoorDyn into your project (see :ref:`"Compiling" <compiling>`), although it
is not strictly required. For instance, if you installed it in the default
folder in Linux, you just need to add the flag "-lmoording" while linking
(either with GCC or CLang).

Below you can find the equivalent example discussed above for the Moordyn python module,
this time developed in C:


.. code-block:: c

    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <moordyn/MoorDyn2.h>

    int main(int, char**)
    {
        int err;
        MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
        if (!system)
            return 1;

        // 3 coupled points x 3 components per point = 9 DoF
        double x[9], xd[9];
        memset(xd, 0.0, sizeof(double));
        // Get the initial positions from the system itself
        for (unsigned int i = 0; i < 3; i++) {
            // 4 = first fairlead id
            MoorDynPoint point = MoorDyn_GetPoint(system, i + 4);
            err = MoorDyn_GetPointPos(point, x + 3 * i);
            if (err != MOORDYN_SUCCESS) {
                MoorDyn_Close(system);
                return 1;
            }
        }

        // Setup the initial condition
        err = MoorDyn_Init(system, x, xd);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }

        // Make the points move at 0.5 m/s to the positive x direction
        for (unsigned int i = 0; i < 3; i++)
            xd[3 * i] = 0.5;
        double t = 0.0, dt = 0.5;
        double f[9];
        err = MoorDyn_Step(system, x, xd, f, &t, &dt);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }

        // Print the position and tension of the line nodes
        unsigned int n_lines;
        err = MoorDyn_GetNumberLines(system, &n_lines);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }
        for (unsigned int i = 0; i < n_lines; i++) {
            const unsigned int line_id = i + 1;
            printf("Line %u\n", line_id);
            printf("=======\n");
            MoorDynLine line = MoorDyn_GetLine(system, line_id);
            if (!line) {
                MoorDyn_Close(system);
                return 1;
            }
            unsigned int n_nodes;
            err = MoorDyn_GetLineNumberNodes(line, &n_nodes);
            if (err != MOORDYN_SUCCESS) {
                MoorDyn_Close(system);
                return 1;
            }
            for (unsigned int j = 0; j < n_nodes; j++) {
                printf("  node %u:\n", j);
                double pos[3], ten[3];
                err = MoorDyn_GetLineNodePos(line, j, pos);
                if (err != MOORDYN_SUCCESS) {
                    MoorDyn_Close(system);
                    return 1;
                }
                printf("  pos = [%g, %g, %g]\n", pos[0], pos[1], pos[2]);
                err = MoorDyn_GetLineNodeTen(line, j, ten);
                if (err != MOORDYN_SUCCESS) {
                    MoorDyn_Close(system);
                    return 1;
                }
                printf("  ten = [%g, %g, %g]\n", ten[0], ten[1], ten[2]);
            }
        }

        // Alright, time to finish!
        err = MoorDyn_Close(system);
        if (err != MOORDYN_SUCCESS)
            return 1;

        return 0;
    }

In the example above everything starts calling

.. doxygenfunction:: MoorDyn_Create

and checking that it returned a non-NULL system. A NULL system would mean that
there were an error building up the system. You can learn more about the
error in the information printed on the terminal.

In C requires explicit type names, while in C++ you can be more
abstract, i.e. you can do something like this:

.. code-block:: c

    auto system = MoorDyn_Create("Mooring/lines.txt");
    auto line = MoorDyn_GetLine(system, 1);

The next step is initializing the system, which computes the
static solution if the TmaxIC flag in the options section is greater than 0. This 
requires the position of the coupled fairleads.

.. doxygenfunction:: MoorDyn_GetPoint
.. doxygenfunction:: MoorDyn_GetPointPos

The :ref:`C API <api_c>` always returns either an
object or an error code:

.. doxygengroup:: moordyn_errors_c

Thus, you can always check that everything properly worked.

With the information of the initial positions of the fairlead, you can initialize MoorDyn:

.. doxygenfunction:: MoorDyn_Init

Afterwards you can run MoorDyn by calling:

.. doxygenfunction:: MoorDyn_Step

In this example, we are just calling it once. In a more complex application the
function will be called in a loop over a time series. In the API there are a number of 
functions that can return information at each timestep that can be implemented in more 
complex drivers. The full list of functions can be found in the 
:ref:`C API section <api_c>`.

It is important to close the MoorDyn system, so that the allocated resources are released:

.. doxygenfunction:: MoorDyn_Close

Fortran
^^^^^^^
This is not to be confused with MoorDyn-F, which relies on modules within the openFAST 
library. MoorDyn-F when compiled includes a driver function with its own driver input 
file. 

This coupling packages MoorDyn-C to be used in standalone Fortran projects. 
Linking the Fortran wrapper of MoorDyn-C is almost the same as linking the C
library. For instance, if you have a Fortran project consisting of a single
source code file, example.f90, then you can compile the driver with the
following CMake code:

.. code-block:: CMake

   CMake_minimum_required (VERSION 3.10)
   project (myproject)

   find_package (MoorDyn REQUIRED)

   add_executable (example example.f90)
   target_link_libraries (example MoorDyn::MoorDyn-F)

Please, note that now you are linking against MoorDyn::MoorDyn-F (not the same as 
the MoorDyn-F in OpenFAST). 

Here is the same example from above, displayed in Fortran:

.. code-block:: fortran

    program main
      use, intrinsic :: iso_fortran_env, only: real64
      use, intrinsic :: iso_c_binding, only: c_ptr, c_associated
      use moordyn

      character(len=28) :: infile
      real(real64), allocatable, target :: x(:)
      real(real64), allocatable, target :: xd(:)
      real(real64), allocatable, target :: f(:)
      real(real64), allocatable, target :: r(:)
      real(real64) :: t, dt
      integer :: err, n_dof, n_points, i_point, n_lines, i_line, n_nodes, i_node
      type(c_ptr) :: system, point, line

      infile = 'Mooring/lines.txt'

      system = MD_Create(infile)
      if ( .not.c_associated(system) ) then
        stop 1
      end if

      err = MD_NCoupledDOF( system, n_dof )
      if ( err /= MD_SUCESS ) then
        stop 1
      elseif ( n_dof /= 9 ) then
        print *,"3x3 = 9 DOFs were expected, not ", n_dof
      end if

      allocate ( x(0:8) )
      allocate ( xd(0:8) )
      allocate ( f(0:8) )
      allocate ( r(0:2) )
      xd = 0.0
      f = 0.0

      ! Get the positions from the points
      err = MD_GetNumberPoints( system, n_points )
      if ( err /= MD_SUCESS ) then
        stop 1
      elseif ( n_points /= 6 ) then
        print *,"6 points were expected, not ", n_points
      end if
      do i_point = 1, 3
        point = MD_GetPoint( system, i_point + 3 )
        if ( .not.c_associated(point) ) then
          stop 1
        end if
        err = MD_GetPointPos( point, r )
        if ( err /= MD_SUCESS ) then
          stop 1
        end if
        do j = 1, 3
          x(3 * i + j) = r(j)
        end do
      end do

      err = MD_Init(system, x, xd)
      if ( err /= MD_SUCESS ) then
        stop 1
      end if

      t = 0
      dt = 0.5
      err = MD_Step(system, x, xd, f, t, dt)
      if ( err /= MD_SUCESS ) then
        stop 1
      end if

      ! Print the position and tension of the line nodes
      err = MD_GetNumberLines(system, n_lines)
      if ( err /= MD_SUCESS ) then
        stop 1
      end if
      do i_line = 1, n_lines
        print *,"Line ", i_line
        print *, "======="
        line = MD_GetLine(system, i_line)
        err = MD_GetLineNumberNodes(line, n_nodes)
        do i_node = 0, n_nodes - 1
          print *,"  node ", i_node, ":"
          err = MD_GetLineNodePos(line, i_node, r)
          print *,"  pos = ", r
          err = MD_GetLineNodeTen(line, i_node, r)
          print *,"  ten = ", r
        end do
      end do

      err = MD_Close(system)
      if ( err /= MD_SUCESS ) then
        stop 1
      end if

      deallocate ( x )
      deallocate ( xd )
      deallocate ( f )
      deallocate ( r )

    end program main

It is very similar to the C code, although the functions have a different
prefix. All the objects (the simulator, the points, the
lines...) take the type type(c_ptr), from the iso_c_binding module. The rest of
the differences are just language syntax.

MATLAB
^^^^^^
To use this feature, insure you used the CMake compile method with the MATLAB install 
turned on. Using MoorDyn in MATLAB is similar to using it in Python. However, in
MATLAB you must manually add the folder where the wrapper files are located to the path. 
To achieve this, in MATLAB go to the HOME menu, section ENVIRONMENT, and click on 
"Set Path". In the window appearing click on "Add Folder...", and set the folder that 
contains the contents of MoorDyn/build/wrappers/matlab/, which by default is:

* C:\Program Files (x86)\MoorDyn\bin in Windows
* /usr/lib in Linux and MacOS

After that you are good to go!

Considering the same example above, the resulting Matlab code would look like
the following:

.. code-block:: matlab

    system = MoorDynM_Create('Mooring/lines.txt');

    %% 3 coupled points x 3 components per point = 9 DoF
    x = zeros(9,1);
    xd = zeros(9,1);
    %% Get the initial positions from the system itself
    for i=1:3
        %% 4 = first fairlead id
        point = MoorDynM_GetPoint(system, i + 3);
        x(1 + 3 * (i - 1):3 * i) = MoorDynM_GetPointPos(point);
    end

    %% Setup the initial condition
    MoorDynM_Init(system, x, xd);

    %% Make the points move at 0.5 m/s to the positive x direction
    for i=1:3
        xd(1 + 3 * (i - 1)) = 0.5;
    end
    t = 0.0;
    dt = 0.5;
    [t, f] = MoorDynM_Step(system, x, xd, t, dt);

    %% Print the position and tension of the line nodes
    n_lines = MoorDynM_GetNumberLines(system);
    for line_id=1:n_lines
        line_id
        line = MoorDynM_GetLine(system, line_id);
        n_nodes = MoorDynM_GetLineNumberNodes(line);
        for node_id=1:n_nodes
            node_id
            pos = MoorDynM_GetLineNodePos(line, node_id - 1);
            pos
            ten = MoorDynM_GetLineNodeTen(line, node_id - 1);
            ten
        end
    end

    %% Alright, time to finish!
    MoorDynM_Close(system);

It is fairly similar to Python. The functions do
not return error codes, only the queried information.
However, the functions do trigger exceptions that can be caught by Matlab.
That feature should be used so that MoorDynM_Close() is
called even if the execution fails.

Simulink
^^^^^^^^
MoorDyn can be used with Simulink (and SimMechanics) models. The challenge is in
supporting MoorDyn's loose-coupling approach where it expects to be called for
sequential time steps and never for correction steps that might repeat a time
step.
A pulse/time-triggering block can be used in Simulink to ensure MoorDyn is
called correctly. An example of this can be seen in WEC-Sim.


Established couplings
---------------------
.. _coupling:

MoorDyn-F with FAST.Farm
^^^^^^^^^^^^^^^^^^^^^^^^

In FAST.Farm, a new ability to use MoorDyn at the array level to simulate shared mooring 
systems has been developed. It is described in 
https://doi.org/10.1016/j.apenergy.2022.120513. An example of the full input file setup 
can be found at https://github.com/FloatingArrayDesign/FASTFarm_10Turbines_Shared.

General Organization
""""""""""""""""""""

The regular ability for each OpenFAST instance to have its own MoorDyn simulation is 
unchanged in FAST.Farm. This ability can be used for any non-shared mooring lines in all 
cases. To enable simulation of shared mooring lines, which are coupled with multiple 
turbines, an additional farm-level MoorDyn instance has been added. This MoorDyn instance 
is not associated with any turbine but instead is called at a higher level by FAST.Farm. 
Attachments to different turbines within this farm-level MoorDyn instance are handled by 
specifying "TurbineN" as the type for any connections that are attached to a turbine, 
where "N" is the specific turbine number as listed in the FAST.Farm input file.


MoorDyn Input File
""""""""""""""""""

The following input file excerpt shows how connections can be specified as attached to 
specific turbines (turbines 3 and 4 in this example). When a connection has "TurbineN" 
as its type, it acts similarly to a "Vessel" type, where the X/Y/Z inputs specify the 
relative location of the fairlead on the platform. In the farm-level MoorDyn input file, 
"Vessel" connection types cannot be used because it is ambiguous which turbine they 
attach to.

.. code-block:: none
 :emphasize-lines: 5,6,12
 
 ----------------------- POINTS ----------------------------------------------
 ID  Attachment     X       Y         Z        Mass    Volume     CdA   Ca
 (-)       (-)        (m)     (m)       (m)      (kg)     (m^3)   (m^2)  (-)
 1         Turbine3   10.0     0      -10.00      0        0        0     0
 3         Turbine4  -10.0     0      -10.00      0        0        0     0
 2         Fixed     267.0    80      -70.00      0        0        0     0
 -------------------------- LINES --------------------------------------------
 ID    LineType      AttachA  AttachB  UnstrLen  NumSegs  LineOutputs

 (-)      (-)        (-)       (-) 	(m)    (-)   (-)
 1     sharedchain    1         2    300.0     20     p
 2     anchorchain    1         3    300.0     20     p

 
In this example, Line 1 is a shared mooring line and Line 2 is an anchored mooring line 
that has a fairlead connection in common with the shared line. Individual mooring systems 
can be modeled in the farm-level MoorDyn instance as well.



FAST.Farm Input File
""""""""""""""""""""

In the branch of FAST.Farm the supports shared mooring capabilities, several additional 
lines have been added to the FAST.Farm primary input file. These are highlighted in the 
example input file excerpt below


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
   
Model Stability and Segment Damping
"""""""""""""""""""""""""""""""""""

Two of the trickier input parameters are the internal damping (BA) for each line type, 
and the mooring simulation time step (dtM). Both relate to the discretization of the 
lines. The highest axial vibration mode of the lumped-mass cable representation would be 
when adjacent nodes oscillate out of phase with each other, as depicted below.
 
In this mode, the midpoint of each segment would not move.  The motion of each node can 
then be characterized by mass-spring-damper values of

.. math::

  m=w L/N \; c=4NBA/L \; k=4NEA/L.

The natural frequency of this mode is then

.. math::

  \omega_n=\sqrt{k/m}=2/l \sqrt{EA/w}=2N/L \sqrt{EA/w}

and the damping ratio, ζ, is related to the internal damping coefficient, BA, by

.. math::

  \zeta =c/c_{crit} = B/l \sqrt{A/Ew} = NBA/L \sqrt{(1/EAw}  \;\;  BA=\zeta \frac{L}{N}\sqrt{EAw}.

The line dynamics frequencies of interest should be lower than ω_n in order to be 
resolved by the model. Accordingly, line dynamics at ω_n, which are likely to be 
dominated by the artificial resonance created by the discretization, can be damped out 
without necessarily impacting the line dynamics of interest. This is advisable because 
the resonances at ω_n can have a large impact on the results. To damp out the segment 
vibrations, a damping ratio approaching the critical value (ζ=1) is recommended. Care 
should be taken to ensure that the line dynamics of interest are not affected.

To simplify things, a desired line segment damping ratio can be specified in the input 
file.  This is done by entering the negative of the desired damping ratio in the BA/-zeta 
field of the Line Types section. A negative value here signals MoorDyn to interpret it as 
a desired damping ratio and then calculate the damping coefficient (BA) for each mooring 
line that will give every line segment that damping ratio (accounting for possible 
differences in segment length between lines).  

Note that the damping ratio is with respect to the critical damping of each segment along 
a mooring line, not with respect to the line as a whole or the floating platform as a 
whole.  It is just a way of letting MoorDyn calculate the damping coefficient 
automatically from the perspective of damping non-physical segment resonances. If the 
model is set up right, this damping can have a negligible contribution to the overall 
damping provided by the moorings on the floating platform.  However, if the damping 
contribution of the mooring lines on the floating platform is supposed to be significant, 
it is best to (1) set the BA value directly to ensure that the expected damping is 
provided and then (2) adjust the number of segments per line to whatever provides 
adequate numerical stability.

FAST/OpenFAST
^^^^^^^^^^^^^

MoorDyn-F, is a core module within OpenFAST and
is available in
`OpenFAST releases <https://github.com/openfast/openfast/releases>`_.
Originally, it was coupled to a modified form of FAST v7. 

WEC-Sim
^^^^^^^

WEC-Sim is coupled with MoorDyn-C v2 as of Spring 2024 with `WEC-Sim PR #1212 <https://github.com/WEC-Sim/WEC-Sim/pull/1212>`_. 
The original coupling was with MoorDyn v1 and the the publication can be found in the 
:ref:`theory section <theory>`. The WEC-Sim source code can be found `here <https://github.com/WEC-Sim/WEC-Sim>`_.

DualSPHysics
^^^^^^^^^^^^

After developing a coupling with MoorDyn, the DualSPHysics team has forked it in
a seperate version called MoorDyn+, specifically dedicated to the coupling with
DualSPHysics.

OpenFOAM
^^^^^^^^

MoorDyn-C has been coupled with OpenFOAM through the `foamMooring <https://gitlab.com/hfchen20/foamMooring>`_ project.

Bladed
^^^^^^

MoorDyn V1 has been coupled with DNV's Bladed software. See the following links for details:

- `MoorDyn-Bladed Coupling Documentation <https://mysoftware.dnv.com/download/public/renewables/bladed/documentation/4_16/workflow/coSimulation/Bladed-MoorDynLink/Overview.html>`_
- `MoorDyn-Bladed Coupling Theory <https://asmedigitalcollection.asme.org/OMAE/proceedings/IOWTC2023/87578/V001T01A011/1195013>`_