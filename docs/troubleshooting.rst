Troubleshooting
===============
.. _troubleshooting:

Here are some general points of advice for using MoorDyn. Another good resource is the
`closed issues page <https://github.com/FloatingArrayDesign/MoorDyn/issues>`_ on the 
MoorDyn-C repository . 
   
Model Stability and Segment Damping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Two of the trickier input parameters are the internal damping (BA) for each line type, 
and the mooring simulation time step (dtM). Both relate to the discretization of the 
lines. The highest axial vibration mode of the lumped-mass cable representation would 
be when adjacent nodes oscillate out of phase with each other, as depicted below.
 
In this mode, the midpoint of each segment would not move.  The motion of each node can
then be characterized by mass-spring-damper values of (where *w* is mass per length):

.. math::

  m = w \frac{L}{N}, \\ c = 4 B A \frac{N}{L}, \\ k = 4 E A \frac{N}{L}.

The natural frequency of this mode is then

.. math::

  \omega_n = \sqrt{\frac{k}{m}} = \frac{2}{l} \sqrt{\frac{E A}{w}}=2 \frac{N}{L} \sqrt{\frac{E A}{w}}

and the damping ratio, ζ, is related to the internal damping coefficient, BA, by

.. math::

  \zeta =\frac{c}{c_{crit}} = \frac{B}{l} \sqrt{\frac{A}{E w}} = B A \frac{N}{L} \sqrt{\frac{1}{E A w}}, \\ B A= \zeta \frac{L}{N} \sqrt{E A w}.

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
whole. It is just a way of letting MoorDyn calculate the damping coefficient automatically 
from the perspective of damping non-physical segment resonances. If the model is set up 
correctly, this damping can have a negligible contribution to the overall damping provided by 
the moorings on the floating platform.  However, if the damping contribution of the mooring 
lines on the floating platform is supposed to be significant, it is best to (1) set the BA 
value directly to ensure that the expected damping is provided and then (2) adjust the number 
of segments per line to whatever provides adequate numerical stability.

Finally, to ensure stability the time step should be significantly smaller than
the natural period,

.. math::

  \Delta t < \frac{2 \pi}{\omega_n}.

However, in contrast to the damping, which can be selected line by line, the
time step is a constant of the whole system, and thus should be selected
considering the minimum natural period of all lines.

Catenary Solve Unsuccessful
^^^^^^^^^^^^^^^^^^^^^^^^^^^

One of the most common issues encountered when using MoorDyn is the failure of the 
catenary solver to converge. The catenary solver is the first step in solving the 
initial conditions of the system. This approach tries to use the properties and geometry 
of the mooring lines to solve for a catenary shape. 

If this routine fails, you will see a "Catenary solve unsuccessful" message in the
the console and the log file. This means that MoorDyn will initialize the lines
as linear between the two defined end locations. After this, the ICgen process begins,
which runs a simulation with no external forcing, allowing the lines to 'fall' into
place. If the lines initialize as linear, then the initialization process will just take
longer, requiring a larger `TmaxIC` value. Explanations about the different initial 
condition generating methods can be found in the :ref:`initialization section <initialization>`.

The "Catenary Solve Unsuccessful" message does not impact the performance of MoorDyn or 
the results it produces, provided the initialization process converges before the simulation 
begins.

Python errors
^^^^^^^^^^^^^

ModuleNotFoundError: No module named 'moordyn'
----------------------------------------------

If you try to import MoorDyn into your Python file or shell and you receive the error 
above, then the Python wrapper is not properly installed. First, if you compiled MoorDyn 
by yourself check that PYTHON_WRAPPER option was ON while configuring with CMake.

Another possible source for the error is that MoorDyn was installed for a
different version of Python. That would happened if either CMake considered the
wrong version or you upgraded your Python installation. Either way, installing
the MoorDyn Python wrapper again should fix the problem.

ImportError
-----------
The import error can show up in a number of ways when importing the MoorDyn module into 
Python:
 
1. ImportError: dlopen(Python/3.9/lib/python/site-packages/cmoordyn.cpython-39-darwin.so, 0x0002): Library not loaded: @rpath/libmoordyn.2.dylib
   Reason: tried: '/usr/local/lib/libmoordyn.2.dylib' (no such file), '/usr/lib/libmoordyn.2.dylib' (no such file, not in dyld cache).

2. ImportError: libmoordyn.so.2: cannot open shared object file: No such file or directory.

If you try to import MoorDyn into your Python file or shell:

.. code-block:: python

   import moordyn
   moordyn.Create()

and you receive the error above, then the Python wrapper has been correctly installed,
but the actual MoorDyn library cannot be found. This is generally caused by an
installation of MoorDyn in a custom path or when installing MoorDyn without admin 
privileges. To resolve this error, add the path to the directory where the MoorDyn 
library was installed to the following environment variables:

* LD_LIBRARY_PATH in Linux and MAC (export LD_LIBRARY_PATH=<insert path here>)
* PATH in Windows

Another solution is to change the -DCMAKE_INSTALL_PREFIX flag for the 
:ref:`CMake compile method <CMake_compile>` to be the path until the lib folder where 
python is searching for the library. For example, to resolve error 1 (installing without 
admin privileges on MacOS), you would set -DCMAKE_INSTALL_PREFIX=/usr/local/ which would 
copy the libraries in the correct place. The total CMake configure command would be: 

.. code-block:: none

 cmake -DCMAKE_INSTALL_PREFIX="/usr/local/" -DCMAKE_BUILD_TYPE=Release
 DPYTHON_WRAPPER_USERINSTALL=ON ../

Note that this error can be slightly different in Linux, Windows, and MAC.
