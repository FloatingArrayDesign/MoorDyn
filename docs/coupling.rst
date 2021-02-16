Couplings and the API
=====================


Established couplings
---------------------

FAST/OpenFAST
^^^^^^^^^^^^^

MoorDyn-F is a core module with OpenFAST and is available included in `OpenFAST releases <https://github.com/openfast/openfast/releases>`_.
Originally, it was coupled to a modified form of FAST v7. 

WEC-Sim
^^^^^^^

WEC-Sim includes methods for coupling with MoorDyn v1, and support will be added for MoorDyn v2 when the opportunity arises.

DualSPHysics
^^^^^^^^^^^^

After developing a coupling with MoorDyn, the DualSPHysics have developed a seperate version of MoorDyn called MoorDyn+ dedicated 
to the coupling with DualSPHysics. The license for MoorDyn+ specifically permits its use with DualSPHysics.


Using from different languages
------------------------------

Python
^^^^^^

MoorDyn can be called from Python scripts.

Matlab
^^^^^^

MoorDyn can be called from Matlab scripts.

Simulink
^^^^^^^^

MoorDyn can be used with Simulink (and SimMechanics) models. The challenge is in supporting MoorDyn's loose-coupling approach 
where it expects to be called for sequential time steps and never for correction steps that might repeat a time step. 
A pulse/time-triggering block can be used in Simulink to ensure MoorDyn is called correctly. An example of this can 
be seen in WEC-Sim.



Calling MoorDyn - the API
-------------------------

(THIS PAGE IN PROGRESS)

C++ Functions
^^^^^^^^^^^^^

.. doxygenfunction:: LinesInit

.. doxygenfunction:: LinesCalc

.. doxygenfunction:: Line::doRHS



C++ Classes
^^^^^^^^^^^

.. doxygenclass:: Line

.. doxygenclass:: Connection



