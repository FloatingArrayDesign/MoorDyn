This example directory contains instructions for common uses of MoorDyn, including OpenFAST, WEC-Sim, and running standalone. 

Before working with these examples, please look at the [MoorDyn documentation](https://moordyn.readthedocs.io/en/latest/drivers.html#python) for an explanation of the code and the different versions (MoorDyn-C and MoorDyn-F). After that, start out with working through the jupyter notebook (`MoorDyn_standalone_demo.ipynb`). This will walk through basic examples for running MoorDyn and visualizing outputs. Visualization is done with MoorPy, however Paraview can also be used as well as the tool [PyDatView](https://github.com/ebranlard/pyDatView). 

Much of this directory was presented during Software Demo Days. A link to that presentation can be found here: https://www.youtube.com/watch?v=FqW7Xpl_VNk

Further examples are the following:

- `MHK_RM1_Floating`: This directory shows how to run MoorDyn-F with OpenFAST.
- `MoorDynF_example`: This directory shows how to use the MoorDyn-F driver from OpenFAST.
- `simpledemo.py`: This is the simplest example of how to run MoorDyn-C with a python script.
- `Running MoorDyn-F (OpenFAST).pdf`: This is a step-by-step list of instructions for running MoorDyn-F with OpenFAST.
- `Running WECSim with MoorDyn.pdf`: This is a step-by-step list of instructions for running MoorDyn-C with WECSim.