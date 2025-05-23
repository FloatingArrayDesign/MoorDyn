This subdirectory is a demonstration of running OpenFAST with MoorDyn-F. These files are very similar to what was shown in the OpenFAST WPTO Software Demo Days demonstration. A few small changes were made, limiting the simulation to run for only one second and changing the output channel list for MoorDyn. The main MoorDyn output file was also disabled, as those outputs can be found in the OpenFAST output file. 

If you are in the CoCalc environment, you can run the code with the following commands in the terminal:

```
1. anaconda2023
2. conda activate openfast_env
3. openfast MHK_RM1_Floating.fst
```

If you are not in the CoCalc environment, you need to locate the OpenFAST executable from the OpenFAST release files, or compile it yourself. More information in this can be found in the [OpenFAST documentation](https://openfast.readthedocs.io/en/main/). Once locating the executable and moving it to this directory, run step 3 above in the terminal. 

