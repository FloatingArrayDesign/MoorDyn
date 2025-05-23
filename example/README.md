This subdirectory is a demonstration of the MoorDyn-F driver. While beyond the scope of what is discussed in the presentation, it is a useful tool if you are looking to work with the MoorDyn-F code directly without going through OpenFAST. This driver is set up to run the body.txt file from the MoorDyn directory.

If you are in the CoCalc environment, you can run the driver with the following commands in the terminal:

```
1. anaconda2023
2. conda activate openfast_env
3. moordyn_driver md_driver.dvr
```

If you are not in the CoCalc environment, you need to locate the MoorDyn driver from the OpenFAST release files, or compile it yourself. More information in this can be found in the [OpenFAST documentation](https://openfast.readthedocs.io/en/main/). Once locating the driver and moving it to this directory, run step 3 above in the terminal. 

More information on the MoorDyn-F driver can be found in the MoorDyn docs [MoorDyn-F driver section](https://moordyn.readthedocs.io/en/latest/inputs.html#moordyn-f-driver-input-file).