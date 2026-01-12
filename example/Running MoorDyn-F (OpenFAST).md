<!-- Copilot generated markdown files from hmtl exports of the instructions word docs -->

# OpenFAST and MoorDyn-F

MoorDyn-F is the core mooring dynamics module in OpenFAST. It is structured to seamlessly fit into the OpenFAST code structure, and thus is less suitable for external couplings than MoorDyn-C. The steps for running MoorDyn with OpenFAST and FAST.Farm are very similar to running any of the other OpenFAST modules. First, enable the module in the main input file, then provide the path to the module input file, and then configure the module input file for your simulation. Finally, run your simulation.

## Running OpenFAST with MoorDyn-F

See the instructions in the `Running OpenFAST.pdf` from the OpenFAST co-calc demonstration for details on setting up OpenFAST more broadly. Additional OpenFAST information can be found in the [OpenFAST documentation](https://openfast.readthedocs.io/en/main/). The general steps for running OpenFAST with MoorDyn-F are as follows:

- Obtain the OpenFAST executables
  - Compile from source code
  - Download executables from [GitHub releases](https://github.com/OpenFAST/openfast/releases)
- Configure the OpenFAST input files for your system
  - Set `CompMooring = 3` in the main OpenFAST `.fst` input file
  - Set the `MooringFile` variable in the main OpenFAST `.fst` input file to the MoorDyn input file path
- Configure the MoorDyn input file for your system
  - Note that coupled MoorDyn objects (bodies, rods, and points) are rigidly attached to the ElastoDyn object instance
- Execute `openfast <input_file>.fst`

## Running MoorDyn-F with the MoorDyn Driver

The MoorDyn driver allows for MoorDyn-F to be run as a stand-alone module, enabling debugging of OpenFAST simulations and MoorDyn-only simulations. To run the driver, an additional input file is required that fills in the information normally provided to MoorDyn by the OpenFAST input file. Information on this file can be found in the [MoorDyn documentation](https://moordyn.readthedocs.io/en/latest/inputs.html#moordyn-f-driver-input-file). The general steps for running MoorDyn-F with the MoorDyn driver are as follows:

- Obtain the driver executable
  - Compile from source code
  - Download executable from [GitHub releases](https://github.com/OpenFAST/openfast/releases)
- Configure the MoorDyn driver input file `.dvr` following the formatting described in the [MoorDyn documentation](https://moordyn.readthedocs.io/en/latest/inputs.html#moordyn-f-driver-input-file).
  - A time series input file is needed for providing motion to coupled MoorDyn objects. The path to this input file is set as the `InputsFile` variable in the MoorDyn driver input file (note that if `InputsMode = 0` all coupled objects positions will be fixed at 0 and the time series file will be ignored)
- Configure the MoorDyn input file
  - Coupled objects will be driven by a time series file provided in the driver input file
- Execute `moordyn_driver <input_file>.dvr`

## Running FAST.Farm with MoorDyn-F

FAST.Farm, the wind farm simulation tool built on OpenFAST, can also be run with MoorDyn. More information on FAST.Farm can be found in the [OpenFAST documentation](https://openfast.readthedocs.io/en/main/source/user/fast.farm/index.html#fast-farm-user-s-guide-and-theory-manual). FAST.Farm runs an OpenFAST instance for each turbine in the farm and thus can have a MoorDyn instance for each turbine. Additionally, there can be a farm level MoorDyn instance, which allows for the simulation of shared mooring lines. Instructions for using MoorDyn with FAST.Farm can be found in the [MoorDyn documentation](https://moordyn.readthedocs.io/en/latest/inputs.html#moordyn-f-with-fast-farm-inputs). The general steps for running FAST.Farm with MoorDyn-F are as follows:

- Obtain the FAST.Farm executables
  - Compile from source code
  - Download executable from [GitHub releases](https://github.com/OpenFAST/openfast/releases)
- Configure the FAST.Farm input files for your system
  - `Mod_SharedMooring = 3` in the main FAST.Farm `.fstf` input file
  - Set the `SharedMoorFile` variable in the main FAST.Farm `.fstf` input file to the MoorDyn input file path
- For each turbine in your system, configure the OpenFAST input files for your system
  - If using MoorDyn at the turbine scale as well, set `CompMooring = 3` in the main OpenFAST `.fst` input file for each turbine. Otherwise set `CompMooring = 0`
  - Set the `MooringFile` variable in the main OpenFAST `.fst` input file to the MoorDyn input file path if using the turbine level MoorDyn instance
- Configure the MoorDyn input files
  - At the turbine level, coupled MoorDyn objects (bodies, rods, and points) are rigidly attached to the ElastoDyn object instance
  - At the farm level MoorDyn instance, the object attachment `Turbine#` is rigidly coupled to the ElastoDyn instance of the given turbine. Coupled objects are not allowed in the farm level MoorDyn input file.
- Execute `fast.farm <input_file>.fstf`
