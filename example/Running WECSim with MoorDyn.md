<!-- Copilot generated markdown files from hmtl exports of the instructions word docs -->

# Running WEC-Sim with MoorDyn

Instructions for running WEC-Sim with MoorDyn can be found in the README for the [WEC-Sim/MoorDyn repository](https://github.com/WEC-Sim/MoorDyn/blob/main/README.md). A short summary of those steps is detailed here:

1. Obtain the MoorDyn libraries, header files, and the MoorDyn caller
   - Download from the [WEC-Sim/MoorDyn repository](https://github.com/WEC-Sim/MoorDyn/blob/main/README.md)
   - Compile MoorDyn-C from source following the [instructions in the documentation](https://moordyn.readthedocs.io/en/latest/compiling.html)
     - This is only needed if libraries from the [WEC-Sim/MoorDyn repository](https://github.com/WEC-Sim/MoorDyn/blob/main/README.md) do not work
2. Move the MoorDyn libraries, `.h` header files, and MoorDyn caller (all the files in the [WEC-Sim/MoorDyn repository](https://github.com/WEC-Sim/MoorDyn/blob/main/README.md)) from step 1 to the `WEC-Sim/source/functions/moorDyn` directory.
   - To test the WEC-Sim MoorDyn setup, run the [WEC-Sim_Application/Mooring/MoorDyn example case](https://github.com/WEC-Sim/WEC-Sim_Applications/tree/main/Mooring/MoorDyn).
3. Configure the MoorDyn input file for your system.
   - Note that WEC-Sim requires a rigid 6 DOF body coupling for each mooring connection (a coupled MoorDyn body). Multiple bodies can be coupled to the WEC-Sim system, allowing the simulation of shared moorings for hydrokinetic devices.
4. Configure the WEC-Sim Simulink model
   - For each MoorDyn connection (a connection can consist of multiple lines and nodes but is between two distinct objects such as a floating body and the seafloor), there should be a MoorDyn Connection block in the Simulink model defining the relative motion between the objects.
   - When using MoorDyn, there should always be exactly one MoorDyn Caller block in the Simulink model. See [WEC-Sim MoorDyn docs](https://wec-sim.github.io/WEC-Sim/dev/user/advanced_features.html#moordyn) for more details.
5. Configure the WEC-Sim input file
   - For each MoorDyn block in your system, you need to have a corresponding `mooring(i)` object, where `i` is the ID number of the body in the MoorDyn input file. Instructions for how to set up the mooring object are in the [WEC-Sim MoorDyn docs](https://wec-sim.github.io/WEC-Sim/dev/user/advanced_features.html#moordyn).
   - The MoorDyn input file needs to be defined as `mooring(1).moorDynInputFile`, as WEC-Sim uses the file path defined in the first Mooring block to load the MoorDyn input file.
6. Run the simulation by executing `wecSim` from the command window.
