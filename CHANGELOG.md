## [2.6.0] - 2026-02-17

### 🚀 Features

- *(core)* Added a function to optionally disable the CMD windows on MoorDyn v1
- *(ci)* Upgrade all (ubuntu, windows and macos) to their latest images
- *(ci)* Enable a branch to test wheels
- *(core)* [**breaking**] Replaced VTK by lean-vtk

### 🐛 Bug Fixes

- *(ci)* Disable Pypy 3.11 (because of MacOS)
- *(ci)* Remove MacOS-arm64 and downgrade Windows to 2022
- *(core)* Fix the minimum input version, so it does not enforce to be the last one
- *(core)* Explicit names for the elastic models
- *(examples)* Removed the tbd field from the Bodies & rods example
- *(ci)* MacOS-13 support dropped from GitHUB actions
- *(ci)* Add the headers path to the setup.py on the wheels
- *(core)* Recovered the VTP support

### 💼 Other

- *(deps)* Bump suisei-cn/actions-download-file from 1.6.0 to 1.6.1
- *(deps)* Bump actions/download-artifact from 4 to 6
- *(deps)* Bump pypa/cibuildwheel from 2.23.1 to 3.2.1
- *(deps)* Bump actions/setup-python from 5 to 6
- *(deps)* Bump actions/checkout from 2 to 5
- *(deps)* Bump pypa/cibuildwheel from 3.2.1 to 3.3.0
- *(deps)* Bump actions/checkout from 5 to 6
- *(deps)* Bump actions/upload-artifact from 4 to 5
- *(deps)* Bump actions/download-artifact from 6 to 7
- *(deps)* Bump actions/upload-artifact from 5 to 6
- *(deps)* Bump mukunku/tag-exists-action from 1.6.0 to 1.7.0
- *(deps)* Bump pypa/cibuildwheel from 3.3.0 to 3.3.1

### 📚 Documentation

- Updates for MD-F SeaState coupling
## [2.4.0] - 2025-03-28

### 🚀 Features

- *(rust)* First test using the Rust wrapper
- *(core)* New flexible states
- *(core)* Create a base class for the instances which provides a unique identifier
- *(core)* [**breaking**] Less flexible but fast extensible state variables
- VIV and viscoelastic model compatability with the flexstate method. Verfied via comparison to MD-F and Thorsen results
- *(core)* New flexible states
- *(core)* Create a base class for the instances which provides a unique identifier
- *(core)* [**breaking**] Less flexible but fast extensible state variables
- Viscoelastic test now regression test of plots in PR
- DisableOutTime added to all test input files. Tests all passing on MacOS

### 🐛 Bug Fixes

- Added dummy AllOutput to old API
- Old AllOutput back to voids, cleans it up a bit
- Rename AllOutput -> WriteOutputs (To match MD-F)
- Mismatch between file and headed
- Change cout to MoorDyn_log for AllOutput wrapper
- Cleaning up after merge
- *(ci)* Run the Rust tests on Linux
- Comment out include random
- Update docs with minor clarifications, move VIV model to internal nodes only
- *(core)* CLang buildings
- *(core)* Removed unused operator
- *(core)* VSCode warnings
- *(core)* Export some more symbols for state tests
- *(core)* Export some more symbols for state tests
- *(core)* VS Code invalid exports
- *(core)* VS Code invalid exports
- *(core)* Add subclasses to the DLL interface
- *(ci)* Drop Ubuntu from MATLAB tests (for the time being)
- Improve input file reading error messages
- *(core)* Use non-static methods that are CLang templates compliant
- *(core)* Initialize the state variables as zeroes by default
- *(core)* StateVarRef not used anymore
- *(core)* Give a common Instance::initialize() method
- Standard initalize structure, standardizing setState calls, and some docs improvements
- Fixed failing polyester test. Added comments for documentation
- Added warning message to loading states from MoorPy
- *(build)* Upgraded the FindRust.cmake (from https://github.com/corrosion-rs/corrosion)
- *(core)* CLang buildings
- *(core)* Removed unused operator
- *(core)* VSCode warnings
- *(core)* Export some more symbols for state tests
- *(core)* Export some more symbols for state tests
- *(core)* VS Code invalid exports
- *(core)* VS Code invalid exports
- *(core)* Add subclasses to the DLL interface
- *(ci)* Drop Ubuntu from MATLAB tests (for the time being)
- *(core)* Use non-static methods that are CLang templates compliant
- *(core)* Initialize the state variables as zeroes by default
- *(core)* StateVarRef not used anymore
- *(core)* Give a common Instance::initialize() method
- Fixed viscoelastic bug by initalizing sub-segment stretch as non-zero, added viscoelastic test based on polyester test
- Hopefully fixes windows and ubuntu failing builds
- Bug fixes in tests
- Added setDependentStates call to Body::initializeUnfreeBody
- Added more debug messages to polyester test for windows compatibility
- Indentation fixes
- *(examples)* Moved VIV to catch2

### 💼 Other

- *(deps)* Bump pypa/cibuildwheel from 2.22.0 to 2.23.0
- *(deps)* Bump pypa/cibuildwheel from 2.23.0 to 2.23.1

### 📚 Documentation

- Updated docs for OpenFAST/openfast#2597
## [2.3.8] - 2024-12-12

### 🚀 Features

- Added the possibility to load initial condition states from other tools
- *(ci)* Compile and test MATLAB wrapper (Linux and MacOS)
- Adds disableOutTime to turn off timestep printing to the console. Useful for the MATLAB wrapper

### 🐛 Bug Fixes

- *(python-wheels)* Test the wheels
- *(ci)* Update to VTK-9.3.1
- *(python)* Enable MUSLLinux builds
- *(python)* Enable pypy builds again
- *(ci)* List of dependencies to publish a package
- *(ci)* Setting the VTK version
- *(ci)* Removed unused env variables
- *(ci)* Use directly the VTK version placeholders
- *(python)* Avoid using brew on MacOS-13 statically linking VTK
- *(python)* Set the install name id on MacOS
- *(ci)* The VTK tar.gz can be directly extracted on MacOS
- *(ci)* Ignore MUSLLinux wheels when testing
- *(ci)* Wrong VTK URI
- *(python)* Set the MACOSX_DEPLOYMENT_TARGET to 10.15
- *(ci)* Remove the MUSLLinux wheels before testing
- *(python)* Use the MUSLLinux VTK compilation when needed
- *(python)* Autonomous MoorPy IC exporter
- *(docs and tests)* Initialization notes with a practical application
- *(python)* Absolute paths are not accepted any longer
- Some small fixes for viscoelastic stuff
- Removing files
- Removing one more file
- Cleaned up some time scheme stuff (added in more spots where Misc states should be accounted for)
- Some more stiffness fixes. Noteably before when reading a non-linear look up table it assumed stress strain. This is not correct and doesnt match theory paper or docs. CHanges to stress tension.
- Dynamic current inflile reading fix
- Some more notes explaining the synchronization model
- Major clean up to the synchronization model
- *(python)* Added the missing time scheme manipulation API entries
- *(python)* Missing GetXXXNumberNodes missing entries
- *(fortran)* Added the time scheme manipulation entries
- *(fortran)* Added the serialization/deserialization
- *(matlab)* Added the time scheme manipulation entries
- *(matlab)* Serialization/deserialization entries
- *(python)* The sea bed force was actually returning the Froude-Krilov one
- *(core)* Water depth warning was printed no matter whether it was corrected or not
- Fixed path errors in waves files.
- *(python)* The time is not required on moordyn.GetWavesKin() anymore
- Removed case sensitivity in the options list keywords
- *(tests)* Ignore the force on the anchor point because it is not computed

### 💼 Other

- *(deps)* Bump pypa/cibuildwheel from 2.20.0 to 2.21.0
- *(deps)* Bump pypa/cibuildwheel from 2.21.0 to 2.21.1
- *(deps)* Bump pypa/cibuildwheel from 2.21.1 to 2.21.2
- *(deps)* Bump pypa/cibuildwheel from 2.21.2 to 2.21.3
- *(deps)* Bump pypa/cibuildwheel from 2.21.3 to 2.22.0
- Conn -> point

### 📚 Documentation

- Updates and typo fixes
- References file rename
- Fixing broken links
- Minor change to update viscoelastic docs
- Clarifying dtout
- Add disable output to the docs and options list

### 🧪 Testing

- Remove initialization for a more fair comparison
- Optimized some time steps
## [2.3.6] - 2024-08-14

### 🐛 Bug Fixes

- Resolves typos and warnings thrown at compiling
- *(python-wheels)* Use delvewheel on Windows
- *(python-wheels)* Add the DLL path to delvewheel
- *(python-wheels)* {project} placeholder seems to be ignored/invalid
- *(python-wheels)* Use the installed headers
- *(python-wheels)* Remove the useless folders after installing
- *(python-wheels)* Temporary disable pypy3.8, which is not working on the CI
- *(python-wheels)* Temporary disable pypy3.9, which is not working on the CI
- *(python-wheels)* Temporary disable pypy3.10, which is not working on the CI

### 📚 Documentation

- Started the documentation for Linux distros with precompiled pacakges
## [2.3.5] - 2024-08-07

### ⚙️ Miscellaneous Tasks

- Overhauled Python wheels crafting
## [2.3.4] - 2024-08-07

### 🚀 Features

- *(body)* Add centripetal forces for rotating bodies

### 🐛 Bug Fixes

- Read first the writelog option, and then anything else
- The quaternions shall be renormalized to get the rotation matrix
- Freeze when writeLog is not the first option
- Centripetal force for parallel axes shall be null
- Rebranding to include centripetal forces on getNetForceAndMass
- Add a centripetal force to bodies with a excentric COG
- EulerZYX -> EulerXYZ on moordyn::Euler2Quat()
- Accelerations of Coupled/fixed bodies/rods when there are several isntances of them
- Odd treatment was meant for indexes from 1 to 3, not 0 to 2, and the matrix indexes were transposed
- EulerXYZ intrinsic angles instead of extrinsic
- Drop the patch to move from extrinsic to intrinsic Euler angles
- Make rod submergence calcs match what is in MDF (verified code)

### 💼 Other

- MinGW needs the DECLDIR on Body::setState

### 📚 Documentation

- Clarify that intrinsic angles are considered, and link to the external resources
- Autogenerated changelog (https://github.com/conventional-changelog/conventional-changelog)

### 🧪 Testing

- Excentrical body test
- Test the centripetal force on a simple case
- Strip the VTK from the test and attach two points to the body to get a more stable orbit
- Test the rotations
- Typo on the config file description

### ⚙️ Miscellaneous Tasks

- Disable memcheck on lowe_and_langley, which is way too slow
## [2.3.3] - 2024-06-24

### 🚀 Features

- *(body)* Add centripetal forces for rotating bodies
- Cleaned up the hack-ish state to generalize it for visco stuff. Made # a comment character in input files. Readded diable VIV during ICgen
- Viscoelastic model with constant and load dependent dynamic stiffness is now live!

### 🐛 Bug Fixes

- Read first the writelog option, and then anything else
- The quaternions shall be renormalized to get the rotation matrix
- Freeze when writeLog is not the first option
- Centripetal force for parallel axes shall be null
- Rebranding to include centripetal forces on getNetForceAndMass
- Add a centripetal force to bodies with a excentric COG
- EulerZYX -> EulerXYZ on moordyn::Euler2Quat()
- Accelerations of Coupled/fixed bodies/rods when there are several isntances of them
- Odd treatment was meant for indexes from 1 to 3, not 0 to 2, and the matrix indexes were transposed
- EulerXYZ intrinsic angles instead of extrinsic
- Drop the patch to move from extrinsic to intrinsic Euler angles
- Make rod submergence calcs match what is in MDF (verified code)
- Make rod submergence calcs match what is in MDF (verified code)

### 💼 Other

- MinGW needs the DECLDIR on Body::setState

### 📚 Documentation

- Clarify that intrinsic angles are considered, and link to the external resources
- Added # comment character instructions to docs

### 🧪 Testing

- Excentrical body test
- Test the centripetal force on a simple case
- Strip the VTK from the test and attach two points to the body to get a more stable orbit
- Test the rotations
- Typo on the config file description
## [2.3.2] - 2024-06-24

### 💼 Other

- Adding pressure bending forces to lines
## [1.01.02] - 2021-02-15
