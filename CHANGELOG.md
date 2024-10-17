## [](https://github.com/core-marine-dev/MoorDyn/compare/v2.3.6...v) (2024-08-23)

### Bug Fixes

* **ci:** Ignore MUSLLinux wheels when testing ([63698bd](https://github.com/core-marine-dev/MoorDyn/commit/63698bd87f4224c8d062f4b3b732b2f52e5dbeef))
* **ci:** List of dependencies to publish a package ([2c9ea79](https://github.com/core-marine-dev/MoorDyn/commit/2c9ea79d89002287bc0e718dcd433b9c891a3b92))
* **ci:** Remove the MUSLLinux wheels before testing ([20a35bf](https://github.com/core-marine-dev/MoorDyn/commit/20a35bf0ad51f4500b84b03e266848f503665720))
* **ci:** Removed unused env variables ([584bda5](https://github.com/core-marine-dev/MoorDyn/commit/584bda5581864d779d467875ee2b1a459cab8b91))
* **ci:** Setting the VTK version ([3be2478](https://github.com/core-marine-dev/MoorDyn/commit/3be24786c21753f8aaba898ee4bdf551ca3196af))
* **ci:** The VTK tar.gz can be directly extracted on MacOS ([9422fb2](https://github.com/core-marine-dev/MoorDyn/commit/9422fb21063254dc3e943f5a8b0399341aed491b))
* **ci:** Update to VTK-9.3.1 ([154ab66](https://github.com/core-marine-dev/MoorDyn/commit/154ab66d5d7442ad15fd31a855577a848f91f5a9))
* **ci:** Use directly the VTK version placeholders ([e528054](https://github.com/core-marine-dev/MoorDyn/commit/e5280542245ae5e7617aafb011c0f0f7a85cf7f9))
* **ci:** Wrong VTK URI ([6ef95f9](https://github.com/core-marine-dev/MoorDyn/commit/6ef95f92ab826f8f4e09661728a54f7d6cb3d870))
* **python-wheels:** Test the wheels ([f534b48](https://github.com/core-marine-dev/MoorDyn/commit/f534b48bd9eb508b00e6ff102e62a7d84addea0a))
* **python:** Avoid using brew on MacOS-13 statically linking VTK ([a521444](https://github.com/core-marine-dev/MoorDyn/commit/a521444281e9117c09135640f49154c2d3d0a18a))
* **python:** Enable MUSLLinux builds ([7f20dab](https://github.com/core-marine-dev/MoorDyn/commit/7f20dab357b8863320d35bee1723f21099900023))
* **python:** Enable pypy builds again ([224f8e4](https://github.com/core-marine-dev/MoorDyn/commit/224f8e4f31271a96b6a4ef170d8337d252305cb7))
* **python:** Set the install name id on MacOS ([32a9fca](https://github.com/core-marine-dev/MoorDyn/commit/32a9fcae8a8aee0558c632fe236f92d9e6bfda8d))
* **python:** Set the MACOSX_DEPLOYMENT_TARGET to 10.15 ([f44e9c4](https://github.com/core-marine-dev/MoorDyn/commit/f44e9c4df1c1c9645d1a08255f16722baa519188))
* **python:** Use the MUSLLinux VTK compilation when needed ([e5fe619](https://github.com/core-marine-dev/MoorDyn/commit/e5fe6190d5fdc67f6bb48c3fc7133a6ed5c5a4ec))
## [](https://github.com/core-marine-dev/MoorDyn/compare/v2.3.5...v) (2024-08-14)

### Bug Fixes

* **python-wheels:** {project} placeholder seems to be ignored/invalid ([0a97f4d](https://github.com/core-marine-dev/MoorDyn/commit/0a97f4dcff2bfa1f83c0b943f0c1695b71e33f69))
* **python-wheels:** Add the DLL path to delvewheel ([9310754](https://github.com/core-marine-dev/MoorDyn/commit/9310754b965ee3602db579e22fd1f242970dff05))
* **python-wheels:** Remove the useless folders after installing ([e6aa341](https://github.com/core-marine-dev/MoorDyn/commit/e6aa341be866183266afacd996accf0afde30837))
* **python-wheels:** Temporary disable pypy3.10, which is not working on the CI ([49dd580](https://github.com/core-marine-dev/MoorDyn/commit/49dd5805db58c21a09a7012cdf6cce685ceeaf72))
* **python-wheels:** Temporary disable pypy3.8, which is not working on the CI ([27166f6](https://github.com/core-marine-dev/MoorDyn/commit/27166f6575586727d2893bc9094eb3fb121aed05))
* **python-wheels:** Temporary disable pypy3.9, which is not working on the CI ([b5b4003](https://github.com/core-marine-dev/MoorDyn/commit/b5b400327b9ec1debeab328c57a314715b0740d9))
* **python-wheels:** Use delvewheel on Windows ([89b5ddc](https://github.com/core-marine-dev/MoorDyn/commit/89b5ddcbc5f64404ff902c8a536e304052ab580a))
* **python-wheels:** Use the installed headers ([ad18025](https://github.com/core-marine-dev/MoorDyn/commit/ad18025c6ff5353f4c99fa7bd58a988c5016c9cd))
* resolves typos and warnings thrown at compiling ([e4a8f08](https://github.com/core-marine-dev/MoorDyn/commit/e4a8f08195e031cbfecfd30574edaad08b15a4d0))
## [](https://github.com/core-marine-dev/MoorDyn/compare/v2.3.3...v) (2024-08-07)

### Features

* **body:** Add centripetal forces for rotating bodies ([f4f816c](https://github.com/core-marine-dev/MoorDyn/commit/f4f816cadb4116f4052f4a3cc36ca2c82f5cabd9))

### Bug Fixes

* accelerations of Coupled/fixed bodies/rods when there are several isntances of them ([1a07a2d](https://github.com/core-marine-dev/MoorDyn/commit/1a07a2d39fe6ffb5f27b6e143830d18ce4f758f2))
* Add a centripetal force to bodies with a excentric COG ([7a56b7c](https://github.com/core-marine-dev/MoorDyn/commit/7a56b7c941c0bfbddb37fab48423f1ad715270a7))
* Centripetal force for parallel axes shall be null ([829c837](https://github.com/core-marine-dev/MoorDyn/commit/829c83711a98ed11178cc3b934b76ce515c61f82))
* Drop the patch to move from extrinsic to intrinsic Euler angles ([6ff56ac](https://github.com/core-marine-dev/MoorDyn/commit/6ff56acc47d756634b1a8238898221ec016c8e0f))
* EulerXYZ intrinsic angles instead of extrinsic ([1eec2e3](https://github.com/core-marine-dev/MoorDyn/commit/1eec2e3005239c9bd8e19f1ebddd05ead8f2f08f))
* EulerZYX -> EulerXYZ on moordyn::Euler2Quat() ([5a5f7fd](https://github.com/core-marine-dev/MoorDyn/commit/5a5f7fdfa8bc161cd580c50b3dcedcca394919c6))
* Freeze when writeLog is not the first option ([d4cce8e](https://github.com/core-marine-dev/MoorDyn/commit/d4cce8ecfb8e5af4819f3877e0f882d59ebb04c8))
* make rod submergence calcs match what is in MDF (verified code) ([0ac0e92](https://github.com/core-marine-dev/MoorDyn/commit/0ac0e9207eec4c202bf6b85f4a6fbc5178a1fb13))
* Odd treatment was meant for indexes from 1 to 3, not 0 to 2, and the matrix indexes were transposed ([0bb4ae2](https://github.com/core-marine-dev/MoorDyn/commit/0bb4ae27c8ed307a7a2382f0a53121ef8d73ebb7))
* Read first the writelog option, and then anything else ([028a567](https://github.com/core-marine-dev/MoorDyn/commit/028a56742226aa27fa900272bf020924eec56b6d))
* Rebranding to include centripetal forces on getNetForceAndMass ([f20a98a](https://github.com/core-marine-dev/MoorDyn/commit/f20a98aa23a584a0475f6055a33acfda30f39718))
* The quaternions shall be renormalized to get the rotation matrix ([7256746](https://github.com/core-marine-dev/MoorDyn/commit/7256746793f0c48b6ec4bc8dd97407ad582a9ae2))
