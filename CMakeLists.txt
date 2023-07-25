cmake_minimum_required(VERSION 3.10)
set(MOORDYN_MAJOR_VERSION 2)
set(MOORDYN_MINOR_VERSION 2)
set(MOORDYN_PATCH_VERSION 0)
set(MOORDYN_VERSION ${MOORDYN_MAJOR_VERSION}.${MOORDYN_MINOR_VERSION})
project(Moordyn VERSION ${MOORDYN_VERSION})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_definitions("MOORDYN_MAJOR_VERSION=${MOORDYN_MAJOR_VERSION}")
add_compile_definitions("MOORDYN_MINOR_VERSION=${MOORDYN_MINOR_VERSION}")
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME Runtime)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# The regular folder where the Moordyn headers are located
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source")

# The user options
set(BUILD_DOCS OFF CACHE BOOL
    "Build the documentation for users and developers")
set(BUILD_BENCHMARKS OFF CACHE BOOL
    "Build the benchmarks. Makes use of Google's Benchmark library and requires an internet connection to download.")
set(EXTERNAL_EIGEN ON CACHE BOOL
    "Use the Eigen3 installed in the system")
set(USE_VTK OFF CACHE BOOL
    "Link with VTK to produce output binary files")
set(FORTRAN_WRAPPER OFF CACHE BOOL
    "Build the Fortran module")
set(MATLAB_WRAPPER OFF CACHE BOOL
    "Build the Matlab wrapper")
set(PYTHON_WRAPPER ON CACHE BOOL
    "Build the Python wrapper")
set(RUST_WRAPPER OFF CACHE BOOL
    "Build the Rust wrapper")
set(PYTHON_WRAPPER_USERINSTALL OFF CACHE BOOL
    "Install the Python wrapper in the user space instead of the root one")
set(MOORDYN_SINGLEPRECISSION OFF CACHE BOOL
    "Use single precission (enhances performance at the cost of accuracy)")
set(MOORDYN_SSE "OFF" CACHE STRING "Use enhanced instructions set")
set_property(CACHE MOORDYN_SSE PROPERTY STRINGS OFF SSE SSE2 SSE4 AVX AVX2 AVX512)
set(MOORDYN_FASTMATH OFF CACHE BOOL "Enhances performance at the cost of accuracy")
mark_as_advanced(PYTHON_WRAPPER_USERINSTALL)
mark_as_advanced(MOORDYN_SINGLEPRECISSION)
mark_as_advanced(MOORDYN_SSE)
mark_as_advanced(MOORDYN_FASTMATH)

# Look for the dependencies
if(${USE_VTK})
    find_package(VTK
                 COMPONENTS CommonCore IOCore IOXML IOGeometry FiltersGeneral
                 REQUIRED)
    add_definitions(-DUSE_VTK)
endif()

if(${FORTRAN_WRAPPER})
    enable_language(Fortran)
endif()

if(${MATLAB_WRAPPER})
    find_package(Matlab REQUIRED MEX_COMPILER ENG_LIBRARY MAIN_PROGRAM)
endif()

if(${PYTHON_WRAPPER})
    find_package(Python
                 COMPONENTS Interpreter Development REQUIRED)
    execute_process(COMMAND ${Python_EXECUTABLE} -m build -h
                    OUTPUT_VARIABLE PYTHON_HAS_BUILD_OUTPUT
                    RESULT_VARIABLE PYTHON_HAS_BUILD
                    ERROR_QUIET)
    if(PYTHON_HAS_BUILD AND NOT PYTHON_HAS_BUILD EQUAL 0)
        message(FATAL_ERROR "PyPA’s build package is required. Please install it typing '${Python_EXECUTABLE} -m pip install --upgrade build'")
    endif()
endif()

if(${RUST_WRAPPER})
    find_package(Rust REQUIRED)
endif()

if(${BUILD_DOCS})
    if(NOT ${PYTHON_WRAPPER})
        find_package(Python
                     COMPONENTS Interpreter
                     REQUIRED)
    endif()
    find_package(Doxygen REQUIRED)
    find_package(Sphinx REQUIRED breathe)

    if(NOT Sphinx_FOUND)
        message(FATAL_ERROR "Sphinx not found. Documentation cannot be built.")
    endif()
endif()

if(${EXTERNAL_EIGEN})
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    mark_as_advanced(Eigen3_DIR)
else()
    list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source/Eigen")
endif()

if(${MOORDYN_SINGLEPRECISSION})
    add_definitions(-DMOORDYN_SINGLEPRECISSION)
endif()

if(NOT ${MOORDYN_SSE} STREQUAL "OFF")
    if(MSVC)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:${MOORDYN_SSE}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:${MOORDYN_SSE}")
    else()
        string(TOLOWER ${MOORDYN_SSE} sse_value)
        if(${MOORDYN_SSE} STREQUAL "AVX512")
            set(sse_value "${sse_value}fp16")
        endif()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m${sse_value}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m${sse_value}")
    endif()
endif()

if(${MOORDYN_FASTMATH})
    if(MSVC)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /fp:fast")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
    else()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffast-math")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
    endif()
endif()

# Compile the library
add_subdirectory(source)
# And the eventual wrappers
add_subdirectory(wrappers)

# The documentation
if(${BUILD_DOCS})
    add_subdirectory(docs)
endif()

# Benchmarks
if(${BUILD_BENCHMARKS})
    add_subdirectory(bench)
endif()

# Testing
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME) AND BUILD_TESTING)
    add_subdirectory(extern/Catch2)
    add_subdirectory(tests)
endif()

# Packaging
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_VENDOR "Jose Luis Cercos-Pita")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${MOORDYN_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${MOORDYN_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${MOORDYN_PATCH_VERSION}")
if(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MoorDyn windows installer")
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "MoorDyn-${MOORDYN_MAJOR_VERSION}.${MOORDYN_MINOR_VERSION}")
    set(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}")
    set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\moordyn.readthedocs.io")
    set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\moordyn.readthedocs.io")
    set(CPACK_NSIS_CONTACT "jlc@core-marine.com")
    set(CPACK_NSIS_MODIFY_PATH ON)
elseif(UNIX)
    set(CPACK_GENERATOR "STGZ")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Redistributable package")
endif()

include(CPack)

if(WIN32)
    configure_file(${PROJECT_SOURCE_DIR}/cmake/NSIS.definitions.nsh.in
                   ${CMAKE_CURRENT_BINARY_DIR}/cmake/NSIS.definitions.nsh)
endif()
