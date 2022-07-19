cmake_minimum_required(VERSION 3.10)
set(MOORDYN_MAJOR_VERSION 2)
set(MOORDYN_MINOR_VERSION 0)
set(MOORDYN_VERSION ${MOORDYN_MAJOR_VERSION}.${MOORDYN_MINOR_VERSION})
project(Moordyn VERSION ${MOORDYN_VERSION})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# The regular folder where the Moordyn headers are located
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source")

# The user options
set(EXTERNAL_EIGEN ON CACHE BOOL
    "Use the Eigen3 installed in the system")
set(PYTHON_WRAPPER ON CACHE BOOL
    "Build the Python wrapper")
set(PYTHON_WRAPPER_USERINSTALL OFF CACHE BOOL
    "Install the Python wrapper in the user space instead of the root one")

# Look for the dependencies
if(${EXTERNAL_EIGEN})
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    mark_as_advanced(Eigen3_DIR)
else()
    list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source/Eigen")
endif()

if(${PYTHON_WRAPPER})
    find_package (Python
                  COMPONENTS Interpreter Development NumPy
                  REQUIRED)
endif()

# Compile the library
add_subdirectory(source)
# And the eventual wrappers
add_subdirectory(wrappers)

# Testing
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME) AND BUILD_TESTING)
    add_subdirectory(test)
endif()
