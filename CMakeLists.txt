cmake_minimum_required(VERSION 3.10)
set(MOORDYN_MAJOR_VERSION 2)
set(MOORDYN_MINOR_VERSION 0)
set(MOORDYN_VERSION ${MOORDYN_MAJOR_VERSION}.${MOORDYN_MINOR_VERSION})
project(Moordyn VERSION ${MOORDYN_VERSION})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# The regular folder where the Moordyn headers are located
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source")

set(EXTERNAL_EIGEN ON CACHE BOOL
    "Use the Eigen3 installed in the system")

if(${EXTERNAL_EIGEN})
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
else()
    list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/Source/Eigen")
endif()

# Compile the library
add_subdirectory(source)

# Testing
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME) AND BUILD_TESTING)
    add_subdirectory(test)
endif()
