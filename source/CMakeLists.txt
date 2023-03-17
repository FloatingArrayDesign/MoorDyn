set(MOORDYN_SRCS
    Body.cpp
    Connection.cpp
    IO.cpp
    Line.cpp
    Log.cpp
    Misc.cpp
    MoorDyn2.cpp
    MoorDyn.cpp
    Rod.cpp
    State.cpp
    Time.cpp
    Waves.cpp
    kiss_fft.cpp
    kiss_fftr.cpp
 "Seafloor.cpp")

set(MOORDYN_HEADERS
    Body.hpp
    Connection.hpp
    IO.hpp
    Line.hpp
    Log.hpp
    Misc.hpp
    MoorDyn2.hpp
    QSlines.hpp
    Rod.hpp
    State.hpp
    Time.hpp
    Waves.hpp
    Body.h
    Connection.h
    Line.h
    MoorDyn2.h
    MoorDynAPI.h
    MoorDyn.h
    Rod.h
    Waves.h
)

set(MOORDYN_PUBLIC_DEPS "")
if(USE_VTK)
    list(APPEND MOORDYN_PUBLIC_DEPS VTK::CommonCore
                                    VTK::IOCore
                                    VTK::IOXML
                                    VTK::IOGeometry
                                    VTK::FiltersGeneral)
endif()
if(EXTERNAL_EIGEN)
    list(APPEND MOORDYN_PUBLIC_DEPS Eigen3::Eigen)
endif()

configure_file(Config.h.in ${CMAKE_BINARY_DIR}/Config.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

add_library(moordyn SHARED ${MOORDYN_SRCS})
set_property(TARGET moordyn PROPERTY VERSION ${MOORDYN_VERSION})
set_property(TARGET moordyn PROPERTY SOVERSION ${MOORDYN_MAJOR_VERSION})
set_property(TARGET moordyn PROPERTY
  INTERFACE_moordyn_MAJOR_VERSION ${MOORDYN_MAJOR_VERSION})
set_property(TARGET moordyn APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING moordyn_MAJOR_VERSION
)

target_compile_definitions(moordyn PUBLIC MoorDyn_EXPORTS)
target_link_libraries(moordyn PUBLIC ${MOORDYN_PUBLIC_DEPS})

install(TARGETS moordyn EXPORT MoorDynTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  FILES
    ${MOORDYN_HEADERS}
    ${CMAKE_BINARY_DIR}/Config.h
  DESTINATION
    include/moordyn
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/moordyn/MoorDynConfigVersion.cmake"
  VERSION ${MOORDYN_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT MoorDynTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/moordyn/MoorDynTargets.cmake"
  NAMESPACE MoorDyn::
)
set(ConfigPackageLocation lib/cmake/moordyn)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/MoorDynConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/moordyn/MoorDynConfig.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation})

install(EXPORT MoorDynTargets
  FILE
    MoorDynTargets.cmake
  NAMESPACE
    MoorDyn::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/moordyn/MoorDynConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/moordyn/MoorDynConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
)
