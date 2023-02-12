set(FORTRAN_SRCS MoorDyn.f90)

add_library(moordynf SHARED ${FORTRAN_SRCS})


set_property(TARGET moordynf PROPERTY VERSION ${MOORDYN_VERSION})
set_property(TARGET moordynf PROPERTY SOVERSION ${MOORDYN_MAJOR_VERSION})
set_property(TARGET moordynf PROPERTY
  INTERFACE_moordynf_MAJOR_VERSION ${MOORDYN_MAJOR_VERSION})
set_property(TARGET moordynf APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING moordynf_MAJOR_VERSION
)
set_target_properties(moordynf PROPERTIES Fortran_MODULE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(moordynf PUBLIC moordyn)

install(TARGETS moordynf EXPORT MoorDynTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if (CMAKE_GENERATOR MATCHES "Visual Studio")
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>/moordyn.mod
  DESTINATION include/moordyn
  COMPONENT Devel
)
else()
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/moordyn.mod
  DESTINATION include/moordyn
)
endif()

