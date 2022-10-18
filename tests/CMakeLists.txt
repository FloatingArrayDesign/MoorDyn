# We need first a custom target to copy all the input files in the compilation
# folder
file(GLOB MooringConfigFiles ${CMAKE_SOURCE_DIR}/tests/Mooring/*.stl
                             ${CMAKE_SOURCE_DIR}/tests/Mooring/*.dat
                             ${CMAKE_SOURCE_DIR}/tests/Mooring/*.txt)
add_custom_target(Test_MooringConfigFiles)
add_custom_command(TARGET Test_MooringConfigFiles PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E
                        copy_directory ${CMAKE_SOURCE_DIR}/tests/Mooring ${CMAKE_CURRENT_BINARY_DIR}/Mooring)
if(WIN32)
    add_custom_command(TARGET Test_MooringConfigFiles POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:moordyn> ${CMAKE_CURRENT_BINARY_DIR}/)
    if(${FORTRAN_WRAPPER})
        add_custom_command(TARGET Test_MooringConfigFiles POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:moordynf> ${CMAKE_CURRENT_BINARY_DIR}/)
    endif()
endif()

set(TESTS "minimal;pendulum;time_schemes;io;bodies_and_rods;wavekin;quasi_static_chain")
if(USE_VTK)
    set(TESTS "${TESTS};vtk")
endif()
foreach(TEST ${TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} moordyn)
    add_dependencies(${TEST} Test_MooringConfigFiles)
    add_test(NAME ${TEST}
             COMMAND ${TEST})
endforeach()

if(${FORTRAN_WRAPPER})
    add_executable(fortran_bindings fortran_bindings.f90)
    target_link_libraries(fortran_bindings moordynf)
    target_include_directories(fortran_bindings PUBLIC $<TARGET_PROPERTY:moordynf,Fortran_MODULE_DIRECTORY>)
    add_dependencies(fortran_bindings Test_MooringConfigFiles)
    add_test(NAME fortran_bindings
             COMMAND fortran_bindings)
endif()

if(${MATLAB_WRAPPER})
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/matlab_bindings.m.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/matlab_bindings.m")

    matlab_add_unit_test(NAME matlab_bindings
                         UNITTEST_FILE ${CMAKE_CURRENT_BINARY_DIR}/matlab_bindings.m
                         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()
