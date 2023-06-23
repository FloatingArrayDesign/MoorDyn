# We need first a custom target to copy all the input files in the compilation # folder
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

set(TESTS 
    minimal
    lifting
    pendulum
    beam
    time_schemes
    io
    rods
    bodies_and_rods
    bodies
    quasi_static_chain
    wavekin_4
    tripod
    seafloor
)

set(CPP_TESTS 
    wavekin
    wavekin_7
)

set(CATCH2_TESTS 
    math_tests
    testDecomposeString
)

function(make_executable test_name)
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE moordyn)
    add_dependencies(${test_name} Test_MooringConfigFiles)
endfunction()

function(make_cpp_executable test_name)
    make_executable(${test_name})
    target_include_directories(${test_name} PUBLIC ${CMAKE_BINARY_DIR}/)
endfunction()

function(make_catch2_executable test_name)
    make_cpp_executable(${test_name})
    target_include_directories(${test_name} PUBLIC ${CMAKE_BINARY_DIR}/)
    target_link_libraries(${test_name} PRIVATE Catch2::Catch2WithMain)
endfunction()

if(USE_VTK)
    set(TESTS "${TESTS};vtk")

    make_cpp_executable(wave_surface)
endif()

foreach(TEST ${TESTS})
    make_executable(${TEST})
    add_test(NAME ${TEST}
             COMMAND ${TEST})
endforeach()

foreach(TEST ${CPP_TESTS})
    make_cpp_executable(${TEST})
    add_test(NAME ${TEST}
             COMMAND ${TEST})
endforeach()

foreach(TEST ${CATCH2_TESTS})
    make_catch2_executable(${TEST})
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
