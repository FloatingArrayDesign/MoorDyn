# We need first a custom target to copy all the input files in the compilation # folder
file(GLOB MooringConfigFiles ${CMAKE_SOURCE_DIR}/tests/Mooring/*.stl
                             ${CMAKE_SOURCE_DIR}/tests/Mooring/*.dat
                             ${CMAKE_SOURCE_DIR}/tests/Mooring/*.txt)
add_custom_target(Test_MooringConfigFiles)
add_custom_command(TARGET Test_MooringConfigFiles PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E
                        copy_directory ${CMAKE_SOURCE_DIR}/tests/Mooring ${CMAKE_CURRENT_BINARY_DIR}/Mooring
)
if(WIN32)
    add_custom_command(TARGET Test_MooringConfigFiles POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:moordyn> ${CMAKE_CURRENT_BINARY_DIR}/
    )
    if(${FORTRAN_WRAPPER})
        add_custom_command(TARGET Test_MooringConfigFiles POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:moordynf> ${CMAKE_CURRENT_BINARY_DIR}/
        )
    endif()
endif()

set(C_TESTS 
    c_api
)

set(CPP_TESTS 
    minimal
    lifting
    pendulum
    time_schemes
    io
    rods
    bodies_and_rods
    wavekin_4
    tripod
    seafloor
    vtk
    wavekin
    wavekin_7
    # wave_surface
)

set(CATCH2_TESTS 
    math_tests
    state_tests
    testDecomposeString
    bodies
    beam
    conveying_fluid
    polyester
    viv
    quasi_static_chain
    lowe_and_langley_2006
    local_euler
    beuler
    midpoint
    aca
    wilson
)

function(make_executable test_name, extension)
    set(TEST_PRIVATE_DEPS "moordyn;")
    add_executable(${test_name} ${test_name}.${extension})
    target_link_libraries(${test_name} PRIVATE ${TEST_PRIVATE_DEPS})
    add_dependencies(${test_name} Test_MooringConfigFiles)
endfunction()

function(make_c_executable test_name)
    make_executable(${test_name}, "c")
endfunction()

function(make_cpp_executable test_name)
    make_executable(${test_name}, "cpp")
    target_include_directories(${test_name} PUBLIC ${CMAKE_BINARY_DIR}/)
endfunction()

function(make_catch2_executable test_name)
    make_cpp_executable(${test_name})
    target_link_libraries(${test_name} PRIVATE Catch2::Catch2WithMain)
endfunction()

foreach(TEST ${C_TESTS})
    make_c_executable(${TEST})
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

if(${PYTHON_WRAPPER})
    add_test (NAME python-tests
        COMMAND ${Python_EXECUTABLE} -m pytest -s
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    set_tests_properties(python-tests
        PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/wrappers/python/:$ENV{PYTHONPATH}")
    set_tests_properties(python-tests
        PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/source/:$ENV{LD_LIBRARY_PATH}")
endif()

if(${MATLAB_WRAPPER})
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/matlab_bindings.m.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/matlab_bindings.m")

    matlab_add_unit_test(NAME matlab_bindings
                         UNITTEST_FILE ${CMAKE_CURRENT_BINARY_DIR}/matlab_bindings.m
                         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

if(${RUST_WRAPPER})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
                   ${CMAKE_CURRENT_BINARY_DIR}/Cargo.toml
                   COPYONLY)
    set(RUST_TESTS 
        rust_minimal
    )
    foreach(TEST ${RUST_TESTS})
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${TEST}.rs
                       ${CMAKE_CURRENT_BINARY_DIR}/${TEST}.rs
                       COPYONLY)
    endforeach()

    add_custom_target(
        rust_make_tests ALL
        ${CMAKE_COMMAND} -E env RUSTFLAGS="-L${CMAKE_BINARY_DIR}/source/" cargo build --release
    )
    add_dependencies(rust_make_tests rust_wrapper)

    foreach(TEST ${RUST_TESTS})
        add_test(NAME ${TEST}
                 COMMAND ${CMAKE_CURRENT_BINARY_DIR}/target/release/${TEST})
        set_tests_properties(${TEST}
            PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/source/:$ENV{LD_LIBRARY_PATH}")
    endforeach()
endif()
