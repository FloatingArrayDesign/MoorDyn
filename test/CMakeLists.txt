# We need first a custom target to copy all the input files in the compilation
# folder
file(GLOB MooringConfigFiles ${CMAKE_SOURCE_DIR}/test/Mooring/*.dat
                             ${CMAKE_SOURCE_DIR}/test/Mooring/*.txt)
add_custom_target(Test_MooringConfigFiles)
add_custom_command(TARGET Test_MooringConfigFiles PRE_BUILD
                    COMMAND ${CMAKE_COMMAND} -E
                        copy_directory ${CMAKE_SOURCE_DIR}/test/Mooring ${CMAKE_CURRENT_BINARY_DIR}/Mooring)

set(TESTS "minimal;bodies_and_rods;wavekin;quasi_static_chain")
foreach(TEST ${TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} moordyn)
    add_dependencies(${TEST} Test_MooringConfigFiles)
    add_test(NAME ${TEST}
             COMMAND ${TEST})
endforeach()
