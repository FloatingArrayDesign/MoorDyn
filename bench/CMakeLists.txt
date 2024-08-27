
include(../cmake/GoogleBenchmark.cmake)


add_custom_target(Bench_MooringConfigFiles)

add_custom_command(TARGET Bench_MooringConfigFiles PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E
                        copy_directory ${CMAKE_SOURCE_DIR}/bench/Mooring ${CMAKE_CURRENT_BINARY_DIR}/Mooring)
if(WIN32)
    add_custom_command(TARGET Bench_MooringConfigFiles POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:moordyn> ${CMAKE_CURRENT_BINARY_DIR}/)
endif()

function (add_bench bench_name src)
    add_executable(${bench_name} ${src})
    add_dependencies(${bench_name} Bench_MooringConfigFiles)
    target_link_libraries(${bench_name} benchmark::benchmark)
    target_link_libraries(${bench_name} moordyn)
    target_include_directories(${bench_name} PUBLIC ${CMAKE_BINARY_DIR}/)
endfunction()

add_bench(WaveBenchmark WaveBench.cpp)
add_bench(MoordynBenchmark MDBench.cpp)
add_bench(LinesBenchmark LinesBench.cpp)

add_custom_target(run_benchmarks
    COMMAND $<TARGET_FILE:WaveBenchmark>
    COMMAND $<TARGET_FILE:MoordynBenchmark>
)