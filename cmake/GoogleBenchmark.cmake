set(BENCHMARK_ENABLE_TESTING NO)

include(FetchContent)

FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG origin/main
)

FetchContent_MakeAvailable(googlebenchmark)
