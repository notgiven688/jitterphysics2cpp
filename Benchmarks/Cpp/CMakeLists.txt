add_executable(JitterBenchmark
    src/ColosseumBenchmark.cpp)

target_link_libraries(JitterBenchmark
    PRIVATE
        Jitter2)

target_compile_features(JitterBenchmark PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(JitterBenchmark PRIVATE /W4 /permissive-)
else()
    target_compile_options(JitterBenchmark PRIVATE -Wall -Wextra -Wpedantic)
endif()
