cmake_minimum_required(VERSION 3.24)

project(Jitter2Cpp
    VERSION 0.1.0
    DESCRIPTION "C++ port of Jitter Physics 2"
    LANGUAGES CXX)

option(JITTER_DOUBLE_PRECISION "Build Jitter2 with double precision Real values" OFF)
option(JITTER_BUILD_TESTS "Build C++ parity tests" ON)
option(JITTER_BUILD_DEMO "Build the C++ JitterDemo application" ON)
option(JITTER_BUILD_BENCHMARKS "Build C++ benchmarks" ON)
option(JITTER_ENABLE_MULTITHREADING "Enable multithreaded engine code paths" ON)
option(JITTER_NATIVE_OPTIMIZATIONS "Enable native CPU tuning flags for local performance comparisons" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(JITTER_NATIVE_OPTIMIZATIONS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-march=native>)
    elseif(MSVC)
        message(WARNING "JITTER_NATIVE_OPTIMIZATIONS is not mapped for MSVC; configure /arch manually if needed.")
    endif()
endif()

add_subdirectory(Jitter2)

if(JITTER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(JitterTests)
endif()

if(JITTER_BUILD_DEMO)
    add_subdirectory(JitterDemo)
endif()

if(JITTER_BUILD_BENCHMARKS)
    add_subdirectory(Benchmarks/Cpp)
endif()
