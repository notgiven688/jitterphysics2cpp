add_library(Jitter2
    src/Parallelization/ThreadPool.cpp
    src/World.cpp)

target_include_directories(Jitter2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(Jitter2 PUBLIC cxx_std_20)

if(JITTER_DOUBLE_PRECISION)
    target_compile_definitions(Jitter2 PUBLIC JITTER_DOUBLE_PRECISION=1)
endif()

if(JITTER_ENABLE_MULTITHREADING)
    target_compile_definitions(Jitter2 PUBLIC JITTER_ENABLE_MULTITHREADING=1)
else()
    target_compile_definitions(Jitter2 PUBLIC JITTER_ENABLE_MULTITHREADING=0)
endif()

if(MSVC)
    target_compile_options(Jitter2 PRIVATE /W4 /permissive-)
else()
    target_compile_options(Jitter2 PRIVATE -Wall -Wextra -Wpedantic)
endif()
