find_package(Catch2 3 CONFIG QUIET)

add_executable(JitterTests
    src/FallbackMain.cpp
    src/BroadPhaseTests.cpp
    src/DebugDrawTests.cpp
    src/DiagnosticTests.cpp
    src/MathTests.cpp
    src/NarrowPhaseTests.cpp
    src/ParallelizationTests.cpp
    src/ShapeTests.cpp
    src/SoftBodyTests.cpp
    src/StorageTests.cpp
    src/RigidBodyTests.cpp)

target_link_libraries(JitterTests PRIVATE Jitter2)
target_compile_features(JitterTests PRIVATE cxx_std_20)

if(Catch2_FOUND)
    target_link_libraries(JitterTests PRIVATE Catch2::Catch2WithMain)
    target_compile_definitions(JitterTests PRIVATE JITTER_USE_CATCH2=1)
    include(Catch)
    catch_discover_tests(JitterTests)
else()
    message(WARNING "Catch2 was not found. Building JitterTests with the temporary fallback runner.")
    target_compile_definitions(JitterTests PRIVATE JITTER_USE_CATCH2=0)
    add_test(NAME JitterTests COMMAND JitterTests)
endif()
