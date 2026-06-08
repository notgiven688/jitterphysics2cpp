include(FetchContent)

find_package(OpenGL REQUIRED)
find_package(ZLIB REQUIRED)

find_package(glfw3 CONFIG QUIET)
if(TARGET glfw)
    set(JITTER_GLFW_TARGET glfw)
elseif(TARGET glfw3)
    set(JITTER_GLFW_TARGET glfw3)
else()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(GLFW3 QUIET IMPORTED_TARGET glfw3)
    endif()

    if(TARGET PkgConfig::GLFW3)
        set(JITTER_GLFW_TARGET PkgConfig::GLFW3)
    else()
        message(STATUS "GLFW was not found locally. Fetching GLFW 3.4.")
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 3.4)
        FetchContent_MakeAvailable(glfw)
        set(JITTER_GLFW_TARGET glfw)
    endif()
endif()

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9b)
FetchContent_MakeAvailable(imgui)

add_library(JitterDemoImGui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)

target_include_directories(JitterDemoImGui
    PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends)

target_link_libraries(JitterDemoImGui
    PUBLIC
        ${JITTER_GLFW_TARGET}
        OpenGL::GL)

target_compile_features(JitterDemoImGui PUBLIC cxx_std_20)

add_executable(JitterDemo
    src/Program.cpp)

target_link_libraries(JitterDemo
    PRIVATE
        Jitter2
        JitterDemoImGui
        ${JITTER_GLFW_TARGET}
        OpenGL::GL
        ZLIB::ZLIB)

target_compile_features(JitterDemo PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(JitterDemo PRIVATE /W4 /permissive-)
else()
    target_compile_options(JitterDemo PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_custom_command(TARGET JitterDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_LIST_DIR}/assets
        $<TARGET_FILE_DIR:JitterDemo>/assets)
