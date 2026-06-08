#if defined(__APPLE__)
#define GLFW_INCLUDE_GLCOREARB
#else
#define GL_GLEXT_PROTOTYPES
#endif

#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <zlib.h>

#include <Jitter2/Jitter2.hpp>

#include <algorithm>
#include <any>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

namespace Shapes = Jitter2::Collision::Shapes;
namespace Constraints = Jitter2::Dynamics::Constraints;
namespace SoftBodies = Jitter2::SoftBodies;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;

constexpr float Pi = 3.14159265358979323846f;

const char* LogLevelName(Jitter2::Logger::LogLevel level)
{
    switch (level)
    {
    case Jitter2::Logger::LogLevel::Information:
        return "Information";
    case Jitter2::Logger::LogLevel::Warning:
        return "Warning";
    case Jitter2::Logger::LogLevel::Error:
        return "Error";
    default:
        return "Unknown";
    }
}

const char* LogLevelColor(Jitter2::Logger::LogLevel level)
{
    switch (level)
    {
    case Jitter2::Logger::LogLevel::Information:
        return "\033[32m";
    case Jitter2::Logger::LogLevel::Warning:
        return "\033[33m";
    case Jitter2::Logger::LogLevel::Error:
        return "\033[31m";
    default:
        return "\033[0m";
    }
}

void InstallJitterLogger()
{
    Jitter2::Logger::Listener =
        [](Jitter2::Logger::LogLevel level, const std::string& message)
        {
            constexpr const char* bold = "\033[1m";
            constexpr const char* reset = "\033[0m";
            std::printf(
                "%s%s[Jitter] %s%s: %s\n",
                LogLevelColor(level),
                bold,
                LogLevelName(level),
                reset,
                message.c_str());
        };
}

void WriteFramebufferPpm(const std::string& path, int width, int height)
{
    if (path.empty() || width <= 0 || height <= 0)
    {
        return;
    }

    std::vector<unsigned char> pixels(static_cast<std::size_t>(width * height * 3));
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    std::ofstream file(path, std::ios::binary);
    if (!file)
    {
        return;
    }

    file << "P6\n" << width << ' ' << height << "\n255\n";
    const int stride = width * 3;
    for (int y = height - 1; y >= 0; --y)
    {
        file.write(
            reinterpret_cast<const char*>(pixels.data() + static_cast<std::size_t>(y * stride)),
            stride);
    }
}


#include "Core/Types.hpp"
#include "Demos/Misc/GearCoupling.hpp"
#include "Demos/Common.hpp"
#include "Renderer/Conversion.hpp"
#include "Renderer/OpenGL/GLResources.hpp"
#include "Renderer/Assets/Mesh.hpp"
#include "Renderer/Drawables.hpp"
#include "Demos/Misc/Octree.hpp"
#include "Demos/Demo21.Types.hpp"
#include "Demos/Demo25.Types.hpp"
#include "Demos/Misc/CcdSolver.hpp"
#include "Demos/Car/ConstraintCar.hpp"
#include "Demos/Misc/Dust.hpp"
#include "Demos/Misc/Player.hpp"
#include "Renderer/DebugRenderer.hpp"
#include "Demos/Car/Wheel.hpp"
#include "Demos/Car/RayCastCar.hpp"
#include "Demos/Demo06.Types.hpp"
#include "Demos/Demo31.Types.hpp"
#include "Renderer/Assets/Primitives.hpp"
#include "Renderer/ShadowCaster.hpp"
#include "Renderer/LitShader.hpp"
#include "Renderer/Skybox.hpp"
#include "Renderer/TextureOverlay.hpp"
#include "Playground.hpp"
#include "Playground.Gui.hpp"


int main(int argc, char** argv)
{
    InstallJitterLogger();

    const LaunchOptions options = ParseOptions(argc, argv);

    try
    {
        glfwSetErrorCallback(GlfwErrorCallback);
        Check(glfwInit() == GLFW_TRUE, "Unable to initialize GLFW.");

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_SAMPLES, 4);
#if defined(__APPLE__)
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

        GLFWwindow* window = glfwCreateWindow(1200, 800, "Jitter2 C++ - Demo", nullptr, nullptr);
        Check(window != nullptr, "Unable to create GLFW window.");

        glfwMakeContextCurrent(window);
        glfwSwapInterval(0);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        ImGui::StyleColorsDark();
        ConfigureGuiTheme();
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init("#version 330");

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        {
            Renderer renderer;
            Skybox skybox;
            ShadowCaster shadowCaster;
            TexturedQuad shadowDebug;
            DebugRenderer debugRenderer;
            DemoScene scene;
            DemoSettings settings;
            if (options.DemoIndex >= 0)
            {
                settings.DemoIndex =
                    std::clamp(options.DemoIndex, 0, static_cast<int>(scene.Demos().size()) - 1);
                scene.SwitchDemo(settings.DemoIndex);
            }
            else
            {
                settings.DemoIndex = scene.CurrentDemo;
            }
            CameraState camera;

            int renderedFrames = 0;
            bool resetKeyWasDown = false;
            bool shootKeyWasDown = false;
            bool multithreadKeyWasDown = false;
            const std::chrono::duration<double> targetFrameTime(1.0 / 100.0);

            while (!glfwWindowShouldClose(window))
            {
                const auto frameStart = std::chrono::steady_clock::now();
                glfwPollEvents();

                if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                {
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
                }

                const bool resetKeyIsDown = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
                if (resetKeyIsDown && !resetKeyWasDown)
                {
                    scene.Reset();
                }
                resetKeyWasDown = resetKeyIsDown;

                const bool shootKeyIsDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
                if (shootKeyIsDown && !shootKeyWasDown)
                {
                    scene.ShootPrimitive(camera.Position, camera.Direction);
                }
                shootKeyWasDown = shootKeyIsDown;

                const bool multithreadKeyIsDown = glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS;
                if (multithreadKeyIsDown && !multithreadKeyWasDown)
                {
                    settings.Multithread = !settings.Multithread;
                }
                multithreadKeyWasDown = multithreadKeyIsDown;

                ImGui_ImplOpenGL3_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();

                UpdateCameraInput(window, camera, io);

                if (!settings.Paused)
                {
                    scene.World.Step(static_cast<Jitter2::Real>(settings.Timestep), settings.Multithread);
                    if (settings.Animate)
                    {
                        scene.Animate(settings.AnimationSpeed);
                    }
                }
                scene.UpdatePicking(window, camera, io);

                int framebufferWidth = 0;
                int framebufferHeight = 0;
                glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);

                if (framebufferWidth > 0 && framebufferHeight > 0)
                {
                    glEnable(GL_DEPTH_TEST);
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);

                    glViewport(0, 0, framebufferWidth, framebufferHeight);
                    glClearColor(73.0f / 255.0f, 76.0f / 255.0f, 92.0f / 255.0f, 1.0f);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    const Vec3 eye = camera.Position;
                    const Mat4 projection = Perspective(
                        camera.FieldOfView,
                        static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight),
                        camera.NearPlane,
                        camera.FarPlane);
                    const Mat4 view = LookAt(eye, eye + camera.Direction, Vec3 {0.0f, 1.0f, 0.0f});
                    const Mat4 viewProjection = Multiply(projection, view);
                    const CameraMatrices cameraMatrices {projection, view, viewProjection};
                    skybox.Draw(cameraMatrices);
                    renderer.QueueWorld(scene.World, scene.FloorShapeForRendering());
                    scene.DrawUpdate(debugRenderer, eye, camera.Direction, window);
                    scene.QueueFrameDrawables();
                    scene.DebugDraw(debugRenderer, settings);

                    shadowCaster.Render(
                        camera,
                        cameraMatrices,
                        framebufferWidth,
                        framebufferHeight,
                        [&renderer, &scene](ShadowCaster& shadow)
                        {
                            renderer.RenderQueuedShadow(shadow);
                            scene.RenderShadowDrawables(shadow);
                        });

                    glViewport(0, 0, framebufferWidth, framebufferHeight);
                    renderer.RenderQueuedLit(cameraMatrices, eye, &shadowCaster, settings.Wireframe);
                    scene.RenderMutableDrawables(renderer, cameraMatrices, eye, &shadowCaster);
                    debugRenderer.Draw(cameraMatrices);

                    if (settings.ShowShadowDebug)
                    {
                        shadowDebug.Position = Vec2 {10.0f, 10.0f};
                        shadowDebug.Size = Vec2 {200.0f, 200.0f};
                        shadowDebug.Draw(shadowCaster.DepthMap(0), framebufferWidth, framebufferHeight);
                    }
                }

                DrawUi(scene, settings);

                ImGui::Render();
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

                if (!options.ScreenshotPath.empty() && renderedFrames == 0)
                {
                    int screenshotWidth = 0;
                    int screenshotHeight = 0;
                    glfwGetFramebufferSize(window, &screenshotWidth, &screenshotHeight);
                    WriteFramebufferPpm(options.ScreenshotPath, screenshotWidth, screenshotHeight);
                }

                glfwSwapBuffers(window);

                ++renderedFrames;
                if (options.FrameLimit > 0 && renderedFrames >= options.FrameLimit)
                {
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
                }

                while (std::chrono::steady_clock::now() - frameStart < targetFrameTime)
                {
                    std::this_thread::yield();
                }
            }
        }

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window);
        glfwTerminate();
    }
    catch (const std::exception& ex)
    {
        std::fprintf(stderr, "%s\n", ex.what());
        glfwTerminate();
        return 1;
    }

    return 0;
}
