#pragma once

struct Vec3;
class DebugRenderer;
class DemoScene;

/// <summary>
/// Defines optional cleanup behavior for demo scenes that own external state.
/// </summary>
class ICleanDemo
{
public:
    virtual ~ICleanDemo() = default;
    /// <summary>
    /// Cleans up resources or callbacks created by the demo.
    /// </summary>
    virtual void CleanUp(DemoScene& pg) = 0;
};

/// <summary>
/// Defines optional per-frame draw/update behavior for demo scenes.
/// </summary>
class IDrawUpdate
{
public:
    virtual ~IDrawUpdate() = default;
    /// <summary>
    /// Updates demo-specific draw state before the frame is rendered.
    /// </summary>
    virtual void DrawUpdate(
        DemoScene& pg,
        DebugRenderer& debugRenderer,
        Vec3 cameraPosition,
        Vec3 cameraDirection,
        ::GLFWwindow* window) = 0;
};

/// <summary>
/// Defines a demo scene that can build physics content in a <see cref="Jitter2::World"/>.
/// </summary>
class IDemo
{
public:
    virtual ~IDemo() = default;
    /// <summary>
    /// Builds the demo scene.
    /// </summary>
    /// <param name="pg">The playground/demo scene helper.</param>
    /// <param name="world">The world into which the demo objects are created.</param>
    virtual void Build(DemoScene& pg, ::Jitter2::World& world) = 0;
    /// <summary>
    /// Gets the display name of the demo.
    /// </summary>
    [[nodiscard]] virtual const char* Name() const = 0;
    /// <summary>
    /// Gets a longer description shown in the demo menu tooltip.
    /// </summary>
    [[nodiscard]] virtual const char* Description() const { return ""; }
    /// <summary>
    /// Gets demo-specific controls shown in the demo menu tooltip.
    /// </summary>
    [[nodiscard]] virtual const char* Controls() const { return ""; }
};
