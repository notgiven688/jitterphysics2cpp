#include <Jitter2/Jitter2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

namespace Shapes = Jitter2::Collision::Shapes;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;

constexpr float Pi = 3.14159265358979323846f;

enum class Scene
{
    Colosseum,
    RotatingCube
};

struct Options
{
    Scene BenchmarkScene = Scene::Colosseum;
    int WarmupFrames = 120;
    int Frames = 1000;
    int Threads = -1;
    bool Multithread = true;
    Jitter2::Real Timestep = static_cast<Jitter2::Real>(1.0 / 100.0);
};

const char* SceneName(Scene scene)
{
    switch (scene)
    {
    case Scene::Colosseum:
        return "colosseum";
    case Scene::RotatingCube:
        return "rotating-cube";
    }

    return "unknown";
}

class ShapeStore
{
public:
    template<typename TShape, typename... TArgs>
    TShape& Create(TArgs&&... args)
    {
        auto shape = std::make_unique<TShape>(std::forward<TArgs>(args)...);
        TShape& reference = *shape;
        Shapes.push_back(std::move(shape));
        return reference;
    }

    std::vector<std::unique_ptr<Shapes::RigidBodyShape>> Shapes;
};

std::size_t CountAttachedShapes(const Jitter2::World& world)
{
    std::size_t count = 0;
    for (const Jitter2::RigidBody* body : world.RigidBodies())
    {
        if (body != nullptr)
        {
            count += body->Shapes().size();
        }
    }
    return count;
}

void AddFloor(Jitter2::World& world, ShapeStore& store)
{
    Jitter2::RigidBody& body = world.CreateRigidBody();
    Shapes::BoxShape& floorShape = store.Create<Shapes::BoxShape>(JVector(200, 200, 200));
    body.Position(JVector(0, -100, 0));
    body.MotionTypeValue(Jitter2::MotionType::Static);
    body.AddShape(floorShape);
}

void CreateRingWall(
    Jitter2::World& world,
    ShapeStore& store,
    JVector position,
    JVector size,
    int height,
    float radius)
{
    const float circumference = Pi * 2.0f * radius;
    const int boxCountPerRing = static_cast<int>(0.9f * circumference / static_cast<float>(size.Z));
    const float increment = (2.0f * Pi) / static_cast<float>(boxCountPerRing);

    for (int ringIndex = 0; ringIndex < height; ++ringIndex)
    {
        for (int i = 0; i < boxCountPerRing; ++i)
        {
            Jitter2::RigidBody& body = world.CreateRigidBody();
            body.AddShape(store.Create<Shapes::BoxShape>(size));

            const float angle = ((ringIndex & 1) == 0
                ? static_cast<float>(i) + 0.5f
                : static_cast<float>(i)) * increment;
            body.Position(position + JVector(
                static_cast<Jitter2::Real>(-std::cos(angle) * radius),
                (static_cast<Jitter2::Real>(ringIndex) + static_cast<Jitter2::Real>(0.5)) * size.Y,
                static_cast<Jitter2::Real>(std::sin(angle) * radius)));
            body.Orientation(JQuaternion::CreateFromAxisAngle(
                JVector::UnitY(),
                static_cast<Jitter2::Real>(angle)));
        }
    }
}

void CreateRingPlatform(
    Jitter2::World& world,
    ShapeStore& store,
    JVector position,
    JVector size,
    float radius)
{
    const float innerCircumference = Pi * 2.0f * (radius - 0.5f * static_cast<float>(size.Z));
    const int boxCount = static_cast<int>(0.95f * innerCircumference / static_cast<float>(size.Y));
    const float increment = (2.0f * Pi) / static_cast<float>(boxCount);

    for (int i = 0; i < boxCount; ++i)
    {
        const float angle = static_cast<float>(i) * increment;

        Jitter2::RigidBody& body = world.CreateRigidBody();
        body.AddShape(store.Create<Shapes::BoxShape>(size));

        body.Position(position + JVector(
            static_cast<Jitter2::Real>(-std::cos(angle) * radius),
            static_cast<Jitter2::Real>(0.5) * size.X,
            static_cast<Jitter2::Real>(std::sin(angle) * radius)));
        body.Orientation(JQuaternion::CreateFromAxisAngle(
            JVector::UnitY(),
            static_cast<Jitter2::Real>(angle + Pi * 0.5f))
            * JQuaternion::CreateFromAxisAngle(
                JVector::UnitZ(),
                static_cast<Jitter2::Real>(Pi * 0.5f)));
    }
}

JVector CreateRing(
    Jitter2::World& world,
    ShapeStore& store,
    JVector position,
    JVector size,
    float radius,
    int heightPerPlatformLevel,
    int platformLevels)
{
    for (int platformIndex = 0; platformIndex < platformLevels; ++platformIndex)
    {
        const Jitter2::Real wallOffset =
            static_cast<Jitter2::Real>(0.5) * size.Z - static_cast<Jitter2::Real>(0.5) * size.X;
        CreateRingWall(world, store, position, size, heightPerPlatformLevel, radius + static_cast<float>(wallOffset));
        CreateRingWall(world, store, position, size, heightPerPlatformLevel, radius - static_cast<float>(wallOffset));

        CreateRingPlatform(
            world,
            store,
            position + JVector(0, static_cast<Jitter2::Real>(heightPerPlatformLevel) * size.Y, 0),
            size,
            radius);
        position.Y += static_cast<Jitter2::Real>(heightPerPlatformLevel) * size.Y + size.X;
    }

    return position;
}

void BuildColosseum(Jitter2::World& world, ShapeStore& store)
{
    AddFloor(world, store);
    world.AllowDeactivation = false;

    const JVector size(
        static_cast<Jitter2::Real>(0.5),
        static_cast<Jitter2::Real>(1),
        static_cast<Jitter2::Real>(3));
    JVector layerPosition;
    constexpr int layerCount = 6;
    constexpr float innerRadius = 15.0f;
    constexpr int heightPerPlatform = 3;
    constexpr int platformsPerLayer = 1;
    constexpr float ringSpacing = 0.5f;

    for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
    {
        const int ringCount = layerCount - layerIndex;
        for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
        {
            CreateRing(
                world,
                store,
                layerPosition,
                size,
                innerRadius
                    + static_cast<float>(ringIndex) * (static_cast<float>(size.Z) + ringSpacing)
                    + static_cast<float>(layerIndex) * (static_cast<float>(size.Z) - static_cast<float>(size.X)),
                heightPerPlatform,
                platformsPerLayer);
        }

        layerPosition.Y += static_cast<Jitter2::Real>(platformsPerLayer)
            * (size.Y * static_cast<Jitter2::Real>(heightPerPlatform) + size.X);
    }
}

Jitter2::RigidBody* BuildRotatingCube(Jitter2::World& world, ShapeStore& store)
{
    Jitter2::RigidBody& rotatingBox = world.CreateRigidBody();

    constexpr Jitter2::Real size = static_cast<Jitter2::Real>(50);

    auto addTransformedBox =
        [&store](Jitter2::RigidBody& body, const JVector& boxSize, const JVector& translation)
        {
            Shapes::BoxShape& box = store.Create<Shapes::BoxShape>(boxSize);
            body.AddShape(store.Create<Shapes::TransformedShape>(box, translation));
        };

    addTransformedBox(rotatingBox, JVector(size, 1, size), JVector(0, +size / 2, 0));
    addTransformedBox(rotatingBox, JVector(size, 1, size), JVector(0, -size / 2, 0));
    addTransformedBox(rotatingBox, JVector(1, size, size), JVector(+size / 2, 0, 0));
    addTransformedBox(rotatingBox, JVector(1, size, size), JVector(-size / 2, 0, 0));
    addTransformedBox(rotatingBox, JVector(size, size, 1), JVector(0, 0, +size / 2));
    addTransformedBox(rotatingBox, JVector(size, size, 1), JVector(0, 0, -size / 2));

    rotatingBox.MotionTypeValue(Jitter2::MotionType::Kinematic);
    rotatingBox.DeactivationTime(std::numeric_limits<Jitter2::Real>::max());
    rotatingBox.SetActivationState(true);

    for (int i = -10; i < 10; ++i)
    {
        for (int e = -10; e < 10; ++e)
        {
            for (int j = -10; j < 10; ++j)
            {
                Jitter2::RigidBody& rigidBody = world.CreateRigidBody();
                rigidBody.AddShape(store.Create<Shapes::BoxShape>(static_cast<Jitter2::Real>(1.5)));
                rigidBody.Position(JVector(i, e, j) * static_cast<Jitter2::Real>(2));
            }
        }
    }

    return &rotatingBox;
}

void UpdateRotatingCube(Jitter2::RigidBody* rotatingCube)
{
    if (rotatingCube != nullptr)
    {
        rotatingCube->AngularVelocity(JVector(
            static_cast<Jitter2::Real>(0.14),
            static_cast<Jitter2::Real>(0.02),
            static_cast<Jitter2::Real>(0.03)));
    }
}

Scene ParseScene(const std::string& value)
{
    if (value == "colosseum")
    {
        return Scene::Colosseum;
    }

    if (value == "rotating-cube")
    {
        return Scene::RotatingCube;
    }

    throw std::out_of_range("Unknown benchmark scene.");
}

Options ParseOptions(int argc, char** argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--frames" && index + 1 < argc)
        {
            options.Frames = std::max(1, std::atoi(argv[++index]));
        }
        else if (argument == "--scene" && index + 1 < argc)
        {
            options.BenchmarkScene = ParseScene(argv[++index]);
        }
        else if (argument == "--warmup" && index + 1 < argc)
        {
            options.WarmupFrames = std::max(0, std::atoi(argv[++index]));
        }
        else if (argument == "--threads" && index + 1 < argc)
        {
            options.Threads = std::max(1, std::atoi(argv[++index]));
        }
        else if (argument == "--single-thread")
        {
            options.Multithread = false;
        }
        else if (argument == "--dt" && index + 1 < argc)
        {
            options.Timestep = static_cast<Jitter2::Real>(std::atof(argv[++index]));
        }
        else if (argument == "--help")
        {
            std::printf(
                "Usage: JitterBenchmark [--scene colosseum|rotating-cube] [--frames N] [--warmup N] [--threads N] [--single-thread] [--dt seconds]\n");
            std::exit(0);
        }
    }
    return options;
}

double SumDebugTimings(const Jitter2::World& world)
{
    double total = 0.0;
    for (const double value : world.DebugTimings())
    {
        total += value;
    }
    return total;
}

void PrintTimingBuckets(const Jitter2::World& world)
{
    const std::span<const double> timings = world.DebugTimings();
    for (std::size_t index = 0; index < timings.size(); ++index)
    {
        const auto timing = static_cast<Jitter2::World::Timings>(index);
        std::printf("  %-18s %8.3f ms\n", Jitter2::World::TimingName(timing), timings[index]);
    }
}

} // namespace

int main(int argc, char** argv)
{
    const Options options = ParseOptions(argc, argv);

    if (options.Threads > 0)
    {
        Jitter2::Parallelization::ThreadPool::Instance().ChangeThreadCount(options.Threads);
    }

    Jitter2::World world;
    ShapeStore store;
    Jitter2::RigidBody* rotatingCube = nullptr;
    if (options.BenchmarkScene == Scene::Colosseum)
    {
        BuildColosseum(world, store);
    }
    else
    {
        rotatingCube = BuildRotatingCube(world, store);
    }

    std::printf("Precision: %s\n", Jitter2::IsDoublePrecision ? "double" : "single");
    std::printf("Scene: %s\n", SceneName(options.BenchmarkScene));
    std::printf(
        "Threads: %d, multithread step: %s\n",
        Jitter2::Parallelization::ThreadPool::Instance().ThreadCount(),
        options.Multithread ? "true" : "false");
    std::printf(
        "Bodies: %zu, shapes: %zu, proxies: %zu\n",
        world.RigidBodies().size() > 0 ? world.RigidBodies().size() - 1 : 0,
        CountAttachedShapes(world),
        world.DynamicTree().Count());
    std::printf("AllowDeactivation: %s\n", world.AllowDeactivation ? "true" : "false");

    for (int index = 0; index < options.WarmupFrames; ++index)
    {
        UpdateRotatingCube(rotatingCube);
        world.Step(options.Timestep, options.Multithread);
    }

    using Clock = std::chrono::steady_clock;
    double debugTotalMilliseconds = 0.0;
    const auto start = Clock::now();
    for (int index = 0; index < options.Frames; ++index)
    {
        UpdateRotatingCube(rotatingCube);
        world.Step(options.Timestep, options.Multithread);
        debugTotalMilliseconds += SumDebugTimings(world);
    }
    const auto end = Clock::now();

    const double wallMilliseconds = std::chrono::duration<double, std::milli>(end - start).count();
    const double averageWallMilliseconds = wallMilliseconds / static_cast<double>(options.Frames);
    const double averageDebugMilliseconds = debugTotalMilliseconds / static_cast<double>(options.Frames);

    std::printf("Frames: %d warmup + %d measured\n", options.WarmupFrames, options.Frames);
    std::printf(
        "Wall avg: %.3f ms (%.0f fps)\n",
        averageWallMilliseconds,
        averageWallMilliseconds > 0.0 ? 1000.0 / averageWallMilliseconds : 0.0);
    std::printf(
        "DebugTimings avg: %.3f ms (%.0f fps)\n",
        averageDebugMilliseconds,
        averageDebugMilliseconds > 0.0 ? 1000.0 / averageDebugMilliseconds : 0.0);
    std::printf("Last timing buckets:\n");
    PrintTimingBuckets(world);

    return 0;
}
