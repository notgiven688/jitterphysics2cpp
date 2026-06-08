using System.Diagnostics;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using ThreadPool = Jitter2.Parallelization.ThreadPool;

Options options = Options.Parse(args);

if (options.Threads > 0)
{
    ThreadPool.Instance.ChangeThreadCount(options.Threads);
}

World world = new();
RigidBody? rotatingCube = options.Scene == Scene.RotatingCube
    ? BuildRotatingCube(world)
    : null;

if (options.Scene == Scene.Colosseum)
{
    BuildColosseum(world);
}

Console.WriteLine($"Precision: {(Precision.IsDoublePrecision ? "double" : "single")}");
Console.WriteLine($"Scene: {options.SceneName}");
Console.WriteLine($"Threads: {ThreadPool.Instance.ThreadCount}, multithread step: {options.Multithread.ToString().ToLowerInvariant()}");
Console.WriteLine($"Bodies: {Math.Max(0, world.RigidBodies.Count - 1)}, shapes: {world.RigidBodies.Sum(body => body.Shapes.Count)}");
Console.WriteLine($"AllowDeactivation: {world.AllowDeactivation.ToString().ToLowerInvariant()}");

for (int index = 0; index < options.WarmupFrames; index++)
{
    UpdateRotatingCube(rotatingCube);
    world.Step(options.Timestep, options.Multithread);
}

double debugTotalMilliseconds = 0.0;
Stopwatch stopwatch = Stopwatch.StartNew();
for (int index = 0; index < options.Frames; index++)
{
    UpdateRotatingCube(rotatingCube);
    world.Step(options.Timestep, options.Multithread);
    debugTotalMilliseconds += SumDebugTimings(world);
}
stopwatch.Stop();

double wallAverageMilliseconds = stopwatch.Elapsed.TotalMilliseconds / options.Frames;
double debugAverageMilliseconds = debugTotalMilliseconds / options.Frames;

Console.WriteLine($"Frames: {options.WarmupFrames} warmup + {options.Frames} measured");
Console.WriteLine($"Wall avg: {wallAverageMilliseconds:0.000} ms ({(wallAverageMilliseconds > 0 ? 1000.0 / wallAverageMilliseconds : 0):0} fps)");
Console.WriteLine($"DebugTimings avg: {debugAverageMilliseconds:0.000} ms ({(debugAverageMilliseconds > 0 ? 1000.0 / debugAverageMilliseconds : 0):0} fps)");
Console.WriteLine("Last timing buckets:");
PrintTimingBuckets(world);

static void AddFloor(World world)
{
    RigidBody body = world.CreateRigidBody();
    BoxShape floorShape = new(200, 200, 200);
    body.Position = new JVector(0, -100, 0);
    body.MotionType = MotionType.Static;
    body.AddShape(floorShape);
}

static void CreateRingWall(World world, JVector position, JVector size, int height, float radius)
{
    float circumference = MathF.PI * 2 * radius;
    int boxCountPerRing = (int)(0.9f * circumference / size.Z);
    float increment = (2.0f * MathF.PI) / boxCountPerRing;

    for (int ringIndex = 0; ringIndex < height; ringIndex++)
    {
        for (int i = 0; i < boxCountPerRing; i++)
        {
            RigidBody body = world.CreateRigidBody();
            body.AddShape(new BoxShape(size));

            float angle = ((ringIndex & 1) == 0 ? i + 0.5f : i) * increment;
            body.Position = position + new JVector(-MathF.Cos(angle) * radius, (ringIndex + 0.5f) * size.Y, MathF.Sin(angle) * radius);
            body.Orientation = JQuaternion.CreateFromAxisAngle(JVector.UnitY, angle);
        }
    }
}

static void CreateRingPlatform(World world, JVector position, JVector size, float radius)
{
    float innerCircumference = MathF.PI * 2 * (radius - 0.5f * size.Z);
    int boxCount = (int)(0.95f * innerCircumference / size.Y);
    float increment = (2.0f * MathF.PI) / boxCount;

    for (int i = 0; i < boxCount; i++)
    {
        float angle = i * increment;

        RigidBody body = world.CreateRigidBody();
        body.AddShape(new BoxShape(size));

        body.Position = position + new JVector(-MathF.Cos(angle) * radius, 0.5f * size.X, MathF.Sin(angle) * radius);
        body.Orientation = JQuaternion.CreateFromAxisAngle(JVector.UnitY, angle + MathF.PI * 0.5f) *
            JQuaternion.CreateFromAxisAngle(JVector.UnitZ, MathF.PI * 0.5f);
    }
}

static JVector CreateRing(World world, JVector position, JVector size, float radius, int heightPerPlatformLevel, int platformLevels)
{
    for (int platformIndex = 0; platformIndex < platformLevels; platformIndex++)
    {
        float wallOffset = 0.5f * size.Z - 0.5f * size.X;
        CreateRingWall(world, position, size, heightPerPlatformLevel, radius + wallOffset);
        CreateRingWall(world, position, size, heightPerPlatformLevel, radius - wallOffset);

        CreateRingPlatform(world, position + new JVector(0, heightPerPlatformLevel * size.Y, 0), size, radius);
        position.Y += heightPerPlatformLevel * size.Y + size.X;
    }

    return position;
}

static void BuildColosseum(World world)
{
    AddFloor(world);
    world.AllowDeactivation = false;

    JVector size = new(0.5f, 1, 3);
    JVector layerPosition = new();
    const int layerCount = 6;
    const float innerRadius = 15f;
    const int heightPerPlatform = 3;
    const int platformsPerLayer = 1;
    const float ringSpacing = 0.5f;

    for (int layerIndex = 0; layerIndex < layerCount; layerIndex++)
    {
        int ringCount = layerCount - layerIndex;
        for (int ringIndex = 0; ringIndex < ringCount; ringIndex++)
        {
            CreateRing(
                world,
                layerPosition,
                size,
                innerRadius + ringIndex * (size.Z + ringSpacing) + layerIndex * (size.Z - size.X),
                heightPerPlatform,
                platformsPerLayer);
        }

        layerPosition.Y += platformsPerLayer * (size.Y * heightPerPlatform + size.X);
    }
}

static RigidBody BuildRotatingCube(World world)
{
    RigidBody rotatingBox = world.CreateRigidBody();

    const float size = 50;

    TransformedShape bs0 = new(new BoxShape(size, 1, size), new JVector(0, +size / 2, 0));
    TransformedShape bs1 = new(new BoxShape(size, 1, size), new JVector(0, -size / 2, 0));
    TransformedShape bs2 = new(new BoxShape(1, size, size), new JVector(+size / 2, 0, 0));
    TransformedShape bs3 = new(new BoxShape(1, size, size), new JVector(-size / 2, 0, 0));
    TransformedShape bs4 = new(new BoxShape(size, size, 1), new JVector(0, 0, +size / 2));
    TransformedShape bs5 = new(new BoxShape(size, size, 1), new JVector(0, 0, -size / 2));

    rotatingBox.AddShapes([bs0, bs1, bs2, bs3, bs4, bs5]);
    rotatingBox.MotionType = MotionType.Kinematic;
    rotatingBox.DeactivationTime = TimeSpan.MaxValue;
    rotatingBox.SetActivationState(true);

    for (int i = -10; i < 10; i++)
    {
        for (int e = -10; e < 10; e++)
        {
            for (int j = -10; j < 10; j++)
            {
                RigidBody rb = world.CreateRigidBody();
                rb.AddShape(new BoxShape(1.5f));
                rb.Position = new JVector(i, e, j) * 2;
            }
        }
    }

    return rotatingBox;
}

static void UpdateRotatingCube(RigidBody? rotatingCube)
{
    if (rotatingCube != null)
    {
        rotatingCube.AngularVelocity = new JVector(0.14f, 0.02f, 0.03f);
    }
}

static double SumDebugTimings(World world)
{
    double total = 0.0;
    foreach (double value in world.DebugTimings)
    {
        total += value;
    }

    return total;
}

static void PrintTimingBuckets(World world)
{
    for (int index = 0; index < (int)World.Timings.Last; index++)
    {
        Console.WriteLine($"  {((World.Timings)index),-18} {world.DebugTimings[index],8:0.000} ms");
    }
}

sealed record Options
{
    public Scene Scene { get; private init; } = Scene.Colosseum;
    public string SceneName => Scene == Scene.Colosseum ? "colosseum" : "rotating-cube";
    public int WarmupFrames { get; private init; } = 120;
    public int Frames { get; private init; } = 1000;
    public int Threads { get; private init; } = -1;
    public bool Multithread { get; private init; } = true;
    public float Timestep { get; private init; } = 1.0f / 100.0f;

    public static Options Parse(string[] args)
    {
        Options options = new();

        for (int index = 0; index < args.Length; index++)
        {
            string argument = args[index];
            if (argument == "--frames" && index + 1 < args.Length)
            {
                options = options with { Frames = Math.Max(1, int.Parse(args[++index])) };
            }
            else if (argument == "--scene" && index + 1 < args.Length)
            {
                options = options with { Scene = ParseScene(args[++index]) };
            }
            else if (argument == "--warmup" && index + 1 < args.Length)
            {
                options = options with { WarmupFrames = Math.Max(0, int.Parse(args[++index])) };
            }
            else if (argument == "--threads" && index + 1 < args.Length)
            {
                options = options with { Threads = Math.Max(1, int.Parse(args[++index])) };
            }
            else if (argument == "--single-thread")
            {
                options = options with { Multithread = false };
            }
            else if (argument == "--dt" && index + 1 < args.Length)
            {
                options = options with { Timestep = float.Parse(args[++index]) };
            }
            else if (argument == "--help")
            {
                Console.WriteLine("Usage: dotnet run -c Release --project Benchmarks/CSharp/JitterBenchmark.CSharp.csproj -- [--scene colosseum|rotating-cube] [--frames N] [--warmup N] [--threads N] [--single-thread] [--dt seconds]");
                Environment.Exit(0);
            }
        }

        return options;
    }

    private static Scene ParseScene(string value)
    {
        return value switch
        {
            "colosseum" => Scene.Colosseum,
            "rotating-cube" => Scene.RotatingCube,
            _ => throw new ArgumentOutOfRangeException(nameof(value), value, "Unknown benchmark scene.")
        };
    }
}

enum Scene
{
    Colosseum,
    RotatingCube
}
