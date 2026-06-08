struct Vec3
{
    float X = 0.0f;
    float Y = 0.0f;
    float Z = 0.0f;
};

struct Vec2
{
    float X = 0.0f;
    float Y = 0.0f;
};

struct Vec4
{
    float X = 0.0f;
    float Y = 0.0f;
    float Z = 0.0f;
    float W = 0.0f;
};

struct Color
{
    float R = 1.0f;
    float G = 1.0f;
    float B = 1.0f;
};

struct Mat4
{
    std::array<float, 16> Values {};

    [[nodiscard]] const float* Data() const { return Values.data(); }
};

struct CameraMatrices
{
    Mat4 Projection;
    Mat4 View;
    Mat4 ViewProjection;
};

struct Vertex
{
    Vec3 Position;
    Vec3 Normal;
    Vec2 Texture;

    Vertex() = default;

    explicit Vertex(Vec3 position)
        : Position(position),
          Normal {1.0f, 0.0f, 0.0f},
          Texture {}
    {
    }

    Vertex(Vec3 position, Vec3 normal)
        : Position(position),
          Normal(normal),
          Texture {}
    {
    }

    Vertex(Vec3 position, Vec3 normal, Vec2 texture)
        : Position(position),
          Normal(normal),
          Texture(texture)
    {
    }
};

struct LaunchOptions
{
    int FrameLimit = 0;
    int DemoIndex = -1;
    std::string ScreenshotPath;
};

struct CameraState
{
    Vec3 Position {0.0f, 4.0f, 8.0f};
    Vec3 Direction {0.0f, 0.0f, -1.0f};
    float FieldOfView = Pi / 4.0f;
    double Theta = Pi / 2.0;
    double Phi = 0.0;
    float NearPlane = 0.1f;
    float FarPlane = 400.0f;
    float MoveSpeed = 0.4f;
    float MouseSensitivity = 0.006f;
    bool Dragging = false;
    double LastMouseX = 0.0;
    double LastMouseY = 0.0;
};

struct DemoSettings
{
    int DemoIndex = -1;
    bool Paused = false;
    bool Animate = false;
    bool Multithread = true;
    bool Wireframe = false;
    bool DebugDrawIslands = false;
    bool DebugDrawContacts = false;
    bool DebugDrawShapes = false;
    bool DebugDrawTree = false;
    int DebugDrawTreeDepth = 1;
    bool ShowShadowDebug = false;
    bool ShowImGuiDemo = false;
    float Timestep = 1.0f / 100.0f;
    float AnimationSpeed = 1.0f;
};

struct BodyTrack
{
    Jitter2::RigidBody* Body = nullptr;
    JVector BasePosition = JVector::Zero();
    float Phase = 0.0f;
    float Bob = 0.0f;
    float Orbit = 0.0f;
    float Spin = 0.0f;
    float Tilt = 0.0f;
};

struct BodyTrackParams
{
    float Phase = 0.0f;
    float Bob = 0.0f;
    float Orbit = 0.0f;
    float Spin = 1.0f;
    float Tilt = 0.0f;
};

struct ConveyorPlank
{
    Jitter2::RigidBody* Body = nullptr;
    float DistanceOffset = 0.0f;
};

struct GearMarker
{
};

struct RigidBodyTag
{
    bool DoNotDraw = true;
};
