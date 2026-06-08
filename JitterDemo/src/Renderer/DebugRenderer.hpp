namespace ClothMaterial
{

Material Create()
{
    static std::shared_ptr<Texture2D> cached;
    if (!cached)
    {
        int width = 0;
        int height = 0;
        std::vector<unsigned char> pixels = LoadTgaBgra("assets/texture_10.tga", width, height);

        cached = std::make_shared<Texture2D>();
        cached->LoadImage(pixels, width, height);
        cached->SetWrap(GL_REPEAT);
        cached->SetAnisotropy(8.0f);
    }

    Material material;
    material.Tint = Vec3 {};
    material.Specular = Vec3 {0.1f, 0.1f, 0.1f};
    material.Shininess = 128.0f;
    material.Alpha = 1.0f;
    material.VertexColorWeight = 0.1f;
    material.TextureWeight = 0.9f;
    material.Texture = cached;
    return material;
}

} // namespace ClothMaterial

class MutableMeshDrawable
{
public:
    struct MeshData
    {
        std::vector<Vertex> Vertices;
        std::vector<TriangleVertexIndex> Indices;
    };

    struct PendingDraw
    {
        Mat4 Model = Identity();
        Vec3 InstanceColor {0.0f, 1.0f, 0.0f};
    };

    MutableMeshDrawable()
    {
        TwoSided = true;
        ShadowsDoubleSided = true;

        glGenVertexArrays(1, &VertexArray_);
        glGenBuffers(1, &VertexBuffer_);
        glGenBuffers(1, &ElementBuffer_);

        glBindVertexArray(VertexArray_);
        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ElementBuffer_);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            reinterpret_cast<void*>(offsetof(Vertex, Position)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            reinterpret_cast<void*>(offsetof(Vertex, Normal)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            reinterpret_cast<void*>(offsetof(Vertex, Texture)));

        glBindVertexArray(0);
    }

    MutableMeshDrawable(const MutableMeshDrawable&) = delete;
    MutableMeshDrawable& operator=(const MutableMeshDrawable&) = delete;

    ~MutableMeshDrawable()
    {
        if (ElementBuffer_ != 0)
        {
            glDeleteBuffers(1, &ElementBuffer_);
        }
        if (VertexBuffer_ != 0)
        {
            glDeleteBuffers(1, &VertexBuffer_);
        }
        if (VertexArray_ != 0)
        {
            glDeleteVertexArrays(1, &VertexArray_);
        }
    }

    void SetTriangles(const std::vector<TriangleVertexIndex>& triangles)
    {
        Mesh.Indices = triangles;

        glBindVertexArray(VertexArray_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ElementBuffer_);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(Mesh.Indices.size() * sizeof(TriangleVertexIndex)),
            Mesh.Indices.data(),
            GL_STATIC_DRAW);
        glBindVertexArray(0);

        std::uint32_t largest = 0;
        for (const TriangleVertexIndex& triangle : Mesh.Indices)
        {
            largest = std::max({largest, triangle.T1, triangle.T2, triangle.T3});
        }

        Mesh.Vertices.assign(static_cast<std::size_t>(largest + 1), Vertex {});
    }

    void RefreshGeometry()
    {
        for (Vertex& vertex : Mesh.Vertices)
        {
            vertex.Normal = Vec3 {};
        }

        for (const TriangleVertexIndex& triangle : Mesh.Indices)
        {
            Vertex& v1 = Mesh.Vertices[static_cast<std::size_t>(triangle.T1)];
            Vertex& v2 = Mesh.Vertices[static_cast<std::size_t>(triangle.T2)];
            Vertex& v3 = Mesh.Vertices[static_cast<std::size_t>(triangle.T3)];

            const Vec3 normal = Cross(v2.Position - v1.Position, v3.Position - v1.Position);
            v1.Normal = v1.Normal + normal;
            v2.Normal = v2.Normal + normal;
            v3.Normal = v3.Normal + normal;
        }

        glBindVertexArray(VertexArray_);
        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(Mesh.Vertices.size() * sizeof(Vertex)),
            Mesh.Vertices.data(),
            GL_DYNAMIC_DRAW);
        glBindVertexArray(0);
    }

    void Push(const Mat4& model, const Vec3& instanceColor)
    {
        Pending.push_back(PendingDraw {model, instanceColor});
    }

    MeshData Mesh;
    std::vector<MeshGroup> MeshGroups;
    std::vector<MaterialSlot> Groups;
    Material DrawableMaterial = Material::Default();
    bool TwoSided = false;
    bool ShadowsDoubleSided = false;

private:
    std::vector<PendingDraw> Pending;
    GLuint VertexArray_ = 0;
    GLuint VertexBuffer_ = 0;
    GLuint ElementBuffer_ = 0;

    friend class Renderer;
    friend class ShadowCaster;
};

class DebugRenderer : public Jitter2::IDebugDrawer
{
public:
    enum class Color
    {
        White = 0,
        Red = 1,
        Green = 2,
        Count = 3,
    };

    DebugRenderer()
        : ShaderProgram_(CreateShaderProgram(DebugVertexShaderSource, DebugFragmentShaderSource))
    {
        ProjectionLocation_ = glGetUniformLocation(ShaderProgram_, "uProjection");
        ViewLocation_ = glGetUniformLocation(ShaderProgram_, "uView");
        ColorLocation_ = glGetUniformLocation(ShaderProgram_, "uColor");

        glGenVertexArrays(1, &VertexArray_);
        glGenBuffers(1, &VertexBuffer_);

        glBindVertexArray(VertexArray_);
        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            reinterpret_cast<void*>(offsetof(Vertex, Position)));
        glBindVertexArray(0);
    }

    DebugRenderer(const DebugRenderer&) = delete;
    DebugRenderer& operator=(const DebugRenderer&) = delete;

    ~DebugRenderer()
    {
        if (VertexBuffer_ != 0)
        {
            glDeleteBuffers(1, &VertexBuffer_);
        }
        if (VertexArray_ != 0)
        {
            glDeleteVertexArrays(1, &VertexArray_);
        }
        if (ShaderProgram_ != 0)
        {
            glDeleteProgram(ShaderProgram_);
        }
    }

    void PushLine(Color color, const Vec3& a, const Vec3& b)
    {
        Bucket& bucket = Buckets_[static_cast<std::size_t>(color)];
        bucket.Vertices.push_back(Vertex {a, Vec3 {0.0f, 1.0f, 0.0f}});
        bucket.Vertices.push_back(Vertex {b, Vec3 {0.0f, 1.0f, 0.0f}});
    }

    void PushBox(Color color, const Vec3& min, const Vec3& max)
    {
        const Vec3 o0 {min.X, min.Y, min.Z};
        const Vec3 o1 {max.X, min.Y, min.Z};
        const Vec3 o2 {min.X, max.Y, min.Z};
        const Vec3 o3 {min.X, min.Y, max.Z};
        const Vec3 o4 {max.X, max.Y, min.Z};
        const Vec3 o5 {min.X, max.Y, max.Z};
        const Vec3 o6 {max.X, min.Y, max.Z};
        const Vec3 o7 {max.X, max.Y, max.Z};

        PushLine(color, o0, o1);
        PushLine(color, o0, o2);
        PushLine(color, o0, o3);
        PushLine(color, o1, o4);
        PushLine(color, o1, o6);
        PushLine(color, o2, o4);
        PushLine(color, o2, o5);
        PushLine(color, o3, o5);
        PushLine(color, o3, o6);
        PushLine(color, o4, o7);
        PushLine(color, o5, o7);
        PushLine(color, o6, o7);
    }

    void PushPoint(Color color, const Vec3& position, float halfSize = 1.0f)
    {
        PushLine(
            color,
            Vec3 {position.X - halfSize, position.Y, position.Z},
            Vec3 {position.X + halfSize, position.Y, position.Z});
        PushLine(
            color,
            Vec3 {position.X, position.Y - halfSize, position.Z},
            Vec3 {position.X, position.Y + halfSize, position.Z});
        PushLine(
            color,
            Vec3 {position.X, position.Y, position.Z - halfSize},
            Vec3 {position.X, position.Y, position.Z + halfSize});
    }

    void DrawSegment(
        const Jitter2::LinearMath::JVector& pA,
        const Jitter2::LinearMath::JVector& pB) override
    {
        PushLine(Color::Green, FromJitterVector(pA), FromJitterVector(pB));
    }

    void DrawTriangle(
        const Jitter2::LinearMath::JVector& pA,
        const Jitter2::LinearMath::JVector& pB,
        const Jitter2::LinearMath::JVector& pC) override
    {
        PushLine(Color::Green, FromJitterVector(pA), FromJitterVector(pB));
        PushLine(Color::Green, FromJitterVector(pB), FromJitterVector(pC));
        PushLine(Color::Green, FromJitterVector(pC), FromJitterVector(pA));
    }

    void DrawPoint(const Jitter2::LinearMath::JVector& p) override
    {
        PushPoint(Color::White, FromJitterVector(p), 0.1f);
    }

    void Draw(const CameraMatrices& cameraMatrices)
    {
        glUseProgram(ShaderProgram_);
        glUniformMatrix4fv(ProjectionLocation_, 1, GL_FALSE, cameraMatrices.Projection.Data());
        glUniformMatrix4fv(ViewLocation_, 1, GL_FALSE, cameraMatrices.View.Data());
        glBindVertexArray(VertexArray_);
        glLineWidth(1.5f);

        for (std::size_t colorIndex = 0; colorIndex < Buckets_.size(); ++colorIndex)
        {
            Bucket& bucket = Buckets_[colorIndex];
            if (bucket.Vertices.empty())
            {
                continue;
            }

            const Vec3 color = ColorValues_[colorIndex];
            glUniform3f(ColorLocation_, color.X, color.Y, color.Z);
            glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
            glBufferData(GL_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(bucket.Vertices.size() * sizeof(Vertex)),
                bucket.Vertices.data(),
                GL_DYNAMIC_DRAW);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(bucket.Vertices.size()));
            bucket.Vertices.clear();
        }

        glLineWidth(1.0f);
        glBindVertexArray(0);
        glUseProgram(0);
    }

private:
    struct Bucket
    {
        std::vector<Vertex> Vertices;
    };

    static Vec3 FromJitterVector(const Jitter2::LinearMath::JVector& value)
    {
        return Vec3 {
            static_cast<float>(value.X),
            static_cast<float>(value.Y),
            static_cast<float>(value.Z),
        };
    }

    static constexpr std::array<Vec3, 3> ColorValues_ {{
        Vec3 {1.0f, 1.0f, 1.0f},
        Vec3 {1.0f, 0.0f, 0.0f},
        Vec3 {0.0f, 1.0f, 0.0f},
    }};

    static constexpr const char* DebugVertexShaderSource = R"GLSL(#version 330 core
layout (location = 0) in vec3 aPosition;

uniform mat4 uProjection;
uniform mat4 uView;

void main()
{
    gl_Position = uProjection * uView * vec4(aPosition, 1.0);
}
)GLSL";

    static constexpr const char* DebugFragmentShaderSource = R"GLSL(#version 330 core
uniform vec3 uColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(uColor, 1.0);
}
)GLSL";

    std::array<Bucket, static_cast<std::size_t>(Color::Count)> Buckets_;
    GLuint ShaderProgram_ = 0;
    GLuint VertexArray_ = 0;
    GLuint VertexBuffer_ = 0;
    GLint ProjectionLocation_ = -1;
    GLint ViewLocation_ = -1;
    GLint ColorLocation_ = -1;
};
