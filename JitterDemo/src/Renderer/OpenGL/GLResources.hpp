class Mesh
{
public:
    Mesh() = default;
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept
        : VertexArray_(other.VertexArray_),
          VertexBuffer_(other.VertexBuffer_),
          ElementBuffer_(other.ElementBuffer_),
          IndexCount_(other.IndexCount_)
    {
        other.VertexArray_ = 0;
        other.VertexBuffer_ = 0;
        other.ElementBuffer_ = 0;
        other.IndexCount_ = 0;
    }

    Mesh& operator=(Mesh&& other) noexcept
    {
        if (this != &other)
        {
            Release();
            VertexArray_ = other.VertexArray_;
            VertexBuffer_ = other.VertexBuffer_;
            ElementBuffer_ = other.ElementBuffer_;
            IndexCount_ = other.IndexCount_;
            other.VertexArray_ = 0;
            other.VertexBuffer_ = 0;
            other.ElementBuffer_ = 0;
            other.IndexCount_ = 0;
        }
        return *this;
    }

    ~Mesh()
    {
        Release();
    }

    void Upload(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices)
    {
        IndexCount_ = static_cast<GLsizei>(indices.size());

        glGenVertexArrays(1, &VertexArray_);
        glGenBuffers(1, &VertexBuffer_);
        glGenBuffers(1, &ElementBuffer_);

        glBindVertexArray(VertexArray_);

        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glBufferData(GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
            vertices.data(),
            GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ElementBuffer_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
            indices.data(),
            GL_STATIC_DRAW);

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

    void Draw() const
    {
        glBindVertexArray(VertexArray_);
        glDrawElements(GL_TRIANGLES, IndexCount_, GL_UNSIGNED_INT, nullptr);
    }

    void Bind() const
    {
        glBindVertexArray(VertexArray_);
    }

    [[nodiscard]] GLsizei IndexCount() const
    {
        return IndexCount_;
    }

private:
    void Release()
    {
        if (ElementBuffer_ != 0)
        {
            glDeleteBuffers(1, &ElementBuffer_);
            ElementBuffer_ = 0;
        }
        if (VertexBuffer_ != 0)
        {
            glDeleteBuffers(1, &VertexBuffer_);
            VertexBuffer_ = 0;
        }
        if (VertexArray_ != 0)
        {
            glDeleteVertexArrays(1, &VertexArray_);
            VertexArray_ = 0;
        }
    }

    GLuint VertexArray_ = 0;
    GLuint VertexBuffer_ = 0;
    GLuint ElementBuffer_ = 0;
    GLsizei IndexCount_ = 0;
};

class Texture2D
{
public:
    Texture2D()
    {
        glGenTextures(1, &Handle_);
        Bind();
    }

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;

    Texture2D(Texture2D&& other) noexcept
        : Handle_(other.Handle_)
    {
        other.Handle_ = 0;
    }

    Texture2D& operator=(Texture2D&& other) noexcept
    {
        if (this != &other)
        {
            Release();
            Handle_ = other.Handle_;
            other.Handle_ = 0;
        }
        return *this;
    }

    ~Texture2D()
    {
        Release();
    }

    void Bind() const
    {
        glBindTexture(GL_TEXTURE_2D, Handle_);
    }

    void Bind(GLenum unit) const
    {
        glActiveTexture(unit);
        Bind();
    }

    void LoadImage(const std::vector<unsigned char>& bgraData, int width, int height, bool generateMipmap = true)
    {
        Bind();
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            width,
            height,
            0,
            GL_BGRA,
            GL_UNSIGNED_BYTE,
            bgraData.data());

        if (generateMipmap)
        {
            SetMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            SetMinMagFilter(GL_LINEAR, GL_LINEAR);
        }
    }

    void AllocateDepth(int width, int height)
    {
        Bind();
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_DEPTH_COMPONENT,
            width,
            height,
            0,
            GL_DEPTH_COMPONENT,
            GL_FLOAT,
            nullptr);
    }

    void SetWrap(GLint wrap)
    {
        Bind();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    }

    void SetMinMagFilter(GLint minFilter, GLint magFilter)
    {
        Bind();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
    }

    void SetAnisotropy(float requested)
    {
        static constexpr GLenum TextureMaxAnisotropy = 0x84FE;
        static constexpr GLenum MaxTextureMaxAnisotropy = 0x84FF;

        if (glfwExtensionSupported("GL_EXT_texture_filter_anisotropic") != GLFW_TRUE)
        {
            return;
        }

        Bind();
        float hardwareMaximum = 1.0f;
        glGetFloatv(MaxTextureMaxAnisotropy, &hardwareMaximum);
        const float value = std::min(requested, hardwareMaximum);
        glTexParameterf(GL_TEXTURE_2D, TextureMaxAnisotropy, value);
    }

    void SetBorderColor(float r, float g, float b, float a)
    {
        const std::array<float, 4> color {r, g, b, a};
        Bind();
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color.data());
    }

    [[nodiscard]] GLuint Handle() const
    {
        return Handle_;
    }

    static std::shared_ptr<Texture2D> Empty()
    {
        static std::shared_ptr<Texture2D> empty = []
        {
            auto texture = std::make_shared<Texture2D>();
            const std::vector<unsigned char> black {0, 0, 0, 0};
            texture->LoadImage(black, 1, 1, false);
            return texture;
        }();
        return empty;
    }

    static std::shared_ptr<Texture2D> White()
    {
        static std::shared_ptr<Texture2D> white = []
        {
            auto texture = std::make_shared<Texture2D>();
            const std::vector<unsigned char> value {255, 255, 255, 255};
            texture->LoadImage(value, 1, 1, false);
            return texture;
        }();
        return white;
    }

private:
    void Release()
    {
        if (Handle_ != 0)
        {
            glDeleteTextures(1, &Handle_);
            Handle_ = 0;
        }
    }

    GLuint Handle_ = 0;
};

struct Instance
{
    Mat4 Transform = Identity();
    Vec3 InstanceColor {1.0f, 1.0f, 1.0f};
};

struct MeshGroup
{
    std::string Name;
    int FromInclusive = 0;
    int ToExclusive = 0;
};

struct Material
{
    Vec3 Tint {};
    Vec3 Specular {0.1f, 0.1f, 0.1f};
    float Shininess = 128.0f;
    float Alpha = 1.0f;
    float VertexColorWeight = 1.0f;
    float TextureWeight = 0.0f;
    bool FlipNormals = false;
    std::shared_ptr<Texture2D> Texture;

    static Material Default()
    {
        return Material {};
    }
};

struct MaterialSlot
{
    int GroupIndex = 0;
    Material MaterialValue = Material::Default();
};

namespace ColorGenerator
{
constexpr int NumColors = 127;

Vec3 ColorFromHsv(float h, float s, float v)
{
    const auto gen =
        [h, s, v](float n)
        {
            const float k = std::fmod(n + h * 6.0f, 6.0f);
            return v - v * s * std::max(0.0f, std::min(std::min(k, 4.0f - k), 1.0f));
        };

    return Vec3 {gen(5.0f), gen(3.0f), gen(1.0f)};
}

const std::array<Vec3, NumColors>& Buffer()
{
    static const std::array<Vec3, NumColors> buffer = []
    {
        std::array<Vec3, NumColors> values {};
        for (int i = 0; i < NumColors; ++i)
        {
            values[static_cast<std::size_t>(i)] =
                ColorFromHsv(static_cast<float>(i) / static_cast<float>(NumColors), 1.0f, 0.6f);
        }
        return values;
    }();
    return buffer;
}

Vec3 GetColor(int seed)
{
    return Buffer()[static_cast<std::size_t>(static_cast<unsigned int>(seed) % NumColors)];
}
}

class InstancedDrawable
{
public:
    explicit InstancedDrawable(Mesh mesh)
        : DrawableMesh_(std::move(mesh))
    {
        CreateInstanceBuffer();
    }

    InstancedDrawable(const InstancedDrawable&) = delete;
    InstancedDrawable& operator=(const InstancedDrawable&) = delete;

    InstancedDrawable(InstancedDrawable&& other) noexcept
        : MeshGroups(std::move(other.MeshGroups)),
          Groups(std::move(other.Groups)),
          MaterialValue(std::move(other.MaterialValue)),
          TwoSided(other.TwoSided),
          ShadowsDoubleSided(other.ShadowsDoubleSided),
          DrawableMesh_(std::move(other.DrawableMesh_)),
          Instances_(std::move(other.Instances_)),
          InstanceBuffer_(other.InstanceBuffer_)
    {
        other.InstanceBuffer_ = 0;
    }

    InstancedDrawable& operator=(InstancedDrawable&& other) noexcept
    {
        if (this != &other)
        {
            Release();
            DrawableMesh_ = std::move(other.DrawableMesh_);
            MeshGroups = std::move(other.MeshGroups);
            Groups = std::move(other.Groups);
            MaterialValue = std::move(other.MaterialValue);
            TwoSided = other.TwoSided;
            ShadowsDoubleSided = other.ShadowsDoubleSided;
            Instances_ = std::move(other.Instances_);
            InstanceBuffer_ = other.InstanceBuffer_;
            other.InstanceBuffer_ = 0;
        }
        return *this;
    }

    ~InstancedDrawable()
    {
        Release();
    }

    void Push(const Mat4& transform)
    {
        Push(transform, Vec3 {1.0f, 1.0f, 1.0f});
    }

    void Push(const Mat4& transform, const Vec3& color)
    {
        Instances_.push_back(Instance {transform, color});
    }

    void Clear()
    {
        Instances_.clear();
    }

    [[nodiscard]] int InstanceCount() const
    {
        return static_cast<int>(Instances_.size());
    }

    void UploadInstances()
    {
        if (Instances_.empty())
        {
            return;
        }

        DrawableMesh_.Bind();
        glBindBuffer(GL_ARRAY_BUFFER, InstanceBuffer_);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(Instances_.size() * sizeof(Instance)),
            Instances_.data(),
            GL_DYNAMIC_DRAW);
    }

    void Bind() const
    {
        DrawableMesh_.Bind();
    }

    [[nodiscard]] GLsizei IndexCount() const
    {
        return DrawableMesh_.IndexCount();
    }

    std::vector<MeshGroup> MeshGroups;
    std::vector<MaterialSlot> Groups;
    Material MaterialValue = Material::Default();
    bool TwoSided = false;
    bool ShadowsDoubleSided = false;

private:
    void CreateInstanceBuffer()
    {
        DrawableMesh_.Bind();
        glGenBuffers(1, &InstanceBuffer_);
        glBindBuffer(GL_ARRAY_BUFFER, InstanceBuffer_);

        const auto stride = static_cast<GLsizei>(sizeof(Instance));
        for (int column = 0; column < 4; ++column)
        {
            const GLuint location = static_cast<GLuint>(3 + column);
            glEnableVertexAttribArray(location);
            glVertexAttribPointer(
                location,
                4,
                GL_FLOAT,
                GL_FALSE,
                stride,
                reinterpret_cast<void*>(offsetof(Instance, Transform)
                    + static_cast<std::size_t>(column) * 4U * sizeof(float)));
            glVertexAttribDivisor(location, 1);
        }

        glEnableVertexAttribArray(7);
        glVertexAttribPointer(
            7,
            3,
            GL_FLOAT,
            GL_FALSE,
            stride,
            reinterpret_cast<void*>(offsetof(Instance, InstanceColor)));
        glVertexAttribDivisor(7, 1);

        glBindVertexArray(0);
    }

    void Release()
    {
        if (InstanceBuffer_ != 0)
        {
            glDeleteBuffers(1, &InstanceBuffer_);
            InstanceBuffer_ = 0;
        }
    }

    Mesh DrawableMesh_;
    std::vector<Instance> Instances_;
    GLuint InstanceBuffer_ = 0;
};
