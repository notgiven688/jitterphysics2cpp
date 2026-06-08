class ShadowCaster
{
public:
    static constexpr int CascadeCount = 3;
    static constexpr int ShadowMapSize = 4096;

    ShadowCaster()
        : ShaderProgram_(CreateShaderProgram(ShadowVertexShaderSource, ShadowFragmentShaderSource))
    {
        LightMatrixLocation_ = glGetUniformLocation(ShaderProgram_, "uLightMatrix");
        ModelLocation_ = glGetUniformLocation(ShaderProgram_, "uModel");
        UseInstancesLocation_ = glGetUniformLocation(ShaderProgram_, "uUseInstances");

        glGenFramebuffers(CascadeCount, Framebuffers_.data());
        for (int i = 0; i < CascadeCount; ++i)
        {
            DepthMaps_[static_cast<std::size_t>(i)].AllocateDepth(ShadowMapSize, ShadowMapSize);
            DepthMaps_[static_cast<std::size_t>(i)].SetMinMagFilter(GL_NEAREST, GL_NEAREST);
            DepthMaps_[static_cast<std::size_t>(i)].SetWrap(GL_CLAMP_TO_BORDER);
            DepthMaps_[static_cast<std::size_t>(i)].SetBorderColor(1.0f, 1.0f, 1.0f, 1.0f);

            glBindFramebuffer(GL_FRAMEBUFFER, Framebuffers_[static_cast<std::size_t>(i)]);
            glFramebufferTexture2D(
                GL_FRAMEBUFFER,
                GL_DEPTH_ATTACHMENT,
                GL_TEXTURE_2D,
                DepthMaps_[static_cast<std::size_t>(i)].Handle(),
                0);
            glDrawBuffer(GL_NONE);
            glReadBuffer(GL_NONE);
            Check(
                glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
                "Unable to create shadow framebuffer.");
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ShadowCaster(const ShadowCaster&) = delete;
    ShadowCaster& operator=(const ShadowCaster&) = delete;

    ~ShadowCaster()
    {
        if (ShaderProgram_ != 0)
        {
            glDeleteProgram(ShaderProgram_);
        }
        glDeleteFramebuffers(CascadeCount, Framebuffers_.data());
    }

    template<typename TDrawCallback>
    void Render(
        const CameraState& camera,
        const CameraMatrices& cameraMatrices,
        int viewportWidth,
        int viewportHeight,
        TDrawCallback drawCallback)
    {
        glViewport(0, 0, ShadowMapSize, ShadowMapSize);
        glUseProgram(ShaderProgram_);

        ComputeLightMatrices(camera, cameraMatrices, viewportWidth, viewportHeight);

        for (int i = 0; i < CascadeCount; ++i)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, Framebuffers_[static_cast<std::size_t>(i)]);
            glClear(GL_DEPTH_BUFFER_BIT);
            glUniformMatrix4fv(
                LightMatrixLocation_,
                1,
                GL_FALSE,
                LightMatrices_[static_cast<std::size_t>(i)].Data());
            drawCallback(*this);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glUseProgram(0);

        BindDepthMaps();
    }

    void BindDepthMaps() const
    {
        for (int i = 0; i < CascadeCount; ++i)
        {
            DepthMaps_[static_cast<std::size_t>(i)].Bind(static_cast<GLenum>(GL_TEXTURE0 + i));
        }
    }

    [[nodiscard]] const std::array<Mat4, CascadeCount>& LightMatrices() const
    {
        return LightMatrices_;
    }

    [[nodiscard]] const Texture2D& DepthMap(int index) const
    {
        return DepthMaps_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] std::array<float, CascadeCount> CascadeSplitsForShader() const
    {
        return std::array<float, CascadeCount> {
            CascadeSplits_[0],
            CascadeSplits_[1],
            std::numeric_limits<float>::max(),
        };
    }

    [[nodiscard]] Vec3 LightDirection() const
    {
        return LightDirection_;
    }

    void DrawInstancedDrawable(InstancedDrawable& drawable) const
    {
        if (drawable.InstanceCount() == 0)
        {
            return;
        }

        drawable.UploadInstances();
        drawable.Bind();
        glUniform1i(UseInstancesLocation_, GL_TRUE);

        if (drawable.ShadowsDoubleSided)
        {
            glDisable(GL_CULL_FACE);
        }
        else
        {
            glCullFace(GL_FRONT);
        }

        glDrawElementsInstanced(
            GL_TRIANGLES,
            drawable.IndexCount(),
            GL_UNSIGNED_INT,
            nullptr,
            drawable.InstanceCount());

        if (drawable.ShadowsDoubleSided)
        {
            glEnable(GL_CULL_FACE);
        }
        glCullFace(GL_BACK);
    }

    void DrawMutableMeshDrawable(MutableMeshDrawable& drawable) const
    {
        if (drawable.Pending.empty() || drawable.Mesh.Indices.empty())
        {
            return;
        }

        glBindVertexArray(drawable.VertexArray_);
        glUniform1i(UseInstancesLocation_, GL_FALSE);

        if (drawable.ShadowsDoubleSided)
        {
            glDisable(GL_CULL_FACE);
        }
        else
        {
            glCullFace(GL_FRONT);
        }

        for (const MutableMeshDrawable::PendingDraw& draw : drawable.Pending)
        {
            glUniformMatrix4fv(ModelLocation_, 1, GL_FALSE, draw.Model.Data());
            glDrawElements(
                GL_TRIANGLES,
                static_cast<GLsizei>(drawable.Mesh.Indices.size() * 3),
                GL_UNSIGNED_INT,
                nullptr);
        }

        if (drawable.ShadowsDoubleSided)
        {
            glEnable(GL_CULL_FACE);
        }
        glCullFace(GL_BACK);
    }

private:
    void ComputeLightMatrices(
        const CameraState& camera,
        const CameraMatrices& cameraMatrices,
        int viewportWidth,
        int viewportHeight)
    {
        const float aspect = static_cast<float>(viewportWidth) / static_cast<float>(std::max(1, viewportHeight));

        for (int i = 0; i < CascadeCount; ++i)
        {
            const float nearPlane = i == 0 ? camera.NearPlane : CascadeSplits_[static_cast<std::size_t>(i - 1)];
            const float farPlane = i < static_cast<int>(CascadeSplits_.size())
                ? CascadeSplits_[static_cast<std::size_t>(i)]
                : camera.FarPlane;
            LightMatrices_[static_cast<std::size_t>(i)] =
                FitLightMatrixToFrustum(camera, cameraMatrices.View, aspect, nearPlane, farPlane);
        }
    }

    Mat4 FitLightMatrixToFrustum(
        const CameraState& camera,
        const Mat4& view,
        float aspect,
        float nearPlane,
        float farPlane)
    {
        const Mat4 projection = Perspective(camera.FieldOfView, aspect, nearPlane, farPlane);
        const std::array<Vec4, 8> corners = FrustumCorners(projection, view);

        Vec3 center {};
        for (const Vec4& corner : corners)
        {
            center = center + Vec3 {corner.X, corner.Y, corner.Z};
        }
        center = center * (1.0f / static_cast<float>(corners.size()));

        const Mat4 lightView = LookAt(center + LightDirection_, center, Vec3 {0.0f, 1.0f, 0.0f});

        Vec3 minimum {
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
        };
        Vec3 maximum {
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
        };

        for (const Vec4& corner : corners)
        {
            const Vec4 transformed = Transform(lightView, corner);
            minimum.X = std::min(minimum.X, transformed.X);
            minimum.Y = std::min(minimum.Y, transformed.Y);
            minimum.Z = std::min(minimum.Z, transformed.Z);
            maximum.X = std::max(maximum.X, transformed.X);
            maximum.Y = std::max(maximum.Y, transformed.Y);
            maximum.Z = std::max(maximum.Z, transformed.Z);
        }

        constexpr float zMultiplier = 3.0f;
        minimum.Z = minimum.Z < 0.0f ? minimum.Z * zMultiplier : minimum.Z / zMultiplier;
        maximum.Z = maximum.Z < 0.0f ? maximum.Z / zMultiplier : maximum.Z * zMultiplier;

        return Multiply(
            OrthographicOffCenter(minimum.X, maximum.X, minimum.Y, maximum.Y, minimum.Z, maximum.Z),
            lightView);
    }

    static std::array<Vec4, 8> FrustumCorners(const Mat4& projection, const Mat4& view)
    {
        Mat4 inverse {};
        Check(Invert(Multiply(projection, view), inverse), "Could not invert projection * view matrix.");

        std::array<Vec4, 8> corners {};
        for (int x = 0; x < 2; ++x)
        {
            for (int y = 0; y < 2; ++y)
            {
                for (int z = 0; z < 2; ++z)
                {
                    Vec4 point {
                        2.0f * static_cast<float>(x) - 1.0f,
                        2.0f * static_cast<float>(y) - 1.0f,
                        2.0f * static_cast<float>(z) - 1.0f,
                        1.0f,
                    };
                    point = Transform(inverse, point);
                    corners[static_cast<std::size_t>(4 * x + 2 * y + z)] = point * (1.0f / point.W);
                }
            }
        }
        return corners;
    }

    static constexpr const char* ShadowVertexShaderSource = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 3) in mat4 aInstanceModel;

uniform mat4 uLightMatrix;
uniform mat4 uModel;
uniform bool uUseInstances;

void main()
{
    mat4 model = uUseInstances ? aInstanceModel : uModel;
    gl_Position = uLightMatrix * model * vec4(aPos, 1.0);
}
)GLSL";

    static constexpr const char* ShadowFragmentShaderSource = R"GLSL(#version 330 core
void main() { }
)GLSL";

    GLuint ShaderProgram_ = 0;
    std::array<GLuint, CascadeCount> Framebuffers_ {};
    std::array<Texture2D, CascadeCount> DepthMaps_;
    std::array<Mat4, CascadeCount> LightMatrices_ {};
    std::array<float, 2> CascadeSplits_ {20.0f, 60.0f};
    Vec3 LightDirection_ = Normalize(Vec3 {1.0f, 2.0f, 1.0f});
    GLint LightMatrixLocation_ = -1;
    GLint ModelLocation_ = -1;
    GLint UseInstancesLocation_ = -1;
};
