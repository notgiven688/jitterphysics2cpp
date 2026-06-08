constexpr const char* VertexShaderSource = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in mat4 aInstanceModel;
layout(location = 7) in vec3 aInstanceColor;

uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uModel;
uniform bool uUseInstances;
uniform vec3 uUniformInstanceColor;

out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec2 vTexCoord;
out vec3 vInstanceColor;

void main()
{
    mat4 model = uUseInstances ? aInstanceModel : uModel;
    vec4 wp = model * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    vWorldNormal = normalize(mat3(transpose(inverse(model))) * aNormal);
    vTexCoord = vec2(aTexCoord.x, 1.0 - aTexCoord.y);
    vInstanceColor = uUseInstances ? aInstanceColor : uUniformInstanceColor;
    gl_Position = uProjection * uView * wp;
}
)GLSL";

constexpr const char* FragmentShaderSource = R"GLSL(#version 330 core
struct Material {
    vec3 tint;
    vec3 specular;
    float shininess;
    float alpha;
    float vertexColorWeight;
    float textureWeight;
    float normalFlip;
};

uniform Material uMat;
uniform vec3 uViewPos;
uniform mat4 uView;
uniform vec3 uSunDir;

uniform mat4 uLightMatrices[3];
uniform float uCascadeSplits[3];

uniform sampler2D uShadowNear;
uniform sampler2D uShadowMid;
uniform sampler2D uShadowFar;
uniform sampler2D uDiffuse;

in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec3 vInstanceColor;

out vec4 FragColor;

float sampleShadow(mat4 lightMatrix, sampler2D shadowmap, vec3 n)
{
    float bias = max(0.6 * (0.4 - dot(n, uSunDir)), 0.0001);
    vec4 lp = lightMatrix * vec4(vWorldPos, 1.0);
    vec3 proj = lp.xyz / lp.w * 0.5 + 0.5;

    if (proj.z > 1.0) return 0.0;

    float mapped = texture(shadowmap, proj.xy).r;
    if (proj.z - bias > mapped)
    {
        return abs(dot(n, uSunDir));
    }
    return 0.0;
}

float shadowForPixel(vec3 n)
{
    float viewDepth = abs((uView * vec4(vWorldPos, 1.0)).z);
    if (viewDepth < uCascadeSplits[0]) return sampleShadow(uLightMatrices[0], uShadowNear, n);
    if (viewDepth < uCascadeSplits[1]) return sampleShadow(uLightMatrices[1], uShadowMid, n);
    return sampleShadow(uLightMatrices[2], uShadowFar, n);
}

void main()
{
    vec3 n = vWorldNormal * uMat.normalFlip;

    vec3 ambient = uMat.vertexColorWeight * vInstanceColor + (1.0 - uMat.vertexColorWeight) * uMat.tint;
    vec3 diffuseBase = mix(vec3(0.6), texture(uDiffuse, vTexCoord).rgb, uMat.textureWeight);

    vec3 lightDirs[4];
    float lightStrengths[4];
    lightDirs[0] = uSunDir;          lightStrengths[0] = 1.0;
    lightDirs[1] = vec3(-1, 0,  1);  lightStrengths[1] = 0.2;
    lightDirs[2] = vec3( 0, 0, -1);  lightStrengths[2] = 0.2;
    lightDirs[3] = vec3(-1, 0, -1);  lightStrengths[3] = 0.2;

    vec3 diffusive = vec3(0);
    for (int i = 0; i < 4; i++)
    {
        vec3 ld = normalize(lightDirs[i]);
        float d = max(dot(n, ld), 0.0);
        diffusive += lightStrengths[i] * d * diffuseBase;
    }

    vec3 viewDir = normalize(uViewPos - vWorldPos);
    vec3 halfway = normalize(normalize(lightDirs[0]) + viewDir);
    float specFactor = pow(max(dot(viewDir, halfway), 0.0), uMat.shininess);
    vec3 specular = specFactor * uMat.specular;

    float shadow = shadowForPixel(n);
    vec3 color = ambient + (diffusive + 0.4 * specular) * (1.0 - shadow);

    FragColor = vec4(color, uMat.alpha);
}
)GLSL";

Color ShapeColor(const Jitter2::RigidBody& body, const Shapes::RigidBodyShape& shape)
{
    const bool customSupportMap =
        dynamic_cast<const EllipsoidShape*>(&shape) != nullptr
        || dynamic_cast<const DoubleSphereShape*>(&shape) != nullptr
        || dynamic_cast<const IcosahedronShape*>(&shape) != nullptr;
    const auto seed = static_cast<int>(customSupportMap
        ? std::hash<const Jitter2::RigidBody*>{}(&body)
        : std::hash<const Shapes::RigidBodyShape*>{}(&shape));
    Vec3 color = ColorGenerator::GetColor(seed);
    if (!body.IsActive())
    {
        color = color + Vec3 {0.2f, 0.2f, 0.2f};
    }
    return Color {color.X, color.Y, color.Z};
}

class Renderer
{
public:
    Renderer()
        : ShaderProgram_(CreateShaderProgram(VertexShaderSource, FragmentShaderSource)),
          Cube_(CreateCubeMesh()),
          Sphere_(CreateSphereMesh()),
          Cylinder_(CreateCylinderMesh()),
          HalfSphere_(CreateHalfSphereMesh()),
          Cone_(CreateConeMesh()),
          Icosahedron_(CreateIcosahedronMesh()),
          CustomEllipsoid_(CreateSupportMapMesh<EllipsoidShape>()),
          CustomDoubleSphere_(CreateSupportMapMesh<DoubleSphereShape>()),
          CustomIcosahedron_(CreateSupportMapMesh<IcosahedronShape>())
    {
        const Material customSupportMapMaterial = CreateCustomSupportMapMaterial();
        CustomEllipsoid_.MaterialValue = customSupportMapMaterial;
        CustomDoubleSphere_.MaterialValue = customSupportMapMaterial;
        CustomIcosahedron_.MaterialValue = customSupportMapMaterial;

        ModelLocation_ = glGetUniformLocation(ShaderProgram_, "uModel");
        ViewLocation_ = glGetUniformLocation(ShaderProgram_, "uView");
        ProjectionLocation_ = glGetUniformLocation(ShaderProgram_, "uProjection");
        ColorLocation_ = glGetUniformLocation(ShaderProgram_, "uUniformInstanceColor");
        UseInstancesLocation_ = glGetUniformLocation(ShaderProgram_, "uUseInstances");
        ViewPositionLocation_ = glGetUniformLocation(ShaderProgram_, "uViewPos");
        SunDirectionLocation_ = glGetUniformLocation(ShaderProgram_, "uSunDir");
        LightMatrixLocations_[0] = glGetUniformLocation(ShaderProgram_, "uLightMatrices[0]");
        LightMatrixLocations_[1] = glGetUniformLocation(ShaderProgram_, "uLightMatrices[1]");
        LightMatrixLocations_[2] = glGetUniformLocation(ShaderProgram_, "uLightMatrices[2]");
        CascadeSplitsLocation_ = glGetUniformLocation(ShaderProgram_, "uCascadeSplits");
        MaterialTintLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.tint");
        MaterialSpecularLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.specular");
        MaterialShininessLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.shininess");
        MaterialAlphaLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.alpha");
        MaterialVertexColorWeightLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.vertexColorWeight");
        MaterialTextureWeightLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.textureWeight");
        MaterialNormalFlipLocation_ = glGetUniformLocation(ShaderProgram_, "uMat.normalFlip");
        ShadowNearTextureLocation_ = glGetUniformLocation(ShaderProgram_, "uShadowNear");
        ShadowMidTextureLocation_ = glGetUniformLocation(ShaderProgram_, "uShadowMid");
        ShadowFarTextureLocation_ = glGetUniformLocation(ShaderProgram_, "uShadowFar");
        DiffuseTextureLocation_ = glGetUniformLocation(ShaderProgram_, "uDiffuse");
        glUseProgram(ShaderProgram_);
        glUniform1i(ShadowNearTextureLocation_, 0);
        glUniform1i(ShadowMidTextureLocation_, 1);
        glUniform1i(ShadowFarTextureLocation_, 2);
        glUniform1i(DiffuseTextureLocation_, 3);
        glUseProgram(0);
    }

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    ~Renderer()
    {
        if (ShaderProgram_ != 0)
        {
            glDeleteProgram(ShaderProgram_);
        }
    }

    void RenderWorld(
        const Jitter2::World& world,
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        bool wireframe,
        const Shapes::RigidBodyShape* floorShape = nullptr)
    {
        QueueWorld(world, floorShape);
        RenderQueuedLit(cameraMatrices, cameraPosition, nullptr, wireframe);
    }

    void QueueWorld(const Jitter2::World& world, const Shapes::RigidBodyShape* floorShape = nullptr)
    {
        for (const Jitter2::RigidBody* body : world.RigidBodies())
        {
            if (body == &world.NullBody())
            {
                continue;
            }
            if (const auto* tag = std::any_cast<RigidBodyTag>(&body->Tag);
                tag != nullptr && tag->DoNotDraw)
            {
                continue;
            }

            for (const Shapes::RigidBodyShape* shape : body->Shapes())
            {
                if (shape == floorShape)
                {
                    continue;
                }

                PushShape(*body, *shape);
            }
        }
    }

    void RenderQueuedShadow(ShadowCaster& shadowCaster)
    {
        shadowCaster.DrawInstancedDrawable(Cube_);
        shadowCaster.DrawInstancedDrawable(Sphere_);
        shadowCaster.DrawInstancedDrawable(Cylinder_);
        shadowCaster.DrawInstancedDrawable(HalfSphere_);
        shadowCaster.DrawInstancedDrawable(Cone_);
        shadowCaster.DrawInstancedDrawable(Icosahedron_);
        shadowCaster.DrawInstancedDrawable(CustomEllipsoid_);
        shadowCaster.DrawInstancedDrawable(CustomDoubleSphere_);
        shadowCaster.DrawInstancedDrawable(CustomIcosahedron_);
    }

    void RenderQueuedLit(
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        const ShadowCaster* shadowCaster,
        bool wireframe)
    {
        glUseProgram(ShaderProgram_);
        BeginLitPass(cameraMatrices, cameraPosition, shadowCaster);

        if (wireframe)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }

        DrawInstancedDrawable(Cube_);
        DrawInstancedDrawable(Sphere_);
        DrawInstancedDrawable(Cylinder_);
        DrawInstancedDrawable(HalfSphere_);
        DrawInstancedDrawable(Cone_);
        DrawInstancedDrawable(Icosahedron_);
        DrawInstancedDrawable(CustomEllipsoid_);
        DrawInstancedDrawable(CustomDoubleSphere_);
        DrawInstancedDrawable(CustomIcosahedron_);

        if (wireframe)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        glBindVertexArray(0);
        glUseProgram(0);
    }

    void RenderMutableMeshDrawable(
        MutableMeshDrawable& drawable,
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        const ShadowCaster* shadowCaster = nullptr)
    {
        if (drawable.Pending.empty() || drawable.Mesh.Indices.empty())
        {
            return;
        }

        glUseProgram(ShaderProgram_);
        BeginLitPass(cameraMatrices, cameraPosition, shadowCaster);
        glUniform1i(UseInstancesLocation_, GL_FALSE);
        ApplyMaterial(drawable.DrawableMaterial);
        glBindVertexArray(drawable.VertexArray_);

        const auto drawRange =
            [&drawable](int from, int to)
            {
                if (to <= from)
                {
                    return;
                }

                glDrawElements(
                    GL_TRIANGLES,
                    static_cast<GLsizei>((to - from) * 3),
                    GL_UNSIGNED_INT,
                    reinterpret_cast<void*>(static_cast<std::uintptr_t>(from * 3 * sizeof(unsigned int))));
            };

        for (const MutableMeshDrawable::PendingDraw& draw : drawable.Pending)
        {
            glUniformMatrix4fv(ModelLocation_, 1, GL_FALSE, draw.Model.Data());
            glUniform3f(ColorLocation_, draw.InstanceColor.X, draw.InstanceColor.Y, draw.InstanceColor.Z);

            if (!drawable.Groups.empty() && !drawable.MeshGroups.empty())
            {
                ApplyMaterial(drawable.DrawableMaterial);
                int cursor = 0;
                for (const MaterialSlot& slot : drawable.Groups)
                {
                    if (slot.GroupIndex < 0
                        || slot.GroupIndex >= static_cast<int>(drawable.MeshGroups.size()))
                    {
                        continue;
                    }

                    const MeshGroup& group = drawable.MeshGroups[static_cast<std::size_t>(slot.GroupIndex)];
                    if (group.FromInclusive > cursor)
                    {
                        drawRange(cursor, group.FromInclusive);
                    }
                    cursor = group.ToExclusive;
                }
                if (cursor < static_cast<int>(drawable.Mesh.Indices.size()))
                {
                    drawRange(cursor, static_cast<int>(drawable.Mesh.Indices.size()));
                }

                for (const MaterialSlot& slot : drawable.Groups)
                {
                    if (slot.GroupIndex < 0
                        || slot.GroupIndex >= static_cast<int>(drawable.MeshGroups.size()))
                    {
                        continue;
                    }

                    ApplyMaterial(slot.MaterialValue);
                    const MeshGroup& group = drawable.MeshGroups[static_cast<std::size_t>(slot.GroupIndex)];
                    drawRange(group.FromInclusive, group.ToExclusive);
                }
            }
            else
            {
                ApplyMaterial(drawable.DrawableMaterial);
                glDrawElements(
                    GL_TRIANGLES,
                    static_cast<GLsizei>(drawable.Mesh.Indices.size() * 3),
                    GL_UNSIGNED_INT,
                    nullptr);
            }

            if (drawable.TwoSided)
            {
                Material flipped = drawable.DrawableMaterial;
                flipped.FlipNormals = !flipped.FlipNormals;
                ApplyMaterial(flipped);
                glCullFace(GL_FRONT);
                drawRange(0, static_cast<int>(drawable.Mesh.Indices.size()));
                glCullFace(GL_BACK);
            }
        }

        drawable.Pending.clear();
        glBindVertexArray(0);
        glUseProgram(0);
    }

    void RenderInstancedDrawable(
        InstancedDrawable& drawable,
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        const ShadowCaster* shadowCaster = nullptr)
    {
        if (drawable.InstanceCount() == 0)
        {
            return;
        }

        glUseProgram(ShaderProgram_);
        BeginLitPass(cameraMatrices, cameraPosition, shadowCaster);
        DrawInstancedDrawable(drawable);
        glBindVertexArray(0);
        glUseProgram(0);
    }

private:
    void BeginLitPass(
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        const ShadowCaster* shadowCaster) const
    {
        SetCameraMatrices(cameraMatrices);
        glUniform3f(ViewPositionLocation_, cameraPosition.X, cameraPosition.Y, cameraPosition.Z);
        const Vec3 sunDirection = shadowCaster != nullptr
            ? shadowCaster->LightDirection()
            : Normalize(Vec3 {1.0f, 2.0f, 1.0f});
        glUniform3f(SunDirectionLocation_, sunDirection.X, sunDirection.Y, sunDirection.Z);

        if (shadowCaster != nullptr)
        {
            const std::array<Mat4, ShadowCaster::CascadeCount>& lightMatrices = shadowCaster->LightMatrices();
            for (int i = 0; i < ShadowCaster::CascadeCount; ++i)
            {
                glUniformMatrix4fv(
                    LightMatrixLocations_[static_cast<std::size_t>(i)],
                    1,
                    GL_FALSE,
                    lightMatrices[static_cast<std::size_t>(i)].Data());
            }

            const std::array<float, ShadowCaster::CascadeCount> cascadeSplits =
                shadowCaster->CascadeSplitsForShader();
            glUniform1fv(
                CascadeSplitsLocation_,
                static_cast<GLsizei>(cascadeSplits.size()),
                cascadeSplits.data());
            shadowCaster->BindDepthMaps();
        }
        else
        {
            const Mat4 identity = Identity();
            for (GLint location : LightMatrixLocations_)
            {
                glUniformMatrix4fv(location, 1, GL_FALSE, identity.Data());
            }

            const std::array<float, 3> cascadeSplits {
                20.0f,
                60.0f,
                std::numeric_limits<float>::max(),
            };
            glUniform1fv(CascadeSplitsLocation_, static_cast<GLsizei>(cascadeSplits.size()), cascadeSplits.data());

            const std::shared_ptr<Texture2D> shadowFallback = Texture2D::White();
            shadowFallback->Bind(GL_TEXTURE0);
            shadowFallback->Bind(GL_TEXTURE1);
            shadowFallback->Bind(GL_TEXTURE2);
        }
    }

    void SetCameraMatrices(const CameraMatrices& cameraMatrices) const
    {
        glUniformMatrix4fv(ProjectionLocation_, 1, GL_FALSE, cameraMatrices.Projection.Data());
        glUniformMatrix4fv(ViewLocation_, 1, GL_FALSE, cameraMatrices.View.Data());
    }

    void PushMesh(InstancedDrawable& drawable, const Mat4& model, Color color)
    {
        drawable.Push(model, Vec3 {color.R, color.G, color.B});
    }

    void DrawInstancedDrawable(InstancedDrawable& drawable)
    {
        if (drawable.InstanceCount() == 0)
        {
            return;
        }

        drawable.UploadInstances();
        drawable.Bind();
        glUniform1i(UseInstancesLocation_, GL_TRUE);

        const auto drawRange =
            [&drawable](int from, int to)
            {
                if (to <= from)
                {
                    return;
                }

                glDrawElementsInstanced(
                    GL_TRIANGLES,
                    static_cast<GLsizei>((to - from) * 3),
                    GL_UNSIGNED_INT,
                    reinterpret_cast<void*>(static_cast<std::uintptr_t>(from * 3 * sizeof(unsigned int))),
                    drawable.InstanceCount());
            };

        if (!drawable.Groups.empty() && !drawable.MeshGroups.empty())
        {
            ApplyMaterial(drawable.MaterialValue);
            int cursor = 0;
            for (const MaterialSlot& slot : drawable.Groups)
            {
                if (slot.GroupIndex < 0
                    || slot.GroupIndex >= static_cast<int>(drawable.MeshGroups.size()))
                {
                    continue;
                }

                const MeshGroup& group = drawable.MeshGroups[static_cast<std::size_t>(slot.GroupIndex)];
                if (group.FromInclusive > cursor)
                {
                    drawRange(cursor, group.FromInclusive);
                }
                cursor = group.ToExclusive;
            }
            if (cursor < drawable.IndexCount() / 3)
            {
                drawRange(cursor, drawable.IndexCount() / 3);
            }

            for (const MaterialSlot& slot : drawable.Groups)
            {
                if (slot.GroupIndex < 0
                    || slot.GroupIndex >= static_cast<int>(drawable.MeshGroups.size()))
                {
                    continue;
                }

                ApplyMaterial(slot.MaterialValue);
                const MeshGroup& group = drawable.MeshGroups[static_cast<std::size_t>(slot.GroupIndex)];
                drawRange(group.FromInclusive, group.ToExclusive);
            }
        }
        else
        {
            ApplyMaterial(drawable.MaterialValue);
            glDrawElementsInstanced(
                GL_TRIANGLES,
                drawable.IndexCount(),
                GL_UNSIGNED_INT,
                nullptr,
                drawable.InstanceCount());
        }

        if (drawable.TwoSided)
        {
            Material flipped = drawable.MaterialValue;
            flipped.FlipNormals = !flipped.FlipNormals;
            ApplyMaterial(flipped);
            glCullFace(GL_FRONT);
            glDrawElementsInstanced(
                GL_TRIANGLES,
                drawable.IndexCount(),
                GL_UNSIGNED_INT,
                nullptr,
                drawable.InstanceCount());
            glCullFace(GL_BACK);
        }

        drawable.Clear();
    }

    void ApplyMaterial(const Material& material)
    {
        const std::shared_ptr<Texture2D> texture = material.Texture ? material.Texture : Texture2D::Empty();
        texture->Bind(GL_TEXTURE3);

        glUniform3f(MaterialTintLocation_, material.Tint.X, material.Tint.Y, material.Tint.Z);
        glUniform3f(MaterialSpecularLocation_, material.Specular.X, material.Specular.Y, material.Specular.Z);
        glUniform1f(MaterialShininessLocation_, material.Shininess);
        glUniform1f(MaterialAlphaLocation_, material.Alpha);
        glUniform1f(MaterialVertexColorWeightLocation_, material.VertexColorWeight);
        glUniform1f(MaterialTextureWeightLocation_, material.TextureWeight);
        glUniform1f(MaterialNormalFlipLocation_, material.FlipNormals ? -1.0f : 1.0f);
    }

    void PushShape(const Jitter2::RigidBody& body, const Shapes::RigidBodyShape& shape)
    {
        const Color color = ShapeColor(body, shape);
        PushShapeMatrix(shape, BodyTransform(body.Position(), body.Orientation()), color);
    }

    void PushShapeMatrix(
        const Shapes::RigidBodyShape& shape,
        const Mat4& transform,
        Color color)
    {
        if (const auto* transformed = dynamic_cast<const Shapes::TransformedShape*>(&shape))
        {
            PushShapeMatrix(
                transformed->OriginalShape(),
                Multiply(
                    Multiply(transform, Translation(transformed->Translation())),
                    FromJitter(transformed->Transformation())),
                color);
            return;
        }

        if (const auto* box = dynamic_cast<const Shapes::BoxShape*>(&shape))
        {
            PushMesh(Cube_, Multiply(transform, Scale(box->Size())), color);
            return;
        }

        if (const auto* sphere = dynamic_cast<const Shapes::SphereShape*>(&shape))
        {
            PushMesh(Sphere_, Multiply(transform, Scale(static_cast<float>(sphere->Radius() * 2))), color);
            return;
        }

        if (const auto* capsule = dynamic_cast<const Shapes::CapsuleShape*>(&shape))
        {
            const Jitter2::Real radius = capsule->Radius();
            const Jitter2::Real length = capsule->Length();
            PushMesh(Cylinder_, Multiply(transform, Scale(JVector(radius, length, radius))), color);

            const Mat4 cap = Multiply(
                Translation(JVector(0, length * static_cast<Jitter2::Real>(0.5), 0)),
                Scale(static_cast<float>(radius * 2)));
            PushMesh(HalfSphere_, Multiply(transform, cap), color);
            PushMesh(HalfSphere_,
                Multiply(Multiply(transform, RotationX(Pi)), cap),
                color);
            return;
        }

        if (const auto* cylinder = dynamic_cast<const Shapes::CylinderShape*>(&shape))
        {
            PushMesh(Cylinder_,
                Multiply(transform, Scale(
                    JVector(cylinder->Radius(), cylinder->Height(), cylinder->Radius()))),
                color);
            return;
        }

        if (const auto* cone = dynamic_cast<const Shapes::ConeShape*>(&shape))
        {
            PushMesh(Cone_,
                Multiply(transform, Scale(
                    JVector(
                        cone->Radius() * static_cast<Jitter2::Real>(2),
                        cone->Height(),
                        cone->Radius() * static_cast<Jitter2::Real>(2)))),
                color);
            return;
        }

        if (dynamic_cast<const EllipsoidShape*>(&shape) != nullptr)
        {
            PushMesh(CustomEllipsoid_, transform, color);
            return;
        }

        if (dynamic_cast<const DoubleSphereShape*>(&shape) != nullptr)
        {
            PushMesh(CustomDoubleSphere_, transform, color);
            return;
        }

        if (dynamic_cast<const IcosahedronShape*>(&shape) != nullptr)
        {
            PushMesh(CustomIcosahedron_, transform, color);
        }
    }

    GLuint ShaderProgram_ = 0;
    InstancedDrawable Cube_;
    InstancedDrawable Sphere_;
    InstancedDrawable Cylinder_;
    InstancedDrawable HalfSphere_;
    InstancedDrawable Cone_;
    InstancedDrawable Icosahedron_;
    InstancedDrawable CustomEllipsoid_;
    InstancedDrawable CustomDoubleSphere_;
    InstancedDrawable CustomIcosahedron_;
    GLint ModelLocation_ = -1;
    GLint ViewLocation_ = -1;
    GLint ProjectionLocation_ = -1;
    GLint ColorLocation_ = -1;
    GLint UseInstancesLocation_ = -1;
    GLint ViewPositionLocation_ = -1;
    GLint SunDirectionLocation_ = -1;
    std::array<GLint, 3> LightMatrixLocations_ {-1, -1, -1};
    GLint CascadeSplitsLocation_ = -1;
    GLint MaterialTintLocation_ = -1;
    GLint MaterialSpecularLocation_ = -1;
    GLint MaterialShininessLocation_ = -1;
    GLint MaterialAlphaLocation_ = -1;
    GLint MaterialVertexColorWeightLocation_ = -1;
    GLint MaterialTextureWeightLocation_ = -1;
    GLint MaterialNormalFlipLocation_ = -1;
    GLint ShadowNearTextureLocation_ = -1;
    GLint ShadowMidTextureLocation_ = -1;
    GLint ShadowFarTextureLocation_ = -1;
    GLint DiffuseTextureLocation_ = -1;
};
