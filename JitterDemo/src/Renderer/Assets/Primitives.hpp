void AddFace(
    std::vector<Vertex>& vertices,
    std::vector<unsigned int>& indices,
    Vec3 normal)
{
    const Vec3 s1 {normal.Y, normal.Z, normal.X};
    const Vec3 s2 = Cross(normal, s1);
    const auto base = static_cast<unsigned int>(vertices.size());
    vertices.push_back(Vertex {(normal - s1 - s2) * 0.5f, normal});
    vertices.push_back(Vertex {(normal - s1 + s2) * 0.5f, normal});
    vertices.push_back(Vertex {(normal + s1 + s2) * 0.5f, normal});
    vertices.push_back(Vertex {(normal + s1 - s2) * 0.5f, normal});
    indices.insert(indices.end(), {base + 1, base + 0, base + 2, base + 2, base + 0, base + 3});
}

Mesh CreateCubeMesh()
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(24);
    indices.reserve(36);

    AddFace(vertices, indices, Vec3 {0.0f, 0.0f, 1.0f});
    AddFace(vertices, indices, Vec3 {0.0f, 0.0f, -1.0f});
    AddFace(vertices, indices, Vec3 {1.0f, 0.0f, 0.0f});
    AddFace(vertices, indices, Vec3 {-1.0f, 0.0f, 0.0f});
    AddFace(vertices, indices, Vec3 {0.0f, 1.0f, 0.0f});
    AddFace(vertices, indices, Vec3 {0.0f, -1.0f, 0.0f});

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Mesh CreateQuadMesh(float halfSize, float uvScale = 1.0f)
{
    std::vector<Vertex> vertices;
    vertices.reserve(4);
    vertices.push_back(Vertex {Vec3 {-halfSize, 0.0f, -halfSize}, Vec3 {0.0f, 1.0f, 0.0f}, Vec2 {0.0f, 0.0f}});
    vertices.push_back(Vertex {Vec3 {-halfSize, 0.0f, +halfSize}, Vec3 {0.0f, 1.0f, 0.0f}, Vec2 {0.0f, uvScale}});
    vertices.push_back(Vertex {Vec3 {+halfSize, 0.0f, -halfSize}, Vec3 {0.0f, 1.0f, 0.0f}, Vec2 {uvScale, 0.0f}});
    vertices.push_back(Vertex {Vec3 {+halfSize, 0.0f, +halfSize}, Vec3 {0.0f, 1.0f, 0.0f}, Vec2 {uvScale, uvScale}});

    std::vector<unsigned int> indices {
        0, 1, 2,
        2, 1, 3,
    };

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

std::shared_ptr<Texture2D> UnitTexture()
{
    static std::shared_ptr<Texture2D> texture = []
    {
        int width = 0;
        int height = 0;
        auto loaded = std::make_shared<Texture2D>();
        std::vector<unsigned char> pixels = LoadTgaBgra("assets/unit.tga", width, height);
        loaded->LoadImage(pixels, width, height);
        loaded->SetWrap(GL_REPEAT);
        loaded->SetAnisotropy(8.0f);
        return loaded;
    }();
    return texture;
}

Material CreateFloorMaterial()
{
    Material material = Material::Default();
    material.Tint = Vec3 {};
    material.Specular = Vec3 {0.1f, 0.1f, 0.1f};
    material.Shininess = 10.0f;
    material.Alpha = 1.0f;
    material.VertexColorWeight = 0.0f;
    material.TextureWeight = 1.0f;
    material.Texture = UnitTexture();
    return material;
}

Mesh CreateSphereMesh(int tesselation = 20)
{
    const int t = tesselation;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(static_cast<std::size_t>(t * t + 2));
    indices.reserve(static_cast<std::size_t>(t * ((t - 1) * 2 + 2) * 3));

    for (int e = 0; e < t; ++e)
    {
        for (int i = 0; i < t; ++i)
        {
            const double alpha = 2.0 * Pi / static_cast<double>(t) * static_cast<double>(i);
            const double beta = Pi / static_cast<double>(t + 1) * static_cast<double>(e + 1);

            Vec3 v {
                static_cast<float>(std::cos(alpha) * std::sin(beta)),
                static_cast<float>(std::cos(beta)),
                static_cast<float>(std::sin(alpha) * std::sin(beta)),
            };

            vertices.push_back(Vertex {v * 0.5f, v});
        }
    }

    vertices.push_back(Vertex {Vec3 {0.0f, -0.5f, 0.0f}, Vec3 {0.0f, -1.0f, 0.0f}});
    vertices.push_back(Vertex {Vec3 {0.0f, +0.5f, 0.0f}, Vec3 {0.0f, +1.0f, 0.0f}});

    for (int e = 0; e < t - 1; ++e)
    {
        for (int i = 0; i < t; ++i)
        {
            indices.push_back(static_cast<unsigned int>(e * t + i));
            indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>((e + 1) * t + i));

            indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>((e + 1) * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>((e + 1) * t + i));
        }
    }

    for (int i = 0; i < t; ++i)
    {
        const int e = t - 1;
        indices.push_back(static_cast<unsigned int>(e * t + i));
        indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
        indices.push_back(static_cast<unsigned int>(t * t + 0));

        indices.push_back(static_cast<unsigned int>((i + 1) % t));
        indices.push_back(static_cast<unsigned int>(i));
        indices.push_back(static_cast<unsigned int>(t * t + 1));
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Mesh CreateHalfSphereMesh(int tesselation = 20)
{
    const int t = tesselation;
    const int halfT = t / 2;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(static_cast<std::size_t>(halfT * t + 1));
    indices.reserve(static_cast<std::size_t>(t * ((halfT - 1) * 2 + 1) * 3));

    for (int e = 0; e < halfT; ++e)
    {
        for (int i = 0; i < t; ++i)
        {
            const double alpha = 2.0 * Pi / static_cast<double>(t) * static_cast<double>(i);
            const double beta = Pi / static_cast<double>(t) * static_cast<double>(e);

            Vec3 v {
                static_cast<float>(std::cos(alpha) * std::cos(beta)),
                static_cast<float>(std::sin(beta)),
                static_cast<float>(std::sin(alpha) * std::cos(beta)),
            };

            vertices.push_back(Vertex {v * 0.5f, v});
        }
    }

    vertices.push_back(Vertex {Vec3 {0.0f, +0.5f, 0.0f}, Vec3 {0.0f, +1.0f, 0.0f}});

    for (int e = 0; e < halfT - 1; ++e)
    {
        for (int i = 0; i < t; ++i)
        {
            indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>(e * t + i));
            indices.push_back(static_cast<unsigned int>((e + 1) * t + i));

            indices.push_back(static_cast<unsigned int>((e + 1) * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
            indices.push_back(static_cast<unsigned int>((e + 1) * t + i));
        }
    }

    for (int i = 0; i < t; ++i)
    {
        const int e = halfT - 1;
        indices.push_back(static_cast<unsigned int>(e * t + (i + 1) % t));
        indices.push_back(static_cast<unsigned int>(e * t + i));
        indices.push_back(static_cast<unsigned int>(halfT * t));
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Mesh CreateCylinderMesh(int tesselation = 20)
{
    const int t = tesselation;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(static_cast<std::size_t>(2 + 4 * t));
    indices.reserve(static_cast<std::size_t>(4 * t * 3));

    vertices.push_back(Vertex {Vec3 {0.0f, -0.5f, 0.0f}, Vec3 {0.0f, -1.0f, 0.0f}});
    vertices.push_back(Vertex {Vec3 {0.0f, +0.5f, 0.0f}, Vec3 {0.0f, +1.0f, 0.0f}});

    for (int i = 0; i < t; ++i)
    {
        const double alpha = 2.0 * Pi / static_cast<double>(t) * static_cast<double>(i);
        Vec3 v {
            static_cast<float>(std::cos(alpha)),
            0.0f,
            static_cast<float>(std::sin(alpha)),
        };

        vertices.push_back(Vertex {v + Vec3 {0.0f, -0.5f, 0.0f}, Vec3 {0.0f, -1.0f, 0.0f}});
        vertices.push_back(Vertex {v + Vec3 {0.0f, +0.5f, 0.0f}, Vec3 {0.0f, +1.0f, 0.0f}});
        vertices.push_back(Vertex {v + Vec3 {0.0f, -0.5f, 0.0f}, v});
        vertices.push_back(Vertex {v + Vec3 {0.0f, +0.5f, 0.0f}, v});
    }

    const int t4 = 4 * t;
    for (int i = 0; i < t; ++i)
    {
        indices.push_back(0);
        indices.push_back(static_cast<unsigned int>(2 + 4 * i));
        indices.push_back(static_cast<unsigned int>(2 + (4 * i + 4) % t4));

        indices.push_back(static_cast<unsigned int>(3 + 4 * i));
        indices.push_back(1);
        indices.push_back(static_cast<unsigned int>(3 + (4 * i + 4) % t4));

        indices.push_back(static_cast<unsigned int>(4 + 4 * i));
        indices.push_back(static_cast<unsigned int>(5 + 4 * i));
        indices.push_back(static_cast<unsigned int>(4 + (4 * i + 4) % t4));

        indices.push_back(static_cast<unsigned int>(5 + 4 * i));
        indices.push_back(static_cast<unsigned int>(5 + (4 * i + 4) % t4));
        indices.push_back(static_cast<unsigned int>(4 + (4 * i + 4) % t4));
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Mesh CreateConeMesh(int tesselation = 20)
{
    const int t = tesselation;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(static_cast<std::size_t>(1 + 3 * t));
    indices.reserve(static_cast<std::size_t>(2 * t * 3));

    constexpr float bottomY = -0.25f;
    constexpr float topY = 0.75f;
    const auto spherical =
        [](double alpha, double beta)
        {
            return Vec3 {
                static_cast<float>(std::cos(alpha) * std::sin(beta)),
                static_cast<float>(std::sin(beta)),
                static_cast<float>(std::sin(alpha) * std::sin(beta)),
            };
        };

    vertices.push_back(Vertex {Vec3 {0.0f, bottomY, 0.0f}, Vec3 {0.0f, -1.0f, 0.0f}});

    for (int i = 0; i < t; ++i)
    {
        const double alpha1 = 2.0 * Pi / static_cast<double>(t) * static_cast<double>(i);
        const double alpha2 = alpha1 + Pi / static_cast<double>(t);
        const double beta = 2.0 / std::sqrt(5.0);

        Vec3 v1 {
            0.5f * static_cast<float>(std::cos(alpha1)),
            0.0f,
            0.5f * static_cast<float>(std::sin(alpha1)),
        };

        const Vec3 n2 = spherical(alpha1, beta);
        const Vec3 n3 = spherical(alpha2, beta);

        vertices.push_back(Vertex {Vec3 {v1.X, bottomY, v1.Z}, Vec3 {0.0f, -1.0f, 0.0f}});
        vertices.push_back(Vertex {Vec3 {v1.X, bottomY, v1.Z}, n2});
        vertices.push_back(Vertex {Vec3 {0.0f, topY, 0.0f}, n3});
    }

    const int t3 = 3 * t;
    for (int i = 0; i < t; ++i)
    {
        indices.push_back(0);
        indices.push_back(static_cast<unsigned int>(1 + 3 * i));
        indices.push_back(static_cast<unsigned int>(1 + (3 * i + 3) % t3));

        indices.push_back(static_cast<unsigned int>(2 + 3 * i));
        indices.push_back(static_cast<unsigned int>(3 + 3 * i));
        indices.push_back(static_cast<unsigned int>(2 + (3 * i + 3) % t3));
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Mesh CreateIcosahedronMesh()
{
    static constexpr std::array<std::array<int, 3>, 20> faces {{
        {{0, 1, 8}}, {{0, 10, 1}}, {{0, 8, 4}}, {{0, 6, 10}}, {{0, 4, 6}},
        {{1, 5, 8}}, {{1, 10, 7}}, {{1, 7, 5}}, {{2, 3, 11}}, {{2, 9, 3}},
        {{2, 6, 4}}, {{2, 11, 6}}, {{2, 4, 9}}, {{3, 5, 7}}, {{3, 9, 5}},
        {{3, 7, 11}}, {{4, 8, 9}}, {{5, 9, 8}}, {{6, 11, 10}}, {{7, 10, 11}},
    }};

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(faces.size() * 3);
    indices.reserve(faces.size() * 3);

    for (const auto& face : faces)
    {
        std::array<Vec3, 3> points {};
        for (int i = 0; i < 3; ++i)
        {
            const JVector& source = IcosahedronShape::Vertices[static_cast<std::size_t>(face[static_cast<std::size_t>(i)])];
            points[static_cast<std::size_t>(i)] = Vec3 {
                static_cast<float>(source.X * static_cast<Jitter2::Real>(0.5)),
                static_cast<float>(source.Y * static_cast<Jitter2::Real>(0.5)),
                static_cast<float>(source.Z * static_cast<Jitter2::Real>(0.5)),
            };
        }

        const Vec3 normal = Normalize(Cross(points[1] - points[0], points[2] - points[0]));
        const auto base = static_cast<unsigned int>(vertices.size());
        vertices.push_back(Vertex {points[0], normal});
        vertices.push_back(Vertex {points[1], normal});
        vertices.push_back(Vertex {points[2], normal});
        indices.insert(indices.end(), {base, base + 1, base + 2});
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

template<typename TShape>
Mesh CreateSupportMapMesh()
{
    TShape shape;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    const std::vector<Jitter2::LinearMath::JTriangle> triangles =
        Shapes::ShapeHelper::Tessellate(shape);
    vertices.reserve(triangles.size() * 3);
    indices.reserve(triangles.size() * 3);

    int index = 0;
    for (const Jitter2::LinearMath::JTriangle& triangle : triangles)
    {
        JVector normal = JVector::Cross(triangle.V1 - triangle.V0, triangle.V2 - triangle.V0);
        normal.Normalize();

        vertices.push_back(Vertex {FromJitter(triangle.V0), FromJitter(normal)});
        vertices.push_back(Vertex {FromJitter(triangle.V1), FromJitter(normal)});
        vertices.push_back(Vertex {FromJitter(triangle.V2), FromJitter(normal)});

        indices.push_back(static_cast<unsigned int>(index + 0));
        indices.push_back(static_cast<unsigned int>(index + 1));
        indices.push_back(static_cast<unsigned int>(index + 2));
        index += 3;
    }

    Mesh mesh;
    mesh.Upload(vertices, indices);
    return mesh;
}

Material CreateCustomSupportMapMaterial()
{
    Material material = Material::Default();
    material.Tint = Vec3 {};
    material.Specular = Vec3 {0.1f, 0.1f, 0.1f};
    material.Shininess = 128.0f;
    material.Alpha = 1.0f;
    material.VertexColorWeight = 0.7f;
    material.TextureWeight = 0.0f;
    return material;
}
