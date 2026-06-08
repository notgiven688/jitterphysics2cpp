struct FractureFragment
{
    Jitter2::RigidBody* Body = nullptr;
    std::vector<JVector> LocalVertices;
    Vec3 Color {};
    int VertexOffset = 0;
};

class FractureFragmentsDrawable : public MutableMeshDrawable
{
public:
    FractureFragmentsDrawable()
    {
        DrawableMaterial = Material {
            Vec3 {0.0f, 0.0f, 0.0f},
            Vec3 {0.14f, 0.14f, 0.14f},
            96.0f,
            1.0f,
            0.0f,
            0.0f,
            false,
            nullptr,
        };
    }

    void SetFragments(const std::vector<FractureFragment>& source)
    {
        Fragments = source;

        int triangleCount = 0;
        for (FractureFragment& fragment : Fragments)
        {
            fragment.VertexOffset = triangleCount * 3;
            triangleCount += static_cast<int>(fragment.LocalVertices.size() / 3);
        }

        std::vector<TriangleVertexIndex> triangles(static_cast<std::size_t>(triangleCount));
        MeshGroups.assign(Fragments.size(), MeshGroup {});
        Groups.assign(Fragments.size(), MaterialSlot {});

        int triangleIndex = 0;
        for (int i = 0; i < static_cast<int>(Fragments.size()); ++i)
        {
            FractureFragment& fragment = Fragments[static_cast<std::size_t>(i)];
            int from = triangleIndex;
            int localTriangleCount = static_cast<int>(fragment.LocalVertices.size() / 3);

            for (int j = 0; j < localTriangleCount; ++j)
            {
                std::uint32_t vertex = static_cast<std::uint32_t>((from + j) * 3);
                triangles[static_cast<std::size_t>(triangleIndex++)] =
                    TriangleVertexIndex {vertex + 0U, vertex + 1U, vertex + 2U};
            }

            MeshGroups[static_cast<std::size_t>(i)] = MeshGroup {
                "fragment_" + std::to_string(i),
                from,
                triangleIndex,
            };

            Material material = Material::Default();
            material.VertexColorWeight = 0.0f;
            material.Tint = fragment.Color;
            material.Specular = Vec3 {0.12f, 0.12f, 0.12f};
            material.Shininess = 80.0f;
            Groups[static_cast<std::size_t>(i)] = MaterialSlot {i, material};
        }

        SetTriangles(triangles);
    }

    void PushFragments()
    {
        if (Fragments.empty())
        {
            return;
        }

        for (int i = 0; i < static_cast<int>(Fragments.size()); ++i)
        {
            FractureFragment& fragment = Fragments[static_cast<std::size_t>(i)];
            if (fragment.Body == nullptr)
            {
                continue;
            }

            const Jitter2::RigidBodyData& data = fragment.Body->Data();
            UpdateFragmentTint(
                i,
                data.IsActive()
                    ? fragment.Color
                    : fragment.Color + InactiveTintBoost);

            int offset = fragment.VertexOffset;

            for (int j = 0; j < static_cast<int>(fragment.LocalVertices.size()); ++j)
            {
                JVector world = JQuaternion::Transform(
                    fragment.LocalVertices[static_cast<std::size_t>(j)],
                    data.Orientation) + data.Position;
                Mesh.Vertices[static_cast<std::size_t>(offset + j)].Position = FromJitter(world);
            }
        }

        RefreshGeometry();
        Push(Identity(), Vec3 {0.0f, 0.0f, 0.0f});
    }

private:
    void UpdateFragmentTint(int fragmentIndex, Vec3 tint)
    {
        if (fragmentIndex < 0 || fragmentIndex >= static_cast<int>(Groups.size()))
        {
            return;
        }

        Groups[static_cast<std::size_t>(fragmentIndex)].MaterialValue.Tint = tint;
    }

    static constexpr Vec3 InactiveTintBoost {0.2f, 0.2f, 0.2f};
    std::vector<FractureFragment> Fragments;
};

struct BreakableBodyTag
{
};

struct FractureFace
{
    explicit FractureFace(std::vector<JVector> vertices = {})
        : Vertices(std::move(vertices))
    {
    }

    std::vector<JVector> Vertices;

    [[nodiscard]] FractureFace Clone() const
    {
        return FractureFace(Vertices);
    }
};

struct ConvexPolyhedron
{
    std::vector<FractureFace> Faces;

    [[nodiscard]] ConvexPolyhedron Clone() const
    {
        ConvexPolyhedron result;
        result.Faces.reserve(Faces.size());
        for (const FractureFace& face : Faces)
        {
            result.Faces.push_back(face.Clone());
        }
        return result;
    }
};

struct Breakable
{
    Jitter2::RigidBody* Body = nullptr;
    JVector Size = JVector::Zero();
    JVector BoundsMin = JVector::Zero();
    JVector BoundsMax = JVector::Zero();
    Jitter2::Real Radius = static_cast<Jitter2::Real>(0);
    int Seed = 0;
    int Generation = 0;
    Jitter2::Real TimeSinceCreated = static_cast<Jitter2::Real>(0);
    bool Queued = false;
    JVector ImpactPoint = JVector::Zero();
    JVector ImpactDirection = JVector::Zero();
    ConvexPolyhedron Source;
    bool SphericalSites = false;
};

struct FractureBounds
{
    JVector Min = JVector::Zero();
    JVector Max = JVector::Zero();
    Jitter2::Real Radius = static_cast<Jitter2::Real>(0);
};

struct PendingFragment
{
    JVector CenterOfMass = JVector::Zero();
    std::vector<JVector> Points;
    std::vector<JVector> LocalVertices;
    ConvexPolyhedron Source;
    JVector BoundsMin = JVector::Zero();
    JVector BoundsMax = JVector::Zero();
    Jitter2::Real Radius = static_cast<Jitter2::Real>(0);
    Jitter2::Real Mass = static_cast<Jitter2::Real>(0);
};
