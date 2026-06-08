struct ShapePairKey
{
    std::uint64_t A = 0;
    std::uint64_t B = 0;

    bool operator==(const ShapePairKey& other) const
    {
        return A == other.A && B == other.B;
    }
};

struct ShapePairKeyHash
{
    std::size_t operator()(const ShapePairKey& key) const
    {
        const std::size_t h1 = std::hash<std::uint64_t> {}(key.A);
        const std::size_t h2 = std::hash<std::uint64_t> {}(key.B);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
    }
};

class IgnoreCollisionBetweenFilter : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    bool Filter(
        Jitter2::Collision::IDynamicTreeProxy& proxyA,
        Jitter2::Collision::IDynamicTreeProxy& proxyB) override
    {
        auto* shapeA = dynamic_cast<Shapes::RigidBodyShape*>(&proxyA);
        auto* shapeB = dynamic_cast<Shapes::RigidBodyShape*>(&proxyB);
        if (shapeA == nullptr || shapeB == nullptr)
        {
            return false;
        }

        return ignore_.find(MakeShapePairKey(*shapeA, *shapeB)) == ignore_.end();
    }

    void IgnoreCollisionBetween(const Shapes::RigidBodyShape& shapeA, const Shapes::RigidBodyShape& shapeB)
    {
        ignore_.insert(MakeShapePairKey(shapeA, shapeB));
    }

private:
    static ShapePairKey MakeShapePairKey(
        const Shapes::RigidBodyShape& shapeA,
        const Shapes::RigidBodyShape& shapeB)
    {
        const std::uint64_t idA = shapeA.ShapeId();
        const std::uint64_t idB = shapeB.ShapeId();
        return idA < idB ? ShapePairKey {idA, idB} : ShapePairKey {idB, idA};
    }

    std::unordered_set<ShapePairKey, ShapePairKeyHash> ignore_;
};

class IgnoreGearCollisionFilter : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    bool Filter(
        Jitter2::Collision::IDynamicTreeProxy& proxyA,
        Jitter2::Collision::IDynamicTreeProxy& proxyB) override
    {
        auto* shapeA = dynamic_cast<Shapes::RigidBodyShape*>(&proxyA);
        auto* shapeB = dynamic_cast<Shapes::RigidBodyShape*>(&proxyB);
        if (shapeA == nullptr || shapeB == nullptr)
        {
            return false;
        }

        const Jitter2::RigidBody* bodyA = shapeA->GetRigidBody();
        const Jitter2::RigidBody* bodyB = shapeB->GetRigidBody();
        if (bodyA == nullptr || bodyB == nullptr)
        {
            return false;
        }

        return !(std::any_cast<GearMarker>(&bodyA->Tag) != nullptr
            && std::any_cast<GearMarker>(&bodyB->Tag) != nullptr);
    }
};

class EllipsoidShape final : public Shapes::RigidBodyShape
{
public:
    EllipsoidShape()
    {
        UpdateWorldBoundingBox();
    }

    void SupportMap(const JVector& direction, JVector& result) const override
    {
        JVector dir = direction;
        dir.X *= static_cast<Jitter2::Real>(0.8);
        dir.Y *= static_cast<Jitter2::Real>(1.2);
        dir.Z *= static_cast<Jitter2::Real>(0.4);
        result = dir;
        result.Normalize();
        result.X *= static_cast<Jitter2::Real>(0.8);
        result.Y *= static_cast<Jitter2::Real>(1.2);
        result.Z *= static_cast<Jitter2::Real>(0.4);
    }

    void GetCenter(JVector& point) const override
    {
        point = JVector::Zero();
    }
};

class DoubleSphereShape final : public Shapes::RigidBodyShape
{
public:
    DoubleSphereShape()
    {
        UpdateWorldBoundingBox();
    }

    void SupportMap(const JVector& direction, JVector& result) const override
    {
        JVector ndir = JVector::Normalize(direction);

        const JVector sphere1 = ndir * static_cast<Jitter2::Real>(1.0)
            + JVector::UnitY() * static_cast<Jitter2::Real>(1.1);
        const JVector sphere2 = ndir * static_cast<Jitter2::Real>(1.5)
            - JVector::UnitY() * static_cast<Jitter2::Real>(0.5);

        if (JVector::Dot(sphere1, ndir) > JVector::Dot(sphere2, ndir))
        {
            result = sphere1 * static_cast<Jitter2::Real>(0.5);
        }
        else
        {
            result = sphere2 * static_cast<Jitter2::Real>(0.5);
        }
    }

    void GetCenter(JVector& point) const override
    {
        point = JVector::Zero();
    }
};

class IcosahedronShape final : public Shapes::RigidBodyShape
{
public:
    IcosahedronShape()
    {
        UpdateWorldBoundingBox();
    }

    void SupportMap(const JVector& direction, JVector& result) const override
    {
        int largestIndex = 0;
        Jitter2::Real maxDot = JVector::Dot(Vertices[0], direction);

        for (int i = 1; i < static_cast<int>(Vertices.size()); ++i)
        {
            const Jitter2::Real dot = JVector::Dot(Vertices[static_cast<std::size_t>(i)], direction);
            if (dot > maxDot)
            {
                maxDot = dot;
                largestIndex = i;
            }
        }

        result = Vertices[static_cast<std::size_t>(largestIndex)] * static_cast<Jitter2::Real>(0.5);
    }

    void GetCenter(JVector& point) const override
    {
        point = JVector::Zero();
    }

    static constexpr Jitter2::Real GoldenRatio =
        static_cast<Jitter2::Real>(1.6180339887498948482);
    static constexpr std::array<JVector, 12> Vertices {
        JVector(0, +1, +GoldenRatio), JVector(0, -1, +GoldenRatio),
        JVector(0, +1, -GoldenRatio), JVector(0, -1, -GoldenRatio),
        JVector(+1, +GoldenRatio, 0), JVector(+1, -GoldenRatio, 0),
        JVector(-1, +GoldenRatio, 0), JVector(-1, -GoldenRatio, 0),
        JVector(+GoldenRatio, 0, +1), JVector(+GoldenRatio, 0, -1),
        JVector(-GoldenRatio, 0, +1), JVector(-GoldenRatio, 0, -1),
    };
};

struct TriangleVertexIndex
{
    std::uint32_t T1 = 0;
    std::uint32_t T2 = 0;
    std::uint32_t T3 = 0;
};

struct JVectorHash
{
    std::size_t operator()(const JVector& value) const
    {
        std::size_t seed = std::hash<Jitter2::Real> {}(value.X);
        seed ^= std::hash<Jitter2::Real> {}(value.Y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<Jitter2::Real> {}(value.Z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

class SoftBodyClothDemo : public SoftBodies::SoftBody
{
public:
    SoftBodyClothDemo(Jitter2::World& world, const std::vector<Jitter2::LinearMath::JTriangle>& triangles)
        : SoftBody(world)
    {
        LoadMesh(triangles);
        Build();
    }

    [[nodiscard]] const std::vector<TriangleVertexIndex>& Triangles() const { return triangles_; }

protected:
    struct Edge
    {
        std::uint16_t IndexA = 0;
        std::uint16_t IndexB = 0;

        bool operator==(const Edge& other) const
        {
            return IndexA == other.IndexA && IndexB == other.IndexB;
        }
    };

    struct EdgeHash
    {
        std::size_t operator()(const Edge& edge) const
        {
            return static_cast<std::size_t>(edge.IndexA) * 24712U + static_cast<std::size_t>(edge.IndexB);
        }
    };

    void LoadMesh(const std::vector<Jitter2::LinearMath::JTriangle>& triangles)
    {
        std::unordered_map<JVector, std::uint16_t, JVectorHash> vertexMap;
        std::unordered_set<Edge, EdgeHash> edgeSet;

        const auto addVertex =
            [&vertexMap, this](const JVector& vertex)
            {
                const auto iterator = vertexMap.find(vertex);
                if (iterator != vertexMap.end())
                {
                    return iterator->second;
                }

                const auto index = static_cast<std::uint16_t>(vertices_.size());
                vertexMap.emplace(vertex, index);
                vertices_.push_back(vertex);
                return index;
            };

        const auto addEdge =
            [&edgeSet, this](std::uint16_t indexA, std::uint16_t indexB)
            {
                Edge edge {indexA, indexB};
                if (edgeSet.insert(edge).second)
                {
                    edges_.push_back(edge);
                }
            };

        for (const Jitter2::LinearMath::JTriangle& triangle : triangles)
        {
            const std::uint16_t u0 = addVertex(triangle.V0);
            const std::uint16_t u1 = addVertex(triangle.V1);
            const std::uint16_t u2 = addVertex(triangle.V2);

            triangles_.push_back(TriangleVertexIndex {u0, u1, u2});

            addEdge(u0, u1);
            addEdge(u0, u2);
            addEdge(u1, u2);
        }
    }

    void Build()
    {
        for (const JVector& vertex : vertices_)
        {
            Jitter2::RigidBody& body = GetWorld().CreateRigidBody();
            body.SetMassInertia(Jitter2::LinearMath::JMatrix::Zero(), static_cast<Jitter2::Real>(100), true);
            body.Position(vertex);
            AddVertex(body);
        }

        for (const Edge& edge : edges_)
        {
            auto& constraint = GetWorld().CreateConstraint<SoftBodies::SpringConstraint>(
                *Vertices()[edge.IndexA],
                *Vertices()[edge.IndexB]);
            constraint.Initialize(Vertices()[edge.IndexA]->Position(), Vertices()[edge.IndexB]->Position());
            constraint.Softness(static_cast<Jitter2::Real>(0.2));
            AddSpring(constraint);
        }

        for (const TriangleVertexIndex& triangle : triangles_)
        {
            auto shape = std::make_unique<SoftBodies::SoftBodyTriangle>(
                *this,
                *Vertices()[triangle.T1],
                *Vertices()[triangle.T2],
                *Vertices()[triangle.T3]);
            shape->UpdateWorldBoundingBox();
            AddShape(*shape);
            ownedTriangles_.push_back(std::move(shape));
        }
    }

private:
    std::vector<JVector> vertices_;
    std::vector<TriangleVertexIndex> triangles_;
    std::vector<Edge> edges_;
    std::vector<std::unique_ptr<SoftBodies::SoftBodyTriangle>> ownedTriangles_;
};

class SoftBodySphereDemo final : public SoftBodyClothDemo
{
public:
    SoftBodySphereDemo(Jitter2::World& world, const JVector& offset)
        : SoftBodyClothDemo(world, GenSphereTriangles(offset))
    {
        for (Jitter2::RigidBody* body : Vertices())
        {
            body->SetMassInertia(Jitter2::LinearMath::JMatrix::Zero(), static_cast<Jitter2::Real>(100), true);
            body->Damping(static_cast<Jitter2::Real>(0.001), static_cast<Jitter2::Real>(0));
        }

        for (Constraints::Constraint* spring : Springs())
        {
            if (auto* springConstraint = dynamic_cast<SoftBodies::SpringConstraint*>(spring))
            {
                springConstraint->Softness(static_cast<Jitter2::Real>(0.5));
            }
        }
    }

    Jitter2::Real Pressure = static_cast<Jitter2::Real>(400);

protected:
    void WorldOnPostStep(Jitter2::Real dt) override
    {
        SoftBodyClothDemo::WorldOnPostStep(dt);

        if (!IsActive())
        {
            return;
        }

        Jitter2::Real volume = static_cast<Jitter2::Real>(0);
        for (SoftBodies::SoftBodyShape* shape : Shapes())
        {
            auto* triangle = dynamic_cast<SoftBodies::SoftBodyTriangle*>(shape);
            if (triangle == nullptr)
            {
                continue;
            }

            const JVector v1 = triangle->Vertex1().Position();
            const JVector v2 = triangle->Vertex2().Position();
            const JVector v3 = triangle->Vertex3().Position();

            volume += ((v2.Y - v1.Y) * (v3.Z - v1.Z)
                - (v2.Z - v1.Z) * (v3.Y - v1.Y)) * (v1.X + v2.X + v3.X);
        }

        const Jitter2::Real invVolume = static_cast<Jitter2::Real>(1)
            / std::max(static_cast<Jitter2::Real>(0.1), volume);

        for (SoftBodies::SoftBodyShape* shape : Shapes())
        {
            auto* triangle = dynamic_cast<SoftBodies::SoftBodyTriangle*>(shape);
            if (triangle == nullptr)
            {
                continue;
            }

            const JVector p0 = triangle->Vertex1().Position();
            const JVector p1 = triangle->Vertex2().Position();
            const JVector p2 = triangle->Vertex3().Position();

            JVector normal = JVector::Cross(p1 - p0, p2 - p0);
            JVector force = normal * Pressure * invVolume;

            constexpr Jitter2::Real maxForce = static_cast<Jitter2::Real>(2);
            const Jitter2::Real forceLengthSquared = force.LengthSquared();
            if (forceLengthSquared > maxForce * maxForce)
            {
                force *= static_cast<Jitter2::Real>(1) / std::sqrt(forceLengthSquared) * maxForce;
            }

            triangle->Vertex1().AddForce(force, false);
            triangle->Vertex2().AddForce(force, false);
            triangle->Vertex3().AddForce(force, false);
        }
    }

private:
    static std::vector<Jitter2::LinearMath::JTriangle> GenSphereTriangles(const JVector& offset)
    {
        std::vector<Jitter2::LinearMath::JTriangle> triangles =
            Shapes::ShapeHelper::Tessellate(
                Jitter2::Collision::SupportPrimitives::CreateSphere(static_cast<Jitter2::Real>(1)),
                4);

        for (Jitter2::LinearMath::JTriangle& triangle : triangles)
        {
            triangle.V0 += offset;
            triangle.V1 += offset;
            triangle.V2 += offset;
        }

        return triangles;
    }
};

class SoftBodyCubeDemo final : public SoftBodies::SoftBody
{
public:
    static constexpr std::array<std::pair<int, int>, 12> Edges {{
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 0},
        {4, 5},
        {5, 6},
        {6, 7},
        {7, 4},
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7},
    }};

    SoftBodyCubeDemo(Jitter2::World& world, const JVector& offset)
        : SoftBody(world)
    {
        std::array<JVector, 8> vertices {};

        vertices[0] = JVector(+1, -1, +1);
        vertices[1] = JVector(+1, -1, -1);
        vertices[2] = JVector(-1, -1, -1);
        vertices[3] = JVector(-1, -1, +1);
        vertices[4] = JVector(+1, +1, +1);
        vertices[5] = JVector(+1, +1, -1);
        vertices[6] = JVector(-1, +1, -1);
        vertices[7] = JVector(-1, +1, +1);

        for (int i = 0; i < 8; ++i)
        {
            Jitter2::RigidBody& body = world.CreateRigidBody();
            body.SetMassInertia(Jitter2::LinearMath::JMatrix::Zero(), static_cast<Jitter2::Real>(5), true);
            body.Position(vertices[static_cast<std::size_t>(i)] + offset);
            AddVertex(body);
        }

        AddTetrahedron(0, 1, 5, 2);
        AddTetrahedron(2, 5, 6, 7);
        AddTetrahedron(3, 0, 2, 7);
        AddTetrahedron(0, 4, 5, 7);
        AddTetrahedron(0, 2, 5, 7);

        center_ = &world.CreateRigidBody();
        center_->Position(offset);
        center_->SetMassInertia(
            Jitter2::LinearMath::JMatrix::Identity() * static_cast<Jitter2::Real>(0.05),
            static_cast<Jitter2::Real>(0.1));

        for (int i = 0; i < 8; ++i)
        {
            auto& constraint = world.CreateConstraint<Constraints::BallSocket>(*center_, *Vertices()[i]);
            constraint.Initialize(Vertices()[i]->Position());
            constraint.Softness(static_cast<Jitter2::Real>(1));
        }
    }

    [[nodiscard]] Jitter2::RigidBody& Center() const { return *center_; }

private:
    void AddTetrahedron(int i1, int i2, int i3, int i4)
    {
        auto tetrahedron = std::make_unique<SoftBodies::SoftBodyTetrahedron>(
            *this,
            *Vertices()[i1],
            *Vertices()[i2],
            *Vertices()[i3],
            *Vertices()[i4]);
        tetrahedron->UpdateWorldBoundingBox();
        AddShape(*tetrahedron);
        ownedTetrahedra_.push_back(std::move(tetrahedron));
    }

    Jitter2::RigidBody* center_ = nullptr;
    std::vector<std::unique_ptr<SoftBodies::SoftBodyTetrahedron>> ownedTetrahedra_;
};
