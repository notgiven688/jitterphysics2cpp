class Dragon : public TriangleMeshDrawable
{
public:
    Dragon()
        : TriangleMeshDrawable("dragon.obj.zip", 5.0f)
    {
        Drawable.MaterialValue = Material {
            Vec3 {},
            Vec3 {0.1f, 0.1f, 0.1f},
            128.0f,
            1.0f,
            1.2f,
            0.5f,
            false,
            nullptr,
        };
    }
};

struct CollisionTriangle
{
    JVector A = JVector::Zero();
    JVector B = JVector::Zero();
    JVector C = JVector::Zero();

    void SupportMap(const JVector& direction, JVector& result) const
    {
        Jitter2::Real min = JVector::Dot(A, direction);
        Jitter2::Real dot = JVector::Dot(B, direction);

        result = A;
        if (dot > min)
        {
            min = dot;
            result = B;
        }

        dot = JVector::Dot(C, direction);
        if (dot > min)
        {
            result = C;
        }
    }

    void GetCenter(JVector& point) const
    {
        point = (static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(3.0)) * (A + B + C);
    }
};

class Octree
{
public:
    struct Node
    {
        Jitter2::LinearMath::JBoundingBox Box;
        std::array<std::uint32_t, 8> Neighbors {};
        std::vector<std::uint32_t> Triangles;
    };

    struct TriangleIndices
    {
        std::uint32_t IndexA = 0;
        std::uint32_t IndexB = 0;
        std::uint32_t IndexC = 0;

        TriangleIndices() = default;
        TriangleIndices(std::uint32_t indexA, std::uint32_t indexB, std::uint32_t indexC)
            : IndexA(indexA),
              IndexB(indexB),
              IndexC(indexC)
        {
        }
    };

    Octree(std::vector<TriangleIndices> indices, std::vector<JVector> vertices)
        : Indices(std::move(indices)),
          Vertices(std::move(vertices)),
          TriangleBoxes(Indices.size())
    {
        Nodes.resize(1024);
        Build();
        std::printf(
            "Build octree (%zu triangles, %u nodes, %d leafs).\n",
            Indices.size(),
            NodeCount,
            NumLeafs);
    }

    [[nodiscard]] const Jitter2::LinearMath::JBoundingBox& Dimensions() const
    {
        return Nodes[0].Box;
    }

    void Query(std::stack<std::uint32_t>& triangles, const Jitter2::LinearMath::JBoundingBox& box) const
    {
        InternalQuery(triangles, box, 0);
    }

    bool Raycast(
        const JVector& origin,
        const JVector& direction,
        JVector& normal,
        Jitter2::Real& lambda) const
    {
        lambda = std::numeric_limits<Jitter2::Real>::max();
        normal = JVector::Zero();

        return InternalRaycast(origin, direction, 0, normal, lambda);
    }

    std::vector<TriangleIndices> Indices;
    std::vector<JVector> Vertices;

private:
    std::uint32_t AllocateNode(const Jitter2::LinearMath::JBoundingBox& size)
    {
        if (Nodes.size() == NodeCount)
        {
            Nodes.resize(Nodes.size() * 2);
        }

        Nodes[NodeCount].Box = size;
        return NodeCount++;
    }

    void InternalQuery(
        std::stack<std::uint32_t>& triangles,
        const Jitter2::LinearMath::JBoundingBox& box,
        std::uint32_t nodeIndex) const
    {
        const Node& node = Nodes[nodeIndex];

        if (node.Box.Contains(box) == Jitter2::LinearMath::JBoundingBox::ContainmentType::Disjoint)
        {
            return;
        }

        if (!node.Triangles.empty())
        {
            for (std::uint32_t t : node.Triangles)
            {
                if (!Jitter2::LinearMath::JBoundingBox::Disjoint(box, TriangleBoxes[t]))
                {
                    triangles.push(t);
                }
            }
        }

        for (int i = 0; i < 8; ++i)
        {
            const std::uint32_t index = node.Neighbors[static_cast<std::size_t>(i)];

            if (index != 0)
            {
                InternalQuery(triangles, box, index);
            }
        }
    }

    void Build()
    {
        for (std::size_t i = 0; i < Indices.size(); ++i)
        {
            Jitter2::LinearMath::JBoundingBox& triangleBox = TriangleBoxes[i];
            TriangleIndices& triangle = Indices[i];

            triangleBox = Jitter2::LinearMath::JBoundingBox::SmallBox();
            Jitter2::LinearMath::JBoundingBox::AddPointInPlace(triangleBox, Vertices[triangle.IndexA]);
            Jitter2::LinearMath::JBoundingBox::AddPointInPlace(triangleBox, Vertices[triangle.IndexB]);
            Jitter2::LinearMath::JBoundingBox::AddPointInPlace(triangleBox, Vertices[triangle.IndexC]);
        }

        Jitter2::LinearMath::JBoundingBox box = Jitter2::LinearMath::JBoundingBox::CreateFromPoints(Vertices);

        JVector delta = box.Max - box.Min;
        const JVector center = box.Center();

        const Jitter2::Real max = std::max(std::max(delta.X, delta.Y), delta.Z);
        delta = JVector(max, max, max);

        box.Max = center + delta * static_cast<Jitter2::Real>(0.5);
        box.Min = center - delta * static_cast<Jitter2::Real>(0.5);

        AllocateNode(box);

        for (std::uint32_t i = 0; i < Indices.size(); ++i)
        {
            AddNode(0, i);
        }
    }

    bool InternalRaycast(
        const JVector& origin,
        const JVector& direction,
        std::uint32_t nodeIndex,
        JVector& normal,
        Jitter2::Real& lambda) const
    {
        const Node& node = Nodes[nodeIndex];

        Jitter2::Real enter = static_cast<Jitter2::Real>(0);
        if (!node.Box.RayIntersect(origin, direction, enter))
        {
            return false;
        }

        bool hit = false;

        if (!node.Triangles.empty())
        {
            for (std::uint32_t triIdx : node.Triangles)
            {
                const TriangleIndices& triangleIndex = Indices[triIdx];

                Jitter2::LinearMath::JTriangle tri(
                    Vertices[triangleIndex.IndexA],
                    Vertices[triangleIndex.IndexB],
                    Vertices[triangleIndex.IndexC]);

                JVector currentNormal;
                Jitter2::Real currentLambda = static_cast<Jitter2::Real>(0);
                if (tri.RayIntersect(
                        origin,
                        direction,
                        Jitter2::LinearMath::JTriangle::CullMode::BackFacing,
                        currentNormal,
                        currentLambda))
                {
                    if (currentLambda < lambda)
                    {
                        lambda = currentLambda;
                        normal = currentNormal;
                        hit = true;
                    }
                }
            }
        }

        for (int i = 0; i < 8; ++i)
        {
            const std::uint32_t childIdx = node.Neighbors[static_cast<std::size_t>(i)];
            if (childIdx == 0)
            {
                continue;
            }

            if (InternalRaycast(origin, direction, childIdx, normal, lambda))
            {
                hit = true;
            }
        }

        return hit;
    }

    int TestSubdivision(const Jitter2::LinearMath::JBoundingBox& parent, std::uint32_t triangle) const
    {
        const Jitter2::LinearMath::JBoundingBox objBox = TriangleBoxes[triangle];
        const JVector center = parent.Center();

        int bits = 0;

        if (objBox.Min.X > center.X)
        {
            bits |= 1;
        }
        else if (objBox.Max.X > center.X)
        {
            return -1;
        }

        if (objBox.Min.Y > center.Y)
        {
            bits |= 2;
        }
        else if (objBox.Max.Y > center.Y)
        {
            return -1;
        }

        if (objBox.Min.Z > center.Z)
        {
            bits |= 4;
        }
        else if (objBox.Max.Z > center.Z)
        {
            return -1;
        }

        return bits;
    }

    void GetSubdivison(
        const Jitter2::LinearMath::JBoundingBox& parent,
        int index,
        Jitter2::LinearMath::JBoundingBox& result) const
    {
        JVector dims = parent.Max - parent.Min;
        dims *= static_cast<Jitter2::Real>(0.5);

        const JVector offset(
            static_cast<Jitter2::Real>((index & (1 << 0)) >> 0),
            static_cast<Jitter2::Real>((index & (1 << 1)) >> 1),
            static_cast<Jitter2::Real>((index & (1 << 2)) >> 2));

        result.Min = JVector(offset.X * dims.X, offset.Y * dims.Y, offset.Z * dims.Z);
        result.Min += parent.Min;
        result.Max = result.Min + dims;

        constexpr Jitter2::Real margin = static_cast<Jitter2::Real>(1e-6);
        const JVector temp = dims * margin;
        result.Min -= temp;
        result.Max += temp;
    }

    void AddNode(std::uint32_t node, std::uint32_t triangle)
    {
        constexpr int maxDepth = 64;
        int depth = 0;

        while (true)
        {
            if (depth++ > maxDepth)
            {
                throw std::runtime_error(
                    "Maximum depth exceeded. Check you model for small or degenerate triangles.");
            }

            Node& nn = Nodes[node];

            const int index = TestSubdivision(nn.Box, triangle);

            if (index == -1)
            {
                if (nn.Triangles.empty())
                {
                    nn.Triangles.reserve(8);
                    ++NumLeafs;
                }

                nn.Triangles.push_back(triangle);
            }
            else
            {
                std::uint32_t newNode = nn.Neighbors[static_cast<std::size_t>(index)];

                if (newNode == 0)
                {
                    Jitter2::LinearMath::JBoundingBox newBox;
                    GetSubdivison(nn.Box, index, newBox);
                    newNode = AllocateNode(newBox);
                    Nodes[node].Neighbors[static_cast<std::size_t>(index)] = newNode;
                }

                node = newNode;
                continue;
            }

            break;
        }
    }

    std::vector<Jitter2::LinearMath::JBoundingBox> TriangleBoxes;
    std::vector<Node> Nodes;
    std::uint32_t NodeCount = 0;
    int NumLeafs = 0;
};

class Tester final : public Jitter2::Collision::IDynamicTreeProxy,
                     public Jitter2::Collision::IRayCastable
{
public:
    explicit Tester(Octree& tree)
        : Tree(tree),
          Box(tree.Dimensions())
    {
    }

    [[nodiscard]] int NodePtr() const override { return NodePointer; }
    void NodePtr(int value) override { NodePointer = value; }
    [[nodiscard]] JVector Velocity() const override { return JVector::Zero(); }
    [[nodiscard]] const Jitter2::LinearMath::JBoundingBox& WorldBoundingBox() const override { return Box; }

    bool RayCast(
        const JVector& origin,
        const JVector& direction,
        JVector& normal,
        Jitter2::Real& lambda) const override
    {
        return Tree.Raycast(origin, direction, normal, lambda);
    }

private:
    Octree& Tree;
    Jitter2::LinearMath::JBoundingBox Box;
    int NodePointer = Jitter2::Collision::DynamicTree::NullNode;
};

class CustomCollisionDetection : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    CustomCollisionDetection(Jitter2::World& world, Tester& shape, Octree& octree)
        : World(world),
          Shape(shape),
          Tree(octree)
    {
        const auto range = Jitter2::World::RequestId(static_cast<int>(Tree.Indices.size()));
        MinIndex = range.first;
    }

    bool Filter(
        Jitter2::Collision::DynamicTree::Proxy& shapeA,
        Jitter2::Collision::DynamicTree::Proxy& shapeB) override
    {
        if (&shapeA != &Shape && &shapeB != &Shape)
        {
            return true;
        }

        const Jitter2::Collision::DynamicTree::Proxy& collider = &shapeA == &Shape ? shapeB : shapeA;

        const auto* rbs = dynamic_cast<const Shapes::RigidBodyShape*>(&collider);
        if (rbs == nullptr)
        {
            return false;
        }

        Jitter2::RigidBody* body = rbs->GetRigidBody();
        if (body == nullptr
            || body->MotionTypeValue() != Jitter2::MotionType::Dynamic
            || !body->IsActive())
        {
            return false;
        }

        thread_local std::stack<std::uint32_t> candidates;
        CollisionTriangle ts;

        Tree.Query(candidates, collider.WorldBoundingBox());

        while (!candidates.empty())
        {
            const std::uint32_t index = candidates.top();
            candidates.pop();
            ts.A = Tree.Vertices[Tree.Indices[index].IndexA];
            ts.B = Tree.Vertices[Tree.Indices[index].IndexB];
            ts.C = Tree.Vertices[Tree.Indices[index].IndexC];

            JVector pointA;
            JVector pointB;
            JVector normal;
            Jitter2::Real penetration = static_cast<Jitter2::Real>(0);
            const bool hit = Jitter2::Collision::NarrowPhase::MprEpa(
                ts,
                *rbs,
                body->Orientation(),
                body->Position(),
                pointA,
                pointB,
                normal,
                penetration);

            if (hit)
            {
                normal = JVector::Cross(ts.B - ts.A, ts.C - ts.A);
                normal.Normalize();

                if (World.EnableAuxiliaryContactPoints)
                {
                    Jitter2::Collision::CollisionManifold manifold;

                    manifold.BuildManifold(
                        ts,
                        *rbs,
                        JQuaternion::Identity(),
                        body->Orientation(),
                        JVector::Zero(),
                        body->Position(),
                        pointA,
                        pointB,
                        normal);

                    World.RegisterContact(
                        rbs->ShapeId(),
                        MinIndex + index,
                        World.NullBody(),
                        *body,
                        normal,
                        manifold);
                }
                else
                {
                    World.RegisterContact(
                        rbs->ShapeId(),
                        MinIndex + index,
                        World.NullBody(),
                        *body,
                        pointB,
                        pointB,
                        normal);
                }
            }
        }

        return false;
    }

private:
    Jitter2::World& World;
    Tester& Shape;
    Octree& Tree;
    std::uint64_t MinIndex = 0;
};

enum class VoxelType : unsigned char
{
    Grass = 0,
    Rock = 1,
    Snow = 2,
};
