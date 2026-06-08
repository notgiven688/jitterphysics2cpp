struct Heightmap
{
    static constexpr int Width = 100;
    static constexpr int Height = 100;
    static constexpr float Amplitude = 2.0f;

    static float GetHeight(int x, int z)
    {
        if (x < 0 || x >= Width || z < 0 || z >= Height)
        {
            throw std::out_of_range("Heightmap coordinate out of range.");
        }

        return std::sin(static_cast<float>(x) * 0.1f)
            * std::cos(static_cast<float>(z) * 0.1f)
            * Amplitude;
    }

    static Jitter2::LinearMath::JBoundingBox GetBoundingBox()
    {
        const JVector min(0, -Amplitude, 0);
        const JVector max(Width - 1, Amplitude, Height - 1);
        return Jitter2::LinearMath::JBoundingBox(min, max);
    }
};

class HeightmapTester final : public Jitter2::Collision::IDynamicTreeProxy,
                              public Jitter2::Collision::IRayCastable
{
public:
    explicit HeightmapTester(const Jitter2::LinearMath::JBoundingBox& box)
        : Box(box)
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
        constexpr Jitter2::Real maxDistance = static_cast<Jitter2::Real>(100);

        Jitter2::Real dirX = direction.X;
        Jitter2::Real dirZ = direction.Z;

        const Jitter2::Real len2 = dirX * dirX + dirZ * dirZ;
        const Jitter2::Real ilen = static_cast<Jitter2::Real>(1) / std::sqrt(len2);

        dirX *= ilen;
        dirZ *= ilen;

        int x = static_cast<int>(std::floor(origin.X));
        int z = static_cast<int>(std::floor(origin.Z));

        const int stepX = dirX > static_cast<Jitter2::Real>(0) ? 1 : -1;
        const int stepZ = dirZ > static_cast<Jitter2::Real>(0) ? 1 : -1;

        const Jitter2::Real nextX =
            dirX > static_cast<Jitter2::Real>(0)
                ? (static_cast<Jitter2::Real>(x + 1) - origin.X)
                : (origin.X - static_cast<Jitter2::Real>(x));
        const Jitter2::Real nextZ =
            dirZ > static_cast<Jitter2::Real>(0)
                ? (static_cast<Jitter2::Real>(z + 1) - origin.Z)
                : (origin.Z - static_cast<Jitter2::Real>(z));

        Jitter2::Real tMaxX = dirX != static_cast<Jitter2::Real>(0)
            ? nextX / std::abs(dirX)
            : std::numeric_limits<Jitter2::Real>::infinity();
        Jitter2::Real tMaxZ = dirZ != static_cast<Jitter2::Real>(0)
            ? nextZ / std::abs(dirZ)
            : std::numeric_limits<Jitter2::Real>::infinity();

        const Jitter2::Real tDeltaX = direction.X != static_cast<Jitter2::Real>(0)
            ? static_cast<Jitter2::Real>(1) / std::abs(dirX)
            : std::numeric_limits<Jitter2::Real>::infinity();
        const Jitter2::Real tDeltaZ = direction.Z != static_cast<Jitter2::Real>(0)
            ? static_cast<Jitter2::Real>(1) / std::abs(dirZ)
            : std::numeric_limits<Jitter2::Real>::infinity();

        Jitter2::Real t = 0;

        while (t <= maxDistance)
        {
            if (!(x < 0 || x + 1 >= Heightmap::Width || z < 0 || z + 1 >= Heightmap::Height))
            {
                const JVector a(
                    x + 0,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 0, z + 0)),
                    z + 0);
                const JVector b(
                    x + 1,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 1, z + 0)),
                    z + 0);
                const JVector c(
                    x + 1,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 1, z + 1)),
                    z + 1);
                const JVector d(
                    x + 0,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 0, z + 1)),
                    z + 1);

                Jitter2::LinearMath::JTriangle tri0(a, c, b);
                Jitter2::LinearMath::JTriangle tri1(a, d, c);

                JVector normal0;
                JVector normal1;
                Jitter2::Real lambda0;
                Jitter2::Real lambda1;
                static_cast<void>(tri0.RayIntersect(
                    origin,
                    direction,
                    Jitter2::LinearMath::JTriangle::CullMode::BackFacing,
                    normal0,
                    lambda0));
                static_cast<void>(tri1.RayIntersect(
                    origin,
                    direction,
                    Jitter2::LinearMath::JTriangle::CullMode::BackFacing,
                    normal1,
                    lambda1));

                if (lambda0 < std::numeric_limits<Jitter2::Real>::max()
                    || lambda1 < std::numeric_limits<Jitter2::Real>::max())
                {
                    if (lambda0 <= lambda1)
                    {
                        normal = normal0;
                        lambda = lambda0;
                    }
                    else
                    {
                        normal = normal1;
                        lambda = lambda1;
                    }

                    return true;
                }
            }

            if (tMaxX < tMaxZ)
            {
                x += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }

        normal = JVector::Zero();
        lambda = static_cast<Jitter2::Real>(0);
        return false;
    }

private:
    Jitter2::LinearMath::JBoundingBox Box;
    int NodePointer = Jitter2::Collision::DynamicTree::NullNode;
};

class HeightmapDetection : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    HeightmapDetection(Jitter2::World& world, HeightmapTester& shape)
        : World(world),
          Shape(shape)
    {
        const auto range = Jitter2::World::RequestId(Heightmap::Width * Heightmap::Height * 2);
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

        const Jitter2::LinearMath::JBoundingBox& box = collider.WorldBoundingBox();

        const int minX = std::max(0, static_cast<int>(box.Min.X));
        const int minZ = std::max(0, static_cast<int>(box.Min.Z));
        const int maxX = std::min(Heightmap::Width - 1, static_cast<int>(box.Max.X) + 1);
        const int maxZ = std::min(Heightmap::Height - 1, static_cast<int>(box.Max.Z) + 1);

        for (int x = minX; x < maxX; ++x)
        {
            for (int z = minZ; z < maxZ; ++z)
            {
                std::uint64_t index = static_cast<std::uint64_t>(
                    2 * (x * Heightmap::Width + z));

                CollisionTriangle triangle;
                triangle.A = JVector(
                    x + 0,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 0, z + 0)),
                    z + 0);
                triangle.B = JVector(
                    x + 1,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 1, z + 1)),
                    z + 1);
                triangle.C = JVector(
                    x + 1,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 1, z + 0)),
                    z + 0);

                JVector normal = JVector::Normalize(
                    JVector::Cross(triangle.B - triangle.A, triangle.C - triangle.A));

                JVector pointA;
                JVector pointB;
                JVector ignoredNormal;
                Jitter2::Real penetration = static_cast<Jitter2::Real>(0);
                bool hit = Jitter2::Collision::NarrowPhase::MprEpa(
                    triangle,
                    *rbs,
                    body->Orientation(),
                    body->Position(),
                    pointA,
                    pointB,
                    ignoredNormal,
                    penetration);

                if (hit)
                {
                    World.RegisterContact(
                        rbs->ShapeId(),
                        MinIndex + index,
                        World.NullBody(),
                        *body,
                        pointA,
                        pointB,
                        normal);
                }

                index += 1;
                triangle.A = JVector(
                    x + 0,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 0, z + 0)),
                    z + 0);
                triangle.B = JVector(
                    x + 0,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 0, z + 1)),
                    z + 1);
                triangle.C = JVector(
                    x + 1,
                    static_cast<Jitter2::Real>(Heightmap::GetHeight(x + 1, z + 1)),
                    z + 1);

                normal = JVector::Normalize(
                    JVector::Cross(triangle.B - triangle.A, triangle.C - triangle.A));

                hit = Jitter2::Collision::NarrowPhase::MprEpa(
                    triangle,
                    *rbs,
                    body->Orientation(),
                    body->Position(),
                    pointA,
                    pointB,
                    ignoredNormal,
                    penetration);

                if (hit)
                {
                    World.RegisterContact(
                        rbs->ShapeId(),
                        MinIndex + index,
                        World.NullBody(),
                        *body,
                        pointA,
                        pointB,
                        normal);
                }
            }
        }

        return false;
    }

private:
    Jitter2::World& World;
    HeightmapTester& Shape;
    std::uint64_t MinIndex = 0;
};
