struct VoxelRenderData
{
    int X = 0;
    int Y = 0;
    int Z = 0;
    VoxelType Type = VoxelType::Grass;
};

struct ChunkKey
{
    int X = 0;
    int Z = 0;

    bool operator==(const ChunkKey& other) const
    {
        return X == other.X && Z == other.Z;
    }
};

struct ChunkKeyHash
{
    std::size_t operator()(const ChunkKey& key) const
    {
        const std::size_t h1 = std::hash<int> {}(key.X);
        const std::size_t h2 = std::hash<int> {}(key.Z);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
    }
};

struct CollisionVoxel
{
    explicit CollisionVoxel(const JVector& position)
        : Position(position)
    {
    }

    void SupportMap(const JVector& direction, JVector& result) const
    {
        constexpr Jitter2::Real halfSize = static_cast<Jitter2::Real>(0.5);
        result.X = Position.X
            + static_cast<Jitter2::Real>(Jitter2::LinearMath::MathHelper::SignBit(direction.X)) * halfSize;
        result.Y = Position.Y
            + static_cast<Jitter2::Real>(Jitter2::LinearMath::MathHelper::SignBit(direction.Y)) * halfSize;
        result.Z = Position.Z
            + static_cast<Jitter2::Real>(Jitter2::LinearMath::MathHelper::SignBit(direction.Z)) * halfSize;
    }

    void GetCenter(JVector& point) const
    {
        point = Position;
    }

    JVector Position;
};

class Noise
{
public:
    static float Calc(float x, float y)
    {
        const auto& p = PermutationTable();
        const int X = static_cast<int>(std::floor(x)) & 255;
        const int Y = static_cast<int>(std::floor(y)) & 255;
        x -= std::floor(x);
        y -= std::floor(y);
        const float u = Fade(x);
        const float v = Fade(y);
        const int A = p[static_cast<std::size_t>(X)] + Y;
        const int B = p[static_cast<std::size_t>(X + 1)] + Y;
        return Lerp(
            v,
            Lerp(
                u,
                Grad(p[static_cast<std::size_t>(A)], x, y, 0.0f),
                Grad(p[static_cast<std::size_t>(B)], x - 1.0f, y, 0.0f)),
            Lerp(
                u,
                Grad(p[static_cast<std::size_t>(A + 1)], x, y - 1.0f, 0.0f),
                Grad(p[static_cast<std::size_t>(B + 1)], x - 1.0f, y - 1.0f, 0.0f)));
    }

    static float Calc3D(float x, float y, float z)
    {
        const auto& p = PermutationTable();
        const int X = static_cast<int>(std::floor(x)) & 255;
        const int Y = static_cast<int>(std::floor(y)) & 255;
        const int Z = static_cast<int>(std::floor(z)) & 255;
        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);
        const float u = Fade(x);
        const float v = Fade(y);
        const float w = Fade(z);
        const int A = p[static_cast<std::size_t>(X)] + Y;
        const int AA = p[static_cast<std::size_t>(A)] + Z;
        const int AB = p[static_cast<std::size_t>(A + 1)] + Z;
        const int B = p[static_cast<std::size_t>(X + 1)] + Y;
        const int BA = p[static_cast<std::size_t>(B)] + Z;
        const int BB = p[static_cast<std::size_t>(B + 1)] + Z;

        return Lerp(
            w,
            Lerp(
                v,
                Lerp(
                    u,
                    Grad(p[static_cast<std::size_t>(AA)], x, y, z),
                    Grad(p[static_cast<std::size_t>(BA)], x - 1.0f, y, z)),
                Lerp(
                    u,
                    Grad(p[static_cast<std::size_t>(AB)], x, y - 1.0f, z),
                    Grad(p[static_cast<std::size_t>(BB)], x - 1.0f, y - 1.0f, z))),
            Lerp(
                v,
                Lerp(
                    u,
                    Grad(p[static_cast<std::size_t>(AA + 1)], x, y, z - 1.0f),
                    Grad(p[static_cast<std::size_t>(BA + 1)], x - 1.0f, y, z - 1.0f)),
                Lerp(
                    u,
                    Grad(p[static_cast<std::size_t>(AB + 1)], x, y - 1.0f, z - 1.0f),
                    Grad(p[static_cast<std::size_t>(BB + 1)], x - 1.0f, y - 1.0f, z - 1.0f))));
    }

private:
    static const std::array<int, 512>& PermutationTable()
    {
        static const std::array<int, 512> p = []
        {
            constexpr std::array<int, 256> permutation {{
                151, 160, 137, 91, 90, 15,
                131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142,
                8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26,
                197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237,
                149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134,
                139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
                230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
                63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200,
                196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3,
                64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82,
                85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223,
                183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101,
                155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113,
                224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193,
                238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14,
                239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176,
                115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114,
                67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180,
            }};

            std::array<int, 512> result {};
            for (int i = 0; i < 256; ++i)
            {
                result[static_cast<std::size_t>(256 + i)] = permutation[static_cast<std::size_t>(i)];
                result[static_cast<std::size_t>(i)] = permutation[static_cast<std::size_t>(i)];
            }
            return result;
        }();
        return p;
    }

    static float Fade(float t)
    {
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }

    static float Lerp(float t, float a, float b)
    {
        return a + t * (b - a);
    }

    static float Grad(int hash, float x, float y, float z)
    {
        const int h = hash & 15;
        const float u = h < 8 ? x : y;
        const float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
};

class VoxelWorld final : public Jitter2::Collision::IDynamicTreeProxy,
                         public Jitter2::Collision::IRayCastable
{
public:
    static constexpr int MinHeight = -40;

    VoxelWorld()
        : Box(JVector(static_cast<Jitter2::Real>(-1e6)), JVector(static_cast<Jitter2::Real>(1e6)))
    {
    }

    static float GetHeight(int x, int z)
    {
        float h = 0.0f;
        h += Noise::Calc(static_cast<float>(x) * 0.01f, static_cast<float>(z) * 0.01f) * 30.0f;
        h += Noise::Calc(static_cast<float>(x) * 0.05f, static_cast<float>(z) * 0.05f) * 5.0f;
        return h;
    }

    static bool IsSolid(int x, int y, int z, float terrainHeight)
    {
        if (y < MinHeight)
        {
            return true;
        }

        const float density = Noise::Calc3D(
            static_cast<float>(x) * 0.05f,
            static_cast<float>(y) * 0.08f,
            static_cast<float>(z) * 0.05f);

        if (static_cast<float>(y) < terrainHeight)
        {
            return density <= 0.3f;
        }

        return false;
    }

    static bool IsSolid(int x, int y, int z)
    {
        return IsSolid(x, y, z, GetHeight(x, z));
    }

    [[nodiscard]] int NodePtr() const override { return NodePointer; }
    void NodePtr(int value) override { NodePointer = value; }
    [[nodiscard]] JVector Velocity() const override { return JVector::Zero(); }
    [[nodiscard]] const Jitter2::LinearMath::JBoundingBox& WorldBoundingBox() const override { return Box; }

    bool RayCast(
        const JVector&,
        const JVector&,
        JVector& normal,
        Jitter2::Real& lambda) const override
    {
        normal = JVector::Zero();
        lambda = static_cast<Jitter2::Real>(0);
        return false;
    }

private:
    Jitter2::LinearMath::JBoundingBox Box;
    int NodePointer = Jitter2::Collision::DynamicTree::NullNode;
};

class VoxelCollisionFilter : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    VoxelCollisionFilter(Jitter2::World& world, VoxelWorld& voxelProxy)
        : World(world),
          VoxelProxy(voxelProxy)
    {
        const auto range = Jitter2::World::RequestId(1'000'000);
        MinIndex = range.first;
    }

    bool Filter(
        Jitter2::Collision::DynamicTree::Proxy& shapeA,
        Jitter2::Collision::DynamicTree::Proxy& shapeB) override
    {
        if (&shapeA != &VoxelProxy && &shapeB != &VoxelProxy)
        {
            return true;
        }

        const Jitter2::Collision::DynamicTree::Proxy& bodyShape =
            &shapeA == &VoxelProxy ? shapeB : shapeA;
        const auto* rbs = dynamic_cast<const Shapes::RigidBodyShape*>(&bodyShape);
        if (rbs == nullptr)
        {
            return false;
        }

        Jitter2::RigidBody* body = rbs->GetRigidBody();
        if (body == nullptr || !body->IsActive())
        {
            return false;
        }

        const Jitter2::LinearMath::JBoundingBox box = bodyShape.WorldBoundingBox();
        const int minX = static_cast<int>(std::floor(box.Min.X));
        const int minY = static_cast<int>(std::floor(box.Min.Y));
        const int minZ = static_cast<int>(std::floor(box.Min.Z));
        const int maxX = static_cast<int>(std::ceil(box.Max.X));
        const int maxY = static_cast<int>(std::ceil(box.Max.Y));
        const int maxZ = static_cast<int>(std::ceil(box.Max.Z));

        constexpr Jitter2::Real normalThreshold = static_cast<Jitter2::Real>(0.5);
        constexpr Jitter2::Real edgeThreshold = static_cast<Jitter2::Real>(0.01);
        constexpr Jitter2::Real closeToEdge = static_cast<Jitter2::Real>(0.5) - edgeThreshold;

        for (int x = minX; x < maxX; ++x)
        {
            for (int z = minZ; z < maxZ; ++z)
            {
                for (int y = minY; y < maxY; ++y)
                {
                    if (!VoxelWorld::IsSolid(x, y, z))
                    {
                        continue;
                    }

                    const std::uint32_t voxelBits =
                        (static_cast<std::uint32_t>(x) * 73856093U)
                        ^ (static_cast<std::uint32_t>(y) * 19349663U)
                        ^ (static_cast<std::uint32_t>(z) * 83492791U);
                    const std::int32_t voxelHash = std::bit_cast<std::int32_t>(voxelBits);
                    const std::uint64_t voxelId =
                        static_cast<std::uint64_t>(static_cast<std::int64_t>(voxelHash));

                    const JVector voxelPos(
                        static_cast<Jitter2::Real>(x) + static_cast<Jitter2::Real>(0.5),
                        static_cast<Jitter2::Real>(y) + static_cast<Jitter2::Real>(0.5),
                        static_cast<Jitter2::Real>(z) + static_cast<Jitter2::Real>(0.5));
                    CollisionVoxel voxel(voxelPos);

                    JVector pointA;
                    JVector pointB;
                    JVector normal;
                    Jitter2::Real penetration = static_cast<Jitter2::Real>(0);
                    const bool hit = Jitter2::Collision::NarrowPhase::MprEpa(
                        voxel,
                        *rbs,
                        body->Orientation(),
                        body->Position(),
                        pointA,
                        pointB,
                        normal,
                        penetration);

                    if (!hit)
                    {
                        continue;
                    }

                    const JVector relPos = pointA - voxelPos;
                    if (relPos.X > closeToEdge && normal.X > normalThreshold && VoxelWorld::IsSolid(x + 1, y, z))
                    {
                        continue;
                    }
                    if (relPos.X < -closeToEdge && normal.X < -normalThreshold && VoxelWorld::IsSolid(x - 1, y, z))
                    {
                        continue;
                    }
                    if (relPos.Y > closeToEdge && normal.Y > normalThreshold && VoxelWorld::IsSolid(x, y + 1, z))
                    {
                        continue;
                    }
                    if (relPos.Y < -closeToEdge && normal.Y < -normalThreshold && VoxelWorld::IsSolid(x, y - 1, z))
                    {
                        continue;
                    }
                    if (relPos.Z > closeToEdge && normal.Z > normalThreshold && VoxelWorld::IsSolid(x, y, z + 1))
                    {
                        continue;
                    }
                    if (relPos.Z < -closeToEdge && normal.Z < -normalThreshold && VoxelWorld::IsSolid(x, y, z - 1))
                    {
                        continue;
                    }

                    World.RegisterContact(
                        rbs->ShapeId(),
                        MinIndex + (voxelId % 1'000'000ULL),
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
    VoxelWorld& VoxelProxy;
    std::uint64_t MinIndex = 0;
};
