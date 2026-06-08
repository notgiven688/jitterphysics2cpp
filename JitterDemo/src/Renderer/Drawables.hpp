Mesh CreateMesh(const CpuMesh& source)
{
    std::vector<unsigned int> indices;
    indices.reserve(source.Indices.size() * 3);
    for (const TriangleVertexIndex& triangle : source.Indices)
    {
        indices.push_back(triangle.T1);
        indices.push_back(triangle.T2);
        indices.push_back(triangle.T3);
    }

    Mesh mesh;
    mesh.Upload(source.Vertices, indices);
    return mesh;
}

class TriangleMeshDrawable
{
public:
    TriangleMeshDrawable(const std::string& filename, float scale = 1.0f, bool reverseWinding = false)
        : CpuData(LoadObj(filename, scale, reverseWinding)),
          Drawable(CreateMesh(CpuData))
    {
        Drawable.MeshGroups = CpuData.Groups;
    }

    void Push(const Mat4& transform, const Vec3& color)
    {
        Drawable.Push(transform, color);
    }

    void Push(const Mat4& transform)
    {
        Drawable.Push(transform);
    }

    CpuMesh CpuData;
    InstancedDrawable Drawable;
};

class DecomposedTeapot : public TriangleMeshDrawable
{
public:
    DecomposedTeapot()
        : TriangleMeshDrawable("teapot_hull.obj", 0.03f)
    {
        Drawable.Groups.clear();
        Drawable.Groups.reserve(CpuData.Groups.size());
        for (int i = 0; i < static_cast<int>(CpuData.Groups.size()); ++i)
        {
            Material material = Material::Default();
            material.Tint = ColorGenerator::GetColor(i * (i << 6));
            material.VertexColorWeight = 0.0f;
            Drawable.Groups.push_back(MaterialSlot {i, material});
        }
    }
};

class Teapot : public TriangleMeshDrawable
{
public:
    Teapot()
        : TriangleMeshDrawable("teapot.obj.zip", 1.0f)
    {
    }
};

class ConvexDecomposition
{
public:
    ConvexDecomposition(
        Jitter2::World& world,
        DecomposedTeapot& drawable,
        std::vector<std::unique_ptr<Shapes::RigidBodyShape>>& ownedShapes)
        : World(world),
          Drawable(drawable),
          OwnedShapes(ownedShapes)
    {
    }

    Jitter2::RigidBody& Spawn(const JVector& position)
    {
        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.Position(position);

        for (const Shapes::ConvexHullShape& shapeToAdd : ShapesToAdd)
        {
            auto shape = std::make_unique<Shapes::ConvexHullShape>(shapeToAdd.Clone());
            Shapes::ConvexHullShape& shapeReference = *shape;
            OwnedShapes.push_back(std::move(shape));
            body.AddShape(shapeReference, Jitter2::MassInertiaUpdateMode::Preserve);
        }

        body.SetMassInertia();
        Bodies.push_back(&body);

        return body;
    }

    void Load()
    {
        CpuMesh& mesh = Drawable.CpuData;

        Jitter2::Real totalMass = static_cast<Jitter2::Real>(0);

        for (const MeshGroup& group : mesh.Groups)
        {
            std::vector<Jitter2::LinearMath::JTriangle> hullTriangles;

            for (int i = group.FromInclusive; i < group.ToExclusive; ++i)
            {
                const TriangleVertexIndex& tvi = mesh.Indices[static_cast<std::size_t>(i)];

                Jitter2::LinearMath::JTriangle jt {
                    ToJitter(mesh.Vertices[tvi.T1].Position),
                    ToJitter(mesh.Vertices[tvi.T2].Position),
                    ToJitter(mesh.Vertices[tvi.T3].Position),
                };

                hullTriangles.push_back(jt);
            }

            Shapes::ConvexHullShape chs(hullTriangles);
            Jitter2::LinearMath::JMatrix inertia;
            JVector cvhCom;
            Jitter2::Real cvhMass = static_cast<Jitter2::Real>(0);
            chs.CalculateMassInertia(inertia, cvhCom, cvhMass);

            Com += cvhCom * cvhMass;
            totalMass += cvhMass;
            ShapesToAdd.push_back(std::move(chs));
        }

        if (totalMass <= static_cast<Jitter2::Real>(0))
        {
            throw std::runtime_error("Convex decomposition did not produce positive mass.");
        }

        Com *= static_cast<Jitter2::Real>(1) / totalMass;

        for (Shapes::ConvexHullShape& shape : ShapesToAdd)
        {
            shape.Shift(-Com);
        }
    }

    void Clear()
    {
        Bodies.clear();
    }

    void PushMatrices()
    {
        for (const Jitter2::RigidBody* body : Bodies)
        {
            const Mat4 mat = BodyTransform(
                body->Position(),
                body->Orientation());
            Drawable.Push(Multiply(mat, Translation(-Com)), Vec3 {});
        }
    }

private:
    Jitter2::World& World;
    DecomposedTeapot& Drawable;
    std::vector<std::unique_ptr<Shapes::RigidBodyShape>>& OwnedShapes;
    std::vector<Jitter2::RigidBody*> Bodies;
    JVector Com = JVector::Zero();
    std::vector<Shapes::ConvexHullShape> ShapesToAdd;
};
