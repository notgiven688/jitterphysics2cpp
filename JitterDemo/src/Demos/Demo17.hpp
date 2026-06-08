// Member functions for DemoScene; included inside class DemoScene.

    void BuildClothScene()
    {
        AddFloor();
        ConfigureSoftBodyCollision();

        constexpr int len = 40;
        constexpr Jitter2::Real scale = static_cast<Jitter2::Real>(0.2);
        constexpr int lenOver2 = len / 2;

        std::vector<Jitter2::LinearMath::JTriangle> triangles;
        triangles.reserve(static_cast<std::size_t>(len * len * 2));

        for (int i = 0; i < len; ++i)
        {
            for (int e = 0; e < len; ++e)
            {
                JVector v0(
                    static_cast<Jitter2::Real>(-lenOver2 + e + 0) * scale,
                    static_cast<Jitter2::Real>(6),
                    static_cast<Jitter2::Real>(-lenOver2 + i + 0) * scale);
                JVector v1(
                    static_cast<Jitter2::Real>(-lenOver2 + e + 0) * scale,
                    static_cast<Jitter2::Real>(6),
                    static_cast<Jitter2::Real>(-lenOver2 + i + 1) * scale);
                JVector v2(
                    static_cast<Jitter2::Real>(-lenOver2 + e + 1) * scale,
                    static_cast<Jitter2::Real>(6),
                    static_cast<Jitter2::Real>(-lenOver2 + i + 0) * scale);
                JVector v3(
                    static_cast<Jitter2::Real>(-lenOver2 + e + 1) * scale,
                    static_cast<Jitter2::Real>(6),
                    static_cast<Jitter2::Real>(-lenOver2 + i + 1) * scale);

                triangles.emplace_back(v0, v1, v2);
                triangles.emplace_back(v3, v2, v1);
            }
        }

        auto cloth = std::make_unique<SoftBodyClothDemo>(World, triangles);
        SoftBodyClothDemo& clothReference = *cloth;
        SoftBodyCloths.push_back(std::move(cloth));

        ClothRenderer = std::make_unique<MutableMeshDrawable>();
        ClothRenderer->SetTriangles(clothReference.Triangles());
        ClothRenderer->DrawableMaterial = ClothMaterial::Create();
        SetClothUVCoordinates(clothReference);

        Jitter2::RigidBody& b0 = World.CreateRigidBody();
        b0.Position(JVector(static_cast<Jitter2::Real>(-1), static_cast<Jitter2::Real>(10), 0));
        b0.AddShape(CreateShape<Shapes::BoxShape>(JVector(static_cast<Jitter2::Real>(1))));
        b0.Orientation(JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(0.4)));

        Jitter2::RigidBody& b1 = World.CreateRigidBody();
        b1.Position(JVector(0, static_cast<Jitter2::Real>(10), 0));
        b1.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.4)));
        b1.Orientation(JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(1)));

        Jitter2::RigidBody& b2 = World.CreateRigidBody();
        b2.Position(JVector(static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(11), 0));
        b2.AddShape(CreateShape<Shapes::SphereShape>(static_cast<Jitter2::Real>(0.5)));

        auto maxVertex =
            [&clothReference](auto scorer)
            {
                return *std::max_element(
                    clothReference.Vertices().begin(),
                    clothReference.Vertices().end(),
                    [&scorer](const Jitter2::RigidBody* left, const Jitter2::RigidBody* right)
                    {
                        return scorer(left->Position()) < scorer(right->Position());
                    });
            };

        Jitter2::RigidBody* fb0 = maxVertex([](const JVector& position)
        {
            return +position.X + position.Z;
        });
        auto& c0 = World.CreateConstraint<Constraints::BallSocket>(*fb0, World.NullBody());
        c0.Initialize(fb0->Position());

        Jitter2::RigidBody* fb1 = maxVertex([](const JVector& position)
        {
            return +position.X - position.Z;
        });
        auto& c1 = World.CreateConstraint<Constraints::BallSocket>(*fb1, World.NullBody());
        c1.Initialize(fb1->Position());

        Jitter2::RigidBody* fb2 = maxVertex([](const JVector& position)
        {
            return -position.X + position.Z;
        });
        auto& c2 = World.CreateConstraint<Constraints::BallSocket>(*fb2, World.NullBody());
        c2.Initialize(fb2->Position());

        Jitter2::RigidBody* fb3 = maxVertex([](const JVector& position)
        {
            return -position.X - position.Z;
        });
        auto& c3 = World.CreateConstraint<Constraints::BallSocket>(*fb3, World.NullBody());
        c3.Initialize(fb3->Position());

        World.SolverIterations(4, 2);
        World.SubstepCount = 3;
    }
