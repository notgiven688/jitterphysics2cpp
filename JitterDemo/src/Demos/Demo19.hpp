// Member functions for DemoScene; included inside class DemoScene.

    void BuildRayCastTestScene()
    {
        AddFloor();

        Jitter2::RigidBody& box = World.CreateRigidBody();
        box.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(1)));
        box.Position(JVector(0, static_cast<Jitter2::Real>(0.5), static_cast<Jitter2::Real>(-6)));
        box.MotionTypeValue(Jitter2::MotionType::Static);
    }

    void DrawRayCastTest(DebugRenderer& debugRenderer)
    {
        const JVector origin(0, static_cast<Jitter2::Real>(20), 0);

        const JVector rayVector(0, static_cast<Jitter2::Real>(-1), 0);

        for (int i = 0; i < 10000; ++i)
        {
            JVector direction =
                Jitter2::LinearMath::JMatrix::CreateRotationX(
                    static_cast<Jitter2::Real>(0.1) + static_cast<Jitter2::Real>(0.0001) * static_cast<Jitter2::Real>(i))
                * rayVector;
            direction =
                (Jitter2::LinearMath::JMatrix::CreateRotationY(
                    static_cast<Jitter2::Real>(0.004) * static_cast<Jitter2::Real>(i))
                    * direction)
                * static_cast<Jitter2::Real>(60);

            debugRenderer.PushLine(
                DebugRenderer::Color::Green,
                FromJitter(origin),
                FromJitter(origin + direction));

            Jitter2::Collision::DynamicTree::Proxy* shape = nullptr;
            JVector normal;
            Jitter2::Real fraction = static_cast<Jitter2::Real>(0);
            const bool hit = World.DynamicTree().RayCast(origin, direction, shape, normal, fraction);

            if (hit)
            {
                debugRenderer.PushPoint(
                    DebugRenderer::Color::White,
                    FromJitter(origin + fraction * direction),
                    0.2f);
            }
        }
    }

