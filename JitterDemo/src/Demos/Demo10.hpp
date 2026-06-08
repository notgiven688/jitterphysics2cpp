// Member functions for DemoScene; included inside class DemoScene.

    void BuildStackedCubesScene()
    {
        AddFloor();

        for (int i = 0; i < 32; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.Position(JVector(0, static_cast<Jitter2::Real>(0.5 + static_cast<double>(i) * 0.999), 0));
            body.AddShape(CreateShape<Shapes::BoxShape>(JVector(1)));
            body.Damping(static_cast<Jitter2::Real>(0.002), static_cast<Jitter2::Real>(0.002));
        }

        for (int i = 0; i < 32; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.Position(JVector(10, static_cast<Jitter2::Real>(0.5 + static_cast<double>(i) * 0.999), 0));
            Shapes::ConeShape& cone = CreateShape<Shapes::ConeShape>();
            body.AddShape(CreateShape<Shapes::TransformedShape>(
                cone,
                JVector::Zero(),
                ScaleMatrix(JVector(static_cast<Jitter2::Real>(0.4), 1, 1))));
            body.Damping(static_cast<Jitter2::Real>(0.002), static_cast<Jitter2::Real>(0.002));
        }

        World.SolverIterations(4, 2);
        World.SubstepCount = 3;
    }
