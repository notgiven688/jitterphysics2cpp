// Member functions for DemoScene; included inside class DemoScene.

    void BuildRestitutionFrictionScene()
    {
        AddFloor();

        World.SolverIterations(20, 4);

        if (FloorShape != nullptr && FloorShape->GetRigidBody() != nullptr)
        {
            FloorShape->GetRigidBody()->Friction(static_cast<Jitter2::Real>(0));
            FloorShape->GetRigidBody()->Restitution(static_cast<Jitter2::Real>(0));
        }

        for (int i = 0; i < 11; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(0.5)));
            body.Position(JVector(static_cast<Jitter2::Real>(-10 + i), 4, -10));
            body.Restitution(static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.1));
            body.Damping(static_cast<Jitter2::Real>(0.001), static_cast<Jitter2::Real>(0.001));
        }

        for (int i = 0; i < 11; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(0.5)));
            body.Position(JVector(static_cast<Jitter2::Real>(2 + i), static_cast<Jitter2::Real>(0.25), 0));
            body.Friction(std::max(
                static_cast<Jitter2::Real>(0),
                static_cast<Jitter2::Real>(1.0) - static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.1)));
            body.Velocity(JVector(0, 0, -10));
            body.Damping(static_cast<Jitter2::Real>(0.001), static_cast<Jitter2::Real>(0.001));
        }
    }
