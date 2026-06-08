// Member functions for DemoScene; included inside class DemoScene.

    void BuildContactManifoldTestScene()
    {
        AddFloor();

        World.SolverIterations(4, 4);

        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.AddShape(CreateShape<Shapes::BoxShape>(
            JVector(static_cast<Jitter2::Real>(5),
                static_cast<Jitter2::Real>(0.5),
                static_cast<Jitter2::Real>(0.5))));
        body.Position(JVector(0, 1, 0));
        body.MotionTypeValue(Jitter2::MotionType::Static);

        Jitter2::RigidBody& body2 = World.CreateRigidBody();
        body2.AddShape(CreateShape<Shapes::CylinderShape>(
            static_cast<Jitter2::Real>(0.5),
            static_cast<Jitter2::Real>(3.0)));
        body2.Position(JVector(0, static_cast<Jitter2::Real>(2.5), 0));
        body2.Friction(static_cast<Jitter2::Real>(0));
    }
