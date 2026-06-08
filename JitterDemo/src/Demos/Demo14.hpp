// Member functions for DemoScene; included inside class DemoScene.

    void BuildCustomShapesScene()
    {
        AddFloor();

        for (int i = 0; i < 100; ++i)
        {
            Jitter2::RigidBody& body1 = World.CreateRigidBody();
            body1.AddShape(CreateShape<EllipsoidShape>());
            body1.Position(JVector(
                static_cast<Jitter2::Real>(-3),
                static_cast<Jitter2::Real>(3 + i * 5),
                static_cast<Jitter2::Real>(0)));

            Jitter2::RigidBody& body2 = World.CreateRigidBody();
            body2.AddShape(CreateShape<DoubleSphereShape>());
            body2.Position(JVector(
                static_cast<Jitter2::Real>(0),
                static_cast<Jitter2::Real>(3 + i * 5),
                static_cast<Jitter2::Real>(0)));

            Jitter2::RigidBody& body3 = World.CreateRigidBody();
            body3.AddShape(CreateShape<IcosahedronShape>());
            body3.Position(JVector(
                static_cast<Jitter2::Real>(3),
                static_cast<Jitter2::Real>(3 + i * 5),
                static_cast<Jitter2::Real>(0)));
        }
    }

