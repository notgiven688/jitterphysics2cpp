// Member functions for DemoScene; included inside class DemoScene.

    void BuildSpeculativeContactsScene()
    {
        AddFloor();
        EnsureIgnoreCollisionFilter();

        World.SolverIterations(12, 4);

        Jitter2::RigidBody& wallBody = World.CreateRigidBody();
        wallBody.AddShape(CreateShape<Shapes::BoxShape>(
            JVector(10, 10, static_cast<Jitter2::Real>(0.02))));
        wallBody.Position(JVector(0, 6, -10));
        wallBody.MotionTypeValue(Jitter2::MotionType::Static);

        Jitter2::RigidBody& sphereBody = World.CreateRigidBody();
        sphereBody.AddShape(CreateShape<Shapes::SphereShape>(
            static_cast<Jitter2::Real>(0.3)));
        sphereBody.Position(JVector(-3, 8, -1));
        sphereBody.Velocity(JVector(0, 0, -107));
        sphereBody.EnableSpeculativeContacts(true);

        Jitter2::RigidBody& boxBody = World.CreateRigidBody();
        boxBody.AddShape(CreateShape<Shapes::BoxShape>(
            JVector(static_cast<Jitter2::Real>(0.3))));
        boxBody.Position(JVector(3, 8, -1));
        boxBody.Velocity(JVector(0, 0, -107));
        boxBody.EnableSpeculativeContacts(true);

        BuildRagdoll(
            JVector(0, 8, -1),
            [](Jitter2::RigidBody& body)
            {
                body.Velocity(JVector(0, 0, -107));
                body.EnableSpeculativeContacts(true);
                body.SetActivationState(false);
            });
    }
