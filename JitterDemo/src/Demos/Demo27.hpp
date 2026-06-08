// Member functions for DemoScene; included inside class DemoScene.

    void BuildCcdScene()
    {
        CcdSolverInstance = std::make_unique<CcdSolver>(World);

        AddFloor();

        Jitter2::RigidBody& paddle = World.CreateRigidBody();
        paddle.AddShape(CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(5),
            static_cast<Jitter2::Real>(1),
            static_cast<Jitter2::Real>(0.01)));
        paddle.Position(JVector(0, 3, -20));
        paddle.AffectedByGravity(false);

        Constraints::HingeJoint hinge(
            World,
            World.NullBody(),
            paddle,
            paddle.Position(),
            JVector::UnitY(),
            Constraints::AngularLimit::Full(),
            false);

        Jitter2::RigidBody& ball = World.CreateRigidBody();
        ball.AddShape(CreateShape<Shapes::SphereShape>(static_cast<Jitter2::Real>(0.2)));
        ball.Position(JVector(
            static_cast<Jitter2::Real>(2.2),
            static_cast<Jitter2::Real>(3),
            static_cast<Jitter2::Real>(400)));
        ball.Velocity(JVector(0, 0, static_cast<Jitter2::Real>(-400)));
        ball.Damping(static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(0));
        ball.AffectedByGravity(false);

        BuildRagdoll(
            JVector(
                static_cast<Jitter2::Real>(-2.2),
                static_cast<Jitter2::Real>(3.5),
                static_cast<Jitter2::Real>(-19.7)),
            [this](Jitter2::RigidBody& body)
            {
                body.AffectedByGravity(false);
                CcdSolverInstance->Add(body);
            });

        CcdSolverInstance->Add(paddle);
        CcdSolverInstance->Add(ball);

        World.SpeculativeRelaxationFactor = static_cast<Jitter2::Real>(0.5);
    }

