// Member functions for DemoScene; included inside class DemoScene.

    void BuildDoublePendulumScene()
    {
        AddFloor();

        Jitter2::RigidBody& b0 = World.CreateRigidBody();
        b0.AddShape(CreateShape<Shapes::SphereShape>(static_cast<Jitter2::Real>(0.2)));
        b0.Position(JVector(0, 12, 0));
        DoublePendulumBody0 = &b0;
        b0.Velocity(JVector(static_cast<Jitter2::Real>(0.01), 0, 0));
        b0.DeactivationTime(std::numeric_limits<Jitter2::Real>::max());

        Jitter2::RigidBody& b1 = World.CreateRigidBody();
        b1.AddShape(CreateShape<Shapes::SphereShape>(static_cast<Jitter2::Real>(0.2)));
        DoublePendulumBody1 = &b1;
        b1.Velocity(JVector(0, 0, static_cast<Jitter2::Real>(0.01)));
        b1.Position(JVector(0, 13, 0));

        auto& c0 = World.CreateConstraint<Constraints::DistanceLimit>(World.NullBody(), b0);
        c0.Initialize(JVector(0, 8, 0), b0.Position());

        auto& c1 = World.CreateConstraint<Constraints::DistanceLimit>(b0, b1);
        c1.Initialize(b0.Position(), b1.Position());

        World.SubstepCount = 10;
        World.SolverIterations(2, 2);

        b0.Damping(static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(0));
        b1.Damping(static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(0));
    }

    void DrawDoublePendulum(DebugRenderer& debugRenderer) const
    {
        if (DoublePendulumBody0 == nullptr || DoublePendulumBody1 == nullptr)
        {
            return;
        }

        const Jitter2::Real ekin = static_cast<Jitter2::Real>(0.5)
            * (DoublePendulumBody0->Velocity().LengthSquared()
                + DoublePendulumBody1->Velocity().LengthSquared());
        const Jitter2::Real epot = -World.Gravity.Y
            * (DoublePendulumBody0->Position().Y + DoublePendulumBody1->Position().Y);

        std::printf(
            "Energy: %.6g Kinetic %.6g; Potential %.6g\n",
            static_cast<double>(ekin + epot),
            static_cast<double>(ekin),
            static_cast<double>(epot));

        debugRenderer.PushLine(
            DebugRenderer::Color::Green,
            FromJitter(JVector(0, 8, 0)),
            FromJitter(DoublePendulumBody0->Position()));
        debugRenderer.PushLine(
            DebugRenderer::Color::White,
            FromJitter(DoublePendulumBody0->Position()),
            FromJitter(DoublePendulumBody1->Position()));
    }
