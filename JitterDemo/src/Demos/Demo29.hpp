// Member functions for DemoScene; included inside class DemoScene.

    void BuildGearsScene()
    {
        AddFloor();

        GearCollisionFilter = std::make_unique<IgnoreGearCollisionFilter>();
        World.BroadPhaseFilter = GearCollisionFilter.get();

        Jitter2::RigidBody& g0 = CreateGear(JVector(-6, 3, static_cast<Jitter2::Real>(0.0)));
        Jitter2::RigidBody& g1 = CreateGear(JVector(-3, 3, static_cast<Jitter2::Real>(0.2)));
        Jitter2::RigidBody& g2 = CreateGear(JVector(0, 3, static_cast<Jitter2::Real>(0.4)));
        Jitter2::RigidBody& g3 = CreateGear(JVector(3, 3, static_cast<Jitter2::Real>(0.2)));
        Jitter2::RigidBody& g4 = CreateGear(JVector(6, 3, static_cast<Jitter2::Real>(0.0)));

        GearCouplings.push_back(std::make_unique<GearCoupling>(
            World, g0, g1, JVector::UnitZ(), JVector::UnitZ(), g0.Position() + JVector(1, 0, 0)));
        GearCouplings.push_back(std::make_unique<GearCoupling>(
            World, g1, g2, JVector::UnitZ(), JVector::UnitZ(), g1.Position() + JVector(1, 0, 0)));
        GearCouplings.push_back(std::make_unique<GearCoupling>(
            World, g2, g3, JVector::UnitZ(), JVector::UnitZ(), g3.Position() - JVector(1, 0, 0)));
        GearCouplings.push_back(std::make_unique<GearCoupling>(
            World, g3, g4, JVector::UnitZ(), JVector::UnitZ(), g4.Position() - JVector(1, 0, 0)));

        World.SolverIterations(6, 2);
        World.SubstepCount = 3;
    }

    Jitter2::RigidBody& CreateGear(const JVector& position)
    {
        Jitter2::RigidBody& gear = World.CreateRigidBody();

        AddTransformedCylinder(
            gear,
            static_cast<Jitter2::Real>(0.2),
            static_cast<Jitter2::Real>(2.0),
            JVector(0, static_cast<Jitter2::Real>(-0.1), 0));
        AddTransformedCylinder(
            gear,
            static_cast<Jitter2::Real>(0.2),
            static_cast<Jitter2::Real>(1.0),
            JVector(0, static_cast<Jitter2::Real>(0.1), 0));
        AddTransformedCylinder(
            gear,
            static_cast<Jitter2::Real>(0.8),
            static_cast<Jitter2::Real>(0.1),
            JVector(static_cast<Jitter2::Real>(-0.8), static_cast<Jitter2::Real>(0.4), 0));

        gear.Orientation(JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(Pi / 2.0f)));
        gear.Position(position);
        gear.AffectedByGravity(false);
        gear.Tag = GearMarker {};
        return gear;
    }

    void AddTransformedCylinder(
        Jitter2::RigidBody& body,
        Jitter2::Real height,
        Jitter2::Real radius,
        const JVector& translation)
    {
        Shapes::CylinderShape& cylinder = CreateShape<Shapes::CylinderShape>(height, radius);
        body.AddShape(CreateShape<Shapes::TransformedShape>(cylinder, translation));
    }

    void DrawGearCouplings(DebugRenderer& debugRenderer) const
    {
        for (const std::unique_ptr<GearCoupling>& coupling : GearCouplings)
        {
            debugRenderer.PushPoint(DebugRenderer::Color::Green, FromJitter(coupling->ContactPoint));
        }
    }
