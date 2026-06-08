class RayCastCar
{
public:
    explicit RayCastCar(Jitter2::World& world)
        : WorldRef(world)
    {
        AccelerationRate = static_cast<Jitter2::Real>(10);
        SteerAngle = static_cast<Jitter2::Real>(40.0f * Pi / 180.0f);
        DriveTorque = static_cast<Jitter2::Real>(340.0);
        SteerRate = static_cast<Jitter2::Real>(5.0);

        Body = &world.CreateRigidBody();

        Shapes::BoxShape& box1 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(3.1),
            static_cast<Jitter2::Real>(1.4),
            static_cast<Jitter2::Real>(8.0));
        Shapes::BoxShape& box2 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(2.4),
            static_cast<Jitter2::Real>(0.8),
            static_cast<Jitter2::Real>(5.0));

        Shapes::TransformedShape& tfs1 = CreateShape<Shapes::TransformedShape>(
            box1,
            JVector(0, static_cast<Jitter2::Real>(0.4), 0));
        Shapes::TransformedShape& tfs2 = CreateShape<Shapes::TransformedShape>(
            box2,
            JVector(0, static_cast<Jitter2::Real>(1.5), static_cast<Jitter2::Real>(1.1)));

        Body->AddShape(tfs1);
        Body->AddShape(tfs2);

        Jitter2::Real mass = static_cast<Jitter2::Real>(100.0);
        JVector sides(static_cast<Jitter2::Real>(3.1), static_cast<Jitter2::Real>(1.0), static_cast<Jitter2::Real>(8.0));

        Jitter2::Real ixx = (static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(12.0))
            * mass * (sides.Y * sides.Y + sides.Z * sides.Z);
        Jitter2::Real iyy = (static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(12.0))
            * mass * (sides.X * sides.X + sides.Z * sides.Z);
        Jitter2::Real izz = (static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(12.0))
            * mass * (sides.X * sides.X + sides.Y * sides.Y);

        Jitter2::LinearMath::JMatrix inertia(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

        Body->Position(JVector(0, static_cast<Jitter2::Real>(0.5), static_cast<Jitter2::Real>(-4)));
        Body->SetMassInertia(inertia, mass);

        Body->Damping(static_cast<Jitter2::Real>(0.0001), static_cast<Jitter2::Real>(0.0001));

        Wheels[0] = std::make_unique<Wheel>(
            world,
            *Body,
            JVector(static_cast<Jitter2::Real>(-1.3), static_cast<Jitter2::Real>(-0.1), static_cast<Jitter2::Real>(-2.5)),
            static_cast<Jitter2::Real>(0.60));
        Wheels[1] = std::make_unique<Wheel>(
            world,
            *Body,
            JVector(static_cast<Jitter2::Real>(+1.3), static_cast<Jitter2::Real>(-0.1), static_cast<Jitter2::Real>(-2.5)),
            static_cast<Jitter2::Real>(0.60));
        Wheels[2] = std::make_unique<Wheel>(
            world,
            *Body,
            JVector(static_cast<Jitter2::Real>(-1.3), static_cast<Jitter2::Real>(-0.1), static_cast<Jitter2::Real>(+2.4)),
            static_cast<Jitter2::Real>(0.60));
        Wheels[3] = std::make_unique<Wheel>(
            world,
            *Body,
            JVector(static_cast<Jitter2::Real>(+1.3), static_cast<Jitter2::Real>(-0.1), static_cast<Jitter2::Real>(+2.4)),
            static_cast<Jitter2::Real>(0.60));

        AdjustWheelValues();
    }

    void AdjustWheelValues()
    {
        Jitter2::Real mass = Body->Mass() / static_cast<Jitter2::Real>(4.0);
        Jitter2::Real wheelMass = Body->Mass() * static_cast<Jitter2::Real>(0.03);

        for (const std::unique_ptr<Wheel>& wheel : Wheels)
        {
            wheel->Inertia = static_cast<Jitter2::Real>(0.5) * (wheel->Radius * wheel->Radius) * wheelMass;
            wheel->Spring = mass * WorldRef.Gravity.Length() / (wheel->WheelTravel * SpringFrac);
            wheel->Damping = static_cast<Jitter2::Real>(2.0)
                * std::sqrt(wheel->Spring * Body->Mass())
                * static_cast<Jitter2::Real>(0.25)
                * DampingFrac;
        }
    }

    void SetInput(Jitter2::Real accelerate, Jitter2::Real steer)
    {
        DestAccelerate = accelerate;
        DestSteering = steer;
    }

    void Step(Jitter2::Real timestep)
    {
        for (const std::unique_ptr<Wheel>& wheel : Wheels)
        {
            wheel->PreStep(timestep);
        }

        Jitter2::Real deltaAccelerate = timestep * AccelerationRate;

        Jitter2::Real deltaSteering = timestep * SteerRate;

        Jitter2::Real dAccelerate = DestAccelerate - Accelerate;
        dAccelerate = std::clamp(dAccelerate, -deltaAccelerate, deltaAccelerate);

        Jitter2::Real dSteering = DestSteering - Steering;
        dSteering = std::clamp(dSteering, -deltaSteering, deltaSteering);

        Accelerate += dAccelerate;
        Steering += dSteering;

        Jitter2::Real maxTorque = DriveTorque * static_cast<Jitter2::Real>(0.5);

        for (const std::unique_ptr<Wheel>& wheel : Wheels)
        {
            wheel->AddTorque(maxTorque * Accelerate);

            if (DestAccelerate == static_cast<Jitter2::Real>(0.0)
                && wheel->AngularVelocity() < static_cast<Jitter2::Real>(0.8))
            {
                wheel->AddTorque(-wheel->AngularVelocity());
            }
        }

        Jitter2::Real alpha = SteerAngle * Steering;

        Wheels[0]->SteerAngle = alpha;
        Wheels[1]->SteerAngle = alpha;

        for (const std::unique_ptr<Wheel>& wheel : Wheels)
        {
            wheel->PostStep(timestep);
        }
    }

    void SetDebugRenderer(DebugRenderer* debugRenderer)
    {
        for (const std::unique_ptr<Wheel>& wheel : Wheels)
        {
            wheel->DebugRendererInstance = debugRenderer;
        }
    }

    Jitter2::RigidBody* Body = nullptr;
    std::array<std::unique_ptr<Wheel>, 4> Wheels {};
    Jitter2::Real SteerAngle = static_cast<Jitter2::Real>(0);
    Jitter2::Real DriveTorque = static_cast<Jitter2::Real>(0);
    Jitter2::Real AccelerationRate = static_cast<Jitter2::Real>(0);
    Jitter2::Real SteerRate = static_cast<Jitter2::Real>(0);

private:
    template<typename TShape, typename... TArgs>
    TShape& CreateShape(TArgs&&... args)
    {
        auto shape = std::make_unique<TShape>(std::forward<TArgs>(args)...);
        TShape& reference = *shape;
        OwnedShapes.push_back(std::move(shape));
        return reference;
    }

    Jitter2::World& WorldRef;
    Jitter2::Real DestSteering = static_cast<Jitter2::Real>(0);
    Jitter2::Real DestAccelerate = static_cast<Jitter2::Real>(0);
    Jitter2::Real Steering = static_cast<Jitter2::Real>(0);
    Jitter2::Real Accelerate = static_cast<Jitter2::Real>(0);
    static constexpr Jitter2::Real DampingFrac = static_cast<Jitter2::Real>(0.8);
    static constexpr Jitter2::Real SpringFrac = static_cast<Jitter2::Real>(0.45);
    std::vector<std::unique_ptr<Shapes::RigidBodyShape>> OwnedShapes;
};
