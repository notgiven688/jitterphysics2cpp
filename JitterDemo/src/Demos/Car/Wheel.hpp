class Wheel
{
private:
    Jitter2::World& WorldRef;
    Jitter2::RigidBody& Car;

public:
    Wheel(Jitter2::World& world, Jitter2::RigidBody& car, JVector position, Jitter2::Real radius)
        : WorldRef(world),
          Car(car)
    {
        Position = position;
        Radius = radius;
        SideFriction = static_cast<Jitter2::Real>(3.0);
        ForwardFriction = static_cast<Jitter2::Real>(5.0);
        Inertia = static_cast<Jitter2::Real>(1.0);
        WheelTravel = static_cast<Jitter2::Real>(0.2);
        MaximumAngularVelocity = static_cast<Jitter2::Real>(200);
        NumberOfRays = 1;
    }

    [[nodiscard]] JVector GetWheelCenter() const
    {
        return Position + JQuaternion::Transform(Up, Car.Orientation()) * Displacement;
    }

    void AddTorque(Jitter2::Real value)
    {
        DriveTorque += value;
    }

    void PostStep(Jitter2::Real timeStep)
    {
        if (timeStep <= static_cast<Jitter2::Real>(0.0))
        {
            return;
        }

        Jitter2::Real origAngVel = AngularVelocityValue;
        UpSpeed = (Displacement - LastDisplacement) / timeStep;

        if (Locked)
        {
            AngularVelocityValue = static_cast<Jitter2::Real>(0);
            Torque = static_cast<Jitter2::Real>(0);
        }
        else
        {
            AngularVelocityValue += Torque * timeStep / Inertia;
            Torque = static_cast<Jitter2::Real>(0);

            if (!OnFloor)
            {
                DriveTorque *= static_cast<Jitter2::Real>(0.1);
            }

            if ((origAngVel > AngularVelocityForGrip && AngularVelocityValue < AngularVelocityForGrip)
                || (origAngVel < AngularVelocityForGrip && AngularVelocityValue > AngularVelocityForGrip))
            {
                AngularVelocityValue = AngularVelocityForGrip;
            }

            AngularVelocityValue += DriveTorque * timeStep / Inertia;
            DriveTorque = static_cast<Jitter2::Real>(0);

            Jitter2::Real maxAngVel = MaximumAngularVelocity;
            AngularVelocityValue = std::clamp(AngularVelocityValue, -maxAngVel, maxAngVel);

            WheelRotation += timeStep * AngularVelocityValue;
        }
    }

    void PreStep(Jitter2::Real timeStep)
    {
        (void)timeStep;
        JVector force = JVector::Zero();
        LastDisplacement = Displacement;
        Displacement = static_cast<Jitter2::Real>(0.0);

        Jitter2::Real vel = Car.Velocity().Length();
        (void)vel;

        JVector worldPos = Car.Position() + JQuaternion::Transform(Position, Car.Orientation());
        JVector worldAxis = JQuaternion::Transform(Up, Car.Orientation());

        JVector forward = JQuaternion::Transform(-JVector::UnitZ(), Car.Orientation());
        Jitter2::LinearMath::JMatrix rotation =
            Jitter2::LinearMath::JMatrix::CreateRotationMatrix(worldAxis, SteerAngle);
        JVector wheelFwd = Jitter2::LinearMath::JMatrix::Transform(forward, rotation);

        JVector wheelLeft = JVector::Cross(worldAxis, wheelFwd);
        wheelLeft.Normalize();

        JVector wheelUp = JVector::Cross(wheelFwd, wheelLeft);

        Jitter2::Real rayLen = static_cast<Jitter2::Real>(2.0) * Radius + WheelTravel;

        JVector wheelRayEnd = worldPos - Radius * worldAxis;
        JVector wheelRayOrigin = wheelRayEnd + rayLen * worldAxis;
        JVector wheelRayDelta = wheelRayEnd - wheelRayOrigin;

        Jitter2::Real deltaFwd = static_cast<Jitter2::Real>(2.0) * Radius
            / static_cast<Jitter2::Real>(NumberOfRays + 1);
        Jitter2::Real deltaFwdStart = deltaFwd;

        OnFloor = false;

        JVector groundNormal = JVector::Zero();
        JVector groundPos = JVector::Zero();
        Jitter2::Real deepestFrac = std::numeric_limits<Jitter2::Real>::max();
        Jitter2::RigidBody* worldBody = nullptr;

        for (int i = 0; i < NumberOfRays; ++i)
        {
            Jitter2::Real distFwd = deltaFwdStart + static_cast<Jitter2::Real>(i) * deltaFwd - Radius;
            Jitter2::Real zOffset = Radius * (static_cast<Jitter2::Real>(1.0)
                - std::cos(static_cast<Jitter2::Real>(Pi / 2.0f) * (distFwd / Radius)));

            JVector newOrigin = wheelRayOrigin + distFwd * wheelFwd + zOffset * wheelUp;

            Jitter2::Collision::DynamicTree::Proxy* shape = nullptr;
            JVector normal;
            Jitter2::Real frac = static_cast<Jitter2::Real>(0);

            const bool result = WorldRef.DynamicTree().RayCast(
                newOrigin,
                wheelRayDelta,
                [this](const Jitter2::Collision::DynamicTree::Proxy& proxy)
                {
                    const auto* rbs = dynamic_cast<const Shapes::RigidBodyShape*>(&proxy);
                    return rbs != nullptr && rbs->GetRigidBody() != &Car;
                },
                {},
                shape,
                normal,
                frac);

            if (result && frac <= static_cast<Jitter2::Real>(1.0))
            {
                auto* rigidBodyShape = dynamic_cast<Shapes::RigidBodyShape*>(shape);
                Jitter2::RigidBody* body = rigidBodyShape != nullptr ? rigidBodyShape->GetRigidBody() : nullptr;

                if (body != nullptr && frac < deepestFrac)
                {
                    deepestFrac = frac;
                    groundPos = newOrigin + frac * wheelRayDelta;
                    worldBody = body;
                    groundNormal = normal;
                }

                OnFloor = true;
            }
        }

        if (!OnFloor)
        {
            return;
        }

        if (groundNormal.LengthSquared() > static_cast<Jitter2::Real>(0.0))
        {
            groundNormal.Normalize();
        }

        Displacement = rayLen * (static_cast<Jitter2::Real>(1.0) - deepestFrac);
        Displacement = std::clamp(Displacement, static_cast<Jitter2::Real>(0.0), WheelTravel);

        Jitter2::Real displacementForceMag = Displacement * Spring;
        displacementForceMag *= JVector::Dot(groundNormal, worldAxis);

        Jitter2::Real dampingForceMag = UpSpeed * Damping;

        Jitter2::Real totalForceMag = displacementForceMag + dampingForceMag;

        if (totalForceMag < static_cast<Jitter2::Real>(0.0))
        {
            totalForceMag = static_cast<Jitter2::Real>(0.0);
        }

        JVector extraForce = totalForceMag * worldAxis;

        force += extraForce;

        JVector groundUp = groundNormal;
        JVector groundLeft = JVector::Cross(groundNormal, wheelFwd);
        if (groundLeft.LengthSquared() > static_cast<Jitter2::Real>(0.0))
        {
            groundLeft.Normalize();
        }

        JVector groundFwd = JVector::Cross(groundLeft, groundUp);

        JVector wheelPointVel = Car.Velocity()
            + JVector::Cross(Car.AngularVelocity(), JQuaternion::Transform(Position, Car.Orientation()));

        JVector rimVel = AngularVelocityValue * JVector::Cross(wheelLeft, groundPos - worldPos);
        wheelPointVel += rimVel;

        if (worldBody == nullptr)
        {
            throw std::runtime_error("car: world body is null.");
        }

        JVector worldVel = worldBody->Velocity()
            + JVector::Cross(worldBody->AngularVelocity(), groundPos - worldBody->Position());

        wheelPointVel -= worldVel;

        Jitter2::Real noslipVel = static_cast<Jitter2::Real>(0.2);
        Jitter2::Real slipVel = static_cast<Jitter2::Real>(0.4);
        Jitter2::Real slipFactor = static_cast<Jitter2::Real>(0.7);

        Jitter2::Real smallVel = static_cast<Jitter2::Real>(3.0);
        Jitter2::Real friction = SideFriction;

        Jitter2::Real sideVel = JVector::Dot(wheelPointVel, groundLeft);

        if (sideVel > slipVel || sideVel < -slipVel)
        {
            friction *= slipFactor;
        }
        else if (sideVel > noslipVel || sideVel < -noslipVel)
        {
            friction *= static_cast<Jitter2::Real>(1.0)
                - (static_cast<Jitter2::Real>(1.0) - slipFactor)
                * (std::abs(sideVel) - noslipVel) / (slipVel - noslipVel);
        }

        if (sideVel < static_cast<Jitter2::Real>(0.0))
        {
            friction *= static_cast<Jitter2::Real>(-1.0);
        }

        if (std::abs(sideVel) < smallVel)
        {
            friction *= std::abs(sideVel) / smallVel;
        }

        Jitter2::Real sideForce = -friction * totalForceMag;

        extraForce = sideForce * groundLeft;
        force += extraForce;

        friction = ForwardFriction;
        Jitter2::Real fwdVel = JVector::Dot(wheelPointVel, groundFwd);

        if (fwdVel > slipVel || fwdVel < -slipVel)
        {
            friction *= slipFactor;
        }
        else if (fwdVel > noslipVel || fwdVel < -noslipVel)
        {
            friction *= static_cast<Jitter2::Real>(1.0)
                - (static_cast<Jitter2::Real>(1.0) - slipFactor)
                * (std::abs(fwdVel) - noslipVel) / (slipVel - noslipVel);
        }

        if (fwdVel < static_cast<Jitter2::Real>(0.0))
        {
            friction *= static_cast<Jitter2::Real>(-1.0);
        }

        if (std::abs(fwdVel) < smallVel)
        {
            friction *= std::abs(fwdVel) / smallVel;
        }

        Jitter2::Real fwdForce = -friction * totalForceMag;

        extraForce = fwdForce * groundFwd;
        force += extraForce;

        JVector wheelCentreVel = Car.Velocity()
            + JVector::Cross(Car.AngularVelocity(), JQuaternion::Transform(Position, Car.Orientation()));

        AngularVelocityForGrip = JVector::Dot(wheelCentreVel, groundFwd) / Radius;
        Torque += -fwdForce * Radius;

        Car.AddForce(force, groundPos);

        if (DebugRendererInstance != nullptr)
        {
            DebugRendererInstance->PushPoint(DebugRenderer::Color::White, FromJitter(groundPos), 0.2f);
        }

        if (worldBody->MotionTypeValue() == Jitter2::MotionType::Dynamic)
        {
            constexpr Jitter2::Real maxOtherBodyAcc = static_cast<Jitter2::Real>(500.0);
            Jitter2::Real maxOtherBodyForce = maxOtherBodyAcc * worldBody->Mass();

            if (force.LengthSquared() > (maxOtherBodyForce * maxOtherBodyForce))
            {
                force *= maxOtherBodyForce / force.Length();
            }

            worldBody->SetActivationState(true);

            worldBody->AddForce(force * static_cast<Jitter2::Real>(-1), groundPos);
        }
    }

    Jitter2::Real SteerAngle = static_cast<Jitter2::Real>(0);
    Jitter2::Real WheelRotation = static_cast<Jitter2::Real>(0);
    Jitter2::Real Damping = static_cast<Jitter2::Real>(0);
    Jitter2::Real Spring = static_cast<Jitter2::Real>(0);
    Jitter2::Real Inertia = static_cast<Jitter2::Real>(1);
    Jitter2::Real Radius = static_cast<Jitter2::Real>(0);
    Jitter2::Real SideFriction = static_cast<Jitter2::Real>(0);
    Jitter2::Real ForwardFriction = static_cast<Jitter2::Real>(0);
    Jitter2::Real WheelTravel = static_cast<Jitter2::Real>(0);
    bool Locked = false;
    Jitter2::Real MaximumAngularVelocity = static_cast<Jitter2::Real>(0);
    int NumberOfRays = 0;
    JVector Position = JVector::Zero();
    [[nodiscard]] Jitter2::Real AngularVelocity() const { return AngularVelocityValue; }

    DebugRenderer* DebugRendererInstance = nullptr;
    const JVector Up = JVector::UnitY();

private:
    Jitter2::Real Displacement = static_cast<Jitter2::Real>(0);
    Jitter2::Real UpSpeed = static_cast<Jitter2::Real>(0);
    Jitter2::Real LastDisplacement = static_cast<Jitter2::Real>(0);
    bool OnFloor = false;
    Jitter2::Real DriveTorque = static_cast<Jitter2::Real>(0);
    Jitter2::Real AngularVelocityValue = static_cast<Jitter2::Real>(0);
    Jitter2::Real AngularVelocityForGrip = static_cast<Jitter2::Real>(0);
    Jitter2::Real Torque = static_cast<Jitter2::Real>(0);
};
