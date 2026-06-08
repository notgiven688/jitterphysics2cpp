// Member functions for DemoScene; included inside class DemoScene.

    void BuildConveyorBeltScene()
    {
        AddFloor();
        ConveyorPlanks.clear();
        ConveyorPhysicsTime = 0.0;

        ConveyorPreSubStepToken = World.PreSubStep.Add([this](Jitter2::Real dt)
        {
            OnConveyorPreStep(dt);
        });

        constexpr float plankWidth = 0.6f;
        const int plankCount = static_cast<int>(ConveyorTotalLength() / plankWidth);
        const float distanceStep = ConveyorTotalLength() / static_cast<float>(plankCount);

        for (int i = 0; i < plankCount; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.AddShape(CreateShape<Shapes::BoxShape>(
                JVector(static_cast<Jitter2::Real>(1.8), static_cast<Jitter2::Real>(0.1), static_cast<Jitter2::Real>(0.55))));
            body.MotionTypeValue(Jitter2::MotionType::Kinematic);
            body.Friction(static_cast<Jitter2::Real>(1));

            const float distance = static_cast<float>(i) * distanceStep;
            JVector position;
            JVector velocity;
            float angularVelocityY = 0.0f;
            ConveyorGetState(distance, position, velocity, angularVelocityY);

            body.Position(position);
            body.Orientation(ConveyorOrientationFromVelocity(velocity));

            ConveyorPlanks.push_back(ConveyorPlank {&body, distance});
        }

        for (int i = 0; i < 8; ++i)
        {
            Jitter2::RigidBody& box = World.CreateRigidBody();
            box.AddShape(CreateShape<Shapes::BoxShape>(JVector(1)));
            box.Position(JVector(static_cast<Jitter2::Real>(-5 + static_cast<double>(i) * 1.5), 6, -ConveyorRadius()));
        }

        Jitter2::RigidBody& floor = World.CreateRigidBody();
        floor.AddShape(CreateShape<Shapes::BoxShape>(JVector(100, 1, 100)));
        floor.Position(JVector(0, -5, 0));
        floor.MotionTypeValue(Jitter2::MotionType::Static);
    }

    static constexpr float ConveyorSpeed()
    {
        return 2.0f;
    }

    static constexpr float ConveyorStraightLength()
    {
        return 12.0f;
    }

    static constexpr float ConveyorRadius()
    {
        return 6.0f;
    }

    static constexpr float ConveyorTotalLength()
    {
        return ConveyorStraightLength() * 2.0f + Pi * ConveyorRadius() * 2.0f;
    }

    static void ConveyorGetState(float distance, JVector& position, JVector& velocity, float& angularVelocityY)
    {
        distance = std::fmod(distance, ConveyorTotalLength());
        if (distance < 0.0f)
        {
            distance += ConveyorTotalLength();
        }

        const float curveLength = Pi * ConveyorRadius();

        if (distance < ConveyorStraightLength())
        {
            const float t = distance;
            position = JVector(
                static_cast<Jitter2::Real>(-ConveyorStraightLength() * 0.5f + t),
                4,
                -ConveyorRadius());
            velocity = JVector(ConveyorSpeed(), 0, 0);
            angularVelocityY = 0.0f;
        }
        else if (distance < ConveyorStraightLength() + curveLength)
        {
            const float t = distance - ConveyorStraightLength();
            const float angle = -Pi * 0.5f + t / ConveyorRadius();
            position = JVector(
                static_cast<Jitter2::Real>(ConveyorStraightLength() * 0.5f + std::cos(angle) * ConveyorRadius()),
                4,
                static_cast<Jitter2::Real>(std::sin(angle) * ConveyorRadius()));
            velocity = JVector(
                static_cast<Jitter2::Real>(-std::sin(angle) * ConveyorSpeed()),
                0,
                static_cast<Jitter2::Real>(std::cos(angle) * ConveyorSpeed()));
            angularVelocityY = ConveyorSpeed() / ConveyorRadius();
        }
        else if (distance < ConveyorStraightLength() * 2.0f + curveLength)
        {
            const float t = distance - (ConveyorStraightLength() + curveLength);
            position = JVector(
                static_cast<Jitter2::Real>(ConveyorStraightLength() * 0.5f - t),
                4,
                ConveyorRadius());
            velocity = JVector(-ConveyorSpeed(), 0, 0);
            angularVelocityY = 0.0f;
        }
        else
        {
            const float t = distance - (ConveyorStraightLength() * 2.0f + curveLength);
            const float angle = Pi * 0.5f + t / ConveyorRadius();
            position = JVector(
                static_cast<Jitter2::Real>(-ConveyorStraightLength() * 0.5f + std::cos(angle) * ConveyorRadius()),
                4,
                static_cast<Jitter2::Real>(std::sin(angle) * ConveyorRadius()));
            velocity = JVector(
                static_cast<Jitter2::Real>(-std::sin(angle) * ConveyorSpeed()),
                0,
                static_cast<Jitter2::Real>(std::cos(angle) * ConveyorSpeed()));
            angularVelocityY = ConveyorSpeed() / ConveyorRadius();
        }
    }

    static JQuaternion ConveyorOrientationFromVelocity(JVector velocity)
    {
        JVector forward = JVector::Normalize(velocity);
        JVector up = JVector::UnitY();
        JVector right = JVector::Normalize(JVector::Cross(up, forward));
        up = JVector::Cross(forward, right);
        const Jitter2::LinearMath::JMatrix orientation =
            Jitter2::LinearMath::JMatrix::FromColumns(right, up, forward);
        return JQuaternion::CreateFromMatrix(orientation);
    }

    void OnConveyorPreStep(Jitter2::Real dt)
    {
        ConveyorPhysicsTime += static_cast<double>(dt);
        const float globalDistance = static_cast<float>(ConveyorPhysicsTime) * ConveyorSpeed();

        for (ConveyorPlank& plank : ConveyorPlanks)
        {
            if (plank.Body == nullptr)
            {
                continue;
            }

            JVector targetPosition;
            JVector targetVelocity;
            float targetAngularVelocityY = 0.0f;
            ConveyorGetState(
                globalDistance + plank.DistanceOffset,
                targetPosition,
                targetVelocity,
                targetAngularVelocityY);

            plank.Body->Velocity(targetVelocity
                + (targetPosition - plank.Body->Position()) * static_cast<Jitter2::Real>(10));

            const JVector currentForward = plank.Body->Orientation().GetBasisZ();
            JVector targetForward = targetVelocity;
            targetForward.Normalize();

            const Jitter2::Real angleError =
                currentForward.Z * targetForward.X - currentForward.X * targetForward.Z;
            const Jitter2::Real correction = angleError * static_cast<Jitter2::Real>(20);
            plank.Body->AngularVelocity(JVector(
                0,
                static_cast<Jitter2::Real>(targetAngularVelocityY) + correction,
                0));
        }
    }

    static void DrawConveyorBelt(DebugRenderer& debugRenderer)
    {
        constexpr int stepMax = 200;
        const float totalLength = ConveyorTotalLength();

        for (int step = 0; step < stepMax; ++step)
        {
            const float d1 = totalLength / static_cast<float>(stepMax) * static_cast<float>(step);
            const float d2 = totalLength / static_cast<float>(stepMax) * static_cast<float>(step + 1);

            JVector p1;
            JVector p2;
            JVector ignoredVelocity;
            float ignoredAngularVelocity = 0.0f;
            ConveyorGetState(d1, p1, ignoredVelocity, ignoredAngularVelocity);
            ConveyorGetState(d2, p2, ignoredVelocity, ignoredAngularVelocity);

            debugRenderer.PushLine(DebugRenderer::Color::Green, FromJitter(p1), FromJitter(p2));
        }
    }
