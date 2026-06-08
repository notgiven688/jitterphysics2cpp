// Common DemoScene helpers; included inside class DemoScene.

    void AddFloor()
    {
        Jitter2::RigidBody& body = World.CreateRigidBody();
        FloorShape = &CreateShape<Shapes::BoxShape>(JVector(200, 200, 200));
        body.Position(JVector(0, -100, 0));
        body.MotionTypeValue(Jitter2::MotionType::Static);
        body.AddShape(*FloorShape);
    }

    template<typename TShape, typename... TArgs>
    TShape& CreateShape(TArgs&&... args)
    {
        auto shape = std::make_unique<TShape>(std::forward<TArgs>(args)...);
        TShape& reference = *shape;
        OwnedShapes.push_back(std::move(shape));
        return reference;
    }

    Jitter2::RigidBody& AddBox(
        const JVector& size,
        const JVector& position,
        Jitter2::MotionType motionType,
        const JQuaternion& orientation = JQuaternion::Identity(),
        BodyTrackParams track = {})
    {
        Jitter2::RigidBody& body = CreateBody(position, orientation, motionType);
        body.AddShape(CreateShape<Shapes::BoxShape>(size));
        if (track.Bob != 0.0f || track.Orbit != 0.0f || track.Spin != 0.0f || track.Tilt != 0.0f)
        {
            AddTrack(body, position, track.Phase, track.Bob, track.Orbit, track.Spin, track.Tilt);
        }
        return body;
    }


    Jitter2::RigidBody& AddSphere(
        Jitter2::Real radius,
        const JVector& position,
        Jitter2::MotionType motionType,
        const JQuaternion& orientation = JQuaternion::Identity())
    {
        Jitter2::RigidBody& body = CreateBody(position, orientation, motionType);
        body.AddShape(CreateShape<Shapes::SphereShape>(radius));
        return body;
    }

    Jitter2::RigidBody& AddCapsule(
        Jitter2::Real radius,
        Jitter2::Real length,
        const JVector& position,
        Jitter2::MotionType motionType,
        const JQuaternion& orientation = JQuaternion::Identity())
    {
        Jitter2::RigidBody& body = CreateBody(position, orientation, motionType);
        body.AddShape(CreateShape<Shapes::CapsuleShape>(radius, length));
        return body;
    }

    Jitter2::RigidBody& AddCylinder(
        Jitter2::Real height,
        Jitter2::Real radius,
        const JVector& position,
        Jitter2::MotionType motionType,
        const JQuaternion& orientation = JQuaternion::Identity())
    {
        Jitter2::RigidBody& body = CreateBody(position, orientation, motionType);
        body.AddShape(CreateShape<Shapes::CylinderShape>(height, radius));
        return body;
    }

    Jitter2::RigidBody& AddCone(
        Jitter2::Real radius,
        Jitter2::Real height,
        const JVector& position,
        Jitter2::MotionType motionType,
        const JQuaternion& orientation = JQuaternion::Identity())
    {
        Jitter2::RigidBody& body = CreateBody(position, orientation, motionType);
        body.AddShape(CreateShape<Shapes::ConeShape>(radius, height));
        return body;
    }

    void BuildPyramid(
        const JVector& position,
        int size,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        for (int i = 0; i < size; ++i)
        {
            for (int e = i; e < size; ++e)
            {
                Jitter2::RigidBody& body = World.CreateRigidBody();
                body.Position(position + JVector(
                        static_cast<Jitter2::Real>((static_cast<double>(e) - static_cast<double>(i) * 0.5) * 1.01),
                        static_cast<Jitter2::Real>(0.5 + static_cast<double>(i)),
                        0));
                Shapes::BoxShape& shape = CreateShape<Shapes::BoxShape>(JVector(1));
                body.AddShape(shape);
                if (action)
                {
                    action(body);
                }
            }
        }
    }


    void BuildTower(
        const JVector& pos,
        int height,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        JQuaternion halfRotationStep =
            JQuaternion::CreateRotationY(static_cast<Jitter2::Real>(Pi * 2.0f / 64.0f));
        JQuaternion fullRotationStep = halfRotationStep * halfRotationStep;
        JQuaternion orientation = JQuaternion::Identity();

        for (int e = 0; e < height; ++e)
        {
            orientation = orientation * halfRotationStep;

            for (int i = 0; i < 32; ++i)
            {
                JVector position = pos + JQuaternion::Transform(
                    JVector(0, static_cast<Jitter2::Real>(0.5 + e), static_cast<Jitter2::Real>(19.5)),
                    orientation);

                Shapes::BoxShape& shape = CreateShape<Shapes::BoxShape>(
                    static_cast<Jitter2::Real>(3),
                    static_cast<Jitter2::Real>(1),
                    static_cast<Jitter2::Real>(0.2));
                Jitter2::RigidBody& body = World.CreateRigidBody();

                body.Orientation(orientation);
                body.Position(JVector(position.X, position.Y, position.Z));

                body.AddShape(shape);

                orientation = orientation * fullRotationStep;

                if (action)
                {
                    action(body);
                }
            }
        }
    }

    void BuildJenga(
        const JVector& position,
        int size,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        const JVector base = position + JVector(0, static_cast<Jitter2::Real>(0.5), 0);
        for (int i = 0; i < size; ++i)
        {
            for (int e = 0; e < 3; ++e)
            {
                Jitter2::RigidBody& body = World.CreateRigidBody();

                if (i % 2 == 0)
                {
                    body.AddShape(CreateShape<Shapes::BoxShape>(JVector(3, 1, 1)));
                    body.Position(base + JVector(0, static_cast<Jitter2::Real>(i), static_cast<Jitter2::Real>(-1 + e)));
                }
                else
                {
                    body.AddShape(CreateShape<Shapes::BoxShape>(JVector(1, 1, 3)));
                    body.Position(base + JVector(static_cast<Jitter2::Real>(-1 + e), static_cast<Jitter2::Real>(i), 0));
                }

                if (action)
                {
                    action(body);
                }
            }
        }
    }

    void BuildPyramidCylinder(
        const JVector& position,
        int size,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        for (int i = 0; i < size; ++i)
        {
            for (int e = i; e < size; ++e)
            {
                Jitter2::RigidBody& body = World.CreateRigidBody();
                body.Position(position + JVector(
                        static_cast<Jitter2::Real>((static_cast<double>(e) - static_cast<double>(i) * 0.5) * 1.01),
                        static_cast<Jitter2::Real>(0.5 + static_cast<double>(i)),
                        0));
                Shapes::CylinderShape& shape = CreateShape<Shapes::CylinderShape>(
                    static_cast<Jitter2::Real>(1),
                    static_cast<Jitter2::Real>(0.5));
                body.AddShape(shape);
                if (action)
                {
                    action(body);
                }
            }
        }
    }

    void BuildWall(
        const JVector& position,
        int sizeX = 20,
        int sizeY = 14,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        for (int i = 0; i < sizeX; ++i)
        {
            for (int e = 0; e < sizeY; ++e)
            {
                Jitter2::RigidBody& body = World.CreateRigidBody();
                body.Position(position + JVector(
                        static_cast<Jitter2::Real>((i % 2 == 0 ? 0.5 : 0.0) + static_cast<double>(e) * 2.01),
                        static_cast<Jitter2::Real>(0.5 + static_cast<double>(i)),
                        0));
                Shapes::BoxShape& shape = CreateShape<Shapes::BoxShape>(JVector(2, 1, 1));
                body.AddShape(shape);
                if (action)
                {
                    action(body);
                }
            }
        }
    }

    void BuildRagdoll(
        const JVector& position,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        enum RagdollParts
        {
            Head,
            UpperLegLeft,
            UpperLegRight,
            LowerLegLeft,
            LowerLegRight,
            UpperArmLeft,
            UpperArmRight,
            LowerArmLeft,
            LowerArmRight,
            Torso,
            RagdollPartCount
        };

        std::array<Jitter2::RigidBody*, RagdollPartCount> parts {};
        for (Jitter2::RigidBody*& part : parts)
        {
            part = &World.CreateRigidBody();
        }

        Jitter2::RigidBody& head = *parts[Head];
        Jitter2::RigidBody& torso = *parts[Torso];
        Jitter2::RigidBody& upperLegLeft = *parts[UpperLegLeft];
        Jitter2::RigidBody& upperLegRight = *parts[UpperLegRight];
        Jitter2::RigidBody& lowerLegLeft = *parts[LowerLegLeft];
        Jitter2::RigidBody& lowerLegRight = *parts[LowerLegRight];
        Jitter2::RigidBody& upperArmLeft = *parts[UpperArmLeft];
        Jitter2::RigidBody& upperArmRight = *parts[UpperArmRight];
        Jitter2::RigidBody& lowerArmLeft = *parts[LowerArmLeft];
        Jitter2::RigidBody& lowerArmRight = *parts[LowerArmRight];

        head.AddShape(CreateShape<Shapes::SphereShape>(static_cast<Jitter2::Real>(0.15)));
        upperLegLeft.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.08), static_cast<Jitter2::Real>(0.3)));
        upperLegRight.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.08), static_cast<Jitter2::Real>(0.3)));
        lowerLegLeft.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.08), static_cast<Jitter2::Real>(0.3)));
        lowerLegRight.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.08), static_cast<Jitter2::Real>(0.3)));
        upperArmLeft.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.07), static_cast<Jitter2::Real>(0.2)));
        upperArmRight.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.07), static_cast<Jitter2::Real>(0.2)));
        lowerArmLeft.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.06), static_cast<Jitter2::Real>(0.2)));
        lowerArmRight.AddShape(CreateShape<Shapes::CapsuleShape>(static_cast<Jitter2::Real>(0.06), static_cast<Jitter2::Real>(0.2)));
        torso.AddShape(CreateShape<Shapes::BoxShape>(
            JVector(static_cast<Jitter2::Real>(0.35), static_cast<Jitter2::Real>(0.6), static_cast<Jitter2::Real>(0.2))));

        head.Position(JVector(0, 0, 0));
        torso.Position(JVector(0, static_cast<Jitter2::Real>(-0.46), 0));
        upperLegLeft.Position(JVector(static_cast<Jitter2::Real>(0.11), static_cast<Jitter2::Real>(-0.85), 0));
        upperLegRight.Position(JVector(static_cast<Jitter2::Real>(-0.11), static_cast<Jitter2::Real>(-0.85), 0));
        lowerLegLeft.Position(JVector(static_cast<Jitter2::Real>(0.11), static_cast<Jitter2::Real>(-1.2), 0));
        lowerLegRight.Position(JVector(static_cast<Jitter2::Real>(-0.11), static_cast<Jitter2::Real>(-1.2), 0));

        const JQuaternion armRotation = JQuaternion::CreateRotationZ(static_cast<Jitter2::Real>(Pi / 2.0f));
        upperArmLeft.Orientation(armRotation);
        upperArmRight.Orientation(armRotation);
        lowerArmLeft.Orientation(armRotation);
        lowerArmRight.Orientation(armRotation);

        upperArmLeft.Position(JVector(static_cast<Jitter2::Real>(0.30), static_cast<Jitter2::Real>(-0.2), 0));
        upperArmRight.Position(JVector(static_cast<Jitter2::Real>(-0.30), static_cast<Jitter2::Real>(-0.2), 0));
        lowerArmLeft.Position(JVector(static_cast<Jitter2::Real>(0.55), static_cast<Jitter2::Real>(-0.2), 0));
        lowerArmRight.Position(JVector(static_cast<Jitter2::Real>(-0.55), static_cast<Jitter2::Real>(-0.2), 0));

        auto& spine0 = World.CreateConstraint<Constraints::BallSocket>(head, torso);
        spine0.Initialize(JVector(0, static_cast<Jitter2::Real>(-0.15), 0));

        auto& spine1 = World.CreateConstraint<Constraints::ConeLimit>(head, torso);
        spine1.Initialize(-JVector::UnitZ(), Constraints::AngularLimit::FromDegree(0, 45));

        auto& hipLeft0 = World.CreateConstraint<Constraints::BallSocket>(torso, upperLegLeft);
        hipLeft0.Initialize(JVector(static_cast<Jitter2::Real>(0.11), static_cast<Jitter2::Real>(-0.7), 0));

        auto& hipLeft1 = World.CreateConstraint<Constraints::TwistAngle>(torso, upperLegLeft);
        hipLeft1.Initialize(JVector::UnitY(), JVector::UnitY(), Constraints::AngularLimit::FromDegree(-80, 80));

        auto& hipLeft2 = World.CreateConstraint<Constraints::ConeLimit>(torso, upperLegLeft);
        hipLeft2.Initialize(-JVector::UnitY(), Constraints::AngularLimit::FromDegree(0, 60));

        auto& hipRight0 = World.CreateConstraint<Constraints::BallSocket>(torso, upperLegRight);
        hipRight0.Initialize(JVector(static_cast<Jitter2::Real>(-0.11), static_cast<Jitter2::Real>(-0.7), 0));

        auto& hipRight1 = World.CreateConstraint<Constraints::TwistAngle>(torso, upperLegRight);
        hipRight1.Initialize(JVector::UnitY(), JVector::UnitY(), Constraints::AngularLimit::FromDegree(-80, 80));

        auto& hipRight2 = World.CreateConstraint<Constraints::ConeLimit>(torso, upperLegRight);
        hipRight2.Initialize(-JVector::UnitY(), Constraints::AngularLimit::FromDegree(0, 60));

        Constraints::HingeJoint kneeLeft(
            World,
            upperLegLeft,
            lowerLegLeft,
            JVector(static_cast<Jitter2::Real>(0.11), static_cast<Jitter2::Real>(-1.05), 0),
            JVector::UnitX(),
            Constraints::AngularLimit::FromDegree(-120, 0));

        Constraints::HingeJoint kneeRight(
            World,
            upperLegRight,
            lowerLegRight,
            JVector(static_cast<Jitter2::Real>(-0.11), static_cast<Jitter2::Real>(-1.05), 0),
            JVector::UnitX(),
            Constraints::AngularLimit::FromDegree(-120, 0));

        Constraints::HingeJoint armLeft(
            World,
            lowerArmLeft,
            upperArmLeft,
            JVector(static_cast<Jitter2::Real>(0.42), static_cast<Jitter2::Real>(-0.2), 0),
            JVector::UnitY(),
            Constraints::AngularLimit::FromDegree(-160, 0));

        Constraints::HingeJoint armRight(
            World,
            lowerArmRight,
            upperArmRight,
            JVector(static_cast<Jitter2::Real>(-0.42), static_cast<Jitter2::Real>(-0.2), 0),
            JVector::UnitY(),
            Constraints::AngularLimit::FromDegree(0, 160));

        kneeLeft.HingeAngleConstraint().LimitSoftness(static_cast<Jitter2::Real>(1));
        kneeLeft.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(1));
        kneeRight.HingeAngleConstraint().LimitSoftness(static_cast<Jitter2::Real>(1));
        kneeRight.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(1));
        armLeft.HingeAngleConstraint().LimitSoftness(static_cast<Jitter2::Real>(1));
        armLeft.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(1));
        armRight.HingeAngleConstraint().LimitSoftness(static_cast<Jitter2::Real>(1));
        armRight.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(1));

        auto& shoulderLeft0 = World.CreateConstraint<Constraints::BallSocket>(upperArmLeft, torso);
        shoulderLeft0.Initialize(JVector(static_cast<Jitter2::Real>(0.20), static_cast<Jitter2::Real>(-0.2), 0));

        auto& shoulderLeft1 = World.CreateConstraint<Constraints::TwistAngle>(torso, upperArmLeft);
        shoulderLeft1.Initialize(JVector::UnitX(), JVector::UnitX(), Constraints::AngularLimit::FromDegree(-20, 60));

        auto& shoulderRight0 = World.CreateConstraint<Constraints::BallSocket>(upperArmRight, torso);
        shoulderRight0.Initialize(JVector(static_cast<Jitter2::Real>(-0.20), static_cast<Jitter2::Real>(-0.2), 0));

        auto& shoulderRight1 = World.CreateConstraint<Constraints::TwistAngle>(torso, upperArmRight);
        shoulderRight1.Initialize(JVector::UnitX(), JVector::UnitX(), Constraints::AngularLimit::FromDegree(-20, 60));

        shoulderLeft1.Bias(static_cast<Jitter2::Real>(0.01));
        shoulderRight1.Bias(static_cast<Jitter2::Real>(0.01));
        hipLeft1.Bias(static_cast<Jitter2::Real>(0.01));
        hipRight1.Bias(static_cast<Jitter2::Real>(0.01));

        IgnoreCollisionBetween(*upperLegLeft.Shapes()[0], *torso.Shapes()[0]);
        IgnoreCollisionBetween(*upperLegRight.Shapes()[0], *torso.Shapes()[0]);
        IgnoreCollisionBetween(*upperLegLeft.Shapes()[0], *lowerLegLeft.Shapes()[0]);
        IgnoreCollisionBetween(*upperLegRight.Shapes()[0], *lowerLegRight.Shapes()[0]);
        IgnoreCollisionBetween(*upperArmLeft.Shapes()[0], *torso.Shapes()[0]);
        IgnoreCollisionBetween(*upperArmRight.Shapes()[0], *torso.Shapes()[0]);
        IgnoreCollisionBetween(*upperArmLeft.Shapes()[0], *lowerArmLeft.Shapes()[0]);
        IgnoreCollisionBetween(*upperArmRight.Shapes()[0], *lowerArmRight.Shapes()[0]);

        for (Jitter2::RigidBody* part : parts)
        {
            part->Position(part->Position() + position);
            if (action)
            {
                action(*part);
            }
        }
    }


    void EnsureIgnoreCollisionFilter()
    {
        if (dynamic_cast<IgnoreCollisionBetweenFilter*>(World.BroadPhaseFilter) == nullptr)
        {
            IgnoreCollisionFilter = std::make_unique<IgnoreCollisionBetweenFilter>();
            World.BroadPhaseFilter = IgnoreCollisionFilter.get();
        }
    }

    void IgnoreCollisionBetween(const Shapes::RigidBodyShape& shapeA, const Shapes::RigidBodyShape& shapeB)
    {
        EnsureIgnoreCollisionFilter();
        auto* filter = dynamic_cast<IgnoreCollisionBetweenFilter*>(World.BroadPhaseFilter);
        filter->IgnoreCollisionBetween(shapeA, shapeB);
    }

    Jitter2::RigidBody& CreateBody(
        const JVector& position,
        const JQuaternion& orientation,
        Jitter2::MotionType motionType)
    {
        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.Position(position);
        body.Orientation(orientation);
        body.MotionTypeValue(motionType);
        return body;
    }

    void AddTrack(
        Jitter2::RigidBody& body,
        const JVector& basePosition,
        float phase,
        float bob,
        float orbit,
        float spin,
        float tilt)
    {
        Tracks.push_back(BodyTrack {&body, basePosition, phase, bob, orbit, spin, tilt});
    }
