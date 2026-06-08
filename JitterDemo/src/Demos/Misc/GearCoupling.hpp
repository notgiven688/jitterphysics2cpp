struct GearCoupling
{
    Constraints::DistanceLimit* DistanceLimit = nullptr;
    std::unique_ptr<Constraints::HingeJoint> HingeJoint1;
    std::unique_ptr<Constraints::HingeJoint> HingeJoint2;
    Jitter2::RigidBody* Body1 = nullptr;
    Jitter2::RigidBody* Body2 = nullptr;
    JQuaternion InitialOrientation1 = JQuaternion::Identity();
    JQuaternion InitialOrientation2 = JQuaternion::Identity();
    JVector ContactPoint = JVector::Zero();
    JVector LocalAxis1 = JVector::UnitZ();
    JVector LocalAxis2 = JVector::UnitZ();
    Jitter2::Real PreviousAngle1 = 0;
    Jitter2::Real PreviousAngle2 = 0;
    Jitter2::Real GearRatio = 1;
    int Rotations1 = 0;
    int Rotations2 = 0;
    Jitter2::World* World = nullptr;
    Jitter2::World::WorldStepFunction::Token PreSubStepToken = 0;

    GearCoupling(
        Jitter2::World& world,
        Jitter2::RigidBody& body1,
        Jitter2::RigidBody& body2,
        JVector rotationAxis1,
        JVector rotationAxis2,
        const JVector& contactPoint)
        : Body1(&body1),
          Body2(&body2),
          InitialOrientation1(body1.Orientation()),
          InitialOrientation2(body2.Orientation()),
          ContactPoint(contactPoint),
          World(&world)
    {
        rotationAxis1.Normalize();
        rotationAxis2.Normalize();

        LocalAxis1 = JQuaternion::ConjugatedTransform(rotationAxis1, body1.Orientation());
        LocalAxis2 = JQuaternion::ConjugatedTransform(rotationAxis2, body2.Orientation());

        HingeJoint1 = std::make_unique<Constraints::HingeJoint>(
            world,
            world.NullBody(),
            body1,
            body1.Position(),
            rotationAxis1,
            Constraints::AngularLimit::Full());
        HingeJoint2 = std::make_unique<Constraints::HingeJoint>(
            world,
            world.NullBody(),
            body2,
            body2.Position(),
            rotationAxis2,
            Constraints::AngularLimit::Full());

        HingeJoint1->HingeAngleConstraint().Bias(static_cast<Jitter2::Real>(0.3));
        HingeJoint2->HingeAngleConstraint().Bias(static_cast<Jitter2::Real>(0.3));
        HingeJoint1->HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(0));
        HingeJoint2->HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(0));

        DistanceLimit = &world.CreateConstraint<Constraints::DistanceLimit>(body1, body2);
        DistanceLimit->Initialize(contactPoint, contactPoint, Constraints::LinearLimit::Fixed());
        DistanceLimit->Bias(static_cast<Jitter2::Real>(0.1));
        DistanceLimit->Softness(static_cast<Jitter2::Real>(0));

        const Jitter2::Real r1 = (body1.Position() - contactPoint).Length();
        const Jitter2::Real r2 = (body2.Position() - contactPoint).Length();
        GearRatio = r1 / r2;

        PreviousAngle1 = GetTwistAngle(body1, InitialOrientation1, LocalAxis1);
        PreviousAngle2 = GetTwistAngle(body2, InitialOrientation2, LocalAxis2);

        PreSubStepToken = world.PreSubStep.Add([this](Jitter2::Real)
        {
            OnPreStep();
        });
    }

    ~GearCoupling()
    {
        Remove();
    }

    static Jitter2::Real GetTwistAngle(
        const Jitter2::RigidBody& body,
        const JQuaternion& initialOrientation,
        const JVector& localAxis)
    {
        JQuaternion q = JQuaternion::MultiplyConjugate(body.Orientation(), initialOrientation);
        JVector axis = JQuaternion::Transform(localAxis, body.Orientation());

        Jitter2::Real y = JVector::Dot(q.Vector(), axis);
        Jitter2::Real x = q.Scalar();

        if (x < static_cast<Jitter2::Real>(0))
        {
            x = -x;
            y = -y;
        }

        return static_cast<Jitter2::Real>(2) * std::atan2(y, x);
    }

    [[nodiscard]] Jitter2::Real GearAngle1() const
    {
        return GetTwistAngle(*Body1, InitialOrientation1, LocalAxis1);
    }

    [[nodiscard]] Jitter2::Real GearAngle2() const
    {
        return GetTwistAngle(*Body2, InitialOrientation2, LocalAxis2);
    }

    Jitter2::Real TrackDeltaAngle()
    {
        const Jitter2::Real angle1 = GearAngle1();
        const Jitter2::Real angle2 = GearAngle2();

        const Jitter2::Real deltaAngle1 = angle1 - PreviousAngle1;
        const Jitter2::Real deltaAngle2 = angle2 - PreviousAngle2;

        if (std::abs(deltaAngle1) > static_cast<Jitter2::Real>(Pi))
        {
            Rotations1 -= deltaAngle1 > static_cast<Jitter2::Real>(0) ? 1 : -1;
        }

        if (std::abs(deltaAngle2) > static_cast<Jitter2::Real>(Pi))
        {
            Rotations2 -= deltaAngle2 > static_cast<Jitter2::Real>(0) ? 1 : -1;
        }

        PreviousAngle1 = angle1;
        PreviousAngle2 = angle2;

        const Jitter2::Real totalRotation1 =
            static_cast<Jitter2::Real>(Rotations1) * static_cast<Jitter2::Real>(Pi * 2.0f) + angle1;
        const Jitter2::Real totalRotation2 =
            static_cast<Jitter2::Real>(Rotations2) * static_cast<Jitter2::Real>(Pi * 2.0f) + angle2;

        return totalRotation2 + GearRatio * totalRotation1;
    }

    void OnPreStep()
    {
        Jitter2::Real error = TrackDeltaAngle();
        error = std::clamp(error, static_cast<Jitter2::Real>(-0.1), static_cast<Jitter2::Real>(0.1));

        const JVector axis1 = JQuaternion::Transform(LocalAxis1, Body1->Orientation());
        const JVector radius = ContactPoint - Body1->Position();
        JVector tangent = JVector::Cross(axis1, radius);
        tangent.Normalize();

        const Jitter2::Real radius2 = (Body2->Position() - ContactPoint).Length();
        const JVector offset = tangent * (error * radius2 * static_cast<Jitter2::Real>(0.5));

        DistanceLimit->Anchor1(ContactPoint + offset);
        DistanceLimit->Anchor2(ContactPoint - offset);
    }

    void Remove()
    {
        if (World != nullptr && PreSubStepToken != 0)
        {
            World->PreSubStep.Remove(PreSubStepToken);
            PreSubStepToken = 0;
        }
    }
};
