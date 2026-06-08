// Member functions for DemoScene; included inside class DemoScene.

    void BuildMotorAndLimitScene()
    {
        AddFloor();

        {
            Jitter2::RigidBody& b0 = World.CreateRigidBody();
            b0.AddShape(CreateShape<Shapes::BoxShape>(
                JVector(2, static_cast<Jitter2::Real>(0.1), static_cast<Jitter2::Real>(0.1))));
            b0.Position(JVector(static_cast<Jitter2::Real>(-1.1), 4, 0));

            Jitter2::RigidBody& b1 = World.CreateRigidBody();
            b1.AddShape(CreateShape<Shapes::BoxShape>(
                JVector(2, static_cast<Jitter2::Real>(0.1), static_cast<Jitter2::Real>(0.1))));
            b1.Position(JVector(static_cast<Jitter2::Real>(1.1), 4, 0));

            Constraints::HingeJoint hinge(
                World,
                World.NullBody(),
                b0,
                b0.Position(),
                JVector::UnitX(),
                Constraints::AngularLimit::Full(),
                true);

            Constraints::UniversalJoint universal(
                World,
                b0,
                b1,
                JVector(0, 4, 0),
                JVector::UnitX(),
                JVector::UnitX());
            (void)universal;

            if (hinge.Motor() != nullptr)
            {
                hinge.Motor()->TargetVelocity(static_cast<Jitter2::Real>(4));
                hinge.Motor()->MaximumForce(static_cast<Jitter2::Real>(1));
            }

            IgnoreCollisionBetween(*b0.Shapes()[0], *b1.Shapes()[0]);
        }

        {
            Jitter2::RigidBody& b0 = World.CreateRigidBody();
            b0.AddShape(CreateShape<Shapes::BoxShape>(
                JVector(2, static_cast<Jitter2::Real>(0.1), 3)));
            b0.AddShape(CreateShape<Shapes::BoxShape>(
                JVector(static_cast<Jitter2::Real>(0.1), 2, static_cast<Jitter2::Real>(2.9))));
            b0.Position(JVector(-5, 3, 0));

            Constraints::HingeJoint hinge(
                World,
                World.NullBody(),
                b0,
                b0.Position(),
                JVector::UnitZ(),
                Constraints::AngularLimit::FromDegree(-120, 120));
            hinge.HingeAngleConstraint().Bias(static_cast<Jitter2::Real>(0.3));
            hinge.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(0));
            hinge.HingeAngleConstraint().LimitBias(static_cast<Jitter2::Real>(0.3));
            hinge.HingeAngleConstraint().LimitSoftness(static_cast<Jitter2::Real>(0));

            for (int i = 0; i < 4; ++i)
            {
                BuildRagdoll(JVector(-4, static_cast<Jitter2::Real>(5 + i * 3), 0));
            }
        }

        {
            const Jitter2::Real angle = static_cast<Jitter2::Real>(Pi / 2.0f);
            const JVector rot1Axis = JQuaternion::Transform(JVector::UnitZ(), JQuaternion::CreateRotationY(angle));

            Jitter2::RigidBody& b0 = World.CreateRigidBody();
            b0.Position(JVector(5, 4, 0));
            b0.Orientation(JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(Pi / 2.0f)));
            b0.AddShape(CreateShape<Shapes::CylinderShape>(
                static_cast<Jitter2::Real>(0.4),
                static_cast<Jitter2::Real>(2.0)));

            Jitter2::RigidBody& b1 = World.CreateRigidBody();
            b1.AddShape(CreateShape<Shapes::CylinderShape>(
                static_cast<Jitter2::Real>(0.4),
                static_cast<Jitter2::Real>(2.0)));
            b1.Position(JVector(static_cast<Jitter2::Real>(9.2), 4, 0));
            b1.Orientation(JQuaternion::CreateRotationY(angle)
                * JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(Pi / 2.0f)));

            Constraints::HingeJoint hinge1(
                World,
                World.NullBody(),
                b0,
                b0.Position(),
                JVector::UnitZ(),
                Constraints::AngularLimit::Full());

            Constraints::HingeJoint hinge2(
                World,
                World.NullBody(),
                b1,
                b1.Position(),
                rot1Axis,
                Constraints::AngularLimit::Full());

            hinge1.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(0));
            hinge2.HingeAngleConstraint().Softness(static_cast<Jitter2::Real>(0));

            auto& relative = World.CreateConstraint<Constraints::TwistAngle>(b0, b1);
            relative.Initialize(JVector::UnitZ(), rot1Axis);
        }

        World.SolverIterations(4, 2);
        World.SubstepCount = 3;
    }
