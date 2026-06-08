// Member functions for DemoScene; included inside class DemoScene.

    void BuildConstraintCarScene()
    {
        ConstraintCarInstance = std::make_unique<ConstraintCar>();
        ConstraintCarHinges.clear();

        AddFloor();

        {
            Jitter2::RigidBody* body = nullptr;

            JVector startPos(-10, 8, -20);
            constexpr int numElements = 30;

            for (int i = 0; i < numElements; ++i)
            {
                Jitter2::RigidBody& nbody = World.CreateRigidBody();
                nbody.AddShape(CreateShape<Shapes::BoxShape>(
                    static_cast<Jitter2::Real>(0.7),
                    static_cast<Jitter2::Real>(0.1),
                    static_cast<Jitter2::Real>(4.0)));
                nbody.Position(startPos + JVector(static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.8), 0, 0));

                if (i == 0)
                {
                    Constraints::HingeJoint hinge(
                        World,
                        World.NullBody(),
                        nbody,
                        startPos + JVector(
                            static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.8)
                                - static_cast<Jitter2::Real>(0.7),
                            0,
                            0),
                        JVector::UnitZ());
                }
                else
                {
                    auto hinge = std::make_unique<Constraints::HingeJoint>(
                        World,
                        *body,
                        nbody,
                        startPos + JVector(
                            static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.8)
                                - static_cast<Jitter2::Real>(0.1),
                            0,
                            0),
                        JVector::UnitZ());

                    hinge->BallSocketConstraint().Softness(static_cast<Jitter2::Real>(0.1));
                    ConstraintCarHinges.push_back(std::move(hinge));
                }

                if (i == numElements - 1)
                {
                    Constraints::HingeJoint hinge(
                        World,
                        nbody,
                        World.NullBody(),
                        startPos + JVector(
                            static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.8)
                                + static_cast<Jitter2::Real>(0.7),
                            0,
                            0),
                        JVector::UnitZ());
                }

                body = &nbody;
            }
        }

        {
            JVector carPos(10, 9, -20);
            JQuaternion rot = JQuaternion::CreateRotationY(static_cast<Jitter2::Real>(Pi / 2.0f));

            ConstraintCarInstance->BuildCar(
                World,
                carPos,
                [carPos, rot](Jitter2::RigidBody& body)
                {
                    body.Position(JQuaternion::Transform(body.Position(), rot) + carPos);
                    body.Orientation(rot);
                });
        }

        World.SubstepCount = 4;
        World.SolverIterations(2, 2);
    }

    void DrawConstraintCar(GLFWwindow* window)
    {
        for (std::size_t i = ConstraintCarHinges.size(); i-- > 0;)
        {
            Constraints::HingeJoint& hinge = *ConstraintCarHinges[i];
            if (hinge.BallSocketConstraint().Impulse().Length() > static_cast<Jitter2::Real>(0.5))
            {
                hinge.Remove();
                ConstraintCarHinges.erase(ConstraintCarHinges.begin() + static_cast<std::ptrdiff_t>(i));
            }
        }

        if (ConstraintCarInstance != nullptr)
        {
            ConstraintCarInstance->UpdateControls(window);
        }
    }
