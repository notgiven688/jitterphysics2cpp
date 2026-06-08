// Member functions for DemoScene; included inside class DemoScene.

    void BuildRotatingCubeScene()
    {
        RotatingBox = &World.CreateRigidBody();

        constexpr Jitter2::Real size = static_cast<Jitter2::Real>(50);

        auto addTransformedBox =
            [this](Jitter2::RigidBody& body, const JVector& boxSize, const JVector& translation)
            {
                Shapes::BoxShape& box = CreateShape<Shapes::BoxShape>(boxSize);
                body.AddShape(CreateShape<Shapes::TransformedShape>(box, translation));
            };

        addTransformedBox(*RotatingBox, JVector(size, 1, size), JVector(0, +size / 2, 0));
        addTransformedBox(*RotatingBox, JVector(size, 1, size), JVector(0, -size / 2, 0));
        addTransformedBox(*RotatingBox, JVector(1, size, size), JVector(+size / 2, 0, 0));
        addTransformedBox(*RotatingBox, JVector(1, size, size), JVector(-size / 2, 0, 0));
        addTransformedBox(*RotatingBox, JVector(size, size, 1), JVector(0, 0, +size / 2));
        addTransformedBox(*RotatingBox, JVector(size, size, 1), JVector(0, 0, -size / 2));

        RotatingBox->Tag = RigidBodyTag {true};
        RotatingBox->MotionTypeValue(Jitter2::MotionType::Kinematic);
        RotatingBox->DeactivationTime(std::numeric_limits<Jitter2::Real>::max());
        RotatingBox->SetActivationState(true);

        for (int i = -10; i < 10; ++i)
        {
            for (int e = -10; e < 10; ++e)
            {
                for (int j = -10; j < 10; ++j)
                {
                    Jitter2::RigidBody& rigidBody = World.CreateRigidBody();
                    rigidBody.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(1.5)));
                    rigidBody.Position(JVector(i, e, j) * static_cast<Jitter2::Real>(2));
                }
            }
        }
    }
