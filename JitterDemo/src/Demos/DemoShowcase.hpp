// Member functions for DemoScene; included inside class DemoScene.

    void BuildShowcaseScene()
    {
        AddFloor();
        AddBox(JVector(static_cast<Jitter2::Real>(1.2), static_cast<Jitter2::Real>(2.2), static_cast<Jitter2::Real>(0.4)),
            JVector(static_cast<Jitter2::Real>(-6.0), static_cast<Jitter2::Real>(0.95), static_cast<Jitter2::Real>(-3.8)),
            Jitter2::MotionType::Static,
            JQuaternion::CreateRotationY(static_cast<Jitter2::Real>(0.28)));

        AddBox(JVector(static_cast<Jitter2::Real>(1.2), static_cast<Jitter2::Real>(1.2), static_cast<Jitter2::Real>(1.2)),
            JVector(static_cast<Jitter2::Real>(-3.9), static_cast<Jitter2::Real>(0.75), static_cast<Jitter2::Real>(-0.8)),
            Jitter2::MotionType::Dynamic,
            JQuaternion::CreateRotationY(static_cast<Jitter2::Real>(0.25)),
            BodyTrackParams {0.2f, 0.34f, 0.12f, 1.0f, 0.18f});

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 4; ++column)
            {
                const float x = -1.9f + static_cast<float>(column) * 1.25f + static_cast<float>(row) * 0.18f;
                const float y = 0.55f + static_cast<float>(row) * 1.15f;
                const float z = -1.3f + static_cast<float>(row) * 0.95f;
                Jitter2::RigidBody& body = AddBox(
                    JVector(static_cast<Jitter2::Real>(0.92), static_cast<Jitter2::Real>(0.92), static_cast<Jitter2::Real>(0.92)),
                    JVector(static_cast<Jitter2::Real>(x), static_cast<Jitter2::Real>(y), static_cast<Jitter2::Real>(z)),
                    Jitter2::MotionType::Dynamic);
                AddTrack(body,
                    body.Position(),
                    static_cast<float>(row * 4 + column) * 0.48f,
                    0.08f,
                    0.06f,
                    0.82f + static_cast<float>(column) * 0.11f,
                    0.12f);
            }
        }

        for (int index = 0; index < 5; ++index)
        {
            const float phase = static_cast<float>(index) * 0.74f + 1.3f;
            Jitter2::RigidBody& body = AddSphere(
                static_cast<Jitter2::Real>(0.48 + 0.07 * static_cast<double>(index % 2)),
                JVector(static_cast<Jitter2::Real>(-4.0 + static_cast<double>(index) * 1.85),
                    static_cast<Jitter2::Real>(3.15 + 0.25 * static_cast<double>(index % 2)),
                    static_cast<Jitter2::Real>(2.45)),
                Jitter2::MotionType::Dynamic);
            AddTrack(body, body.Position(), phase, 0.34f, 0.18f, 0.55f, 0.0f);
        }

        for (int index = 0; index < 4; ++index)
        {
            const float x = -3.4f + static_cast<float>(index) * 2.25f;
            JQuaternion orientation = JQuaternion::CreateRotationZ(static_cast<Jitter2::Real>(0.55))
                * JQuaternion::CreateRotationY(static_cast<Jitter2::Real>(0.22 * static_cast<float>(index)));
            orientation.Normalize();

            Jitter2::RigidBody& body = AddCapsule(
                static_cast<Jitter2::Real>(0.34),
                static_cast<Jitter2::Real>(1.35 + 0.2 * static_cast<double>(index % 2)),
                JVector(static_cast<Jitter2::Real>(x), static_cast<Jitter2::Real>(1.05), static_cast<Jitter2::Real>(4.15)),
                Jitter2::MotionType::Dynamic,
                orientation);
            AddTrack(body, body.Position(), static_cast<float>(index) * 0.62f + 2.2f, 0.12f, 0.16f, 1.15f, 0.36f);
        }

        for (int index = 0; index < 3; ++index)
        {
            Jitter2::RigidBody& cylinder = AddCylinder(
                static_cast<Jitter2::Real>(1.25),
                static_cast<Jitter2::Real>(0.42),
                JVector(static_cast<Jitter2::Real>(2.7 + static_cast<double>(index) * 1.55),
                    static_cast<Jitter2::Real>(2.25 + 0.2 * static_cast<double>(index)),
                    static_cast<Jitter2::Real>(-3.0)),
                Jitter2::MotionType::Dynamic,
                JQuaternion::CreateRotationZ(static_cast<Jitter2::Real>(0.28 * static_cast<double>(index + 1))));
            AddTrack(cylinder, cylinder.Position(), static_cast<float>(index) * 0.7f + 3.4f, 0.09f, 0.1f, 0.7f, 0.22f);

            Jitter2::RigidBody& cone = AddCone(
                static_cast<Jitter2::Real>(0.46),
                static_cast<Jitter2::Real>(1.25),
                JVector(static_cast<Jitter2::Real>(2.7 + static_cast<double>(index) * 1.55),
                    static_cast<Jitter2::Real>(3.8 + 0.2 * static_cast<double>(index)),
                    static_cast<Jitter2::Real>(-4.25)),
                Jitter2::MotionType::Dynamic,
                JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(0.22 * static_cast<double>(index + 1))));
            AddTrack(cone, cone.Position(), static_cast<float>(index) * 0.83f + 4.6f, 0.1f, 0.12f, 0.88f, 0.2f);
        }
    }

