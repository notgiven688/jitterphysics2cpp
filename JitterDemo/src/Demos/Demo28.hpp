// Member functions for DemoScene; included inside class DemoScene.

    void CreateRingWall(JVector position, JVector size, int height, float radius)
    {
        const float circumference = Pi * 2.0f * radius;
        const int boxCountPerRing = static_cast<int>(0.9f * circumference / static_cast<float>(size.Z));
        const float increment = (2.0f * Pi) / static_cast<float>(boxCountPerRing);

        for (int ringIndex = 0; ringIndex < height; ++ringIndex)
        {
            for (int i = 0; i < boxCountPerRing; ++i)
            {
                const float angle = ((ringIndex & 1) == 0
                    ? static_cast<float>(i) + 0.5f
                    : static_cast<float>(i)) * increment;

                Jitter2::RigidBody& body = World.CreateRigidBody();
                body.AddShape(CreateShape<Shapes::BoxShape>(size));

                body.Position(position + JVector(
                    static_cast<Jitter2::Real>(-std::cos(angle) * radius),
                    (static_cast<Jitter2::Real>(ringIndex) + static_cast<Jitter2::Real>(0.5)) * size.Y,
                    static_cast<Jitter2::Real>(std::sin(angle) * radius)));
                body.Orientation(JQuaternion::CreateFromAxisAngle(
                    JVector::UnitY(),
                    static_cast<Jitter2::Real>(angle)));
            }
        }
    }

    void CreateRingPlatform(JVector position, JVector size, float radius)
    {
        const float innerCircumference = Pi * 2.0f * (radius - 0.5f * static_cast<float>(size.Z));
        const int boxCount = static_cast<int>(0.95f * innerCircumference / static_cast<float>(size.Y));
        const float increment = (2.0f * Pi) / static_cast<float>(boxCount);

        for (int i = 0; i < boxCount; ++i)
        {
            const float angle = static_cast<float>(i) * increment;

            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.AddShape(CreateShape<Shapes::BoxShape>(size));

            body.Position(position + JVector(
                static_cast<Jitter2::Real>(-std::cos(angle) * radius),
                static_cast<Jitter2::Real>(0.5) * size.X,
                static_cast<Jitter2::Real>(std::sin(angle) * radius)));
            body.Orientation(JQuaternion::CreateFromAxisAngle(
                JVector::UnitY(),
                static_cast<Jitter2::Real>(angle + Pi * 0.5f))
                * JQuaternion::CreateFromAxisAngle(
                    JVector::UnitZ(),
                    static_cast<Jitter2::Real>(Pi * 0.5f)));
        }
    }

    JVector CreateRing(
        JVector position,
        JVector size,
        float radius,
        int heightPerPlatformLevel,
        int platformLevels)
    {
        for (int platformIndex = 0; platformIndex < platformLevels; ++platformIndex)
        {
            const Jitter2::Real wallOffset =
                static_cast<Jitter2::Real>(0.5) * size.Z - static_cast<Jitter2::Real>(0.5) * size.X;
            CreateRingWall(position, size, heightPerPlatformLevel, radius + static_cast<float>(wallOffset));
            CreateRingWall(position, size, heightPerPlatformLevel, radius - static_cast<float>(wallOffset));

            CreateRingPlatform(
                position + JVector(0, static_cast<Jitter2::Real>(heightPerPlatformLevel) * size.Y, 0),
                size,
                radius);
            position.Y += static_cast<Jitter2::Real>(heightPerPlatformLevel) * size.Y + size.X;
        }

        return position;
    }

    void BuildColosseumScene()
    {
        AddFloor();
        World.AllowDeactivation = false;

        const JVector size(
            static_cast<Jitter2::Real>(0.5),
            static_cast<Jitter2::Real>(1),
            static_cast<Jitter2::Real>(3));
        JVector layerPosition;
        constexpr int layerCount = 6;
        constexpr float innerRadius = 15.0f;
        constexpr int heightPerPlatform = 3;
        constexpr int platformsPerLayer = 1;
        constexpr float ringSpacing = 0.5f;

        for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
        {
            const int ringCount = layerCount - layerIndex;
            for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
            {
                CreateRing(
                    layerPosition,
                    size,
                    innerRadius
                        + static_cast<float>(ringIndex) * (static_cast<float>(size.Z) + ringSpacing)
                        + static_cast<float>(layerIndex) * (static_cast<float>(size.Z) - static_cast<float>(size.X)),
                    heightPerPlatform,
                    platformsPerLayer);
            }

            layerPosition.Y += static_cast<Jitter2::Real>(platformsPerLayer)
                * (size.Y * static_cast<Jitter2::Real>(heightPerPlatform) + size.X);
        }
    }
