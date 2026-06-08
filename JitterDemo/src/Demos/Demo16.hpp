// Member functions for DemoScene; included inside class DemoScene.

    void BuildSoftBodyCubesScene()
    {
        AddFloor();
        ConfigureSoftBodyCollision();

        for (int i = 0; i < 3; ++i)
        {
            JVector position(0, static_cast<Jitter2::Real>(5 + i * 3), 0);

            auto cube = std::make_unique<SoftBodyCubeDemo>(World, position);
            SoftBodyCubes.push_back(std::move(cube));

            std::array<JVector, 4> offset {{
                JVector(static_cast<Jitter2::Real>(-0.5), static_cast<Jitter2::Real>(-1.5), static_cast<Jitter2::Real>(-0.5)),
                JVector(static_cast<Jitter2::Real>(-0.5), static_cast<Jitter2::Real>(-1.5), static_cast<Jitter2::Real>(+0.5)),
                JVector(static_cast<Jitter2::Real>(+0.5), static_cast<Jitter2::Real>(-1.5), static_cast<Jitter2::Real>(-0.5)),
                JVector(static_cast<Jitter2::Real>(+0.5), static_cast<Jitter2::Real>(-1.5), static_cast<Jitter2::Real>(+0.5)),
            }};

            for (int e = 0; e < 4; ++e)
            {
                Jitter2::RigidBody& body = World.CreateRigidBody();
                body.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(1)));
                body.Position(position + offset[static_cast<std::size_t>(e)]);
            }
        }

        JVector position(10, 1, 0);

        for (int i = 0; i < 3; ++i)
        {
            for (int e = i; e < 3; ++e)
            {
                JVector cubePosition = position
                    + JVector(
                        static_cast<Jitter2::Real>((e - i * 0.5f) * 1.01f),
                        static_cast<Jitter2::Real>(0.5f + i * 1.0f),
                        0) * static_cast<Jitter2::Real>(2);
                SoftBodyCubes.push_back(std::make_unique<SoftBodyCubeDemo>(World, cubePosition));
            }
        }

        World.SolverIterations(4, 2);
        World.SubstepCount = 4;
    }

