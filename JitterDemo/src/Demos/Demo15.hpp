// Member functions for DemoScene; included inside class DemoScene.

    void BuildPressurizedSoftBodiesScene()
    {
        AddFloor();
        ConfigureSoftBodyCollision();

        for (int i = 0; i < 3; ++i)
        {
            auto sphere = std::make_unique<SoftBodySphereDemo>(
                World,
                JVector(static_cast<Jitter2::Real>(-3 + i * 3), static_cast<Jitter2::Real>(2), 0));
            sphere->Pressure = static_cast<Jitter2::Real>(300);
            SoftBodySpheres.push_back(std::move(sphere));
        }

        World.SolverIterations(4, 2);
        World.SubstepCount = 3;
    }

