// Member functions for DemoScene; included inside class DemoScene.

    void BuildManyRagdollsScene()
    {
        AddFloor();

        for (int i = 0; i < 100; ++i)
        {
            BuildRagdoll(JVector(0, static_cast<Jitter2::Real>(3 + 2 * i), 0));
        }

        World.SolverIterations(8, 4);
    }

