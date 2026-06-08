// Member functions for DemoScene; included inside class DemoScene.

    void BuildManyPyramidsScene()
    {
        AddFloor();

        World.SolverIterations(4, 2);

        for (int e = 0; e < 2; ++e)
        {
            for (int i = 0; i < 30; ++i)
            {
                BuildPyramid(
                    JVector(
                        static_cast<Jitter2::Real>(-20 + 40 * e),
                        0,
                        static_cast<Jitter2::Real>(-75 + 5 * i)),
                    20,
                    [](Jitter2::RigidBody& body)
                    {
                        body.SetActivationState(false);
                    });
            }
        }
    }

