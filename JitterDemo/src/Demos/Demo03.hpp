// Member functions for DemoScene; included inside class DemoScene.

    void BuildPyramidScene()
    {
        AddFloor();
        BuildPyramid(JVector::Zero(), 40);
        BuildPyramidCylinder(JVector(10, 0, 10), 20);
        World.SolverIterations(4, 4);
    }

