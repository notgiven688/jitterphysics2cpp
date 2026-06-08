// Member functions for DemoScene; included inside class DemoScene.

    void BuildTowerScene()
    {
        AddFloor();
        BuildTower(JVector::Zero(), 40);
        World.SolverIterations(12, 4);
    }

