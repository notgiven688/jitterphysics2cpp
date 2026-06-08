// Member functions for DemoScene; included inside class DemoScene.

    void BuildConvexDecompositionScene()
    {
        AddFloor();

        TeapotRenderer = std::make_unique<DecomposedTeapot>();
        TeapotDecomp = std::make_unique<ConvexDecomposition>(World, *TeapotRenderer, OwnedShapes);
        TeapotDecomp->Load();

        for (int i = 0; i < 6; ++i)
        {
            TeapotDecomp->Spawn(JVector(0, static_cast<Jitter2::Real>(10 + i * 3), -14));
            TeapotDecomp->Spawn(JVector(0, static_cast<Jitter2::Real>(10 + i * 3), -6));
            TeapotDecomp->Spawn(JVector(5, static_cast<Jitter2::Real>(10 + i * 3), -14));
            TeapotDecomp->Spawn(JVector(5, static_cast<Jitter2::Real>(10 + i * 3), -6));
        }

        World.SolverIterations(8, 4);
    }

