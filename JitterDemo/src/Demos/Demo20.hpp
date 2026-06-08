// Member functions for DemoScene; included inside class DemoScene.

    void BuildOctreeScene()
    {
        AddFloor();

        DragonRenderer = std::make_unique<Dragon>();

        std::vector<Octree::TriangleIndices> indices;
        indices.reserve(DragonRenderer->CpuData.Indices.size());
        for (const TriangleVertexIndex& i : DragonRenderer->CpuData.Indices)
        {
            indices.emplace_back(i.T1, i.T2, i.T3);
        }

        std::vector<JVector> vertices;
        vertices.reserve(DragonRenderer->CpuData.Vertices.size());
        for (const Vertex& vertex : DragonRenderer->CpuData.Vertices)
        {
            vertices.push_back(ToJitter(vertex.Position));
        }

        DragonOctree = std::make_unique<Octree>(std::move(indices), std::move(vertices));

        OctreeTestShape = std::make_unique<Tester>(*DragonOctree);
        World.DynamicTree().AddProxy(*OctreeTestShape, false);

        OctreeCollisionFilter = std::make_unique<CustomCollisionDetection>(
            World,
            *OctreeTestShape,
            *DragonOctree);
        World.BroadPhaseFilter = OctreeCollisionFilter.get();
    }
