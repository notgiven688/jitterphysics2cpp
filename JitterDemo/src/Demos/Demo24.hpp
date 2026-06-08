// Member functions for DemoScene; included inside class DemoScene.

    void BuildPointCloudTeapotScene()
    {
        AddFloor();

        PointCloudTeapotBodies.clear();
        PointCloudTeapotRenderer = std::make_unique<Teapot>();

        std::unordered_set<JVector, JVectorHash> seenVertices;
        seenVertices.reserve(PointCloudTeapotRenderer->CpuData.Vertices.size());
        std::vector<JVector> vertices;
        vertices.reserve(PointCloudTeapotRenderer->CpuData.Vertices.size());
        for (const Vertex& vertex : PointCloudTeapotRenderer->CpuData.Vertices)
        {
            JVector point = ToJitter(vertex.Position);
            if (seenVertices.insert(point).second)
            {
                vertices.push_back(point);
            }
        }

        std::vector<JVector> reducedVertices = Shapes::ShapeHelper::SampleHull(vertices, 3);

        Shapes::PointCloudShape pointCloud(reducedVertices);
        JVector center;
        pointCloud.GetCenter(center);
        const JVector shift = -center;
        pointCloud.Shift(shift);
        PointCloudTeapotShift = Translation(shift);

        for (int i = 0; i < 16; ++i)
        {
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.Position(JVector(0, static_cast<Jitter2::Real>(10 + i * 3), 0));

            body.AddShape(CreateShape<Shapes::PointCloudShape>(pointCloud.Clone()));
            PointCloudTeapotBodies.push_back(&body);
        }
    }

    void DrawPointCloudTeapots()
    {
        for (const Jitter2::RigidBody* body : PointCloudTeapotBodies)
        {
            if (body == nullptr || body->Handle().IsZero())
            {
                continue;
            }

            Vec3 color = ColorGenerator::GetColor(static_cast<int>(std::hash<const Jitter2::RigidBody*>{}(body)));
            if (!body->IsActive())
            {
                color = color + Vec3 {0.2f, 0.2f, 0.2f};
            }

            PointCloudTeapotRenderer->Push(
                Multiply(
                    BodyTransform(body->Position(), body->Orientation()),
                    PointCloudTeapotShift),
                color);
        }
    }
