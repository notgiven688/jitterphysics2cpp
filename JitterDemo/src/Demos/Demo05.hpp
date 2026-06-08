// Member functions for DemoScene; included inside class DemoScene.

    void BuildLevelGeometryScene()
    {
        LevelRenderer = std::make_unique<Dust>();

        AddFloor();

        LevelBody = &World.CreateRigidBody();
        CreateLevelShapes(*LevelBody);
        LevelBody->Tag = RigidBodyTag {true};
        LevelBody->MotionTypeValue(Jitter2::MotionType::Static);

        BuildJenga(
            JVector(-2, 6, 24),
            20,
            [](Jitter2::RigidBody& rigidBody)
            {
                rigidBody.Friction(static_cast<Jitter2::Real>(0.3));
            });

        PlayerInstance = std::make_unique<Player>(World, JVector(-6, 7, 32));
    }

    void DrawLevelGeometry(DebugRenderer& debugRenderer, GLFWwindow* window)
    {
        if (LevelRenderer != nullptr && LevelBody != nullptr)
        {
            LevelRenderer->Push(
                BodyTransform(
                    LevelBody->Position(),
                    LevelBody->Orientation()),
                Vec3 {0.35f, 0.35f, 0.35f});
        }

        if (LevelDebugDraw && LevelRenderer != nullptr)
        {
            for (const TriangleVertexIndex& triangle : LevelRenderer->CpuData.Indices)
            {
                Vec3 a = LevelRenderer->CpuData.Vertices[triangle.T1].Position;
                Vec3 b = LevelRenderer->CpuData.Vertices[triangle.T2].Position;
                Vec3 c = LevelRenderer->CpuData.Vertices[triangle.T3].Position;

                debugRenderer.PushLine(DebugRenderer::Color::Green, a, b);
                debugRenderer.PushLine(DebugRenderer::Color::Green, b, c);
                debugRenderer.PushLine(DebugRenderer::Color::Green, c, a);
            }
        }

        const bool oIsDown = window != nullptr && glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS;
        if (oIsDown && !LevelPreviousO)
        {
            LevelDebugDraw = !LevelDebugDraw;
        }
        LevelPreviousO = oIsDown;

        if (PlayerInstance == nullptr)
        {
            return;
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            PlayerInstance->SetAngularInput(static_cast<Jitter2::Real>(-1.0));
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            PlayerInstance->SetAngularInput(static_cast<Jitter2::Real>(1.0));
        }
        else
        {
            PlayerInstance->SetAngularInput(static_cast<Jitter2::Real>(0.0));
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        {
            PlayerInstance->SetLinearInput(-JVector::UnitZ());
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        {
            PlayerInstance->SetLinearInput(JVector::UnitZ());
        }
        else
        {
            PlayerInstance->SetLinearInput(JVector::Zero());
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        {
            PlayerInstance->Jump();
        }
    }

    void CreateLevelShapes(Jitter2::RigidBody& level)
    {
        std::vector<int> indices;
        indices.reserve(LevelRenderer->CpuData.Indices.size() * 3);
        for (const TriangleVertexIndex& triangle : LevelRenderer->CpuData.Indices)
        {
            indices.push_back(static_cast<int>(triangle.T1));
            indices.push_back(static_cast<int>(triangle.T2));
            indices.push_back(static_cast<int>(triangle.T3));
        }

        std::vector<JVector> vertices;
        vertices.reserve(LevelRenderer->CpuData.Vertices.size());
        for (const Vertex& vertex : LevelRenderer->CpuData.Vertices)
        {
            vertices.push_back(ToJitter(vertex.Position));
        }

        LevelMesh = std::make_unique<Shapes::TriangleMesh>(vertices, indices);
        for (int index = 0; index < static_cast<int>(LevelMesh->Indices().size()); ++index)
        {
            level.AddShape(
                CreateShape<Shapes::TriangleShape>(*LevelMesh, index),
                Jitter2::MassInertiaUpdateMode::Preserve);
        }
    }
