// Member functions for DemoScene; included inside class DemoScene.

    void BuildAngularSweepScene()
    {
        AngularSweepStaticBar = &CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(10),
            static_cast<Jitter2::Real>(10),
            static_cast<Jitter2::Real>(0.1));
        AngularSweepDynamicBox = &CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(5),
            static_cast<Jitter2::Real>(1),
            static_cast<Jitter2::Real>(1));

        AngularSweepPosition = JVector(0, 0, 10);
        AngularSweepVelocity = JVector(0, 0, -10);
        AngularSweepAngularVelocity = JVector(1, 2, 2);
        AngularSweepCubeRenderer = std::make_unique<InstancedDrawable>(CreateCubeMesh());
    }

    static Mat4 CreateAngularSweepMatrix(
        const JVector& position,
        const JVector& velocity,
        const JVector& angularVelocity,
        Jitter2::Real dt)
    {
        const JQuaternion quat = Jitter2::LinearMath::MathHelper::RotationQuaternion(angularVelocity, dt);
        const Mat4 orientation =
            FromJitter(Jitter2::LinearMath::JMatrix::CreateFromQuaternion(quat));
        const Mat4 translation = Translation(position + velocity * dt);
        const Mat4 scale = Scale(JVector(
            static_cast<Jitter2::Real>(5),
            static_cast<Jitter2::Real>(1),
            static_cast<Jitter2::Real>(1)));

        return Multiply(Multiply(translation, orientation), scale);
    }

    void DrawAngularSweep(DebugRenderer& debugRenderer, GLFWwindow* window)
    {
        if (AngularSweepStaticBar == nullptr
            || AngularSweepDynamicBox == nullptr
            || AngularSweepCubeRenderer == nullptr)
        {
            return;
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
        {
            AngularSweepPosition += JVector(0, 0, static_cast<Jitter2::Real>(0.01));
        }
        if (window != nullptr && glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        {
            AngularSweepPosition -= JVector(0, 0, static_cast<Jitter2::Real>(0.01));
        }

        AngularSweepCubeRenderer->Push(
            Scale(JVector(
                static_cast<Jitter2::Real>(10),
                static_cast<Jitter2::Real>(10),
                static_cast<Jitter2::Real>(0.1))),
            Vec3 {0.2f, 0.2f, 0.2f});

        JVector positionA;
        JVector positionB;
        JVector normal;
        Jitter2::Real lambda = static_cast<Jitter2::Real>(0);
        const bool result = Jitter2::Collision::NarrowPhase::Sweep(
            *AngularSweepStaticBar,
            *AngularSweepDynamicBox,
            JQuaternion::Identity(),
            JQuaternion::Identity(),
            JVector::Zero(),
            AngularSweepPosition,
            JVector::Zero(),
            AngularSweepVelocity,
            JVector::Zero(),
            AngularSweepAngularVelocity,
            static_cast<Jitter2::Real>(10),
            static_cast<Jitter2::Real>(10),
            positionA,
            positionB,
            normal,
            lambda);

        if (!result)
        {
            return;
        }

        for (int i = 0; i <= 10; ++i)
        {
            AngularSweepCubeRenderer->Push(
                CreateAngularSweepMatrix(
                    AngularSweepPosition,
                    AngularSweepVelocity,
                    AngularSweepAngularVelocity,
                    static_cast<Jitter2::Real>(i) * static_cast<Jitter2::Real>(0.1) * lambda),
                ColorGenerator::GetColor(i * 4));
        }

        debugRenderer.PushPoint(DebugRenderer::Color::White, FromJitter(positionA), 2.0f);
        debugRenderer.PushPoint(DebugRenderer::Color::White, FromJitter(positionB), 2.0f);
    }
