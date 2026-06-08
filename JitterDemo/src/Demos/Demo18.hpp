// Member functions for DemoScene; included inside class DemoScene.

    void BuildPointTestScene()
    {
        PointTestShape = std::make_unique<Shapes::ConeShape>(static_cast<Jitter2::Real>(1));
    }

    void DrawPointTest(DebugRenderer& debugRenderer)
    {
        if (PointTestShape == nullptr)
        {
            return;
        }

        const auto time = static_cast<Jitter2::Real>(glfwGetTime());

        const Jitter2::LinearMath::JMatrix orientation =
            Jitter2::LinearMath::JMatrix::CreateRotationX(static_cast<Jitter2::Real>(1.1) * time)
            * Jitter2::LinearMath::JMatrix::CreateRotationY(static_cast<Jitter2::Real>(2.1) * time)
            * Jitter2::LinearMath::JMatrix::CreateRotationZ(static_cast<Jitter2::Real>(1.7) * time);

        for (int i = -15; i < 16; ++i)
        {
            for (int e = -15; e < 16; ++e)
            {
                for (int k = -15; k < 16; ++k)
                {
                    const JVector point =
                        JVector(
                            static_cast<Jitter2::Real>(i),
                            static_cast<Jitter2::Real>(e),
                            static_cast<Jitter2::Real>(k))
                        * static_cast<Jitter2::Real>(0.1);

                    const bool result = Jitter2::Collision::NarrowPhase::PointTest(
                        *PointTestShape,
                        orientation,
                        JVector::Zero(),
                        point);

                    if (result)
                    {
                        debugRenderer.PushPoint(DebugRenderer::Color::White, FromJitter(point), 0.01f);
                    }
                }
            }
        }
    }
