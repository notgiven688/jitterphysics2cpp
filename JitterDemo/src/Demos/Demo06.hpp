// Member functions for DemoScene; included inside class DemoScene.

    static Mat4 Demo06RotationY(Jitter2::Real radians)
    {
        return RotationY(static_cast<float>(radians));
    }

    static Mat4 Demo06RotationX(Jitter2::Real radians)
    {
        return RotationX(static_cast<float>(radians));
    }

    void BuildRayCastCarScene()
    {
        AddFloor();

        RayCastCarInstance = std::make_unique<RayCastCar>(World);
        RayCastCarInstance->Body->Position(JVector(0, 2, 0));
        RayCastCarInstance->Body->DeactivationTime(std::numeric_limits<Jitter2::Real>::max());
        RayCastCarInstance->Body->Tag = RigidBodyTag {};

        CarRenderer = std::make_unique<CarMesh>();
        WheelRenderer = std::make_unique<WheelMesh>();

        BuildPyramid(-JVector::UnitZ() * static_cast<Jitter2::Real>(20), 10);
        BuildJenga(JVector(-20, 0, -10), 10);
        BuildWall(JVector(30, 0, -20), 4);

        World.SolverIterations(4, 4);
        World.SubstepCount = 2;
    }

    void DrawRayCastCar(DebugRenderer& debugRenderer, GLFWwindow* window)
    {
        if (RayCastCarInstance == nullptr)
        {
            return;
        }

        if (CarRenderer != nullptr)
        {
            CarRenderer->Push(
                Multiply(
                    BodyTransform(
                        RayCastCarInstance->Body->Position(),
                        RayCastCarInstance->Body->Orientation()),
                    Translation(JVector(0, static_cast<Jitter2::Real>(-0.3), static_cast<Jitter2::Real>(0.8)))));
        }

        if (WheelRenderer != nullptr)
        {
            for (int i = 0; i < 4; ++i)
            {
                Wheel& wheel = *RayCastCarInstance->Wheels[static_cast<std::size_t>(i)];

                Mat4 rotate = Identity();
                if (i == 1 || i == 3)
                {
                    rotate = Demo06RotationY(static_cast<Jitter2::Real>(Pi));
                }

                Mat4 bodyTransform = BodyTransform(
                    RayCastCarInstance->Body->Position(),
                    RayCastCarInstance->Body->Orientation());

                Mat4 wheelTransform = Multiply(bodyTransform, Translation(wheel.GetWheelCenter()));
                wheelTransform = Multiply(wheelTransform, Demo06RotationY(wheel.SteerAngle));
                wheelTransform = Multiply(wheelTransform, Demo06RotationX(-wheel.WheelRotation));
                wheelTransform = Multiply(wheelTransform, rotate);

                WheelRenderer->Push(wheelTransform);
            }
        }

        Jitter2::Real steer = static_cast<Jitter2::Real>(0);
        Jitter2::Real accelerate = static_cast<Jitter2::Real>(0);

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        {
            accelerate = static_cast<Jitter2::Real>(1);
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        {
            accelerate = static_cast<Jitter2::Real>(-1);
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            steer = static_cast<Jitter2::Real>(1);
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            steer = static_cast<Jitter2::Real>(-1);
        }

        RayCastCarInstance->SetInput(accelerate, steer);
        RayCastCarInstance->SetDebugRenderer(&debugRenderer);
        RayCastCarInstance->Step(static_cast<Jitter2::Real>(1.0 / 100.0));
        RayCastCarInstance->SetDebugRenderer(nullptr);
    }
