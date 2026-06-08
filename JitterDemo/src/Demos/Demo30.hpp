// Member functions for DemoScene; included inside class DemoScene.

    static bool KeyPressBegin(GLFWwindow* window, int key, bool& previous)
    {
        const bool down = window != nullptr && glfwGetKey(window, key) == GLFW_PRESS;
        const bool result = down && !previous;
        previous = down;
        return result;
    }

    void BuildSweepCastsScene()
    {
        CurrentSweepCastKind = SweepCastKind::Sphere;
        SweepCastPreviousO = false;
        SweepCastPreviousP = false;

        SweepSphereRenderer = std::make_unique<InstancedDrawable>(CreateSphereMesh());
        SweepBoxRenderer = std::make_unique<InstancedDrawable>(CreateCubeMesh());
        SweepCylinderRenderer = std::make_unique<InstancedDrawable>(CreateCylinderMesh());
        SweepHalfSphereRenderer = std::make_unique<InstancedDrawable>(CreateHalfSphereMesh());

        AddFloor();

        BuildJenga(JVector(-8, 0, -10), 12);
        BuildPyramid(JVector(6, 0, -12), 10);
        BuildWall(JVector(-4, 0, -22), 8, 8);
    }

    void HandleSweepCastInput(GLFWwindow* window)
    {
        if (KeyPressBegin(window, GLFW_KEY_O, SweepCastPreviousO))
        {
            CurrentSweepCastKind = static_cast<SweepCastKind>(
                (static_cast<int>(CurrentSweepCastKind) + 3) % 4);
        }

        if (KeyPressBegin(window, GLFW_KEY_P, SweepCastPreviousP))
        {
            CurrentSweepCastKind = static_cast<SweepCastKind>(
                (static_cast<int>(CurrentSweepCastKind) + 1) % 4);
        }
    }

    void DrawSweepCastShape(
        SweepCastKind kind,
        const JVector& position,
        const JQuaternion& orientation,
        const Vec3& color)
    {
        constexpr Jitter2::Real radius = static_cast<Jitter2::Real>(0.45);
        const JVector boxHalfExtents(
            static_cast<Jitter2::Real>(0.75),
            static_cast<Jitter2::Real>(0.5),
            static_cast<Jitter2::Real>(1.1));
        constexpr Jitter2::Real halfLength = static_cast<Jitter2::Real>(0.45);
        constexpr Jitter2::Real halfHeight = static_cast<Jitter2::Real>(0.45);

        const Mat4 mat = Multiply(
            Translation(position),
            FromJitter(Jitter2::LinearMath::JMatrix::CreateFromQuaternion(orientation)));

        switch (kind)
        {
        case SweepCastKind::Sphere:
            SweepSphereRenderer->Push(
                Multiply(mat, Scale(static_cast<float>(radius * static_cast<Jitter2::Real>(2)))),
                color);
            break;

        case SweepCastKind::Box:
            SweepBoxRenderer->Push(
                Multiply(mat, Scale(boxHalfExtents * static_cast<Jitter2::Real>(2))),
                color);
            break;

        case SweepCastKind::Capsule:
        {
            SweepCylinderRenderer->Push(
                Multiply(
                    mat,
                    Scale(JVector(radius, halfLength * static_cast<Jitter2::Real>(2), radius))),
                color);

            const Mat4 cap = Multiply(
                Translation(JVector(0, halfLength, 0)),
                Scale(static_cast<float>(radius * static_cast<Jitter2::Real>(2))));
            SweepHalfSphereRenderer->Push(
                Multiply(mat, cap),
                color);
            SweepHalfSphereRenderer->Push(
                Multiply(Multiply(mat, RotationX(static_cast<float>(Pi))), cap),
                color);
            break;
        }

        case SweepCastKind::Cylinder:
            SweepCylinderRenderer->Push(
                Multiply(
                    mat,
                    Scale(JVector(radius, halfHeight * static_cast<Jitter2::Real>(2), radius))),
                color);
            break;
        }
    }

    void DrawSweepCasts(Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window)
    {
        if (SweepSphereRenderer == nullptr
            || SweepBoxRenderer == nullptr
            || SweepCylinderRenderer == nullptr
            || SweepHalfSphereRenderer == nullptr)
        {
            return;
        }

        HandleSweepCastInput(window);

        constexpr Jitter2::Real radius = static_cast<Jitter2::Real>(0.45);
        const JVector boxHalfExtents(
            static_cast<Jitter2::Real>(0.75),
            static_cast<Jitter2::Real>(0.5),
            static_cast<Jitter2::Real>(1.1));
        constexpr Jitter2::Real halfLength = static_cast<Jitter2::Real>(0.45);
        constexpr Jitter2::Real halfHeight = static_cast<Jitter2::Real>(0.45);
        const Vec3 castColor {0.35f, 0.35f, 0.35f};

        const JVector origin = ToJitter(cameraPosition);
        JVector direction = JVector::NormalizeSafe(ToJitter(cameraDirection));
        const JQuaternion shapeOrientation = JQuaternion::Identity();

        Jitter2::Collision::DynamicTree::Proxy* proxy = nullptr;
        JVector pointA;
        JVector pointB;
        JVector normal;
        Jitter2::Real lambda = static_cast<Jitter2::Real>(0);

        bool hit = false;
        switch (CurrentSweepCastKind)
        {
        case SweepCastKind::Sphere:
            hit = World.DynamicTree().SweepCastSphere(
                radius,
                origin,
                direction,
                Jitter2::Collision::DynamicTree::SweepCastPreFilter {},
                Jitter2::Collision::DynamicTree::SweepCastPostFilter {},
                proxy,
                pointA,
                pointB,
                normal,
                lambda);
            break;

        case SweepCastKind::Box:
            hit = World.DynamicTree().SweepCastBox(
                boxHalfExtents,
                shapeOrientation,
                origin,
                direction,
                Jitter2::Collision::DynamicTree::SweepCastPreFilter {},
                Jitter2::Collision::DynamicTree::SweepCastPostFilter {},
                proxy,
                pointA,
                pointB,
                normal,
                lambda);
            break;

        case SweepCastKind::Capsule:
            hit = World.DynamicTree().SweepCastCapsule(
                radius,
                halfLength,
                shapeOrientation,
                origin,
                direction,
                Jitter2::Collision::DynamicTree::SweepCastPreFilter {},
                Jitter2::Collision::DynamicTree::SweepCastPostFilter {},
                proxy,
                pointA,
                pointB,
                normal,
                lambda);
            break;

        case SweepCastKind::Cylinder:
            hit = World.DynamicTree().SweepCastCylinder(
                radius,
                halfHeight,
                shapeOrientation,
                origin,
                direction,
                Jitter2::Collision::DynamicTree::SweepCastPreFilter {},
                Jitter2::Collision::DynamicTree::SweepCastPostFilter {},
                proxy,
                pointA,
                pointB,
                normal,
                lambda);
            break;
        }

        if (!hit)
        {
            return;
        }

        const JVector hitPosition = origin + direction * lambda;
        DrawSweepCastShape(
            CurrentSweepCastKind,
            hitPosition,
            CurrentSweepCastKind == SweepCastKind::Sphere ? JQuaternion::Identity() : shapeOrientation,
            castColor);
    }
