class ConstraintCar
{
public:
    void BuildCar(
        Jitter2::World& world,
        JVector,
        const std::function<void(Jitter2::RigidBody&)>& action = {})
    {
        OwnedShapes.clear();
        Bodies.clear();

        Car = &world.CreateRigidBody();
        Bodies.push_back(Car);

        Shapes::BoxShape& box1 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(1.5),
            static_cast<Jitter2::Real>(0.60),
            static_cast<Jitter2::Real>(3));
        Shapes::BoxShape& box2 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(1.0),
            static_cast<Jitter2::Real>(0.45),
            static_cast<Jitter2::Real>(1.5));

        Shapes::TransformedShape& tfs1 = CreateShape<Shapes::TransformedShape>(
            box1,
            JVector(0, static_cast<Jitter2::Real>(-0.3), static_cast<Jitter2::Real>(0.0)));
        Shapes::TransformedShape& tfs2 = CreateShape<Shapes::TransformedShape>(
            box2,
            JVector(0, static_cast<Jitter2::Real>(0.20), static_cast<Jitter2::Real>(0.3)));

        Car->AddShape(tfs1);
        Car->AddShape(tfs2);
        Car->Position(JVector(0, 2, 0));
        Car->SetMassInertia(
            Jitter2::LinearMath::JMatrix(
                static_cast<Jitter2::Real>(0.4), 0, 0,
                0, static_cast<Jitter2::Real>(0.4), 0,
                0, 0, static_cast<Jitter2::Real>(1.0)),
            static_cast<Jitter2::Real>(1.0));

        for (int i = 0; i < 4; ++i)
        {
            Damper[static_cast<std::size_t>(i)] = &world.CreateRigidBody();
            Damper[static_cast<std::size_t>(i)]->AddShape(CreateShape<Shapes::BoxShape>(
                static_cast<Jitter2::Real>(0.2)));
            Damper[static_cast<std::size_t>(i)]->SetMassInertia(static_cast<Jitter2::Real>(0.1));

            Wheels[static_cast<std::size_t>(i)] = &world.CreateRigidBody();

            Shapes::CylinderShape& shape = CreateShape<Shapes::CylinderShape>(
                static_cast<Jitter2::Real>(0.1),
                static_cast<Jitter2::Real>(0.3));
            Shapes::TransformedShape& tf = CreateShape<Shapes::TransformedShape>(
                shape,
                JVector::Zero(),
                Jitter2::LinearMath::JMatrix::CreateRotationZ(static_cast<Jitter2::Real>(Pi / 2.0f)));

            Wheels[static_cast<std::size_t>(i)]->AddShape(tf);

            Bodies.push_back(Wheels[static_cast<std::size_t>(i)]);
            Bodies.push_back(Damper[static_cast<std::size_t>(i)]);
        }

        Car->DeactivationTime(std::numeric_limits<Jitter2::Real>::max());

        Damper[FrontLeft]->Position(JVector(
            static_cast<Jitter2::Real>(-0.75),
            static_cast<Jitter2::Real>(1.4),
            static_cast<Jitter2::Real>(-1.1)));
        Damper[FrontRight]->Position(JVector(
            static_cast<Jitter2::Real>(+0.75),
            static_cast<Jitter2::Real>(1.4),
            static_cast<Jitter2::Real>(-1.1)));

        Damper[BackLeft]->Position(JVector(
            static_cast<Jitter2::Real>(-0.75),
            static_cast<Jitter2::Real>(1.4),
            static_cast<Jitter2::Real>(1.1)));
        Damper[BackRight]->Position(JVector(
            static_cast<Jitter2::Real>(+0.75),
            static_cast<Jitter2::Real>(1.4),
            static_cast<Jitter2::Real>(1.1)));

        for (int i = 0; i < 4; ++i)
        {
            Wheels[static_cast<std::size_t>(i)]->Position(Damper[static_cast<std::size_t>(i)]->Position());
        }

        for (int i = 0; i < 4; ++i)
        {
            DamperJoints[static_cast<std::size_t>(i)] = std::make_unique<Constraints::PrismaticJoint>(
                world,
                *Car,
                *Damper[static_cast<std::size_t>(i)],
                Damper[static_cast<std::size_t>(i)]->Position(),
                JVector::UnitY(),
                Constraints::LinearLimit::Fixed(),
                false);

            DamperJoints[static_cast<std::size_t>(i)]->Slider().LimitBias(static_cast<Jitter2::Real>(2));
            DamperJoints[static_cast<std::size_t>(i)]->Slider().LimitSoftness(static_cast<Jitter2::Real>(0.6));
            DamperJoints[static_cast<std::size_t>(i)]->Slider().Bias(static_cast<Jitter2::Real>(0.2));

            DamperJoints[static_cast<std::size_t>(i)]->HingeAngleConstraint()->LimitBias(
                static_cast<Jitter2::Real>(0.6));
            DamperJoints[static_cast<std::size_t>(i)]->HingeAngleConstraint()->LimitSoftness(
                static_cast<Jitter2::Real>(0.01));
        }

        DamperJoints[FrontLeft]->HingeAngleConstraint()->Limit(
            Constraints::AngularLimit::FromDegree(-MaxAngle, MaxAngle));
        DamperJoints[FrontRight]->HingeAngleConstraint()->Limit(
            Constraints::AngularLimit::FromDegree(-MaxAngle, MaxAngle));
        DamperJoints[BackLeft]->HingeAngleConstraint()->Limit(Constraints::AngularLimit::Fixed());
        DamperJoints[BackRight]->HingeAngleConstraint()->Limit(Constraints::AngularLimit::Fixed());

        for (int i = 0; i < 4; ++i)
        {
            Sockets[static_cast<std::size_t>(i)] = std::make_unique<Constraints::HingeJoint>(
                world,
                *Damper[static_cast<std::size_t>(i)],
                *Wheels[static_cast<std::size_t>(i)],
                Wheels[static_cast<std::size_t>(i)]->Position(),
                JVector::UnitX(),
                true);
        }

        auto* filter = dynamic_cast<IgnoreCollisionBetweenFilter*>(world.BroadPhaseFilter);
        if (filter == nullptr)
        {
            IgnoreCollisionFilter = std::make_unique<IgnoreCollisionBetweenFilter>();
            filter = IgnoreCollisionFilter.get();
            world.BroadPhaseFilter = filter;
        }

        for (int i = 0; i < 4; ++i)
        {
            filter->IgnoreCollisionBetween(*Car->Shapes()[0], *Damper[static_cast<std::size_t>(i)]->Shapes()[0]);
            filter->IgnoreCollisionBetween(
                *Wheels[static_cast<std::size_t>(i)]->Shapes()[0],
                *Damper[static_cast<std::size_t>(i)]->Shapes()[0]);
            filter->IgnoreCollisionBetween(*Car->Shapes()[0], *Wheels[static_cast<std::size_t>(i)]->Shapes()[0]);
        }

        SteerMotor[FrontLeft] = &world.CreateConstraint<Constraints::AngularMotor>(*Car, *Damper[FrontLeft]);
        SteerMotor[FrontLeft]->Initialize(JVector::UnitY());
        SteerMotor[FrontRight] = &world.CreateConstraint<Constraints::AngularMotor>(*Car, *Damper[FrontRight]);
        SteerMotor[FrontRight]->Initialize(JVector::UnitY());

        if (action)
        {
            for (Jitter2::RigidBody* body : Bodies)
            {
                action(*body);
            }
        }
    }

    void UpdateControls(GLFWwindow* window)
    {
        Jitter2::Real accelerate = static_cast<Jitter2::Real>(0.0);
        if (window != nullptr && glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        {
            accelerate = static_cast<Jitter2::Real>(1.0);
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        {
            accelerate = static_cast<Jitter2::Real>(-1.0);
        }

        if (window != nullptr && glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            Steer += static_cast<Jitter2::Real>(0.1);
        }
        else if (window != nullptr && glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            Steer -= static_cast<Jitter2::Real>(0.1);
        }
        else
        {
            Steer *= static_cast<Jitter2::Real>(0.9);
        }

        Steer = std::clamp(Steer, static_cast<Jitter2::Real>(-1.0), static_cast<Jitter2::Real>(1.0));

        const Jitter2::Real targetAngle = Steer * static_cast<Jitter2::Real>(MaxAngle / 180.0f * Pi);
        const Jitter2::Real currentAngleL = DamperJoints[FrontLeft]->HingeAngleConstraint()->Angle();
        const Jitter2::Real currentAngleR = DamperJoints[FrontRight]->HingeAngleConstraint()->Angle();

        SteerMotor[FrontLeft]->MaximumForce(
            static_cast<Jitter2::Real>(10.0) * std::abs(targetAngle - currentAngleL));
        SteerMotor[FrontLeft]->TargetVelocity(
            static_cast<Jitter2::Real>(10.0) * (targetAngle - currentAngleL));

        SteerMotor[FrontRight]->MaximumForce(
            static_cast<Jitter2::Real>(10.0) * std::abs(targetAngle - currentAngleR));
        SteerMotor[FrontRight]->TargetVelocity(
            static_cast<Jitter2::Real>(10.0) * (targetAngle - currentAngleR));

        for (int i = 0; i < 4; ++i)
        {
            Wheels[static_cast<std::size_t>(i)]->Friction(static_cast<Jitter2::Real>(0.8));
            Sockets[static_cast<std::size_t>(i)]->Motor()->MaximumForce(
                static_cast<Jitter2::Real>(1.0) * std::abs(accelerate));
            Sockets[static_cast<std::size_t>(i)]->Motor()->TargetVelocity(
                static_cast<Jitter2::Real>(-80.0) * accelerate);
        }
    }

private:
    template<typename TShape, typename... TArgs>
    TShape& CreateShape(TArgs&&... args)
    {
        auto shape = std::make_unique<TShape>(std::forward<TArgs>(args)...);
        TShape& reference = *shape;
        OwnedShapes.push_back(std::move(shape));
        return reference;
    }

    static constexpr int FrontLeft = 0;
    static constexpr int FrontRight = 1;
    static constexpr int BackLeft = 2;
    static constexpr int BackRight = 3;
    static constexpr Jitter2::Real MaxAngle = static_cast<Jitter2::Real>(40);

    Jitter2::RigidBody* Car = nullptr;
    std::array<Jitter2::RigidBody*, 4> Damper {};
    std::array<Jitter2::RigidBody*, 4> Wheels {};
    std::array<std::unique_ptr<Constraints::HingeJoint>, 4> Sockets {};
    std::array<std::unique_ptr<Constraints::PrismaticJoint>, 4> DamperJoints {};
    std::array<Constraints::AngularMotor*, 2> SteerMotor {};
    Jitter2::Real Steer = static_cast<Jitter2::Real>(0);
    std::vector<Jitter2::RigidBody*> Bodies;
    std::vector<std::unique_ptr<Shapes::RigidBodyShape>> OwnedShapes;
    std::unique_ptr<IgnoreCollisionBetweenFilter> IgnoreCollisionFilter;
};
