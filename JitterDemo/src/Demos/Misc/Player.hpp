class Player
{
public:
    Player(Jitter2::World& world, JVector position)
        : WorldRef(world)
    {
        Body = &world.CreateRigidBody();
        Body->AddShape(CreateShape<Shapes::CapsuleShape>());
        Body->Position(position);

        Body->Damping(static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(0));
        Body->DeactivationTime(std::numeric_limits<Jitter2::Real>::max());

        Shapes::BoxShape& armShape1 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(0.2),
            static_cast<Jitter2::Real>(0.8),
            static_cast<Jitter2::Real>(0.2));
        Shapes::BoxShape& armShape2 = CreateShape<Shapes::BoxShape>(
            static_cast<Jitter2::Real>(0.2),
            static_cast<Jitter2::Real>(0.8),
            static_cast<Jitter2::Real>(0.2));

        Shapes::TransformedShape& arm1 = CreateShape<Shapes::TransformedShape>(
            armShape1,
            JVector(static_cast<Jitter2::Real>(+0.5), static_cast<Jitter2::Real>(0.3), 0));
        Shapes::TransformedShape& arm2 = CreateShape<Shapes::TransformedShape>(
            armShape2,
            JVector(static_cast<Jitter2::Real>(-0.5), static_cast<Jitter2::Real>(0.3), 0));

        Body->AddShape(arm1, Jitter2::MassInertiaUpdateMode::Preserve);
        Body->AddShape(arm2, Jitter2::MassInertiaUpdateMode::Preserve);

        auto& ur = world.CreateConstraint<Constraints::HingeAngle>(*Body, world.NullBody());
        ur.Initialize(JVector::UnitY(), Constraints::AngularLimit::Full());

        Body->Friction(static_cast<Jitter2::Real>(0.8));

        AngularMovement = &world.CreateConstraint<Constraints::AngularMotor>(*Body, world.NullBody());
        AngularMovement->Initialize(JVector::UnitY(), JVector::UnitY());
        AngularMovement->MaximumForce(static_cast<Jitter2::Real>(1000));
    }

    void SetAngularInput(Jitter2::Real rotate)
    {
        AngularMovement->TargetVelocity(rotate);
    }

    void Jump()
    {
        Jitter2::RigidBody* floorBody = nullptr;
        JVector hitPoint;
        if (CanJump(floorBody, hitPoint))
        {
            Jitter2::Real newYVel = static_cast<Jitter2::Real>(5.0);

            if (floorBody != nullptr)
            {
                newYVel += floorBody->Velocity().Y;
            }

            Jitter2::Real deltaVel = Body->Velocity().Y - newYVel;

            Body->Velocity(JVector(Body->Velocity().X, newYVel, Body->Velocity().Z));

            if (floorBody != nullptr && floorBody->MotionTypeValue() == Jitter2::MotionType::Dynamic)
            {
                Jitter2::Real force = Body->Mass() * deltaVel * static_cast<Jitter2::Real>(100.0);
                floorBody->SetActivationState(true);
                floorBody->AddForce(JVector::UnitY() * force, floorBody->Position() + hitPoint);
            }
        }
    }

    void SetLinearInput(JVector deltaMove)
    {
        Jitter2::RigidBody* floor = nullptr;
        JVector hitpoint;
        if (!CanJump(floor, hitpoint))
        {
            return;
        }

        deltaMove *= static_cast<Jitter2::Real>(3.0);

        Jitter2::Real deltaMoveLen = deltaMove.Length();

        JVector bodyVel = Body->Velocity();
        bodyVel.Y = 0;

        Jitter2::Real bodyVelLen = bodyVel.Length();

        if (deltaMoveLen > static_cast<Jitter2::Real>(0.01))
        {
            if (bodyVelLen < static_cast<Jitter2::Real>(5))
            {
                JVector force = JQuaternion::Transform(deltaMove, Body->Orientation()) * static_cast<Jitter2::Real>(10.0);

                Body->AddForce(force);

                if (floor != nullptr)
                {
                    floor->AddForce(-force, Body->Position() + hitpoint);
                }
            }
        }
    }

    Jitter2::RigidBody* Body = nullptr;
    Constraints::AngularMotor* AngularMovement = nullptr;

private:
    template<typename TShape, typename... TArgs>
    TShape& CreateShape(TArgs&&... args)
    {
        auto shape = std::make_unique<TShape>(std::forward<TArgs>(args)...);
        TShape& reference = *shape;
        OwnedShapes.push_back(std::move(shape));
        return reference;
    }

    bool CanJump(Jitter2::RigidBody*& floor, JVector& hitPoint)
    {
        for (Jitter2::Arbiter* contact : Body->Contacts())
        {
            Jitter2::ContactData& cd = contact->Data();
            int numContacts = 0;
            hitPoint = JVector::Zero();

            unsigned int mask = cd.UsageMask >> 4U;

            bool isBody1 = &contact->Body1() == Body;
            floor = isBody1 ? &contact->Body2() : &contact->Body1();

            if ((mask & Jitter2::ContactData::MaskContact0) != 0U)
            {
                hitPoint += isBody1 ? cd.Contacts[0].RelativePosition1 : cd.Contacts[0].RelativePosition2;
                ++numContacts;
            }
            if ((mask & Jitter2::ContactData::MaskContact1) != 0U)
            {
                hitPoint += isBody1 ? cd.Contacts[1].RelativePosition1 : cd.Contacts[1].RelativePosition2;
                ++numContacts;
            }
            if ((mask & Jitter2::ContactData::MaskContact2) != 0U)
            {
                hitPoint += isBody1 ? cd.Contacts[2].RelativePosition1 : cd.Contacts[2].RelativePosition2;
                ++numContacts;
            }
            if ((mask & Jitter2::ContactData::MaskContact3) != 0U)
            {
                hitPoint += isBody1 ? cd.Contacts[3].RelativePosition1 : cd.Contacts[3].RelativePosition2;
                ++numContacts;
            }

            if (numContacts == 0)
            {
                continue;
            }

            hitPoint *= static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(numContacts);

            if (hitPoint.Y <= static_cast<Jitter2::Real>(-0.8))
            {
                return true;
            }
        }

        hitPoint = JVector::Zero();
        floor = nullptr;

        return false;
    }

    Jitter2::World& WorldRef;
    std::vector<std::unique_ptr<Shapes::RigidBodyShape>> OwnedShapes;
};
