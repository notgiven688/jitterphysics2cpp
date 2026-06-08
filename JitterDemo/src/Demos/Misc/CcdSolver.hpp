class CcdSolver
{
public:
    explicit CcdSolver(Jitter2::World& world)
        : World(world)
    {
        PreStepToken = World.PreStep.Add([this](Jitter2::Real dt)
        {
            PreStep(dt);
        });
    }

    ~CcdSolver()
    {
        Destroy();
    }

    bool Enabled = true;

    void Add(Jitter2::RigidBody& body)
    {
        Bodies.push_back(&body);
    }

    void Remove(Jitter2::RigidBody& body)
    {
        Bodies.erase(std::remove(Bodies.begin(), Bodies.end(), &body), Bodies.end());
    }

    void Destroy()
    {
        if (PreStepToken != 0)
        {
            World.PreStep.Remove(PreStepToken);
            PreStepToken = 0;
        }

        Bodies.clear();
    }

private:
    static constexpr int SelfConsistencyIterations = 4;

    void PreStep(Jitter2::Real dt)
    {
        if (!Enabled)
        {
            return;
        }

        for (int iter = 0; iter < SelfConsistencyIterations; ++iter)
        {
            for (Jitter2::RigidBody* body : Bodies)
            {
                if (body == nullptr || body->Handle().IsZero())
                {
                    throw std::runtime_error(
                        "RigidBody has been removed from the world, but is still registered with the CCD solver.");
                }

                JVector predictedPosition;
                JQuaternion predictedOrientation;
                body->PredictPose(dt, predictedPosition, predictedOrientation);

                for (Shapes::RigidBodyShape* shape : body->Shapes())
                {
                    Jitter2::LinearMath::JBoundingBox predictedBox;
                    shape->CalculateBoundingBox(predictedOrientation, predictedPosition, predictedBox);
                    const Jitter2::LinearMath::JBoundingBox box =
                        Jitter2::LinearMath::JBoundingBox::CreateMerged(
                            shape->WorldBoundingBox(),
                            predictedBox);

                    OverlapList.clear();
                    World.DynamicTree().Query(OverlapList, box);

                    CreateAndSolve(OverlapList, *shape, dt);
                }
            }
        }
    }

    void CreateAndSolve(
        const std::vector<Jitter2::Collision::DynamicTree::Proxy*>& proxies,
        Shapes::RigidBodyShape& shape,
        Jitter2::Real dt)
    {
        Shapes::RigidBodyShape* otherShape = nullptr;

        JVector bestPointA;
        JVector bestPointB;
        JVector bestNormal;

        Jitter2::Real smallestToi = std::numeric_limits<Jitter2::Real>::max();

        for (Jitter2::Collision::DynamicTree::Proxy* proxy : proxies)
        {
            auto* candidate = dynamic_cast<Shapes::RigidBodyShape*>(proxy);
            if (candidate == nullptr)
            {
                continue;
            }
            if (candidate->GetRigidBody() == shape.GetRigidBody())
            {
                continue;
            }

            Jitter2::RigidBody* body = shape.GetRigidBody();
            Jitter2::RigidBody* candidateBody = candidate->GetRigidBody();
            if (body == nullptr || candidateBody == nullptr)
            {
                continue;
            }

            const Jitter2::Real extentA = std::max(
                (shape.WorldBoundingBox().Max - body->Position()).Length(),
                (shape.WorldBoundingBox().Min - body->Position()).Length());
            const Jitter2::Real extentB = std::max(
                (candidate->WorldBoundingBox().Max - candidateBody->Position()).Length(),
                (candidate->WorldBoundingBox().Min - candidateBody->Position()).Length());

            JVector pointA;
            JVector pointB;
            JVector normal;
            Jitter2::Real toi = static_cast<Jitter2::Real>(0);

            const bool success = Jitter2::Collision::NarrowPhase::Sweep(
                shape,
                *candidate,
                body->Orientation(),
                candidateBody->Orientation(),
                body->Position(),
                candidateBody->Position(),
                body->Velocity(),
                candidateBody->Velocity(),
                body->AngularVelocity(),
                candidateBody->AngularVelocity(),
                extentA,
                extentB,
                pointA,
                pointB,
                normal,
                toi);

            if (!success || toi > dt || toi == static_cast<Jitter2::Real>(0))
            {
                continue;
            }

            if (World.NarrowPhaseFilter != nullptr
                && !World.NarrowPhaseFilter->Filter(shape, *candidate, pointA, pointB, normal, toi))
            {
                continue;
            }

            if (toi < smallestToi)
            {
                smallestToi = toi;
                bestPointA = pointA;
                bestPointB = pointB;
                bestNormal = normal;
                otherShape = candidate;
            }
        }

        if (!(smallestToi < std::numeric_limits<Jitter2::Real>::max()) || otherShape == nullptr)
        {
            return;
        }

        Jitter2::RigidBody* body = shape.GetRigidBody();
        Jitter2::RigidBody* otherBody = otherShape->GetRigidBody();
        if (body == nullptr || otherBody == nullptr)
        {
            return;
        }

        Jitter2::Arbiter* arbiter = nullptr;
        if (shape.ShapeId() < otherShape->ShapeId())
        {
            World.GetOrCreateArbiter(shape.ShapeId(), otherShape->ShapeId(), *body, *otherBody, arbiter);
            World.RegisterContact(*arbiter, bestPointA, bestPointB, bestNormal);
        }
        else
        {
            World.GetOrCreateArbiter(otherShape->ShapeId(), shape.ShapeId(), *otherBody, *body, arbiter);
            World.RegisterContact(*arbiter, bestPointB, bestPointA, -bestNormal);
        }

        arbiter->Data().PrepareForIteration(static_cast<Jitter2::Real>(1) / dt);
        arbiter->Data().Iterate(false);
    }

    Jitter2::World& World;
    std::vector<Jitter2::RigidBody*> Bodies;
    std::vector<Jitter2::Collision::DynamicTree::Proxy*> OverlapList;
    Jitter2::World::WorldStepFunction::Token PreStepToken = 0;
};
