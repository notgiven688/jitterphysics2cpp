#pragma once

#include <utility>

#include <Jitter2/Collision/CollisionFilter/IBroadPhaseFilter.hpp>
#include <Jitter2/Collision/DynamicTree/DynamicTree.hpp>
#include <Jitter2/Collision/NarrowPhase/NarrowPhase.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Dynamics/Contact.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/SoftBodies/SoftBody.hpp>
#include <Jitter2/SoftBodies/SoftBodyShape.hpp>

namespace Jitter2::SoftBodies
{

// A broad-phase filter that handles collisions involving soft body shapes.
// It delegates collision detection to the narrow phase and registers contacts with the
// closest rigid body vertices of the soft body.
class BroadPhaseCollisionFilter : public Collision::IBroadPhaseFilter
{
public:

    // Initializes a new instance of the BroadPhaseCollisionFilter class.
    // world: The world instance.
    explicit BroadPhaseCollisionFilter(World& world)
        : world_(&world)
    {
    }


    bool Filter(
        Collision::DynamicTree::Proxy& proxyA,
        Collision::DynamicTree::Proxy& proxyB) override
    {
        static constexpr LinearMath::JQuaternion IdentityOrientation =
            LinearMath::JQuaternion::Identity();
        static constexpr LinearMath::JVector ZeroPosition = LinearMath::JVector::Zero();

        const SoftBodyShape* softA = dynamic_cast<const SoftBodyShape*>(&proxyA);
        const SoftBodyShape* softB = dynamic_cast<const SoftBodyShape*>(&proxyB);

        if (softA != nullptr && softB != nullptr)
        {
            if (softB->ShapeId() < softA->ShapeId())
            {
                std::swap(softA, softB);
            }

            if (!softA->GetSoftBody().IsActive() && !softB->GetSoftBody().IsActive())
            {
                return false;
            }

            LinearMath::JVector pointA;
            LinearMath::JVector pointB;
            LinearMath::JVector normal;
            Real penetration = static_cast<Real>(0);
            if (!Collision::NarrowPhase::MprEpa(
                    *softA,
                    *softB,
                    IdentityOrientation,
                    IdentityOrientation,
                    ZeroPosition,
                    ZeroPosition,
                    pointA,
                    pointB,
                    normal,
                    penetration))
            {
                return false;
            }

            RigidBody& closestA = softA->GetClosest(pointA);
            RigidBody& closestB = softB->GetClosest(pointB);
            world_->RegisterContact(
                closestA.RigidBodyId(),
                closestB.RigidBodyId(),
                closestA,
                closestB,
                pointA,
                pointB,
                normal);

            return false;
        }

        if (softA != nullptr)
        {
            const auto* rigid = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyB);
            if (rigid == nullptr || rigid->GetRigidBody() == nullptr)
            {
                return false;
            }

            RigidBody& body = *rigid->GetRigidBody();
            if (!softA->GetSoftBody().IsActive() && !body.IsActive())
            {
                return false;
            }

            LinearMath::JVector pointA;
            LinearMath::JVector pointB;
            LinearMath::JVector normal;
            Real penetration = static_cast<Real>(0);
            if (!Collision::NarrowPhase::MprEpa(
                    *softA,
                    *rigid,
                    IdentityOrientation,
                    body.Orientation(),
                    ZeroPosition,
                    body.Position(),
                    pointA,
                    pointB,
                    normal,
                    penetration))
            {
                return false;
            }

            RigidBody& closest = softA->GetClosest(pointA);
            world_->RegisterContact(
                closest.RigidBodyId(),
                body.RigidBodyId(),
                closest,
                body,
                pointA,
                pointB,
                normal,
                ContactData::SolveMode::AngularBody1);

            return false;
        }

        if (softB != nullptr)
        {
            const auto* rigid = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyA);
            if (rigid == nullptr || rigid->GetRigidBody() == nullptr)
            {
                return false;
            }

            RigidBody& body = *rigid->GetRigidBody();
            if (!softB->GetSoftBody().IsActive() && !body.IsActive())
            {
                return false;
            }

            LinearMath::JVector pointA;
            LinearMath::JVector pointB;
            LinearMath::JVector normal;
            Real penetration = static_cast<Real>(0);
            if (!Collision::NarrowPhase::MprEpa(
                    *softB,
                    *rigid,
                    IdentityOrientation,
                    body.Orientation(),
                    ZeroPosition,
                    body.Position(),
                    pointA,
                    pointB,
                    normal,
                    penetration))
            {
                return false;
            }

            RigidBody& closest = softB->GetClosest(pointA);
            world_->RegisterContact(
                closest.RigidBodyId(),
                body.RigidBodyId(),
                closest,
                body,
                pointA,
                pointB,
                normal,
                ContactData::SolveMode::AngularBody1);

            return false;
        }

        return true;
    }

private:
    World* world_ = nullptr;
};

} // namespace Jitter2::SoftBodies
