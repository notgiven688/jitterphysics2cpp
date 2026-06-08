#pragma once

#include <Jitter2/Collision/DynamicTree/DynamicTree.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/SoftBodies/SoftBodyShape.hpp>

namespace Jitter2::SoftBodies
{

// Provides a collision filter that prevents self-collisions within soft bodies
// and shapes attached to the same rigid body.
class DynamicTreeCollisionFilter
{
public:

    // Filters collision pairs to exclude self-collisions.
    // proxyA: The first proxy.
    // proxyB: The second proxy.
    // Returns: true if the pair should be processed for collision; false if it should be skipped.
    static bool Filter(
        const Collision::DynamicTree::Proxy& proxyA,
        const Collision::DynamicTree::Proxy& proxyB)
    {
        const auto* rigidA = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyA);
        const auto* rigidB = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyB);
        if (rigidA != nullptr && rigidB != nullptr)
        {
            return rigidA->GetRigidBody() != rigidB->GetRigidBody();
        }

        const auto* softA = dynamic_cast<const SoftBodyShape*>(&proxyA);
        const auto* softB = dynamic_cast<const SoftBodyShape*>(&proxyB);
        if (softA != nullptr && softB != nullptr)
        {
            return &softA->GetSoftBody() != &softB->GetSoftBody();
        }

        return true;
    }
};

} // namespace Jitter2::SoftBodies
