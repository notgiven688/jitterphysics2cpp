#pragma once

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Precision.hpp>

namespace Jitter2::Collision
{

// Provides a hook into the narrowphase collision detection pipeline.
// Implement this interface to intercept collisions after contact generation.
// This can be used to modify contact data, implement custom collision responses,
// or filter out specific collisions.
class INarrowPhaseFilter
{
public:
    virtual ~INarrowPhaseFilter() = default;

    // Called for each detected collision with its contact data.
    // shapeA: The first shape in the collision.
    // shapeB: The second shape in the collision.
    // pointA: Contact point on shape A (modifiable).
    // pointB: Contact point on shape B (modifiable).
    // normal: Collision normal from B to A (modifiable).
    // penetration: Penetration depth (modifiable).
    // Returns: true to keep the collision; false to discard it.
    virtual bool Filter(
        const Shapes::RigidBodyShape& shapeA,
        const Shapes::RigidBodyShape& shapeB,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration) = 0;
};

} // namespace Jitter2::Collision
