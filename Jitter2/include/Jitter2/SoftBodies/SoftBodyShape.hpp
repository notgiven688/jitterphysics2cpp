#pragma once

#include <Jitter2/Collision/NarrowPhase/NarrowPhase.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::SoftBodies
{

class SoftBody;

// Abstract base class for shapes used in soft body simulations.
class SoftBodyShape : public Collision::Shapes::Shape
{
public:
    explicit SoftBodyShape(SoftBody& softBody)
        : softBody_(&softBody)
    {
    }

    // Gets the soft body instance this shape belongs to.
    [[nodiscard]] SoftBody& GetSoftBody() const { return *softBody_; }

    // Gets the rigid body closest to the specified position.
    // position: The position in world coordinates.
    // Returns: The closest rigid body (vertex) of this shape.
    virtual RigidBody& GetClosest(const LinearMath::JVector& position) const = 0;


    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        return Collision::NarrowPhase::RayCast(*this, origin, direction, lambda, normal);
    }


    bool Sweep(

        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& sweep,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        const bool hit = Collision::NarrowPhase::Sweep(
            *this,

            support,
            orientation,
            position,
            sweep,
            pointB,
            pointA,
            normal,
            lambda);
        normal = -normal;
        return hit;
    }


    bool Distance(
        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const override
    {
        return Collision::NarrowPhase::Distance(
            support,
            *this,
            orientation,
            LinearMath::JQuaternion::Identity(),
            position,
            LinearMath::JVector::Zero(),
            pointA,
            pointB,
            normal,
            distance);
    }

private:
    SoftBody* softBody_ = nullptr;
};

} // namespace Jitter2::SoftBodies
