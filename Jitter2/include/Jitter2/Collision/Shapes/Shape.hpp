#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <Jitter2/ArgumentCheck.hpp>
#include <Jitter2/Collision/DynamicTree/IDynamicTreeProxy.hpp>
#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2
{
class RigidBody;
}

namespace Jitter2::Collision::Shapes
{

inline Real Positive(Real value, const char* name)
{
    return ArgumentCheck::Positive(value, name);
}

inline Real NonNegative(Real value, const char* name)
{
    return ArgumentCheck::NonNegative(value, name);
}

inline LinearMath::JVector PositiveComponents(const LinearMath::JVector& value, const char* name)
{
    return ArgumentCheck::PositiveComponents(value, name);
}

// The main entity of the collision system. Implements ISupportMappable for
// narrow-phase and IDynamicTreeProxy for broad-phase collision detection.
// The shape itself does not have a position or orientation. Shapes can be associated with
// instances of RigidBody.
class Shape : public Collision::IDynamicTreeProxy,
              public Collision::IUpdatableBoundingBox,
              public Collision::IRayCastable,
              public Collision::ISweepTestable,
              public Collision::IDistanceTestable,
              public Collision::ISupportMappable
{
public:
    Shape();
    virtual ~Shape() = default;

    Shape(const Shape&) = delete;
    Shape& operator=(const Shape&) = delete;
    Shape(Shape&&) = default;
    Shape& operator=(Shape&&) = default;

    [[nodiscard]] std::uint64_t ShapeId() const { return shapeId_; }
    [[nodiscard]] const LinearMath::JBoundingBox& WorldBoundingBox() const override { return worldBoundingBox_; }
    [[nodiscard]] bool IsRegistered() const { return SetIndex != -1; }
    [[nodiscard]] int NodePtr() const override { return nodePtr_; }
    void NodePtr(int value) override { nodePtr_ = value; }

    LinearMath::JVector Velocity() const override { return LinearMath::JVector::Zero(); }
    void UpdateWorldBoundingBox(Real dt = 0) override = 0;
    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override = 0;
    bool Sweep(
        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& sweep,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const override = 0;
    bool Distance(
        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const override = 0;

    virtual void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const = 0;

    virtual void GetCenter(LinearMath::JVector& point) const = 0;

protected:
    void SweptExpandBoundingBox(Real dt)
    {
        const Real swept = dt * Velocity().Length();
        worldBoundingBox_.Min -= LinearMath::JVector(swept);
        worldBoundingBox_.Max += LinearMath::JVector(swept);
    }

    LinearMath::JBoundingBox worldBoundingBox_;

private:
    std::uint64_t shapeId_;
    int nodePtr_ = -1;
};

// Represents the abstract base class for shapes that can be attached to a rigid body.
class RigidBodyShape : public Shape
{
public:
    // The RigidBody instance to which this shape is attached.
    [[nodiscard]] ::Jitter2::RigidBody* GetRigidBody() const { return rigidBody_; }

    LinearMath::JVector Position = LinearMath::JVector::Zero();
    LinearMath::JQuaternion Orientation = LinearMath::JQuaternion::Identity();

    [[nodiscard]] LinearMath::JVector WorldPosition() const;
    [[nodiscard]] LinearMath::JQuaternion WorldOrientation() const;

    LinearMath::JVector Velocity() const override;
    void UpdateWorldBoundingBox(Real dt = 0) override;

    virtual void CalculateBoundingBox(
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const;

    // Calculates the mass and inertia of the shape. Can be overridden by child classes to improve
    // performance or accuracy. The default implementation relies on an approximation of the shape
    // constructed using the support map function.
    // The inertia tensor is computed relative to the coordinate system origin (0,0,0),
    // not the center of mass.
    virtual void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const;

    // Performs a local ray cast against the shape, checking if a ray originating from a specified point
    // and traveling in a specified direction intersects with the object. It does not take into account the
    // transformation of the associated rigid body.
    // origin: The starting point of the ray.
    // direction: The direction of the ray. This vector does not need to be normalized.
    // normal: The surface normal at the point of intersection, if an intersection occurs.
    // lambda: The scalar value representing the distance along the ray's direction vector
    // from origin to the intersection point. The hit point can be calculated as:
    // origin + lambda * direction.
    // Returns: true if the ray intersects with the object; otherwise, false.
    virtual bool LocalRayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const;

    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        const LinearMath::JVector position = WorldPosition();
        const LinearMath::JQuaternion orientation = WorldOrientation();
        const LinearMath::JQuaternion inverse = orientation.Conjugate();
        const LinearMath::JVector transformedOrigin =
            LinearMath::JQuaternion::Transform(origin - position, inverse);
        const LinearMath::JVector transformedDirection =
            LinearMath::JQuaternion::Transform(direction, inverse);

        const bool hit = LocalRayCast(transformedOrigin, transformedDirection, normal, lambda);
        normal = LinearMath::JQuaternion::Transform(normal, orientation);
        return hit;
    }

    bool Sweep(
        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& sweep,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const override;

    bool Distance(
        const Collision::ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const override;

private:
    void AttachTo(::Jitter2::RigidBody* body)
    {
        rigidBody_ = body;
    }

    ::Jitter2::RigidBody* rigidBody_ = nullptr;

    friend class ::Jitter2::RigidBody;
};

} // namespace Jitter2::Collision::Shapes
