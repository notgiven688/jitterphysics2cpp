#pragma once

#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Precision.hpp>

namespace Jitter2::Collision
{

// Defines an interface for a generic convex shape characterized by its support function.
// The support function is the fundamental operation for GJK-based collision detection algorithms.
// Any convex shape can be represented implicitly through its support mapping without requiring
// explicit vertex or face data.
class ISupportMappable
{
public:
    virtual ~ISupportMappable() = default;

    // Computes the point on the shape that is furthest in the specified direction.
    // direction: The search direction in local space. Does not need to be normalized.
    // result: The point on the shape's surface furthest along direction.
    virtual void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const = 0;

    // Computes a point deep within the shape, used as an initial search point in GJK-based algorithms.
    // point: A point that must lie strictly inside the convex hull.
    virtual void GetCenter(LinearMath::JVector& point) const = 0;
};

class IDynamicTreeProxy
{
public:
    virtual ~IDynamicTreeProxy() = default;

    int SetIndex = -1;

    [[nodiscard]] virtual int NodePtr() const = 0;
    virtual void NodePtr(int value) = 0;
    [[nodiscard]] virtual LinearMath::JVector Velocity() const = 0;
    [[nodiscard]] virtual const LinearMath::JBoundingBox& WorldBoundingBox() const = 0;
};

class IUpdatableBoundingBox
{
public:
    virtual ~IUpdatableBoundingBox() = default;

    virtual void UpdateWorldBoundingBox(Real dt = static_cast<Real>(0)) = 0;
};

class IRayCastable
{
public:
    virtual ~IRayCastable() = default;

    virtual bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const = 0;
};

class IDistanceTestable
{
public:
    virtual ~IDistanceTestable() = default;

    // Finds the closest points between this object and the query shape.
    // T: The query support-map type.
    // support: The query shape.
    // orientation: The query shape orientation in world space.
    // position: The query shape position in world space.
    // pointA: Closest point on the query shape in world space. Undefined when the shapes overlap.
    // pointB: Closest point on this object in world space. Undefined when the shapes overlap.
    // normal: Unit direction from the query shape toward this object, or JVector.Zero when
    // the shapes overlap. Do not use this to test whether a result was found.
    // distance: The separation distance between the shapes. Zero when overlapping.
    // Returns: true if the shapes are separated; false if they overlap.
    virtual bool Distance(
        const ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const = 0;
};

class ISweepTestable
{
public:
    virtual ~ISweepTestable() = default;

    // Performs a sweep test against this object.
    // T: The query support-map type.
    // support: The query shape.
    // orientation: The query shape orientation in world space.
    // position: The query shape position in world space.
    // sweep: The query shape translation in world space.
    // pointA: Collision point on the query shape in world space at the sweep origin. Undefined when the shapes already overlap.
    // pointB: Collision point on this object in world space at the sweep origin. Undefined when the shapes already overlap.
    // normal: Collision normal in world space, or JVector.Zero if the shapes already overlap.
    // Use the return value to determine whether a hit occurred; do not rely on this being non-zero.
    // lambda: The time of impact expressed in units of sweep. Zero if the shapes already overlap.
    // Returns: true if the query shape hits or already overlaps this object; otherwise, false.
    virtual bool Sweep(
        const ISupportMappable& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& sweep,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const = 0;
};

} // namespace Jitter2::Collision
