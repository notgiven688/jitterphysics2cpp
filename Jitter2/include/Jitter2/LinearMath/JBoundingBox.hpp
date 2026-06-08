#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::LinearMath
{

struct JBoundingBox
{
    enum class ContainmentType
    {

// The two boxes are completely separated and do not touch or overlap.

        Disjoint,

// The other box is completely inside this box.

        Contains,
        Intersects
    };

    static constexpr Real Epsilon = static_cast<Real>(1e-12);

// The minimum corner of the bounding box (smallest X, Y, Z coordinates).

    JVector Min;

// The maximum corner of the bounding box (largest X, Y, Z coordinates).

    JVector Max;

    constexpr JBoundingBox() : Min(), Max() {}

    // Represents an axis-aligned bounding box (AABB), a rectangular bounding box whose edges are parallel to the coordinate axes.
    constexpr JBoundingBox(const JVector& min, const JVector& max) : Min(min), Max(max) {}

    static constexpr JBoundingBox LargeBox()
    {
        return JBoundingBox(JVector::MinValue(), JVector::MaxValue());
    }

    static constexpr JBoundingBox SmallBox()
    {
        return JBoundingBox(JVector::MaxValue(), JVector::MinValue());
    }

    // Creates a new AABB that encloses the original box after it has been rotated by the given orientation matrix.
    // Rotating an AABB usually results in a larger AABB to fit the rotated geometry.
    // box: The original bounding box.
    // orientation: The rotation matrix to apply.
    // Returns: A new AABB enclosing the rotated box.
    static JBoundingBox CreateTransformed(const JBoundingBox& box, const JMatrix& orientation)
    {
        JVector halfExtents = static_cast<Real>(0.5) * (box.Max - box.Min);
        JVector center = static_cast<Real>(0.5) * (box.Max + box.Min);

        center = orientation * center;
        halfExtents = JMatrix::Absolute(orientation) * halfExtents;

        return JBoundingBox(center - halfExtents, center + halfExtents);
    }

    // Determines whether the bounding box contains the specified point.
    bool Contains(const JVector& point) const
    {
        return Min.X <= point.X && point.X <= Max.X
            && Min.Y <= point.Y && point.Y <= Max.Y
            && Min.Z <= point.Z && point.Z <= Max.Z;
    }

    // Determines the relationship between this box and another box.
    // box: The other bounding box to test.
    // Returns: ContainmentType.Disjoint if they do not touch.
    // ContainmentType.Contains if box is strictly inside this box.
    // ContainmentType.Intersects if they overlap but one does not strictly contain the other.
    ContainmentType Contains(const JBoundingBox& box) const
    {
        if (Disjoint(*this, box))
        {
            return ContainmentType::Disjoint;
        }

        // Determines whether the outer box completely contains the inner box.
        // outer: The outer bounding box.
        // inner: The inner bounding box to test.
        // Returns: true if inner is entirely within the boundaries of outer; otherwise, false.
        return Contains(*this, box) ? ContainmentType::Contains : ContainmentType::Intersects;
    }

    std::array<JVector, 8> GetCorners() const
    {
        return {
            JVector(Min.X, Max.Y, Max.Z),
            JVector(Max.X, Max.Y, Max.Z),
            JVector(Max.X, Min.Y, Max.Z),
            JVector(Min.X, Min.Y, Max.Z),
            JVector(Min.X, Max.Y, Min.Z),
            JVector(Max.X, Max.Y, Min.Z),
            JVector(Max.X, Min.Y, Min.Z),
            JVector(Min.X, Min.Y, Min.Z),
        };
    }

    // Expands the bounding box to include the specified point.
    // box: The bounding box to expand.
    // point: The point to include.
    static void AddPointInPlace(JBoundingBox& box, const JVector& point)
    {
        box.Min = JVector::Min(box.Min, point);
        box.Max = JVector::Max(box.Max, point);
    }

    // Creates a bounding box that exactly encompasses a collection of points.
    // points: The collection of points to encompass.
    // Returns: A bounding box containing all the points.
    static JBoundingBox CreateFromPoints(const std::vector<JVector>& points)
    {
        JBoundingBox box = SmallBox();
        for (const JVector& point : points)
        {
            AddPointInPlace(box, point);
        }
        return box;
    }

    // Determines whether the two boxes are completely separated (disjoint).
    // left: The first bounding box.
    // right: The second bounding box.
    // Returns: true if there is a gap between the boxes on at least one axis; otherwise, false.
    static bool Disjoint(const JBoundingBox& left, const JBoundingBox& right)
    {
        return left.Max.X < right.Min.X || left.Min.X > right.Max.X
            || left.Max.Y < right.Min.Y || left.Min.Y > right.Max.Y
            || left.Max.Z < right.Min.Z || left.Min.Z > right.Max.Z;
    }

    static bool Contains(const JBoundingBox& outer, const JBoundingBox& inner)
    {
        return outer.Min.X <= inner.Min.X && outer.Max.X >= inner.Max.X
            && outer.Min.Y <= inner.Min.Y && outer.Max.Y >= inner.Max.Y
            && outer.Min.Z <= inner.Min.Z && outer.Max.Z >= inner.Max.Z;
    }

    // Creates a new bounding box that is the union of two other bounding boxes.
    // original: The first bounding box.
    // additional: The second bounding box.
    // Returns: A bounding box encompassing both inputs.
    static JBoundingBox CreateMerged(const JBoundingBox& original, const JBoundingBox& additional)
    {
        return JBoundingBox(
            JVector::Min(original.Min, additional.Min),
            JVector::Max(original.Max, additional.Max));
    }

    JVector Center() const
    {
        return (Min + Max) * static_cast<Real>(0.5);
    }

    JVector ClosestPoint(const JVector& point) const
    {
        return JVector(
            std::clamp(point.X, Min.X, Max.X),
            std::clamp(point.Y, Min.Y, Max.Y),
            std::clamp(point.Z, Min.Z, Max.Z));
    }

    Real DistanceSquared(const JVector& point) const
    {
        const JVector closest = ClosestPoint(point);
        return (closest - point).LengthSquared();
    }

    Real Distance(const JVector& point) const
    {
        return std::sqrt(DistanceSquared(point));
    }

    // Calculates the volume of the bounding box.
    Real GetVolume() const
    {
        const JVector len = Max - Min;
        return len.X * len.Y * len.Z;
    }

    // Calculates the surface area of the bounding box.
    Real GetSurfaceArea() const
    {
        const JVector len = Max - Min;
        return static_cast<Real>(2) * (len.X * len.Y + len.Y * len.Z + len.Z * len.X);
    }

    // Checks if a finite line segment intersects this bounding box.
    // origin: The start point of the segment.
    // direction: The vector from start to end (End = Origin + Direction).
    // Returns: true if the segment passes through the box; otherwise, false.
    bool SegmentIntersect(const JVector& origin, const JVector& direction) const
    {
        Real enter = 0;
        Real exit = 1;
        return Intersect1D(origin.X, direction.X, Min.X, Max.X, enter, exit)
            && Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, enter, exit)
            && Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, enter, exit);
    }

    // Checks if an infinite ray intersects this bounding box.
    // origin: The origin of the ray.
    // direction: The direction of the ray (not necessarily normalized).
    // Returns: true if the ray intersects the box; otherwise, false.
    bool RayIntersect(const JVector& origin, const JVector& direction) const
    {
        Real enter = 0;

        // Checks if an infinite ray intersects this bounding box and calculates the entry distance.
        // origin: The origin of the ray.
        // direction: The direction of the ray (not necessarily normalized).
        // enter: Outputs the distance along the direction vector where the ray enters the box. Returns 0 if the origin is inside.
        // Returns: true if the ray intersects the box; otherwise, false.
        return RayIntersect(origin, direction, enter);
    }

    bool RayIntersect(const JVector& origin, const JVector& direction, Real& enter) const
    {
        enter = 0;
        Real exit = std::numeric_limits<Real>::max();
        return Intersect1D(origin.X, direction.X, Min.X, Max.X, enter, exit)
            && Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, enter, exit)
            && Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, enter, exit);
    }

private:
    static bool Intersect1D(Real start, Real dir, Real min, Real max, Real& enter, Real& exit)
    {
        if (dir * dir < Epsilon * Epsilon)
        {
            return start >= min && start <= max;
        }

        Real t0 = (min - start) / dir;
        Real t1 = (max - start) / dir;
        if (t0 > t1)
        {
            std::swap(t0, t1);
        }

        if (t0 > exit || t1 < enter)
        {
            return false;
        }

        if (t0 > enter)
        {
            enter = t0;
        }
        if (t1 < exit)
        {
            exit = t1;
        }

        return true;
    }
};

static_assert(sizeof(JBoundingBox) == 6 * sizeof(Real));

constexpr bool operator==(const JBoundingBox& left, const JBoundingBox& right)
{
    return left.Min == right.Min && left.Max == right.Max;
}

} // namespace Jitter2::LinearMath
