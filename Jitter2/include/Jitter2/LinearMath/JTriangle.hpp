#pragma once

#include <cmath>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>

#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::LinearMath
{

struct JTriangle
{
    enum class CullMode
    {

// Cull triangles that are front-facing.
// A triangle is front-facing if its vertices are ordered counter-clockwise (CCW).

        FrontFacing,

// Cull triangles that are back-facing.
// A triangle is back-facing if its vertices are ordered clockwise (CW).
// This is the most common culling mode.

        BackFacing,
        None
    };

    // The first vertex of the triangle.
    JVector V0;
    // The second vertex of the triangle.
    JVector V1;
    // The third vertex of the triangle.
    JVector V2;

    constexpr JTriangle() = default;

    // Represents a triangle defined by three vertices.
    constexpr JTriangle(const JVector& v0, const JVector& v1, const JVector& v2)
        : V0(v0),
          V1(v1),
          V2(v2)
    {
    }

    [[nodiscard]] bool RayIntersect(

        // Checks if a ray intersects the triangle.
        // origin: The starting point (origin) of the ray.
        // direction: The direction vector of the ray.
        // cullMode: Determines whether to ignore triangles based on their winding order (Front/Back facing).
        // normal: Output: The normalized surface normal at the point of intersection.
        // lambda: Output: The distance along the direction vector where the intersection occurs (hit point = origin + lambda * direction).
        // Returns: true if the ray intersects the triangle; otherwise, false.
        const JVector& origin,
        const JVector& direction,
        CullMode cullMode,
        JVector& normal,
        Real& lambda) const
    {
        const JVector u = V0 - V1;
        const JVector v = V0 - V2;

        normal = JVector::Cross(u, v);
        const Real inverseLengthSquared = static_cast<Real>(1) / normal.LengthSquared();
        const Real denominator = JVector::Dot(direction, normal);

        if (std::abs(denominator) < static_cast<Real>(1e-6))
        {
            lambda = std::numeric_limits<Real>::max();
            normal = JVector::Zero();
            return false;
        }

        lambda = JVector::Dot(V0 - origin, normal);
        if ((cullMode == CullMode::FrontFacing && lambda < static_cast<Real>(0))
            || (cullMode == CullMode::BackFacing && lambda > static_cast<Real>(0)))
        {
            lambda = std::numeric_limits<Real>::max();
            normal = JVector::Zero();
            return false;
        }

        lambda /= denominator;
        const JVector hitPoint = origin + lambda * direction;

        const JVector at = V0 - hitPoint;
        JVector tmp = JVector::Cross(u, at);
        const Real gamma = JVector::Dot(tmp, normal) * inverseLengthSquared;
        tmp = JVector::Cross(at, v);
        const Real beta = JVector::Dot(tmp, normal) * inverseLengthSquared;
        const Real alpha = static_cast<Real>(1) - gamma - beta;

        if (alpha > static_cast<Real>(0) && beta > static_cast<Real>(0) && gamma > static_cast<Real>(0))
        {
            normal *= -static_cast<Real>(MathHelper::SignBit(denominator)) * std::sqrt(inverseLengthSquared);
            return true;
        }

        lambda = std::numeric_limits<Real>::max();
        normal = JVector::Zero();
        return false;
    }

    // Calculates the face normal of the triangle.
    // The direction follows the Right-Hand Rule (counter-clockwise winding).
    // Returns: The non-normalized normal vector.
    [[nodiscard]] JVector GetNormal() const
    {
        return JVector::Cross(V1 - V0, V2 - V0);
    }

    // Returns a string representation of the JTriangle.
    [[nodiscard]] std::string ToString() const
    {
        std::ostringstream stream;
        stream << "V0={" << V0 << "}, V1={" << V1 << "}, V2={" << V2 << "}";
        return stream.str();
    }

    // Calculates the geometric center (centroid) of the triangle.
    [[nodiscard]] JVector GetCenter() const
    {
        return (V0 + V1 + V2) * static_cast<Real>(1.0 / 3.0);
    }

    // Calculates the area of the triangle.
    [[nodiscard]] Real GetArea() const
    {
        return GetNormal().Length() * static_cast<Real>(0.5);
    }

    // Calculates the axis-aligned bounding box (AABB) of this triangle.
    [[nodiscard]] JBoundingBox GetBoundingBox() const
    {
        JBoundingBox box = JBoundingBox::SmallBox();
        JBoundingBox::AddPointInPlace(box, V0);
        JBoundingBox::AddPointInPlace(box, V1);
        JBoundingBox::AddPointInPlace(box, V2);
        return box;
    }

    // Finds the closest point on the triangle surface to a specified point.
    // point: The query point.
    // Returns: The point on the triangle closest to the query point.
    [[nodiscard]] JVector ClosestPoint(const JVector& point) const
    {
        const JVector ab = V1 - V0;
        const JVector ac = V2 - V0;
        const JVector ap = point - V0;

        const Real d1 = JVector::Dot(ab, ap);
        const Real d2 = JVector::Dot(ac, ap);
        if (d1 <= static_cast<Real>(0) && d2 <= static_cast<Real>(0))
        {
            return V0;
        }

        const JVector bp = point - V1;
        const Real d3 = JVector::Dot(ab, bp);
        const Real d4 = JVector::Dot(ac, bp);
        if (d3 >= static_cast<Real>(0) && d4 <= d3)
        {
            return V1;
        }

        const Real vc = d1 * d4 - d3 * d2;
        if (vc <= static_cast<Real>(0) && d1 >= static_cast<Real>(0) && d3 <= static_cast<Real>(0))
        {
            const Real v = d1 / (d1 - d3);
            return V0 + v * ab;
        }

        const JVector cp = point - V2;
        const Real d5 = JVector::Dot(ab, cp);
        const Real d6 = JVector::Dot(ac, cp);
        if (d6 >= static_cast<Real>(0) && d5 <= d6)
        {
            return V2;
        }

        const Real vb = d5 * d2 - d1 * d6;
        if (vb <= static_cast<Real>(0) && d2 >= static_cast<Real>(0) && d6 <= static_cast<Real>(0))
        {
            const Real w = d2 / (d2 - d6);
            return V0 + w * ac;
        }

        const Real va = d3 * d6 - d5 * d4;
        if (va <= static_cast<Real>(0) && (d4 - d3) >= static_cast<Real>(0) && (d5 - d6) >= static_cast<Real>(0))
        {
            const Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return V1 + w * (V2 - V1);
        }

        const Real denominator = static_cast<Real>(1) / (va + vb + vc);
        const Real v = vb * denominator;
        const Real w = vc * denominator;
        return V0 + ab * v + ac * w;
    }
};

constexpr bool operator==(const JTriangle& left, const JTriangle& right)
{
    return left.V0 == right.V0 && left.V1 == right.V1 && left.V2 == right.V2;
}

constexpr bool operator!=(const JTriangle& left, const JTriangle& right)
{
    return !(left == right);
}

inline std::ostream& operator<<(std::ostream& stream, const JTriangle& triangle)
{
    return stream << triangle.ToString();
}

} // namespace Jitter2::LinearMath
