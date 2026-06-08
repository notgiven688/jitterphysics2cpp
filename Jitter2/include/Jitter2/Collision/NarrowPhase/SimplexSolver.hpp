#pragma once

#include <array>
#include <limits>

#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Collision
{

// Implements the Gilbert-Johnson-Keerthi (GJK) simplex solver for finding the closest point
// to the origin on a simplex (point, line segment, triangle, or tetrahedron).
// This solver iteratively reduces the simplex to the feature closest to the origin.
// It handles degeneracy by falling back to lower-dimensional features when the current
// simplex is near-degenerate.
class SimplexSolver
{
public:

    // Resets the solver to an empty simplex.
    void Reset()
    {
        usageMask_ = 0;
    }

    // Adds a vertex to the simplex and computes the new closest point to the origin.
    // vertex: The vertex to add (a point on the Minkowski difference).
    // closest: When this method returns, contains the point on the reduced simplex closest to the origin.
    // Returns: true if the origin is not contained within the simplex;
    // false if the origin is enclosed by the tetrahedron.
    bool AddVertex(const LinearMath::JVector& vertex, LinearMath::JVector& closest)
    {
        std::array<int, 4> indices {};
        int useCount = 0;
        int freeSlot = 0;

        for (int i = 0; i < 4; ++i)
        {
            if ((usageMask_ & (1u << i)) != 0)
            {
                indices[static_cast<std::size_t>(useCount++)] = i;
            }
            else
            {
                freeSlot = i;
            }
        }

        indices[static_cast<std::size_t>(useCount++)] = freeSlot;
        vertices_[static_cast<std::size_t>(freeSlot)] = vertex;

        switch (useCount)
        {
        case 1:
        {
            const int i0 = indices[0];
            closest = vertices_[static_cast<std::size_t>(i0)];
            usageMask_ = 1u << i0;
            return true;
        }
        case 2:
        {
            closest = ClosestSegment(indices[0], indices[1], usageMask_);
            return true;
        }
        case 3:
        {
            closest = ClosestTriangle(indices[0], indices[1], indices[2], usageMask_);
            return true;
        }
        case 4:
        {
            closest = ClosestTetrahedron(usageMask_);
            return usageMask_ != 0b1111;
        }
        default:
            return false;
        }
    }

private:
    static constexpr Real Epsilon = static_cast<Real>(1e-8);

    static Real Determinant(
        const LinearMath::JVector& a,
        const LinearMath::JVector& b,
        const LinearMath::JVector& c,
        const LinearMath::JVector& d)
    {
        return LinearMath::JVector::Dot(b - a, LinearMath::JVector::Cross(c - a, d - a));
    }

    LinearMath::JVector ClosestSegment(int i0, int i1, unsigned int& mask) const
    {
        const LinearMath::JVector a = vertices_[static_cast<std::size_t>(i0)];
        const LinearMath::JVector b = vertices_[static_cast<std::size_t>(i1)];
        const LinearMath::JVector v = b - a;
        const Real vsq = v.LengthSquared();
        const bool degenerate = vsq < Epsilon;

        Real t = -LinearMath::JVector::Dot(a, v) / vsq;
        Real lambda0 = static_cast<Real>(1) - t;
        Real lambda1 = t;

        mask = (1u << i0) | (1u << i1);
        if (lambda0 < static_cast<Real>(0) || degenerate)
        {
            mask = 1u << i1;
            lambda0 = static_cast<Real>(0);
            lambda1 = static_cast<Real>(1);
        }
        else if (lambda1 < static_cast<Real>(0))
        {
            mask = 1u << i0;
            lambda0 = static_cast<Real>(1);
            lambda1 = static_cast<Real>(0);
        }

        return lambda0 * a + lambda1 * b;
    }

    LinearMath::JVector ClosestTriangle(int i0, int i1, int i2, unsigned int& mask) const
    {
        mask = 0;

        const LinearMath::JVector a = vertices_[static_cast<std::size_t>(i0)];
        const LinearMath::JVector b = vertices_[static_cast<std::size_t>(i1)];
        const LinearMath::JVector c = vertices_[static_cast<std::size_t>(i2)];

        const LinearMath::JVector u = a - b;
        const LinearMath::JVector v = a - c;
        const LinearMath::JVector normal = LinearMath::JVector::Cross(u, v);
        const Real t = normal.LengthSquared();
        const Real inverseT = static_cast<Real>(1) / t;
        const bool degenerate = t < Epsilon;

        LinearMath::JVector cross = LinearMath::JVector::Cross(u, a);
        const Real lambda2 = LinearMath::JVector::Dot(cross, normal) * inverseT;
        cross = LinearMath::JVector::Cross(a, v);
        const Real lambda1 = LinearMath::JVector::Dot(cross, normal) * inverseT;
        const Real lambda0 = static_cast<Real>(1) - lambda2 - lambda1;

        Real bestDistance = std::numeric_limits<Real>::max();
        LinearMath::JVector closestPoint = LinearMath::JVector::Zero();

        if (lambda0 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestSegment(i1, i2, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                bestDistance = distance;
                closestPoint = closest;
            }
        }

        if (lambda1 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestSegment(i0, i2, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                bestDistance = distance;
                closestPoint = closest;
            }
        }

        if (lambda2 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestSegment(i0, i1, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                closestPoint = closest;
            }
        }

        if (mask != 0)
        {
            return closestPoint;
        }

        mask = (1u << i0) | (1u << i1) | (1u << i2);
        return lambda0 * a + lambda1 * b + lambda2 * c;
    }

    LinearMath::JVector ClosestTetrahedron(unsigned int& mask) const
    {
        const auto& v0 = vertices_[0];
        const auto& v1 = vertices_[1];
        const auto& v2 = vertices_[2];
        const auto& v3 = vertices_[3];

        const Real detT = Determinant(v0, v1, v2, v3);
        const Real inverseDetT = static_cast<Real>(1) / detT;
        const bool degenerate = detT * detT < Epsilon;

        const Real lambda0 = Determinant(LinearMath::JVector::Zero(), v1, v2, v3) * inverseDetT;
        const Real lambda1 = Determinant(v0, LinearMath::JVector::Zero(), v2, v3) * inverseDetT;
        const Real lambda2 = Determinant(v0, v1, LinearMath::JVector::Zero(), v3) * inverseDetT;
        const Real lambda3 = static_cast<Real>(1) - lambda0 - lambda1 - lambda2;

        Real bestDistance = std::numeric_limits<Real>::max();
        LinearMath::JVector closestPoint = LinearMath::JVector::Zero();
        mask = 0;

        if (lambda0 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestTriangle(1, 2, 3, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                bestDistance = distance;
                closestPoint = closest;
            }
        }

        if (lambda1 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestTriangle(0, 2, 3, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                bestDistance = distance;
                closestPoint = closest;
            }
        }

        if (lambda2 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestTriangle(0, 1, 3, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                bestDistance = distance;
                closestPoint = closest;
            }
        }

        if (lambda3 < static_cast<Real>(0) || degenerate)
        {
            unsigned int m = 0;
            const LinearMath::JVector closest = ClosestTriangle(0, 1, 2, m);
            const Real distance = closest.LengthSquared();
            if (distance < bestDistance)
            {
                mask = m;
                closestPoint = closest;
            }
        }

        if (mask != 0)
        {
            return closestPoint;
        }

        mask = 0b1111;
        return LinearMath::JVector::Zero();
    }

    std::array<LinearMath::JVector, 4> vertices_ {};
    unsigned int usageMask_ = 0;
};

} // namespace Jitter2::Collision
