#pragma once

#include <cmath>
#include <limits>

#include <Jitter2/Collision/NarrowPhase/ConvexPolytope.hpp>
#include <Jitter2/Collision/NarrowPhase/MinkowskiDifference.hpp>
#include <Jitter2/Collision/NarrowPhase/SimplexSolver.hpp>
#include <Jitter2/Collision/NarrowPhase/SimplexSolverAB.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>
#include <Jitter2/Logger.hpp>

namespace Jitter2::Collision::NarrowPhase
{

inline constexpr Real NumericEpsilon = static_cast<Real>(1e-16);

inline LinearMath::JVector NormalizeSafe(const LinearMath::JVector& value, Real epsilonSquared = NumericEpsilon)
{
    const Real lengthSq = value.LengthSquared();
    if (lengthSq < epsilonSquared)
    {
        return LinearMath::JVector::Zero();
    }

    return value * (static_cast<Real>(1) / std::sqrt(lengthSq));
}

namespace Detail
{

class MprEpaSolver
{
public:
    // Determines whether two convex shapes overlap, providing detailed information for both overlapping and separated
    // cases. It assumes that support shape A is at position zero and not rotated.
    // Internally, the method employs the Expanding Polytope Algorithm (EPA) to gather collision information.
    // supportA: The support function of shape A.
    // supportB: The support function of shape B.
    // orientationB: The orientation of shape B.
    // positionB: The position of shape B.
    // pointA: For the overlapping case: the deepest point on shape A inside shape B; for the separated case: the
    // closest point on shape A to shape B.
    // pointB: For the overlapping case: the deepest point on shape B inside shape A; for the separated case: the
    // closest point on shape B to shape A.
    // normal: The normalized collision normal pointing from pointB to pointA. This normal remains defined even
    // if pointA and pointB coincide. It denotes the direction in which the shapes should be moved by the minimum distance
    // (defined by the penetration depth) to either separate them in the overlapping case or bring them into contact in
    // the separated case.
    // penetration: The penetration depth.
    // Returns true if the algorithm completes successfully, false otherwise. In case of algorithm convergence
    // failure, collision information reverts to the type's default values.
    template<typename TSupportA, typename TSupportB>
    bool Collision(
        const TSupportA& supportA,
        const TSupportB& supportB,
        const LinearMath::JQuaternion& orientationB,
        const LinearMath::JVector& positionB,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration)
    {
        const Real collideEpsilon = static_cast<Real>(1e-4);
        constexpr int maxIter = 85;

        MinkowskiDifference::Vertex centerVertex;
        MinkowskiDifference::GetCenter(supportA, supportB, orientationB, positionB, centerVertex);
        const LinearMath::JVector center = centerVertex.V;

        convexPolytope_.InitHeap();
        convexPolytope_.InitTetrahedron(center);

        int iter = 0;
        ConvexPolytope::Triangle closestTriangle;

        while (++iter < maxIter)
        {
            closestTriangle = convexPolytope_.GetClosestTriangle();

            LinearMath::JVector searchDir = closestTriangle.ClosestToOrigin;
            Real searchDirSq = closestTriangle.ClosestToOriginSq;

            if (!convexPolytope_.OriginEnclosed())
            {
                searchDir = -searchDir;
            }

            if (closestTriangle.ClosestToOriginSq < NumericEpsilon)
            {
                searchDir = closestTriangle.Normal;
                searchDirSq = closestTriangle.NormalSq;
            }

            MinkowskiDifference::Vertex vertex;
            MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, searchDir, vertex);

            const Real deltaDist = LinearMath::JVector::Dot(closestTriangle.ClosestToOrigin - vertex.V, searchDir);

            if (deltaDist * deltaDist <= collideEpsilon * collideEpsilon * searchDirSq)
            {
                convexPolytope_.CalculatePoints(closestTriangle, pointA, pointB);

                penetration = std::sqrt(closestTriangle.ClosestToOriginSq);
                if (!convexPolytope_.OriginEnclosed())
                {
                    penetration *= static_cast<Real>(-1);
                }

                if (std::abs(penetration) > NumericEpsilon)
                {
                    normal = closestTriangle.ClosestToOrigin * (static_cast<Real>(1) / penetration);
                }
                else
                {
                    normal = closestTriangle.Normal * (static_cast<Real>(1) / std::sqrt(closestTriangle.NormalSq));
                }

                return true;
            }

            if (!convexPolytope_.AddVertex(vertex))
            {
                convexPolytope_.CalculatePoints(closestTriangle, pointA, pointB);

                penetration = std::sqrt(closestTriangle.ClosestToOriginSq);
                if (!convexPolytope_.OriginEnclosed())
                {
                    penetration *= static_cast<Real>(-1);
                }

                if (std::abs(penetration) > NumericEpsilon)
                {
                    normal = closestTriangle.ClosestToOrigin * (static_cast<Real>(1) / penetration);
                }
                else
                {
                    normal = closestTriangle.Normal * (static_cast<Real>(1) / std::sqrt(closestTriangle.NormalSq));
                }

                return true;
            }
        }

        pointA = LinearMath::JVector::Zero();
        pointB = LinearMath::JVector::Zero();
        normal = LinearMath::JVector::Zero();
        penetration = static_cast<Real>(0);
        Logger::Warning(
            "{0}: EPA, Could not converge within {1} iterations.",
            "NarrowPhase",
            maxIter);
        return false;
    }

    template<typename TSupportA, typename TSupportB>
    bool SolveMpr(
        const TSupportA& supportA,
        const TSupportB& supportB,
        const LinearMath::JQuaternion& orientationB,
        const LinearMath::JVector& positionB,
        Real epaThreshold,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration)
    {
        const Real collideEpsilon = static_cast<Real>(1e-5);
        constexpr int maxIter = 34;

        MinkowskiDifference::Vertex v0;
        MinkowskiDifference::Vertex v1;
        MinkowskiDifference::Vertex v2;
        MinkowskiDifference::Vertex v3;
        MinkowskiDifference::Vertex v4;

        LinearMath::JVector temp1;
        LinearMath::JVector temp2;
        LinearMath::JVector temp3;

        penetration = static_cast<Real>(0);

        MinkowskiDifference::GetCenter(supportA, supportB, orientationB, positionB, v0);

        if (std::abs(v0.V.X) < NumericEpsilon
            && std::abs(v0.V.Y) < NumericEpsilon
            && std::abs(v0.V.Z) < NumericEpsilon)
        {
            v0.V.X = static_cast<Real>(1e-5);
        }

        normal = -v0.V;

        MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, normal, v1);

        pointA = v1.A;
        pointB = v1.B;

        if (LinearMath::JVector::Dot(v1.V, normal) <= static_cast<Real>(0))
        {
            return false;
        }

        normal = LinearMath::JVector::Cross(v1.V, v0.V);

        const Real sphericalEpsilon = static_cast<Real>(1e-12);
        if (normal.LengthSquared() < sphericalEpsilon)
        {
            normal = v1.V - v0.V;
            normal.Normalize();

            temp1 = v1.A - v1.B;
            penetration = LinearMath::JVector::Dot(temp1, normal);

            return true;
        }

        MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, normal, v2);

        if (LinearMath::JVector::Dot(v2.V, normal) <= static_cast<Real>(0))
        {
            return false;
        }

        temp1 = v1.V - v0.V;
        temp2 = v2.V - v0.V;
        normal = LinearMath::JVector::Cross(temp1, temp2);

        Real dist = LinearMath::JVector::Dot(normal, v0.V);

        if (dist > static_cast<Real>(0))
        {
            std::swap(v1, v2);
            normal = -normal;
        }

        int phase2 = 0;
        int phase1 = 0;
        bool hit = false;

        while (true)
        {
            if (phase1 > maxIter)
            {
                return false;
            }

            ++phase1;

            MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, normal, v3);

            if (LinearMath::JVector::Dot(v3.V, normal) <= static_cast<Real>(0))
            {
                return false;
            }

            temp1 = LinearMath::JVector::Cross(v1.V, v3.V);
            if (LinearMath::JVector::Dot(temp1, v0.V) < static_cast<Real>(0))
            {
                v2 = v3;
                temp1 = v1.V - v0.V;
                temp2 = v3.V - v0.V;
                normal = LinearMath::JVector::Cross(temp1, temp2);
                continue;
            }

            temp1 = LinearMath::JVector::Cross(v3.V, v2.V);
            if (LinearMath::JVector::Dot(temp1, v0.V) < static_cast<Real>(0))
            {
                v1 = v3;
                temp1 = v3.V - v0.V;
                temp2 = v2.V - v0.V;
                normal = LinearMath::JVector::Cross(temp1, temp2);
                continue;
            }

            break;
        }

        while (true)
        {
            ++phase2;

            temp1 = v2.V - v1.V;
            temp2 = v3.V - v1.V;
            normal = LinearMath::JVector::Cross(temp1, temp2);

            const Real normalSq = normal.LengthSquared();

            if (normalSq < NumericEpsilon)
            {
                return false;
            }

            if (!hit)
            {
                const Real d = LinearMath::JVector::Dot(normal, v1.V);
                hit = d >= static_cast<Real>(0);
            }

            MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, normal, v4);

            temp3 = v4.V - v3.V;
            const Real delta = LinearMath::JVector::Dot(temp3, normal);
            penetration = LinearMath::JVector::Dot(v4.V, normal);

            if (delta * delta <= collideEpsilon * collideEpsilon * normalSq
                || penetration <= static_cast<Real>(0)
                || phase2 > maxIter)
            {
                if (hit)
                {
                    const Real invnormal = static_cast<Real>(1) / std::sqrt(normalSq);

                    penetration *= invnormal;

                    if (penetration > epaThreshold)
                    {
                        convexPolytope_.InitHeap();
                        convexPolytope_.GetVertex(0) = v0;
                        convexPolytope_.GetVertex(1) = v1;
                        convexPolytope_.GetVertex(2) = v2;
                        convexPolytope_.GetVertex(3) = v3;

                        if (SolveMprEpa(
                                supportA,
                                supportB,
                                orientationB,
                                positionB,
                                pointA,
                                pointB,
                                normal,
                                penetration))
                        {
                            return true;
                        }
                    }

                    normal *= invnormal;

                    temp3 = LinearMath::JVector::Cross(v1.V, temp1);
                    const Real gamma = LinearMath::JVector::Dot(temp3, normal) * invnormal;
                    temp3 = LinearMath::JVector::Cross(temp2, v1.V);
                    const Real beta = LinearMath::JVector::Dot(temp3, normal) * invnormal;
                    const Real alpha = static_cast<Real>(1) - gamma - beta;

                    pointA = alpha * v1.A + beta * v2.A + gamma * v3.A;
                    pointB = alpha * v1.B + beta * v2.B + gamma * v3.B;
                }

                return hit;
            }

            temp1 = LinearMath::JVector::Cross(v4.V, v0.V);
            Real dot = LinearMath::JVector::Dot(temp1, v1.V);

            if (dot >= static_cast<Real>(0))
            {
                dot = LinearMath::JVector::Dot(temp1, v2.V);

                if (dot >= static_cast<Real>(0))
                {
                    v1 = v4;
                }
                else
                {
                    v3 = v4;
                }
            }
            else
            {
                dot = LinearMath::JVector::Dot(temp1, v3.V);

                if (dot >= static_cast<Real>(0))
                {
                    v2 = v4;
                }
                else
                {
                    v1 = v4;
                }
            }
        }
    }

private:
    template<typename TSupportA, typename TSupportB>
    bool SolveMprEpa(
        const TSupportA& supportA,
        const TSupportB& supportB,
        const LinearMath::JQuaternion& orientationB,
        const LinearMath::JVector& positionB,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration)
    {
        const Real collideEpsilon = static_cast<Real>(1e-5);
        constexpr int maxIter = 85;

        convexPolytope_.InitTetrahedron();

        int iter = 0;
        ConvexPolytope::Triangle closestTriangle;

        while (++iter < maxIter)
        {
            closestTriangle = convexPolytope_.GetClosestTriangle();

            LinearMath::JVector searchDir = closestTriangle.ClosestToOrigin;
            Real searchDirSq = closestTriangle.ClosestToOriginSq;

            if (closestTriangle.ClosestToOriginSq < NumericEpsilon)
            {
                searchDir = closestTriangle.Normal;
                searchDirSq = closestTriangle.NormalSq;
            }

            MinkowskiDifference::Vertex vertex;
            MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, searchDir, vertex);

            const Real deltaDist = LinearMath::JVector::Dot(closestTriangle.ClosestToOrigin - vertex.V, searchDir);

            if (deltaDist * deltaDist <= collideEpsilon * collideEpsilon * searchDirSq)
            {
                convexPolytope_.CalculatePoints(closestTriangle, pointA, pointB);

                normal = closestTriangle.Normal * (static_cast<Real>(1) / std::sqrt(closestTriangle.NormalSq));
                penetration = std::sqrt(closestTriangle.ClosestToOriginSq);

                return true;
            }

            if (!convexPolytope_.AddVertex(vertex))
            {
                convexPolytope_.CalculatePoints(closestTriangle, pointA, pointB);

                normal = closestTriangle.Normal * (static_cast<Real>(1) / std::sqrt(closestTriangle.NormalSq));
                penetration = std::sqrt(closestTriangle.ClosestToOriginSq);

                return true;
            }
        }

        Logger::Warning(
            "{0}: EPA, Could not converge within {1} iterations.",
            "NarrowPhase",
            maxIter);
        return false;
    }

    ConvexPolytope convexPolytope_;
};

} // namespace Detail

// Check if a point is inside a shape.
// support: Support map representing the shape.
// point: Point to check.
// Returns true if the point is contained within the shape, false otherwise.
template<typename TSupport>
bool PointTest(const TSupport& support, const LinearMath::JVector& point)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 34;

    LinearMath::JVector center;
    support.GetCenter(center);

    const LinearMath::JVector x = point;
    LinearMath::JVector v = x - center;

    SimplexSolver simplexSolver;
    simplexSolver.Reset();

    int iter = maxIter;
    Real distSq = v.LengthSquared();

    while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
    {
        LinearMath::JVector p;
        support.SupportMap(v, p);

        const LinearMath::JVector w = x - p;
        const Real vw = LinearMath::JVector::Dot(v, w);

        if (vw >= static_cast<Real>(0))
        {
            return false;
        }

        if (!simplexSolver.AddVertex(w, v))
        {
            break;
        }

        distSq = v.LengthSquared();
    }

    return true;
}

// Check if a point is inside a shape.
// support: Support map representing the shape.
// orientation: Orientation of the shape.
// position: Position of the shape.
// point: Point to check.
// Returns true if the point is contained within the shape, false otherwise.
template<typename TSupport>
bool PointTest(
    const TSupport& support,
    const LinearMath::JMatrix& orientation,
    const LinearMath::JVector& position,
    const LinearMath::JVector& point)
{
    const LinearMath::JVector transformedOrigin =
        LinearMath::JMatrix::TransposedTransform(point - position, orientation);
    return PointTest(support, transformedOrigin);
}

// Performs a ray cast against a shape.
// support: The support function of the shape.
// orientation: The orientation of the shape in world space.
// position: The position of the shape in world space.
// origin: The origin of the ray.
// direction: The direction of the ray; normalization is not necessary.
// lambda: Specifies the hit point of the ray, calculated as 'origin + lambda * direction'.
// Zero if the origin is inside the shape, Real.PositiveInfinity if the ray does not hit.
// normal: The normalized normal vector perpendicular to the surface, pointing outwards. Zero if the ray does not hit or
// the ray origin is inside the shape.
// Returns true if the ray intersects with the shape; otherwise, false.
template<typename TSupport>
bool RayCast(
    const TSupport& support,
    const LinearMath::JVector& origin,
    const LinearMath::JVector& direction,
    Real& lambda,
    LinearMath::JVector& normal)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 34;

    normal = LinearMath::JVector::Zero();
    lambda = static_cast<Real>(0);

    const LinearMath::JVector r = direction;
    LinearMath::JVector x = origin;

    LinearMath::JVector center;
    support.GetCenter(center);
    LinearMath::JVector v = x - center;

    SimplexSolver simplexSolver;
    simplexSolver.Reset();

    int iter = maxIter;
    Real distSq = v.LengthSquared();

    while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
    {
        LinearMath::JVector p;
        support.SupportMap(v, p);

        LinearMath::JVector w = x - p;
        const Real vdotW = LinearMath::JVector::Dot(v, w);

        if (vdotW > static_cast<Real>(0))
        {
            const Real vdotR = LinearMath::JVector::Dot(v, r);

            if (vdotR >= -NumericEpsilon)
            {
                lambda = std::numeric_limits<Real>::infinity();
                return false;
            }

            lambda -= vdotW / vdotR;
            x = origin + r * lambda;
            w = x - p;
            normal = v;
        }

        if (!simplexSolver.AddVertex(w, v))
        {
            break;
        }

        distSq = v.LengthSquared();
    }

    normal = NormalizeSafe(normal);
    return true;
}

// Performs a ray cast against a shape.
// support: The support function of the shape.
// origin: The origin of the ray.
// direction: The direction of the ray; normalization is not necessary.
// lambda: Specifies the hit point of the ray, calculated as 'origin + lambda * direction'.
// Zero if the origin is inside the shape, Real.PositiveInfinity if the ray does not hit.
// normal: The normalized normal vector perpendicular to the surface, pointing outwards. Zero if the ray does not hit or
// the ray origin is inside the shape.
// Returns true if the ray intersects with the shape; otherwise, false.
template<typename TSupport>
bool RayCast(
    const TSupport& support,
    const LinearMath::JQuaternion& orientation,
    const LinearMath::JVector& position,
    const LinearMath::JVector& origin,
    const LinearMath::JVector& direction,
    Real& lambda,
    LinearMath::JVector& normal)
{
    const LinearMath::JVector transformedDir =
        LinearMath::JQuaternion::ConjugatedTransform(direction, orientation);
    const LinearMath::JVector transformedOrigin =
        LinearMath::JQuaternion::ConjugatedTransform(origin - position, orientation);

    const bool result = RayCast(support, transformedOrigin, transformedDir, lambda, normal);
    normal = LinearMath::JQuaternion::Transform(normal, orientation);
    return result;
}

// Determines whether two convex shapes overlap, providing detailed information for both overlapping and separated
// cases. Internally, the method employs the Expanding Polytope Algorithm (EPA) to gather collision information.
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationA: The orientation of shape A in world space.
// orientationB: The orientation of shape B in world space.
// positionA: The position of shape A in world space.
// positionB: The position of shape B in world space.
// pointA: For the overlapping case: the deepest point on shape A inside shape B; for the separated case: the
// closest point on shape A to shape B.
// pointB: For the overlapping case: the deepest point on shape B inside shape A; for the separated case: the
// closest point on shape B to shape A.
// normal: The normalized collision normal pointing from pointB to pointA. This normal remains defined even
// if pointA and pointB coincide. It denotes the direction in which the shapes should be moved by the minimum distance
// (defined by the penetration depth) to either separate them in the overlapping case or bring them into contact in
// the separated case.
// penetration: The penetration depth.
// Returns true if the algorithm completes successfully, false otherwise. In case of algorithm convergence
// failure, collision information reverts to the type's default values.
template<typename TSupportA, typename TSupportB>
bool Collision(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& penetration)
{
    thread_local Detail::MprEpaSolver solver;
    return solver.Collision(supportA, supportB, orientationB, positionB, pointA, pointB, normal, penetration);
}

template<typename TSupportA, typename TSupportB>
bool Collision(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& penetration)
{
    const LinearMath::JQuaternion orientation =
        LinearMath::JQuaternion::ConjugateMultiply(orientationA, orientationB);
    const LinearMath::JVector position =
        LinearMath::JQuaternion::ConjugatedTransform(positionB - positionA, orientationA);

    thread_local Detail::MprEpaSolver solver;
    const bool success = solver.Collision(supportA, supportB, orientation, position, pointA, pointB, normal, penetration);

    pointA = LinearMath::JQuaternion::Transform(pointA, orientationA) + positionA;
    pointB = LinearMath::JQuaternion::Transform(pointB, orientationA) + positionA;
    normal = LinearMath::JQuaternion::Transform(normal, orientationA);

    return success;
}

// Provides the distance and closest points for non-overlapping shapes. It
// assumes that support shape A is located at position zero and not rotated.
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationB: The orientation of shape B in world space.
// positionB: The position of shape B in world space.
// pointA: Closest point on shape A. Undefined for the overlapping case.
// pointB: Closest point on shape B. Undefined for the overlapping case.
// normal: Unit direction from shape A toward shape B. Undefined for the overlapping case.
// distance: The distance between the separating shapes. Zero if shapes overlap.
// Returns true if the shapes do not overlap and distance information
// can be provided.
template<typename TSupportA, typename TSupportB>
bool Distance(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& distance)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 34;

    SimplexSolverAB simplexSolver;
    simplexSolver.Reset();

    int iter = maxIter;

    MinkowskiDifference::Vertex center;
    MinkowskiDifference::GetCenter(supportA, supportB, orientationB, positionB, center);

    LinearMath::JVector v = center.V;
    Real distSq = v.LengthSquared();

    while (iter-- != 0)
    {
        if (distSq < collideEpsilon * collideEpsilon)
        {
            distance = static_cast<Real>(0);
            normal = LinearMath::JVector::Zero();
            simplexSolver.GetClosest(pointA, pointB);
            return false;
        }

        MinkowskiDifference::Vertex w;
        MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, -v, w);

        const Real deltaDist = LinearMath::JVector::Dot(v - w.V, v);
        if (deltaDist * deltaDist < collideEpsilon * collideEpsilon * distSq)
        {
            break;
        }

        if (!simplexSolver.AddVertex(w, v))
        {
            distance = static_cast<Real>(0);
            normal = LinearMath::JVector::Zero();
            simplexSolver.GetClosest(pointA, pointB);
            return false;
        }

        distSq = v.LengthSquared();
    }

    distance = std::sqrt(distSq);
    normal = v * (static_cast<Real>(-1) / distance);
    simplexSolver.GetClosest(pointA, pointB);
    return true;
}

// Provides the distance and closest points for non-overlapping shapes.
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationA: The orientation of shape A in world space.
// orientationB: The orientation of shape B in world space.
// positionA: The position of shape A in world space.
// positionB: The position of shape B in world space.
// pointA: Closest point on shape A. Undefined for the overlapping case.
// pointB: Closest point on shape B. Undefined for the overlapping case.
// normal: Unit direction from shape A toward shape B. Undefined for the overlapping case.
// distance: The distance between the separating shapes. Zero if shapes overlap.
// Returns true if the shapes do not overlap and distance information
// can be provided.
template<typename TSupportA, typename TSupportB>
bool Distance(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& distance)
{
    const LinearMath::JQuaternion orientation =
        LinearMath::JQuaternion::ConjugateMultiply(orientationA, orientationB);
    const LinearMath::JVector position =
        LinearMath::JQuaternion::ConjugatedTransform(positionB - positionA, orientationA);

    const bool result = Distance(supportA, supportB, orientation, position, pointA, pointB, normal, distance);

    pointA = LinearMath::JQuaternion::Transform(pointA, orientationA) + positionA;
    pointB = LinearMath::JQuaternion::Transform(pointB, orientationA) + positionA;
    normal = LinearMath::JQuaternion::Transform(normal, orientationA);

    return result;
}

// Performs an overlap test. It assumes that support shape A is located
// at position zero and not rotated.
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationB: The orientation of shape B in world space.
// positionB: The position of shape B in world space.
// Returns true of the shapes overlap, and false otherwise.
template<typename TSupportA, typename TSupportB>
bool Overlap(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 34;

    SimplexSolverAB simplexSolver;
    simplexSolver.Reset();

    int iter = maxIter;

    MinkowskiDifference::Vertex center;
    MinkowskiDifference::GetCenter(supportA, supportB, orientationB, positionB, center);

    LinearMath::JVector v = center.V;
    Real distSq = v.LengthSquared();

    while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
    {
        MinkowskiDifference::Vertex w;
        MinkowskiDifference::Support(supportA, supportB, orientationB, positionB, -v, w);
        const Real vw = LinearMath::JVector::Dot(v, w.V);
        if (vw >= static_cast<Real>(0))
        {
            return false;
        }

        if (!simplexSolver.AddVertex(w, v))
        {
            return true;
        }

        distSq = v.LengthSquared();
    }

    return true;
}

// Performs an overlap test.
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationA: The orientation of shape A in world space.
// orientationB: The orientation of shape B in world space.
// positionA: The position of shape A in world space.
// positionB: The position of shape B in world space.
// Returns true of the shapes overlap, and false otherwise.
template<typename TSupportA, typename TSupportB>
bool Overlap(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB)
{
    const LinearMath::JQuaternion orientation =
        LinearMath::JQuaternion::ConjugateMultiply(orientationA, orientationB);
    const LinearMath::JVector position =
        LinearMath::JQuaternion::ConjugatedTransform(positionB - positionA, orientationA);

    return Overlap(supportA, supportB, orientation, position);
}

inline constexpr Real EpaPenetrationThreshold = static_cast<Real>(0.02);

// Detects whether two convex shapes overlap and provides detailed collision information for overlapping shapes.
// Internally, this method utilizes the Minkowski Portal Refinement (MPR) to obtain the collision information.
// Although MPR is not exact, it delivers a strict upper bound for the penetration depth. If the upper bound exceeds
// a predefined threshold, the results are further refined using the Expanding Polytope Algorithm (EPA).
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationA: The orientation of shape A in world space.
// orientationB: The orientation of shape B in world space.
// positionA: The position of shape A in world space.
// positionB: The position of shape B in world space.
// pointA: The deepest point on shape A that is inside shape B.
// pointB: The deepest point on shape B that is inside shape A.
// normal: The normalized collision normal pointing from pointB to pointA. This normal remains defined even
// if pointA and pointB coincide, representing the direction in which the shapes must be separated by the minimal
// distance (determined by the penetration depth) to avoid overlap.
// penetration: The penetration depth.
// epaThreshold: Penetration depth threshold above which MPR results are refined with EPA.
// Returns true if the shapes overlap (collide), and false otherwise.
template<typename TSupportA, typename TSupportB>
bool MprEpa(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& penetration,
    Real epaThreshold = EpaPenetrationThreshold)
{
    thread_local Detail::MprEpaSolver solver;
    return solver.SolveMpr(
        supportA,
        supportB,
        orientationB,
        positionB,
        epaThreshold,
        pointA,
        pointB,
        normal,
        penetration);
}

// Detects whether two convex shapes overlap and provides detailed collision information for overlapping shapes.
// It assumes that support shape A is at position zero and not rotated.
// Internally, this method utilizes the Minkowski Portal Refinement (MPR) to obtain the collision information.
// Although MPR is not exact, it delivers a strict upper bound for the penetration depth. If the upper bound
// exceeds a predefined threshold, the results are further refined using the Expanding Polytope Algorithm (EPA).
// supportA: The support function of shape A.
// supportB: The support function of shape B.
// orientationB: The orientation of shape B.
// positionB: The position of shape B.
// pointA: The deepest point on shape A that is inside shape B.
// pointB: The deepest point on shape B that is inside shape A.
// normal: The normalized collision normal pointing from pointB to pointA. This normal remains defined even
// if pointA and pointB coincide, representing the direction in which the shapes must be separated by the minimal
// distance (determined by the penetration depth) to avoid overlap.
// penetration: The penetration depth.
// epaThreshold: Penetration depth threshold above which MPR results are refined with EPA.
// Returns true if the shapes overlap (collide), and false otherwise.
template<typename TSupportA, typename TSupportB>
bool MprEpa(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& penetration,
    Real epaThreshold = EpaPenetrationThreshold)
{
    const LinearMath::JQuaternion orientation =
        LinearMath::JQuaternion::ConjugateMultiply(orientationA, orientationB);
    const LinearMath::JVector position =
        LinearMath::JQuaternion::ConjugatedTransform(positionB - positionA, orientationA);

    thread_local Detail::MprEpaSolver solver;
    const bool result = solver.SolveMpr(
        supportA,
        supportB,
        orientation,
        position,
        epaThreshold,
        pointA,
        pointB,
        normal,
        penetration);

    pointA = LinearMath::JQuaternion::Transform(pointA, orientationA) + positionA;
    pointB = LinearMath::JQuaternion::Transform(pointB, orientationA) + positionA;
    normal = LinearMath::JQuaternion::Transform(normal, orientationA);

    return result;
}

// Calculates the time of impact (TOI) and the collision points in world space for two shapes with linear
// velocities sweepA and sweepB and angular velocities
// sweepAngularA and sweepAngularB.
// supportA: Shape A.
// supportB: Shape B.
// orientationA: Orientation of shape A in world space.
// orientationB: Orientation of shape B in world space.
// positionA: Position of shape A in world space.
// positionB: Position of shape B in world space.
// sweepA: Linear velocity of shape A.
// sweepB: Linear velocity of shape B.
// sweepAngularA: Angular velocity of shape A.
// sweepAngularB: Angular velocity of shape B.
// extentA: Bounding sphere radius of shape A. Used to bound the angular displacement contribution.
// extentB: Bounding sphere radius of shape B. Used to bound the angular displacement contribution.
// pointA: Collision point on shape A in world space at the sweep origin.
// pointB: Collision point on shape B in world space at the sweep origin.
// normal: Normalized collision normal in world space at time of impact (points from A to B). Zero if the shapes already
// overlap or do not hit.
// lambda: Time of impact. Real.PositiveInfinity if no hit is detected. Zero if shapes overlap.
// Returns: True if the shapes hit or already overlap, false otherwise.
// Uses conservative advancement for continuous collision detection. May fail to converge to the correct TOI
// and collision points in certain edge cases due to limitations in linear motion approximation and
// distance gradient estimation.
template<typename TSupportA, typename TSupportB>
bool Sweep(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    const LinearMath::JVector& sweepB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& lambda)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 34;

    SimplexSolverAB simplexSolver;
    simplexSolver.Reset();

    MinkowskiDifference::Vertex center;
    MinkowskiDifference::GetCenter(supportA, supportB, orientationB, positionB, center);

    LinearMath::JVector posB = positionB;

    lambda = static_cast<Real>(0);
    pointA = LinearMath::JVector::Zero();
    pointB = LinearMath::JVector::Zero();

    const LinearMath::JVector r = sweepB;
    LinearMath::JVector v = -center.V;

    normal = LinearMath::JVector::Zero();

    int iter = maxIter;
    Real distSq = v.LengthSquared();

    while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
    {
        MinkowskiDifference::Vertex vertex;
        MinkowskiDifference::Support(supportA, supportB, orientationB, posB, v, vertex);
        const LinearMath::JVector w = vertex.V;

        const Real vDotW = -LinearMath::JVector::Dot(v, w);

        if (vDotW > static_cast<Real>(0))
        {
            const Real vDotR = LinearMath::JVector::Dot(v, r);

            if (vDotR >= static_cast<Real>(-1e-12))
            {
                lambda = std::numeric_limits<Real>::infinity();
                return false;
            }

            lambda -= vDotW / vDotR;
            posB = positionB + lambda * r;
            normal = v;
        }

        if (!simplexSolver.AddVertex(vertex, v))
        {
            break;
        }

        v = -v;
        distSq = v.LengthSquared();
    }

    simplexSolver.GetClosest(pointA, pointB);
    normal = NormalizeSafe(normal);

    return true;
}

// Calculates the time of impact and the collision points in world space for two shapes with linear velocities
// sweepA and sweepB.
// supportA: Shape A.
// supportB: Shape B.
// orientationA: Orientation of shape A in world space.
// orientationB: Orientation of shape B in world space.
// positionA: Position of shape A in world space.
// positionB: Position of shape B in world space.
// sweepA: Linear velocity of shape A.
// sweepB: Linear velocity of shape B.
// pointA: Collision point on shape A in world space at the sweep origin.
// pointB: Collision point on shape B in world space at the sweep origin.
// normal: Normalized collision normal in world space (points from A to B). Zero if the shapes already overlap or do not hit.
// lambda: Time of impact. Real.PositiveInfinity if no hit is detected, zero if shapes overlap.
// Returns: True if the shapes will hit or already overlap, false otherwise.
template<typename TSupportA, typename TSupportB>
bool Sweep(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB,
    const LinearMath::JVector& sweepA,
    const LinearMath::JVector& sweepB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& lambda)
{
    const LinearMath::JQuaternion orientation =
        LinearMath::JQuaternion::ConjugateMultiply(orientationA, orientationB);
    const LinearMath::JVector position =
        LinearMath::JQuaternion::ConjugatedTransform(positionB - positionA, orientationA);

    LinearMath::JVector sweep = sweepB - sweepA;
    sweep = LinearMath::JQuaternion::ConjugatedTransform(sweep, orientationA);

    const bool result = Sweep(supportA, supportB, orientation, position, sweep, pointA, pointB, normal, lambda);
    if (!result)
    {
        return false;
    }

    pointA = LinearMath::JQuaternion::Transform(pointA, orientationA) + positionA;
    pointB = LinearMath::JQuaternion::Transform(pointB, orientationA) + positionA;
    normal = LinearMath::JQuaternion::Transform(normal, orientationA);
    pointB += lambda * (sweepA - sweepB);

    return true;
}

// Performs a sweep test where shape A is fixed at the origin with identity orientation and zero velocity.
// supportA: Shape A (fixed at the origin, identity orientation).
// supportB: Shape B.
// orientationB: Orientation of shape B in world space.
// positionB: Position of shape B in world space.
// sweepB: Linear velocity of shape B.
// pointA: Collision point on shape A in world space at the sweep origin.
// pointB: Collision point on shape B in world space at the sweep origin.
// normal: Normalized collision normal in world space (points from A to B). Zero if the shapes already overlap or do not hit.
// lambda: Time of impact. Real.PositiveInfinity if no hit is detected, zero if shapes overlap.
// Returns: True if the shapes hit or already overlap, false otherwise.
template<typename TSupportA, typename TSupportB>
bool Sweep(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationA,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionA,
    const LinearMath::JVector& positionB,
    const LinearMath::JVector& sweepA,
    const LinearMath::JVector& sweepB,
    const LinearMath::JVector& sweepAngularA,
    const LinearMath::JVector& sweepAngularB,
    Real extentA,
    Real extentB,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& lambda)
{
    const Real collideEpsilon = static_cast<Real>(1e-4);
    constexpr int maxIter = 64;

    const Real maxAngularSpeed = extentA * sweepAngularA.Length() + extentB * sweepAngularB.Length();
    const Real combinedRadius = extentA + extentB;

    LinearMath::JVector posA = positionA;
    LinearMath::JVector posB = positionB;

    LinearMath::JQuaternion oriA = orientationA;
    LinearMath::JQuaternion oriB = orientationB;

    lambda = static_cast<Real>(0);

    int iter = 0;

    LinearMath::JQuaternion sweepAngularDeltaA = LinearMath::JQuaternion::Identity();
    LinearMath::JQuaternion sweepAngularDeltaB = LinearMath::JQuaternion::Identity();

    Real distance;
    Distance(supportA, supportB, oriA, oriB, posA, posB, pointA, pointB, normal, distance);

    if (distance < collideEpsilon)
    {
        return true;
    }

    while (true)
    {
        const Real sweepLinearProj = LinearMath::JVector::Dot(normal, sweepA - sweepB);
        const Real sweepLen = sweepLinearProj + maxAngularSpeed;

        if (sweepLen < NumericEpsilon || (sweepLinearProj < static_cast<Real>(0) && distance > combinedRadius))
        {
            normal = LinearMath::JVector::Zero();
            lambda = std::numeric_limits<Real>::infinity();
            return false;
        }

        const Real tmpLambda = distance / sweepLen;
        lambda += tmpLambda;

        sweepAngularDeltaA = LinearMath::MathHelper::RotationQuaternion(sweepAngularA, lambda);
        sweepAngularDeltaB = LinearMath::MathHelper::RotationQuaternion(sweepAngularB, lambda);

        oriA = sweepAngularDeltaA * orientationA;
        oriB = sweepAngularDeltaB * orientationB;

        posA = positionA + sweepA * lambda;
        posB = positionB + sweepB * lambda;

        if (iter++ > maxIter)
        {
            break;
        }

        LinearMath::JVector newNormal;
        const bool result = Distance(supportA, supportB, oriA, oriB, posA, posB, pointA, pointB, newNormal, distance);
        if (result)
        {
            normal = newNormal;
        }

        if (distance < collideEpsilon)
        {
            break;
        }
    }

    const LinearMath::JVector linearTransformationA = sweepA * lambda;
    const LinearMath::JVector linearTransformationB = sweepB * lambda;

    const LinearMath::JVector deltaA = pointA - posA;
    const LinearMath::JVector deltaB = pointB - posB;

    pointA -= linearTransformationA
        + (deltaA - LinearMath::JQuaternion::ConjugatedTransform(deltaA, sweepAngularDeltaA));
    pointB -= linearTransformationB
        + (deltaB - LinearMath::JQuaternion::ConjugatedTransform(deltaB, sweepAngularDeltaB));

    return true;
}

} // namespace Jitter2::Collision::NarrowPhase
