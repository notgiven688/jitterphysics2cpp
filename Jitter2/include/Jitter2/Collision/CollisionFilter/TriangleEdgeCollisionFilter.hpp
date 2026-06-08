#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Jitter2/Collision/CollisionFilter/INarrowPhaseFilter.hpp>
#include <Jitter2/Collision/NarrowPhase/NarrowPhase.hpp>
#include <Jitter2/Collision/Shapes/TriangleShape.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/LinearMath/JAngle.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Precision.hpp>

namespace Jitter2::Collision
{

// Filters internal edge collisions for TriangleShape geometry. Adjusts collision
// normals at shared edges to match neighboring triangles, or discards the collision if the normal
// cannot be resolved. Requires triangle adjacency information; boundary edges (no neighbor)
// are left unmodified. Back-face collisions are discarded.
class TriangleEdgeCollisionFilter : public INarrowPhaseFilter
{
public:

    // Gets or sets the distance threshold for edge collision detection, in world units.
    // Larger values are more aggressive at filtering edges but may incorrectly affect
    // legitimate collisions near triangle boundaries.
    // Value: The default value is 0.01 world units.
    [[nodiscard]] Real EdgeThreshold() const { return edgeThreshold_; }

    // Gets or sets the distance threshold for edge collision detection, in world units.
    void EdgeThreshold(Real value) { edgeThreshold_ = value; }

    // Gets or sets the minimum length of the projected collision normal required to keep the contact.
    // When the collision normal is projected onto the plane formed by the triangle normal and its
    // neighbor's normal, a very short projection indicates the collision is occurring along the
    // edge crease itself and is discarded. Lower values allow more edge collisions through;
    // higher values are more aggressive at filtering.
    // Value: The default value is 0.5.
    [[nodiscard]] Real ProjectionThreshold() const { return projectionThreshold_; }

    // Gets or sets the minimum length of the projected collision normal required to keep the contact.
    void ProjectionThreshold(Real value) { projectionThreshold_ = value; }

    // Gets or sets the angle threshold used for two purposes: determining when two triangle normals
    // are considered coplanar (using simpler snapping logic), and discarding back-face collisions
    // whose normal is anti-parallel to the triangle normal within this angle.
    // Value: The default value is approximately 2.5 degrees.
    [[nodiscard]] LinearMath::JAngle AngleThreshold() const
    {
        return LinearMath::JAngle::FromRadian(StableMath::Acos(cosAngle_));
    }

    // Gets or sets the angle threshold used for two purposes: determining when two triangle normals
    // are considered coplanar (using simpler snapping logic), and discarding back-face collisions
    // whose normal is anti-parallel to the triangle normal within this angle.
    void AngleThreshold(LinearMath::JAngle value)
    {
        cosAngle_ = StableMath::Cos(value.Radian);
    }


    bool Filter(
        const Shapes::RigidBodyShape& shapeA,
        const Shapes::RigidBodyShape& shapeB,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration) override
    {
        const auto* triangleA = dynamic_cast<const Shapes::TriangleShape*>(&shapeA);
        const auto* triangleB = dynamic_cast<const Shapes::TriangleShape*>(&shapeB);

        const bool isTriangleA = triangleA != nullptr;
        const bool isTriangleB = triangleB != nullptr;

        if (isTriangleA == isTriangleB)
        {
            return true;
        }

        const Shapes::TriangleShape& triangleShape = isTriangleA ? *triangleA : *triangleB;
        LinearMath::JVector triangleNormal;
        LinearMath::JVector neighborNormal;
        EdgeContactEvaluation evaluation = EvaluateTriangleContact(
            triangleShape,
            isTriangleB,
            isTriangleA ? pointA : pointB,
            normal,
            triangleNormal,
            neighborNormal);

        if (evaluation == EdgeContactEvaluation::Discard)
        {
            return false;
        }

        if (evaluation == EdgeContactEvaluation::Keep)
        {
            return true;
        }

        RigidBody* bodyA = shapeA.GetRigidBody();
        RigidBody* bodyB = shapeB.GetRigidBody();
        if (bodyA == nullptr || bodyB == nullptr)
        {
            return true;
        }

        const bool isSpeculative = penetration < static_cast<Real>(0);

        if (!isSpeculative)
        {
            const bool result = NarrowPhase::MprEpa(
                shapeA,
                shapeB,
                bodyA->Data().Orientation,
                bodyB->Data().Orientation,
                bodyA->Data().Position,
                bodyB->Data().Position,
                pointA,
                pointB,
                normal,
                penetration,
                static_cast<Real>(0));

            if (!result)
            {
                return false;
            }

            evaluation = EvaluateTriangleContact(
                triangleShape,
                isTriangleB,
                isTriangleA ? pointA : pointB,
                normal,
                triangleNormal,
                neighborNormal);

            if (evaluation == EdgeContactEvaluation::Discard)
            {
                return false;
            }

            if (evaluation == EdgeContactEvaluation::Keep)
            {
                return true;
            }
        }

        const LinearMath::JVector midPoint = static_cast<Real>(0.5) * (pointA + pointB);

        if (LinearMath::JVector::Dot(triangleNormal, neighborNormal) > cosAngle_)
        {
            if (LinearMath::JVector::Dot(normal, neighborNormal)
                > LinearMath::JVector::Dot(normal, triangleNormal))
            {
                if (!isSpeculative)
                {
                    penetration = static_cast<Real>(0);
                    pointA = midPoint;
                    pointB = midPoint;
                }

                normal = neighborNormal;
            }
            else
            {
                if (!isSpeculative)
                {
                    penetration = static_cast<Real>(0);
                    pointA = midPoint;
                    pointB = midPoint;
                }

                normal = triangleNormal;
            }

            return true;
        }

        const LinearMath::JVector cross = LinearMath::JVector::Cross(neighborNormal, triangleNormal);
        const Real crossLenSq = cross.LengthSquared();
        const LinearMath::JVector projection =
            normal - (LinearMath::JVector::Dot(cross, normal) / crossLenSq) * cross;

        if (projection.LengthSquared() < projectionThreshold_ * projectionThreshold_)
        {
            return false;
        }

        const Real f1 = LinearMath::JVector::Dot(LinearMath::JVector::Cross(projection, neighborNormal), cross);
        const Real f2 = LinearMath::JVector::Dot(LinearMath::JVector::Cross(projection, triangleNormal), cross);
        const bool between = f1 <= static_cast<Real>(0) && f2 >= static_cast<Real>(0);

        if (!between)
        {
            if (LinearMath::JVector::Dot(normal, neighborNormal)
                > LinearMath::JVector::Dot(normal, triangleNormal))
            {
                if (!isSpeculative)
                {
                    penetration = static_cast<Real>(0);
                    pointA = midPoint;
                    pointB = midPoint;
                }

                normal = neighborNormal;
            }
            else
            {
                if (!isSpeculative)
                {
                    penetration = static_cast<Real>(0);
                    pointA = midPoint;
                    pointB = midPoint;
                }

                normal = triangleNormal;
            }
        }

        return true;
    }

private:
    enum class EdgeContactEvaluation
    {
        Keep,
        Discard,
        Edge
    };

    static void ProjectPointOnPlane(
        LinearMath::JVector& point,
        const LinearMath::JVector& a,
        const LinearMath::JVector& b,
        const LinearMath::JVector& c)
    {
        const LinearMath::JVector ab = b - a;
        const LinearMath::JVector ac = c - a;
        LinearMath::JVector planeNormal = LinearMath::JVector::Cross(ab, ac);
        planeNormal.Normalize();

        const LinearMath::JVector ap = point - a;
        const Real distance = LinearMath::JVector::Dot(ap, planeNormal);
        point -= distance * planeNormal;
    }

    EdgeContactEvaluation EvaluateTriangleContact(
        const Shapes::TriangleShape& triangleShape,
        bool triangleIsShapeB,
        const LinearMath::JVector& contactPoint,
        const LinearMath::JVector& normal,
        LinearMath::JVector& triangleNormal,
        LinearMath::JVector& neighborNormal) const
    {
        const auto& triangle = triangleShape.Mesh().Indices().at(static_cast<std::size_t>(triangleShape.Index()));
        RigidBody* triangleBody = triangleShape.GetRigidBody();
        if (triangleBody == nullptr)
        {
            return EdgeContactEvaluation::Keep;
        }

        triangleNormal = LinearMath::JQuaternion::Transform(triangle.Normal, triangleBody->Data().Orientation);
        neighborNormal = LinearMath::JVector::Zero();

        if (triangleIsShapeB)
        {
            triangleNormal = -triangleNormal;
        }

        if (LinearMath::JVector::Dot(normal, triangleNormal) < -cosAngle_)
        {
            return EdgeContactEvaluation::Discard;
        }

        LinearMath::JVector a;
        LinearMath::JVector b;
        LinearMath::JVector c;
        triangleShape.GetWorldVertices(a, b, c);

        LinearMath::JVector projectedContactPoint = contactPoint;
        ProjectPointOnPlane(projectedContactPoint, a, b, c);

        LinearMath::JVector n = b - a;
        LinearMath::JVector pma = projectedContactPoint - a;
        const Real d0 = (pma - LinearMath::JVector::Dot(pma, n)
            * n * (static_cast<Real>(1) / n.LengthSquared())).LengthSquared();

        n = c - a;
        pma = projectedContactPoint - a;
        const Real d1 = (pma - LinearMath::JVector::Dot(pma, n)
            * n * (static_cast<Real>(1) / n.LengthSquared())).LengthSquared();

        n = c - b;
        pma = projectedContactPoint - b;
        const Real d2 = (pma - LinearMath::JVector::Dot(pma, n)
            * n * (static_cast<Real>(1) / n.LengthSquared())).LengthSquared();

        if (std::min(std::min(d0, d1), d2) > edgeThreshold_ * edgeThreshold_)
        {
            return EdgeContactEvaluation::Keep;
        }

        if (d0 < d1 && d0 < d2)
        {
            if (triangle.NeighborC == -1)
            {
                return EdgeContactEvaluation::Keep;
            }
            neighborNormal = triangleShape.Mesh().Indices().at(static_cast<std::size_t>(triangle.NeighborC)).Normal;
        }
        else if (d1 < d2)
        {
            if (triangle.NeighborB == -1)
            {
                return EdgeContactEvaluation::Keep;
            }
            neighborNormal = triangleShape.Mesh().Indices().at(static_cast<std::size_t>(triangle.NeighborB)).Normal;
        }
        else
        {
            if (triangle.NeighborA == -1)
            {
                return EdgeContactEvaluation::Keep;
            }
            neighborNormal = triangleShape.Mesh().Indices().at(static_cast<std::size_t>(triangle.NeighborA)).Normal;
        }

        neighborNormal = LinearMath::JQuaternion::Transform(neighborNormal, triangleBody->Data().Orientation);

        if (triangleIsShapeB)
        {
            neighborNormal = -neighborNormal;
        }

        return EdgeContactEvaluation::Edge;
    }

    Real edgeThreshold_ = static_cast<Real>(0.01);
    Real cosAngle_ = static_cast<Real>(0.999);
    Real projectionThreshold_ = static_cast<Real>(0.5);
};

} // namespace Jitter2::Collision
