#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>

namespace Jitter2::Collision
{

// Represents a contact manifold between two convex shapes.
// The manifold is constructed from support points sampled around the collision normal,
// then reduced to a small contact set suitable for the solver.
class CollisionManifold
{
public:
    static constexpr int MaxSupportPoints = 6;
    static constexpr int MaxManifoldPoints = 12;
    static constexpr int SolverContactLimit = 4;

    [[nodiscard]] int Count() const { return manifoldCount_; }
    [[nodiscard]] const LinearMath::JVector& ManifoldA(int index) const
    {
        return manifoldA_[static_cast<std::size_t>(index)];
    }
    [[nodiscard]] const LinearMath::JVector& ManifoldB(int index) const
    {
        return manifoldB_[static_cast<std::size_t>(index)];
    }

    // Builds the contact manifold between two shapes given their transforms and initial contact.
    // Ta: The type of support shape A.
    // Tb: The type of support shape B.
    // shapeA: The first shape.
    // shapeB: The second shape.
    // quaternionA: Orientation of shape A.
    // quaternionB: Orientation of shape B.
    // positionA: Position of shape A.
    // positionB: Position of shape B.
    // pA: Initial contact point on shape A.
    // pB: Initial contact point on shape B.
    // normal: The collision normal (from B to A).
    template<typename TSupportA, typename TSupportB>
    void BuildManifold(
        const TSupportA& shapeA,
        const TSupportB& shapeB,
        const LinearMath::JQuaternion& quaternionA,
        const LinearMath::JQuaternion& quaternionB,
        const LinearMath::JVector& positionA,
        const LinearMath::JVector& positionB,
        const LinearMath::JVector& pointA,
        const LinearMath::JVector& pointB,
        const LinearMath::JVector& normal)
    {
        leftCount_ = 0;
        rightCount_ = 0;
        manifoldCount_ = 0;

        const LinearMath::JVector crossVector1 = LinearMath::MathHelper::CreateOrthonormal(normal);
        const LinearMath::JVector crossVector2 = LinearMath::JVector::Cross(normal, crossVector1);

        const LinearMath::JVector relativePositionB = positionB - positionA;
        const LinearMath::JVector pointALocal = pointA - positionA;
        const LinearMath::JVector pointBLocal = pointB - positionA;

        std::array<LinearMath::JVector, MaxSupportPoints> left {};
        std::array<LinearMath::JVector, MaxSupportPoints> right {};

        for (int e = 0; e < MaxSupportPoints; ++e)
        {
            LinearMath::JVector perturbedNormal = normal
                + HexagonVertices[static_cast<std::size_t>(2 * e + 0)] * Perturbation * crossVector1
                + HexagonVertices[static_cast<std::size_t>(2 * e + 1)] * Perturbation * crossVector2;

            LinearMath::JVector tmp = LinearMath::JQuaternion::ConjugatedTransform(perturbedNormal, quaternionA);
            LinearMath::JVector supportA;
            shapeA.SupportMap(tmp, supportA);
            supportA = LinearMath::JQuaternion::Transform(supportA, quaternionA);
            PushLeft(left, supportA);

            perturbedNormal = -perturbedNormal;

            tmp = LinearMath::JQuaternion::ConjugatedTransform(perturbedNormal, quaternionB);
            LinearMath::JVector supportB;
            shapeB.SupportMap(tmp, supportB);
            supportB = LinearMath::JQuaternion::Transform(supportB, quaternionB) + relativePositionB;
            PushRight(right, supportB);
        }

        Real leftSign = static_cast<Real>(0);
        Real rightSign = static_cast<Real>(0);
        const bool leftPolygon = leftCount_ > 2 && TryGetPolygonSign(left, leftCount_, normal, leftSign);
        const bool rightPolygon = rightCount_ > 2 && TryGetPolygonSign(right, rightCount_, normal, rightSign);

        if (leftPolygon)
        {
            for (int e = 0; e < rightCount_; ++e)
            {
                const LinearMath::JVector point = right[static_cast<std::size_t>(e)];
                if (!ContainsPoint(left, leftCount_, point, normal, leftSign))
                {
                    continue;
                }

                AddPointOnB(point, pointALocal, normal);
                if (manifoldCount_ == MaxManifoldPoints)
                {
                    goto finalize;
                }
            }

            if (rightCount_ == 2)
            {
                std::array<LinearMath::JVector, 2> clipped {};
                const int clippedCount = ClipSegmentAgainstPolygon(
                    right[0],
                    right[1],
                    left,
                    leftCount_,
                    normal,
                    leftSign,
                    clipped);

                for (int i = 0; i < clippedCount; ++i)
                {
                    AddPointOnB(clipped[static_cast<std::size_t>(i)], pointALocal, normal);
                    if (manifoldCount_ == MaxManifoldPoints)
                    {
                        goto finalize;
                    }
                }
            }
        }

        if (rightPolygon)
        {
            for (int e = 0; e < leftCount_; ++e)
            {
                const LinearMath::JVector point = left[static_cast<std::size_t>(e)];
                if (!ContainsPoint(right, rightCount_, point, normal, rightSign))
                {
                    continue;
                }

                AddPointOnA(point, pointBLocal, normal);
                if (manifoldCount_ == MaxManifoldPoints)
                {
                    goto finalize;
                }
            }

            if (leftCount_ == 2)
            {
                std::array<LinearMath::JVector, 2> clipped {};
                const int clippedCount = ClipSegmentAgainstPolygon(
                    left[0],
                    left[1],
                    right,
                    rightCount_,
                    normal,
                    rightSign,
                    clipped);

                for (int i = 0; i < clippedCount; ++i)
                {
                    AddPointOnA(clipped[static_cast<std::size_t>(i)], pointBLocal, normal);
                    if (manifoldCount_ == MaxManifoldPoints)
                    {
                        goto finalize;
                    }
                }
            }
        }

        if (leftCount_ == 2 && rightCount_ == 2)
        {
            std::array<LinearMath::JVector, 2> clippedA {};
            std::array<LinearMath::JVector, 2> clippedB {};
            const int clippedCount = IntersectSegments(left[0], left[1], right[0], right[1], normal, clippedA, clippedB);

            for (int i = 0; i < clippedCount; ++i)
            {
                AddPointPair(clippedA[static_cast<std::size_t>(i)], clippedB[static_cast<std::size_t>(i)], normal);
                if (manifoldCount_ == MaxManifoldPoints)
                {
                    goto finalize;
                }
            }
        }

        if (leftPolygon && rightPolygon)
        {
            std::array<LinearMath::JVector, 2> clippedA {};
            std::array<LinearMath::JVector, 2> clippedB {};

            for (int i = 0; i < leftCount_; ++i)
            {
                const LinearMath::JVector startA = left[static_cast<std::size_t>(i)];
                const LinearMath::JVector endA = left[static_cast<std::size_t>((i + 1) % leftCount_)];

                for (int j = 0; j < rightCount_; ++j)
                {
                    const LinearMath::JVector startB = right[static_cast<std::size_t>(j)];
                    const LinearMath::JVector endB = right[static_cast<std::size_t>((j + 1) % rightCount_)];

                    const int clippedCount = IntersectSegments(startA, endA, startB, endB, normal, clippedA, clippedB);

                    for (int k = 0; k < clippedCount; ++k)
                    {
                        AddPointPair(
                            clippedA[static_cast<std::size_t>(k)],
                            clippedB[static_cast<std::size_t>(k)],
                            normal);
                        if (manifoldCount_ == MaxManifoldPoints)
                        {
                            goto finalize;
                        }
                    }
                }
            }
        }

        if (manifoldCount_ == 0)
        {
            manifoldA_[static_cast<std::size_t>(manifoldCount_)] = pointALocal;
            manifoldB_[static_cast<std::size_t>(manifoldCount_++)] = pointBLocal;
        }

    finalize:
        ReduceManifold(normal);

        for (int i = 0; i < manifoldCount_; ++i)
        {
            manifoldA_[static_cast<std::size_t>(i)] += positionA;
            manifoldB_[static_cast<std::size_t>(i)] += positionA;
        }
    }

    // Builds the contact manifold between two rigid body shapes using their current transforms.
    // Ta: The type of shape A.
    // Tb: The type of shape B.
    // shapeA: The first rigid body shape.
    // shapeB: The second rigid body shape.
    // pA: Initial contact point on shape A.
    // pB: Initial contact point on shape B.
    // normal: The collision normal (from B to A).
    template<typename TShapeA, typename TShapeB>
    void BuildManifold(
        const TShapeA& shapeA,
        const TShapeB& shapeB,
        const LinearMath::JVector& pointA,
        const LinearMath::JVector& pointB,
        const LinearMath::JVector& normal)
    {
        BuildManifold(
            shapeA,
            shapeB,
            shapeA.Orientation,
            shapeB.Orientation,
            shapeA.Position,
            shapeB.Position,
            pointA,
            pointB,
            normal);
    }

private:
    static constexpr Real DuplicatePointDistanceSq = static_cast<Real>(0.001);
    static constexpr Real Sqrt3Over2 = static_cast<Real>(0.86602540378);
    static constexpr Real Perturbation = static_cast<Real>(0.01);
    static constexpr std::array<Real, MaxSupportPoints * 2> HexagonVertices {
        static_cast<Real>(1), static_cast<Real>(0),
        static_cast<Real>(0.5), Sqrt3Over2,
        static_cast<Real>(-0.5), Sqrt3Over2,
        static_cast<Real>(-1), static_cast<Real>(0),
        static_cast<Real>(-0.5), -Sqrt3Over2,
        static_cast<Real>(0.5), -Sqrt3Over2,
    };

    void PushLeft(std::array<LinearMath::JVector, MaxSupportPoints>& left, const LinearMath::JVector& value)
    {
        if (leftCount_ > 0
            && (left[0] - value).LengthSquared() < DuplicatePointDistanceSq)
        {
            return;
        }

        if (leftCount_ > 1
            && (left[static_cast<std::size_t>(leftCount_ - 1)] - value).LengthSquared() < DuplicatePointDistanceSq)
        {
            return;
        }

        left[static_cast<std::size_t>(leftCount_++)] = value;
    }

    void PushRight(std::array<LinearMath::JVector, MaxSupportPoints>& right, const LinearMath::JVector& value)
    {
        if (rightCount_ > 0
            && (right[0] - value).LengthSquared() < DuplicatePointDistanceSq)
        {
            return;
        }

        if (rightCount_ > 1
            && (right[static_cast<std::size_t>(rightCount_ - 1)] - value).LengthSquared() < DuplicatePointDistanceSq)
        {
            return;
        }

        right[static_cast<std::size_t>(rightCount_++)] = value;
    }

    static bool TryGetPolygonSign(
        const std::array<LinearMath::JVector, MaxSupportPoints>& polygon,
        int count,
        const LinearMath::JVector& normal,
        Real& sign)
    {
        const Real epsilon = static_cast<Real>(1e-6);

        Real area = static_cast<Real>(0);
        LinearMath::JVector previous = polygon[static_cast<std::size_t>(count - 1)];

        for (int i = 0; i < count; ++i)
        {
            const LinearMath::JVector current = polygon[static_cast<std::size_t>(i)];
            area += LinearMath::JVector::Dot(LinearMath::JVector::Cross(previous, current), normal);
            previous = current;
        }

        if (std::abs(area) <= epsilon)
        {
            sign = static_cast<Real>(0);
            return false;
        }

        sign = area < static_cast<Real>(0) ? static_cast<Real>(-1) : static_cast<Real>(1);
        return true;
    }

    static bool ContainsPoint(
        const std::array<LinearMath::JVector, MaxSupportPoints>& polygon,
        int count,
        const LinearMath::JVector& point,
        const LinearMath::JVector& normal,
        Real sign)
    {
        const Real epsilon = static_cast<Real>(1e-3);

        LinearMath::JVector previous = polygon[static_cast<std::size_t>(count - 1)];

        for (int i = 0; i < count; ++i)
        {
            const LinearMath::JVector current = polygon[static_cast<std::size_t>(i)];
            const Real side = sign * LinearMath::JVector::Dot(
                LinearMath::JVector::Cross(current - previous, point - previous),
                normal);

            if (side < -epsilon)
            {
                return false;
            }

            previous = current;
        }

        return true;
    }

    static int ClipSegmentAgainstPolygon(
        const LinearMath::JVector& start,
        const LinearMath::JVector& end,
        const std::array<LinearMath::JVector, MaxSupportPoints>& polygon,
        int polygonCount,
        const LinearMath::JVector& normal,
        Real sign,
        std::array<LinearMath::JVector, 2>& clipped)
    {
        const Real epsilon = static_cast<Real>(1e-3);

        Real enter = static_cast<Real>(0);
        Real exit = static_cast<Real>(1);
        const LinearMath::JVector delta = end - start;

        for (int i = 0; i < polygonCount; ++i)
        {
            const LinearMath::JVector edgeStart = polygon[static_cast<std::size_t>(i)];
            const LinearMath::JVector edgeEnd = polygon[static_cast<std::size_t>((i + 1) % polygonCount)];

            const Real startSide = sign * LinearMath::JVector::Dot(
                LinearMath::JVector::Cross(edgeEnd - edgeStart, start - edgeStart),
                normal);
            const Real endSide = sign * LinearMath::JVector::Dot(
                LinearMath::JVector::Cross(edgeEnd - edgeStart, end - edgeStart),
                normal);

            const bool startInside = startSide >= -epsilon;
            const bool endInside = endSide >= -epsilon;

            if (!startInside && !endInside)
            {
                return 0;
            }

            if (startInside && endInside)
            {
                continue;
            }

            const Real denominator = startSide - endSide;
            if (std::abs(denominator) <= epsilon)
            {
                return 0;
            }

            const Real t = std::clamp(startSide / denominator, static_cast<Real>(0), static_cast<Real>(1));

            if (!startInside)
            {
                enter = std::max(enter, t);
            }
            else
            {
                exit = std::min(exit, t);
            }

            if (exit < enter)
            {
                return 0;
            }
        }

        clipped[0] = start + enter * delta;

        if ((start + exit * delta - clipped[0]).LengthSquared() < epsilon)
        {
            return 1;
        }

        clipped[1] = start + exit * delta;
        return 2;
    }

    static Real CrossAlongNormal(
        const LinearMath::JVector& left,
        const LinearMath::JVector& right,
        const LinearMath::JVector& normal)
    {
        return LinearMath::JVector::Dot(LinearMath::JVector::Cross(left, right), normal);
    }

    static int IntersectSegments(
        const LinearMath::JVector& startA,
        const LinearMath::JVector& endA,
        const LinearMath::JVector& startB,
        const LinearMath::JVector& endB,
        const LinearMath::JVector& normal,
        std::array<LinearMath::JVector, 2>& clippedA,
        std::array<LinearMath::JVector, 2>& clippedB)
    {
        const Real epsilon = static_cast<Real>(1e-3);

        const LinearMath::JVector deltaA = endA - startA;
        const LinearMath::JVector deltaB = endB - startB;
        const LinearMath::JVector offset = startB - startA;

        const Real cross = CrossAlongNormal(deltaA, deltaB, normal);

        if (std::abs(cross) <= epsilon)
        {
            if (std::abs(CrossAlongNormal(offset, deltaA, normal)) > epsilon)
            {
                return 0;
            }

            const Real lengthASquared = deltaA.LengthSquared();
            const Real lengthBSquared = deltaB.LengthSquared();

            if (lengthASquared <= epsilon || lengthBSquared <= epsilon)
            {
                return 0;
            }

            const Real parameterB0 = LinearMath::JVector::Dot(startB - startA, deltaA) / lengthASquared;
            const Real parameterB1 = LinearMath::JVector::Dot(endB - startA, deltaA) / lengthASquared;

            const Real enter = std::max(static_cast<Real>(0), std::min(parameterB0, parameterB1));
            const Real exit = std::min(static_cast<Real>(1), std::max(parameterB0, parameterB1));

            if (exit < enter - epsilon)
            {
                return 0;
            }

            clippedA[0] = startA + enter * deltaA;

            const Real parameterA0 = std::clamp(
                LinearMath::JVector::Dot(clippedA[0] - startB, deltaB) / lengthBSquared,
                static_cast<Real>(0),
                static_cast<Real>(1));
            clippedB[0] = startB + parameterA0 * deltaB;

            clippedA[1] = startA + exit * deltaA;

            if ((clippedA[1] - clippedA[0]).LengthSquared() <= epsilon)
            {
                return 1;
            }

            const Real parameterA1 = std::clamp(
                LinearMath::JVector::Dot(clippedA[1] - startB, deltaB) / lengthBSquared,
                static_cast<Real>(0),
                static_cast<Real>(1));
            clippedB[1] = startB + parameterA1 * deltaB;

            return 2;
        }

        Real t = CrossAlongNormal(offset, deltaB, normal) / cross;
        Real u = CrossAlongNormal(offset, deltaA, normal) / cross;

        if (t < -epsilon || t > static_cast<Real>(1) + epsilon
            || u < -epsilon || u > static_cast<Real>(1) + epsilon)
        {
            return 0;
        }

        t = std::clamp(t, static_cast<Real>(0), static_cast<Real>(1));
        u = std::clamp(u, static_cast<Real>(0), static_cast<Real>(1));

        clippedA[0] = startA + t * deltaA;
        clippedB[0] = startB + u * deltaB;

        return 1;
    }

    bool IsDuplicatePoint(const std::array<LinearMath::JVector, MaxManifoldPoints>& manifold, const LinearMath::JVector& point) const
    {
        for (int i = 0; i < manifoldCount_; ++i)
        {
            if ((manifold[static_cast<std::size_t>(i)] - point).LengthSquared() < DuplicatePointDistanceSq)
            {
                return true;
            }
        }

        return false;
    }

    void AddPointOnA(
        const LinearMath::JVector& point,
        const LinearMath::JVector& pointB,
        const LinearMath::JVector& normal)
    {
        if (manifoldCount_ == MaxManifoldPoints)
        {
            return;
        }

        const Real diff = LinearMath::JVector::Dot(point - pointB, normal);
        if (diff < static_cast<Real>(0))
        {
            return;
        }

        if (IsDuplicatePoint(manifoldA_, point))
        {
            return;
        }

        manifoldA_[static_cast<std::size_t>(manifoldCount_)] = point;
        manifoldB_[static_cast<std::size_t>(manifoldCount_++)] = point - diff * normal;
    }

    void AddPointOnB(
        const LinearMath::JVector& point,
        const LinearMath::JVector& pointA,
        const LinearMath::JVector& normal)
    {
        if (manifoldCount_ == MaxManifoldPoints)
        {
            return;
        }

        const Real diff = LinearMath::JVector::Dot(point - pointA, normal);
        if (diff > static_cast<Real>(0))
        {
            return;
        }

        if (IsDuplicatePoint(manifoldB_, point))
        {
            return;
        }

        manifoldB_[static_cast<std::size_t>(manifoldCount_)] = point;
        manifoldA_[static_cast<std::size_t>(manifoldCount_++)] = point - diff * normal;
    }

    void AddPointPair(
        const LinearMath::JVector& pointA,
        const LinearMath::JVector& pointB,
        const LinearMath::JVector& normal)
    {
        if (manifoldCount_ == MaxManifoldPoints)
        {
            return;
        }

        if (LinearMath::JVector::Dot(pointA - pointB, normal) < static_cast<Real>(0))
        {
            return;
        }

        if (IsDuplicatePoint(manifoldA_, pointA) || IsDuplicatePoint(manifoldB_, pointB))
        {
            return;
        }

        manifoldA_[static_cast<std::size_t>(manifoldCount_)] = pointA;
        manifoldB_[static_cast<std::size_t>(manifoldCount_++)] = pointB;
    }

    void ReduceManifold(const LinearMath::JVector& normal)
    {
        if (manifoldCount_ <= SolverContactLimit)
        {
            return;
        }

        LinearMath::JVector centroid = LinearMath::JVector::Zero();

        for (int i = 0; i < manifoldCount_; ++i)
        {
            centroid += manifoldA_[static_cast<std::size_t>(i)];
        }

        centroid *= static_cast<Real>(1) / static_cast<Real>(manifoldCount_);

        const LinearMath::JVector tangent1 = LinearMath::MathHelper::CreateOrthonormal(normal);
        const LinearMath::JVector tangent2 = LinearMath::JVector::Cross(normal, tangent1);

        std::array<double, MaxManifoldPoints> angles {};
        std::array<int, MaxManifoldPoints> order {};

        for (int i = 0; i < manifoldCount_; ++i)
        {
            const LinearMath::JVector delta = manifoldA_[static_cast<std::size_t>(i)] - centroid;
            angles[static_cast<std::size_t>(i)] = static_cast<double>(
                StableMath::Atan2(
                    LinearMath::JVector::Dot(delta, tangent2),
                    LinearMath::JVector::Dot(delta, tangent1)));
            order[static_cast<std::size_t>(i)] = i;
        }

        for (int i = 1; i < manifoldCount_; ++i)
        {
            const int current = order[static_cast<std::size_t>(i)];
            const double currentAngle = angles[static_cast<std::size_t>(current)];
            int j = i - 1;

            while (j >= 0 && angles[static_cast<std::size_t>(order[static_cast<std::size_t>(j)])] > currentAngle)
            {
                order[static_cast<std::size_t>(j + 1)] = order[static_cast<std::size_t>(j)];
                --j;
            }

            order[static_cast<std::size_t>(j + 1)] = current;
        }

        std::array<LinearMath::JVector, SolverContactLimit> reducedA {};
        std::array<LinearMath::JVector, SolverContactLimit> reducedB {};

        for (int i = 0; i < SolverContactLimit; ++i)
        {
            const int selected = order[static_cast<std::size_t>(
                ((2 * i + 1) * manifoldCount_) / (2 * SolverContactLimit))];
            reducedA[static_cast<std::size_t>(i)] = manifoldA_[static_cast<std::size_t>(selected)];
            reducedB[static_cast<std::size_t>(i)] = manifoldB_[static_cast<std::size_t>(selected)];
        }

        for (int i = 0; i < SolverContactLimit; ++i)
        {
            manifoldA_[static_cast<std::size_t>(i)] = reducedA[static_cast<std::size_t>(i)];
            manifoldB_[static_cast<std::size_t>(i)] = reducedB[static_cast<std::size_t>(i)];
        }

        manifoldCount_ = SolverContactLimit;
    }

    std::array<LinearMath::JVector, MaxManifoldPoints> manifoldA_ {};
    std::array<LinearMath::JVector, MaxManifoldPoints> manifoldB_ {};
    int leftCount_ = 0;
    int rightCount_ = 0;
    int manifoldCount_ = 0;
};

} // namespace Jitter2::Collision
