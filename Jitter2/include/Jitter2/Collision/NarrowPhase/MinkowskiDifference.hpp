#pragma once

#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Collision::MinkowskiDifference
{

// Represents a vertex on the Minkowski difference of two shapes.
struct Vertex
{
    LinearMath::JVector V = LinearMath::JVector::Zero();
    LinearMath::JVector A = LinearMath::JVector::Zero();
    LinearMath::JVector B = LinearMath::JVector::Zero();

    Vertex() = default;

    // Creates a vertex with only the difference point set.
    // v: The Minkowski difference point.
    explicit Vertex(const LinearMath::JVector& v)
        : V(v)
    {
    }
};

// Computes the support function S_{A-B}(d) = S_A(d) - S_B(-d) for the Minkowski difference.
// Ta: The type of support shape A.
// Tb: The type of support shape B.
// supportA: The support function of shape A (at origin, not rotated).
// supportB: The support function of shape B.
// orientationB: The orientation of shape B.
// positionB: The position of shape B.
// direction: The search direction.
// v: The resulting vertex containing support points from both shapes.
template<typename TSupportA, typename TSupportB>
void Support(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    const LinearMath::JVector& direction,
    Vertex& vertex)
{
    supportA.SupportMap(direction, vertex.A);

    const LinearMath::JVector inverseDirection =
        LinearMath::JQuaternion::ConjugatedTransform(-direction, orientationB);
    supportB.SupportMap(inverseDirection, vertex.B);
    vertex.B = LinearMath::JQuaternion::Transform(vertex.B, orientationB) + positionB;
    // The support point on shape B in world space.
    vertex.V = vertex.A - vertex.B;
}

// Computes a point guaranteed to be inside the Minkowski difference.
// Ta: The type of support shape A.
// Tb: The type of support shape B.
// supportA: The support function of shape A (at origin, not rotated).
// supportB: The support function of shape B.
// orientationB: The orientation of shape B.
// positionB: The position of shape B.
// center: The resulting vertex representing the center of the Minkowski difference.
template<typename TSupportA, typename TSupportB>
void GetCenter(
    const TSupportA& supportA,
    const TSupportB& supportB,
    const LinearMath::JQuaternion& orientationB,
    const LinearMath::JVector& positionB,
    Vertex& center)
{
    supportA.GetCenter(center.A);
    supportB.GetCenter(center.B);
    center.B = LinearMath::JQuaternion::Transform(center.B, orientationB) + positionB;
    center.V = center.A - center.B;
}

} // namespace Jitter2::Collision::MinkowskiDifference
