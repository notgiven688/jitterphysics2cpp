#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <stack>
#include <unordered_set>
#include <vector>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Collision/Shapes/VertexSupportMap.hpp>
#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JTriangle.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Collision::Shapes::ShapeHelper
{

inline constexpr Real GoldenRatio = static_cast<Real>(1.6180339887498948482045L);

inline constexpr std::array<LinearMath::JVector, 12> IcosahedronVertices {{
    LinearMath::JVector(0, +1, +GoldenRatio),
    LinearMath::JVector(0, -1, +GoldenRatio),
    LinearMath::JVector(0, +1, -GoldenRatio),
    LinearMath::JVector(0, -1, -GoldenRatio),
    LinearMath::JVector(+1, +GoldenRatio, 0),
    LinearMath::JVector(+1, -GoldenRatio, 0),
    LinearMath::JVector(-1, +GoldenRatio, 0),
    LinearMath::JVector(-1, -GoldenRatio, 0),
    LinearMath::JVector(+GoldenRatio, 0, +1),
    LinearMath::JVector(+GoldenRatio, 0, -1),
    LinearMath::JVector(-GoldenRatio, 0, +1),
    LinearMath::JVector(-GoldenRatio, 0, -1),
}};

inline constexpr std::array<std::array<int, 3>, 20> IcosahedronIndices {{
    {{1, 0, 10}},
    {{0, 1, 8}},
    {{0, 4, 6}},
    {{4, 0, 8}},
    {{0, 6, 10}},
    {{5, 1, 7}},
    {{1, 5, 8}},
    {{7, 1, 10}},
    {{2, 3, 11}},
    {{3, 2, 9}},
    {{4, 2, 6}},
    {{2, 4, 9}},
    {{6, 2, 11}},
    {{3, 5, 7}},
    {{5, 3, 9}},
    {{3, 7, 11}},
    {{4, 8, 9}},
    {{8, 5, 9}},
    {{10, 6, 11}},
    {{7, 10, 11}},
}};

struct JVectorHash
{
    std::size_t operator()(const LinearMath::JVector& value) const
    {
        const std::size_t hx = std::hash<Real> {}(value.X);
        const std::size_t hy = std::hash<Real> {}(value.Y);
        const std::size_t hz = std::hash<Real> {}(value.Z);
        std::size_t seed = hx;
        seed ^= hy + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        seed ^= hz + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

template<typename TSupport, typename TCollection>
void Subdivide(
    const TSupport& support,
    TCollection& hullCollection,
    LinearMath::JVector v1,
    LinearMath::JVector v2,
    LinearMath::JVector v3,
    LinearMath::JVector p1,
    LinearMath::JVector p2,
    LinearMath::JVector p3,
    int subdivisions)
{
    if (subdivisions <= 1)
    {
        const LinearMath::JVector normal = LinearMath::JVector::Cross(p3 - p1, p2 - p1);
        if (normal.LengthSquared() > static_cast<Real>(1e-16))
        {
            hullCollection.push_back(LinearMath::JTriangle(p1, p2, p3));
        }

        return;
    }

    const LinearMath::JVector h1 = (v1 + v2) * static_cast<Real>(0.5);
    const LinearMath::JVector h2 = (v2 + v3) * static_cast<Real>(0.5);
    const LinearMath::JVector h3 = (v3 + v1) * static_cast<Real>(0.5);

    LinearMath::JVector sp1;
    LinearMath::JVector sp2;
    LinearMath::JVector sp3;
    support.SupportMap(h1, sp1);
    support.SupportMap(h2, sp2);
    support.SupportMap(h3, sp3);

    subdivisions -= 1;

    Subdivide(support, hullCollection, v1, h1, h3, p1, sp1, sp3, subdivisions);
    Subdivide(support, hullCollection, h1, v2, h2, sp1, p2, sp2, subdivisions);
    Subdivide(support, hullCollection, h3, h2, v3, sp3, sp2, p3, subdivisions);
    Subdivide(support, hullCollection, h2, h3, h1, sp2, sp3, sp1, subdivisions);
}

// hullCollection: A collection to which the triangles are added.
template<typename TSupport, typename TCollection>
void Tessellate(const TSupport& support, TCollection& hullCollection, int subdivisions = 3)
{
    for (int i = 0; i < 20; ++i)
    {
        const LinearMath::JVector v1 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][0])];
        const LinearMath::JVector v2 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][1])];
        const LinearMath::JVector v3 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][2])];

        LinearMath::JVector sv1;
        LinearMath::JVector sv2;
        LinearMath::JVector sv3;
        support.SupportMap(v1, sv1);
        support.SupportMap(v2, sv2);
        support.SupportMap(v3, sv3);

        Subdivide(support, hullCollection, v1, v2, v3, sv1, sv2, sv3, subdivisions);
    }
}

// Creates a tessellation of a shape defined by its support map.
// support: The support map interface implemented by the shape.
// subdivisions: The number of subdivisions used for hull generation.
// The tessellated hull may not be perfectly convex. It is therefore not suited to be used with
// ConvexHullShape. The time complexity is O(4^n), where n is the number of subdivisions.
template<typename TSupport>
std::vector<LinearMath::JTriangle> Tessellate(const TSupport& support, int subdivisions = 3)
{
    std::vector<LinearMath::JTriangle> triangles;

    // Creates a tessellation of a shape defined by its support map and appends all generated triangles to the specified sink.
    // TSupport: The support shape type.
    // TSink: The sink type receiving the generated triangles.
    // support: The support map interface implemented by the shape.
    // hullSink: The sink receiving the generated triangles.
    // subdivisions: The number of subdivisions used for hull generation.
    // The tessellated hull may not be perfectly convex. It is therefore not suited to be used with
    // ConvexHullShape. The time complexity is O(4^n), where n is the number of subdivisions.

    Tessellate(support, triangles, subdivisions);
    return triangles;
}

inline std::vector<LinearMath::JTriangle> Tessellate(
    const std::vector<LinearMath::JVector>& vertices,
    int subdivisions = 3)
{
    return Tessellate(VertexSupportMap(vertices), subdivisions);
}

// Approximates the convex hull of a given set of 3D vertices by sampling support points
// generated through recursive subdivision of an icosahedron.
// vertices: The vertices used to approximate the hull.
// subdivisions: The number of recursive subdivisions applied to each icosahedron triangle.
// Higher values produce more sampling directions and better coverage,
// but increase computation. Default is 3.
// Returns: A list of JVector points on the convex hull, sampled using directional support mapping from a
// refined spherical distribution.
// This method begins with a regular icosahedron and recursively subdivides each triangular face into smaller
// triangles, projecting new vertices onto the unit sphere. Each final vertex direction is passed to the support
// mapper to generate a hull point. The time complexity is O(4^n), where n is the number of subdivisions.
// Thrown when vertices is empty.
template<typename TSupport>
std::vector<LinearMath::JVector> SampleHull(const TSupport& support, int subdivisions = 3)
{
    std::stack<std::pair<LinearMath::JTriangle, int>> stack;

    for (int i = 0; i < 20; ++i)
    {
        const LinearMath::JVector v1 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][0])];
        const LinearMath::JVector v2 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][1])];
        const LinearMath::JVector v3 =
            IcosahedronVertices[static_cast<std::size_t>(IcosahedronIndices[static_cast<std::size_t>(i)][2])];

        stack.emplace(LinearMath::JTriangle(v1, v2, v3), subdivisions);
    }

    std::unordered_set<LinearMath::JVector, JVectorHash> hull;

    while (!stack.empty())
    {
        auto [triangle, depth] = stack.top();
        stack.pop();

        if (depth <= 1)
        {
            LinearMath::JVector sv0;
            LinearMath::JVector sv1;
            LinearMath::JVector sv2;
            support.SupportMap(triangle.V0, sv0);
            support.SupportMap(triangle.V1, sv1);
            support.SupportMap(triangle.V2, sv2);

            hull.insert(sv0);
            hull.insert(sv1);
            hull.insert(sv2);
            continue;
        }

        const LinearMath::JVector ab =
            LinearMath::JVector::Normalize((triangle.V0 + triangle.V1) * static_cast<Real>(0.5));
        const LinearMath::JVector bc =
            LinearMath::JVector::Normalize((triangle.V1 + triangle.V2) * static_cast<Real>(0.5));
        const LinearMath::JVector ca =
            LinearMath::JVector::Normalize((triangle.V2 + triangle.V0) * static_cast<Real>(0.5));

        stack.emplace(LinearMath::JTriangle(triangle.V0, ab, ca), depth - 1);
        stack.emplace(LinearMath::JTriangle(ab, triangle.V1, bc), depth - 1);
        stack.emplace(LinearMath::JTriangle(ca, bc, triangle.V2), depth - 1);
        stack.emplace(LinearMath::JTriangle(ab, bc, ca), depth - 1);
    }

    return std::vector<LinearMath::JVector>(hull.begin(), hull.end());
}

inline std::vector<LinearMath::JVector> SampleHull(
    const std::vector<LinearMath::JVector>& vertices,
    int subdivisions = 3)
{
    return SampleHull(VertexSupportMap(vertices), subdivisions);
}

// Calculates the axis-aligned bounding box of a shape given its orientation and position.
// support: The support map interface implemented by the shape.
// orientation: The orientation of the shape.
// position: The position of the shape.
// box: The resulting bounding box.
template<typename TSupport>
void CalculateBoundingBox(
    const TSupport& support,
    const LinearMath::JQuaternion& orientation,
    const LinearMath::JVector& position,
    LinearMath::JBoundingBox& box)
{
    const LinearMath::JMatrix orientationT =
        LinearMath::JMatrix::Transpose(LinearMath::JMatrix::CreateFromQuaternion(orientation));

    const LinearMath::JVector axisX = orientationT.GetColumn(0);
    const LinearMath::JVector axisY = orientationT.GetColumn(1);
    const LinearMath::JVector axisZ = orientationT.GetColumn(2);

    LinearMath::JVector supportPoint;
    support.SupportMap(axisX, supportPoint);
    box.Max.X = LinearMath::JVector::Dot(axisX, supportPoint);

    support.SupportMap(axisY, supportPoint);
    box.Max.Y = LinearMath::JVector::Dot(axisY, supportPoint);

    support.SupportMap(axisZ, supportPoint);
    box.Max.Z = LinearMath::JVector::Dot(axisZ, supportPoint);

    support.SupportMap(-axisX, supportPoint);
    box.Min.X = LinearMath::JVector::Dot(axisX, supportPoint);

    support.SupportMap(-axisY, supportPoint);
    box.Min.Y = LinearMath::JVector::Dot(axisY, supportPoint);

    support.SupportMap(-axisZ, supportPoint);
    box.Min.Z = LinearMath::JVector::Dot(axisZ, supportPoint);

    box.Min += position;
    box.Max += position;
}

// Calculates the mass properties of an implicitly defined shape, assuming unit mass density.
// The shape is approximated via surface tessellation using the specified number of subdivisions.
// Note on Reference Frame: The calculated inertia tensor is expressed relative to the coordinate system origin (0,0,0),
// not the calculated centerOfMass.
// support: The support map interface implemented by the shape.
// inertia: Output parameter for the inertia tensor calculated relative to the Origin (0,0,0).
// centerOfMass: Output parameter for the calculated center of mass vector (relative to the Origin).
// mass: Output parameter for the calculated mass (Volume * density 1.0).
// subdivisions: The recursion depth for the surface tessellation (default 4).
template<typename TSupport>
void CalculateMassInertia(
    const TSupport& support,
    LinearMath::JMatrix& inertia,
    LinearMath::JVector& centerOfMass,
    Real& mass,
    int subdivisions = 4)
{
    centerOfMass = LinearMath::JVector::Zero();
    inertia = LinearMath::JMatrix::Zero();
    mass = static_cast<Real>(0);

    constexpr Real a = static_cast<Real>(1.0 / 60.0);
    constexpr Real b = static_cast<Real>(1.0 / 120.0);
    const LinearMath::JMatrix canonicalInertia(a, b, b, b, a, b, b, b, a);

    for (const LinearMath::JTriangle& triangle : Tessellate(support, subdivisions))
    {
        const LinearMath::JMatrix transformation =
            LinearMath::JMatrix::FromColumns(triangle.V0, triangle.V1, triangle.V2);
        const Real determinant = transformation.Determinant();

        const LinearMath::JMatrix tetrahedronInertia =
            (transformation * canonicalInertia * LinearMath::JMatrix::Transpose(transformation)) * determinant;
        const LinearMath::JVector tetrahedronCenter =
            static_cast<Real>(0.25) * (triangle.V0 + triangle.V1 + triangle.V2);
        const Real tetrahedronMass = static_cast<Real>(1.0 / 6.0) * determinant;

        inertia = inertia + tetrahedronInertia;
        centerOfMass += tetrahedronMass * tetrahedronCenter;
        mass += tetrahedronMass;
    }

    inertia = LinearMath::JMatrix::Identity() * inertia.Trace() - inertia;
    centerOfMass *= static_cast<Real>(1) / mass;
}

} // namespace Jitter2::Collision::Shapes::ShapeHelper
