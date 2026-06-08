#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

#include <Jitter2/Collision/NarrowPhase/MinkowskiDifference.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Collision
{

// Represents a convex polytope builder used in the Expanding Polytope Algorithm (EPA)
// for computing penetration depth and contact information.
// The polytope is iteratively expanded by adding vertices from the Minkowski difference
// until convergence. Call InitHeap at least once before use to allocate
// memory for vertices and triangles.
// Memory is allocated from the unmanaged heap and reused across EPA iterations.
class ConvexPolytope
{
public:
    struct Triangle
    {
        short A = 0;
        short B = 0;
        short C = 0;
        bool FacingOrigin = false;

        LinearMath::JVector Normal = LinearMath::JVector::Zero();
        LinearMath::JVector ClosestToOrigin = LinearMath::JVector::Zero();

        Real NormalSq = static_cast<Real>(0);
        Real ClosestToOriginSq = static_cast<Real>(0);

        short operator[](int index) const
        {
            switch (index)
            {
            case 0:
                return A;
            case 1:
                return B;
            default:
                return C;
            }
        }
    };

    static constexpr int MaxVertices = 128;
    static constexpr int MaxTriangles = 2 * MaxVertices;

    MinkowskiDifference::Vertex& GetVertex(int index)
    {
        assert(index < MaxVertices);
        return vertices_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] const MinkowskiDifference::Vertex& GetVertex(int index) const
    {
        assert(index < MaxVertices);
        return vertices_[static_cast<std::size_t>(index)];
    }

    // Indicates whether the origin is enclosed within the polyhedron.
    // Only valid after GetClosestTriangle has been called, which updates this flag.
    [[nodiscard]] bool OriginEnclosed() const
    {
        return originEnclosed_;
    }

    // Computes the closest points on shapes A and B from a triangle on the polytope.
    // ctri: The triangle from GetClosestTriangle.
    // pointA: The closest point on shape A.
    // pointB: The closest point on shape B.
    void CalculatePoints(const Triangle& ctri, LinearMath::JVector& pointA, LinearMath::JVector& pointB) const
    {
        LinearMath::JVector bc;
        CalcBarycentric(ctri, bc);
        pointA = bc.X * vertices_[static_cast<std::size_t>(ctri.A)].A
            + bc.Y * vertices_[static_cast<std::size_t>(ctri.B)].A
            + bc.Z * vertices_[static_cast<std::size_t>(ctri.C)].A;
        pointB = bc.X * vertices_[static_cast<std::size_t>(ctri.A)].B
            + bc.Y * vertices_[static_cast<std::size_t>(ctri.B)].B
            + bc.Z * vertices_[static_cast<std::size_t>(ctri.C)].B;
    }

    // Finds the triangle on the polytope closest to the origin.
    // Returns: A reference to the closest triangle. Also updates OriginEnclosed.
    const Triangle& GetClosestTriangle()
    {
        int closestIndex = -1;
        Real currentMin = std::numeric_limits<Real>::max();

        const bool skipTest = originEnclosed_;
        originEnclosed_ = true;

        for (int i = 0; i < tPointer_; ++i)
        {
            const Triangle& triangle = triangles_[static_cast<std::size_t>(i)];
            if (triangle.ClosestToOriginSq < currentMin)
            {
                currentMin = triangle.ClosestToOriginSq;
                closestIndex = i;
            }

            if (!triangle.FacingOrigin)
            {
                originEnclosed_ = skipTest;
            }
        }

        assert(closestIndex >= 0);
        return triangles_[static_cast<std::size_t>(closestIndex)];
    }

    // Initializes the polytope with a tetrahedron formed from the first four vertices.
    // The first four vertices must be set before calling this method.
    void InitTetrahedron()
    {
        originEnclosed_ = false;
        vPointer_ = 4;
        tPointer_ = 0;

        center_ = static_cast<Real>(0.25)
            * (vertices_[0].V + vertices_[1].V + vertices_[2].V + vertices_[3].V);

        CreateTriangle(0, 2, 1);
        CreateTriangle(0, 1, 3);
        CreateTriangle(0, 3, 2);
        CreateTriangle(1, 2, 3);
    }

    // Initializes the polytope with a small tetrahedron centered at the specified point.
    // point: The center point of the initial tetrahedron.
    void InitTetrahedron(const LinearMath::JVector& point)
    {
        originEnclosed_ = false;
        vPointer_ = 4;
        tPointer_ = 0;
        center_ = point;

        const Real scale = static_cast<Real>(1e-2);
        vertices_[0] = MinkowskiDifference::Vertex(center_
            + scale * LinearMath::JVector(
                std::sqrt(static_cast<Real>(8.0 / 9.0)),
                static_cast<Real>(0),
                -static_cast<Real>(1.0 / 3.0)));
        vertices_[1] = MinkowskiDifference::Vertex(center_
            + scale * LinearMath::JVector(
                -std::sqrt(static_cast<Real>(2.0 / 9.0)),
                std::sqrt(static_cast<Real>(2.0 / 3.0)),
                -static_cast<Real>(1.0 / 3.0)));
        vertices_[2] = MinkowskiDifference::Vertex(center_
            + scale * LinearMath::JVector(
                -std::sqrt(static_cast<Real>(2.0 / 9.0)),
                -std::sqrt(static_cast<Real>(2.0 / 3.0)),
                -static_cast<Real>(1.0 / 3.0)));
        vertices_[3] = MinkowskiDifference::Vertex(center_
            + scale * LinearMath::JVector(static_cast<Real>(0), static_cast<Real>(0), static_cast<Real>(1)));

        CreateTriangle(2, 0, 1);
        CreateTriangle(1, 0, 3);
        CreateTriangle(3, 0, 2);
        CreateTriangle(2, 1, 3);
    }

    // Allocates unmanaged memory for vertices and triangles.
    // Must be called before any other method. Safe to call multiple times; allocation occurs only once.
    void InitHeap()
    {
    // The C# implementation allocates unmanaged scratch memory lazily. The C++ port uses fixed
    // in-object storage, so this method intentionally only preserves the public lifecycle call.
    }

    // Adds a vertex to the polytope and rebuilds the convex hull.
    // vertex: The Minkowski difference vertex to add.
    // Returns: true if the vertex was incorporated; false if the polytope could not expand.
    // This operation invalidates references from previous GetClosestTriangle calls.
    bool AddVertex(const MinkowskiDifference::Vertex& vertex)
    {
        assert(vPointer_ < MaxVertices);

        std::array<Edge, MaxVertices * 3 / 2> edges {};
        vertices_[static_cast<std::size_t>(vPointer_)] = vertex;

        int ePointer = 0;
        for (int index = tPointer_; index-- > 0;)
        {
            if (!IsLit(index, vPointer_))
            {
                continue;
            }

            for (int k = 0; k < 3; ++k)
            {
                Edge edge(
                    triangles_[static_cast<std::size_t>(index)][(k + 0) % 3],
                    triangles_[static_cast<std::size_t>(index)][(k + 1) % 3]);
                bool added = true;
                for (int e = ePointer; e-- > 0;)
                {
                    if (Edge::Equals(edges[static_cast<std::size_t>(e)], edge))
                    {
                        edges[static_cast<std::size_t>(e)] = edges[static_cast<std::size_t>(--ePointer)];
                        added = false;
                    }
                }

                if (added)
                {
                    edges[static_cast<std::size_t>(ePointer++)] = edge;
                }
            }

            triangles_[static_cast<std::size_t>(index)] = triangles_[static_cast<std::size_t>(--tPointer_)];
        }

        if (ePointer == 0)
        {
            return false;
        }

        for (int i = 0; i < ePointer; ++i)
        {
            if (!CreateTriangle(edges[static_cast<std::size_t>(i)].A, edges[static_cast<std::size_t>(i)].B, vPointer_))
            {
                return false;
            }
        }

        ++vPointer_;
        return true;
    }

private:
    struct Edge
    {
        short A = 0;
        short B = 0;

        Edge() = default;
        Edge(short a, short b) : A(a), B(b) {}

        static bool Equals(const Edge& a, const Edge& b)
        {
            return (a.A == b.A && a.B == b.B) || (a.A == b.B && a.B == b.A);
        }
    };

    static constexpr Real NumericEpsilon = static_cast<Real>(1e-16);

    bool CalcBarycentric(const Triangle& tri, LinearMath::JVector& result) const
    {
        bool clamped = false;

        const LinearMath::JVector a = vertices_[static_cast<std::size_t>(tri.A)].V;
        const LinearMath::JVector b = vertices_[static_cast<std::size_t>(tri.B)].V;
        const LinearMath::JVector c = vertices_[static_cast<std::size_t>(tri.C)].V;

        const LinearMath::JVector u = a - b;
        const LinearMath::JVector v = a - c;

        Real t = static_cast<Real>(1) / tri.NormalSq;

        LinearMath::JVector tmp = LinearMath::JVector::Cross(u, a);
        Real gamma = LinearMath::JVector::Dot(tmp, tri.Normal) * t;
        tmp = LinearMath::JVector::Cross(a, v);
        Real beta = LinearMath::JVector::Dot(tmp, tri.Normal) * t;
        Real alpha = static_cast<Real>(1) - gamma - beta;

        if (alpha >= static_cast<Real>(0) && beta < static_cast<Real>(0))
        {
            t = LinearMath::JVector::Dot(a, u);
            if (gamma < static_cast<Real>(0) && t > static_cast<Real>(0))
            {
                beta = std::min(static_cast<Real>(1), t / u.LengthSquared());
                alpha = static_cast<Real>(1) - beta;
                gamma = static_cast<Real>(0);
            }
            else
            {
                gamma = std::min(
                    static_cast<Real>(1),
                    std::max(static_cast<Real>(0), LinearMath::JVector::Dot(a, v) / v.LengthSquared()));
                alpha = static_cast<Real>(1) - gamma;
                beta = static_cast<Real>(0);
            }

            clamped = true;
        }
        else if (beta >= static_cast<Real>(0) && gamma < static_cast<Real>(0))
        {
            const LinearMath::JVector w = b - c;
            t = LinearMath::JVector::Dot(b, w);
            if (alpha < static_cast<Real>(0) && t > static_cast<Real>(0))
            {
                gamma = std::min(static_cast<Real>(1), t / w.LengthSquared());
                beta = static_cast<Real>(1) - gamma;
                alpha = static_cast<Real>(0);
            }
            else
            {
                alpha = std::min(
                    static_cast<Real>(1),
                    std::max(static_cast<Real>(0), -LinearMath::JVector::Dot(b, u) / u.LengthSquared()));
                beta = static_cast<Real>(1) - alpha;
                gamma = static_cast<Real>(0);
            }

            clamped = true;
        }
        else if (gamma >= static_cast<Real>(0) && alpha < static_cast<Real>(0))
        {
            const LinearMath::JVector w = b - c;
            t = -LinearMath::JVector::Dot(c, v);
            if (beta < static_cast<Real>(0) && t > static_cast<Real>(0))
            {
                alpha = std::min(static_cast<Real>(1), t / v.LengthSquared());
                gamma = static_cast<Real>(1) - alpha;
                beta = static_cast<Real>(0);
            }
            else
            {
                beta = std::min(
                    static_cast<Real>(1),
                    std::max(static_cast<Real>(0), -LinearMath::JVector::Dot(c, w) / w.LengthSquared()));
                gamma = static_cast<Real>(1) - beta;
                alpha = static_cast<Real>(0);
            }

            clamped = true;
        }

        result.X = alpha;
        result.Y = beta;
        result.Z = gamma;
        return clamped;
    }

    bool IsLit(int candidate, int w) const
    {
        const Triangle& triangle = triangles_[static_cast<std::size_t>(candidate)];
        const LinearMath::JVector deltaA =
            vertices_[static_cast<std::size_t>(w)].V - vertices_[static_cast<std::size_t>(triangle.A)].V;
        return LinearMath::JVector::Dot(deltaA, triangle.Normal) > static_cast<Real>(0);
    }

    bool CreateTriangle(short a, short b, short c)
    {
        Triangle& triangle = triangles_[static_cast<std::size_t>(tPointer_)];
        triangle.A = a;
        triangle.B = b;
        triangle.C = c;

        const LinearMath::JVector u =
            vertices_[static_cast<std::size_t>(a)].V - vertices_[static_cast<std::size_t>(b)].V;
        const LinearMath::JVector v =
            vertices_[static_cast<std::size_t>(a)].V - vertices_[static_cast<std::size_t>(c)].V;
        triangle.Normal = LinearMath::JVector::Cross(u, v);
        triangle.NormalSq = triangle.Normal.LengthSquared();

        if (triangle.NormalSq < NumericEpsilon)
        {
            return false;
        }

        Real delta = LinearMath::JVector::Dot(triangle.Normal, vertices_[static_cast<std::size_t>(a)].V - center_);

        if (delta < static_cast<Real>(0))
        {
            std::swap(triangle.A, triangle.B);
            triangle.Normal = -triangle.Normal;
        }

        delta = LinearMath::JVector::Dot(triangle.Normal, vertices_[static_cast<std::size_t>(a)].V);
        triangle.FacingOrigin = delta >= static_cast<Real>(0);

        LinearMath::JVector bc;
        if (CalcBarycentric(triangle, bc))
        {
            triangle.ClosestToOrigin = bc.X * vertices_[static_cast<std::size_t>(triangle.A)].V
                + bc.Y * vertices_[static_cast<std::size_t>(triangle.B)].V
                + bc.Z * vertices_[static_cast<std::size_t>(triangle.C)].V;
            triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
        }
        else
        {
            triangle.ClosestToOrigin = triangle.Normal * (delta / triangle.NormalSq);
            triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
        }

        ++tPointer_;
        return true;
    }

    std::array<Triangle, MaxTriangles> triangles_ {};
    std::array<MinkowskiDifference::Vertex, MaxVertices> vertices_ {};

    short tPointer_ = 0;
    short vPointer_ = 0;

    bool originEnclosed_ = false;
    LinearMath::JVector center_ = LinearMath::JVector::Zero();
};

} // namespace Jitter2::Collision
