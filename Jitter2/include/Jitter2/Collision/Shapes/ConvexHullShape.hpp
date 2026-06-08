#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Jitter2/Collision/Shapes/ICloneableShape.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/LinearMath/JTriangle.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a convex hull shape defined by a set of triangles.
class ConvexHullShape final : public RigidBodyShape, public ICloneableShape<ConvexHullShape>
{
public:

    // Initializes a new instance of the ConvexHullShape class, creating a convex hull.
    // triangles: All vertices defining the convex hull. The vertices must strictly lie
    // on the surface of the convex hull to avoid incorrect results or indefinite hangs in the collision algorithm.
    // Thrown when triangles is empty, or when the convex hull consists of more than
    // ushort.MaxValue vertices.
    // Thrown when triangles does not define a non-degenerate volume.
    explicit ConvexHullShape(const std::vector<LinearMath::JTriangle>& triangles)
    {
        if (triangles.empty())
        {
            throw std::invalid_argument("Triangle set must contain at least one triangle.");
        }

        Build(triangles);

    // Updates the shape's cached mass, inertia, and bounding box.
    // Thrown when the convex hull does not define a non-degenerate volume.

        UpdateShape();
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] ConvexHullShape Clone() const override
    {
        ConvexHullShape result(CloneTag {});
        result.data_ = data_;
        result.shift_ = shift_;
        result.cachedBoundingBox_ = cachedBoundingBox_;
        result.cachedInertia_ = cachedInertia_;
        result.cachedCenter_ = cachedCenter_;
        result.cachedMass_ = cachedMass_;
        result.UpdateWorldBoundingBox();
        return result;
    }

    [[nodiscard]] LinearMath::JVector Shift() const { return shift_; }

    void Shift(const LinearMath::JVector& value)
    {
        shift_ = value;
        UpdateShape();
        UpdateWorldBoundingBox();
    }

    void UpdateShape()
    {

    // Recalculates the mass, center of mass, and inertia tensor from the convex hull triangles.
    // Thrown when the convex hull does not define a non-degenerate volume.

        CalculateMassInertia();
        CalculateInitialBox();
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        InternalSupportMap(direction, result);
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        point = cachedCenter_;
    }

    void CalculateBoundingBox(
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const override
    {
        const LinearMath::JVector halfSize = static_cast<Real>(0.5) * (cachedBoundingBox_.Max - cachedBoundingBox_.Min);
        const LinearMath::JVector center = static_cast<Real>(0.5) * (cachedBoundingBox_.Max + cachedBoundingBox_.Min);

        const LinearMath::JMatrix orientationMatrix = LinearMath::JMatrix::CreateFromQuaternion(orientation);
        const LinearMath::JVector transformedHalfSize =
            LinearMath::JMatrix::Absolute(orientationMatrix) * halfSize;
        const LinearMath::JVector transformedCenter =
            LinearMath::JQuaternion::Transform(center, orientation);

        box.Min = position + transformedCenter - transformedHalfSize;
        box.Max = position + transformedCenter + transformedHalfSize;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        inertia = cachedInertia_;
        centerOfMass = cachedCenter_;
        mass = cachedMass_;
    }

private:
    struct CloneTag
    {
    };

    struct HullVector
    {
        LinearMath::JVector Vertex;
        std::uint16_t NeighborMinIndex = 0;
        std::uint16_t NeighborMaxIndex = 0;
    };

    struct HullTriangle
    {
        std::uint16_t A = 0;
        std::uint16_t B = 0;
        std::uint16_t C = 0;
    };

    struct HullData
    {
        std::vector<HullVector> Vertices;
        std::vector<HullTriangle> Indices;
        std::vector<std::uint16_t> NeighborList;
    };


    explicit ConvexHullShape(CloneTag)
    {
    }

    static std::uint16_t FindOrAddVertex(
        std::vector<HullVector>& vertices,
        const LinearMath::JVector& vertex)
    {
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            if (vertices[index].Vertex == vertex)
            {
                return static_cast<std::uint16_t>(index);
            }
        }

        if (vertices.size() >= std::numeric_limits<std::uint16_t>::max())
        {
            throw std::invalid_argument("The convex hull consists of too many vertices (>65535).");
        }

        vertices.push_back(HullVector {vertex});
        return static_cast<std::uint16_t>(vertices.size() - 1);
    }

    // Helper to sort and add unique elements from source to destination.
    // Replaces LINQ Distinct() for better performance and zero allocations.
    static void AddDistinct(std::vector<std::uint16_t>& source, std::vector<std::uint16_t>& destination)
    {
        if (source.empty())
        {
            return;
        }

        std::sort(source.begin(), source.end());

        std::uint16_t previous = source[0];
        destination.push_back(previous);

        for (std::size_t index = 1; index < source.size(); ++index)
        {
            const std::uint16_t current = source[index];
            if (current != previous)
            {
                destination.push_back(current);
                previous = current;
            }
        }
    }

    void Build(const std::vector<LinearMath::JTriangle>& triangles)
    {
        auto data = std::make_shared<HullData>();

        data->Vertices.reserve(triangles.size() * 3);
        data->Indices.resize(triangles.size());

        for (const LinearMath::JTriangle& triangle : triangles)
        {
            FindOrAddVertex(data->Vertices, triangle.V0);
            FindOrAddVertex(data->Vertices, triangle.V1);
            FindOrAddVertex(data->Vertices, triangle.V2);
        }

        std::vector<std::vector<std::uint16_t>> tmpNeighbors(data->Vertices.size());

        for (std::size_t index = 0; index < triangles.size(); ++index)
        {
            const LinearMath::JTriangle& triangle = triangles[index];

            const std::uint16_t a = FindOrAddVertex(data->Vertices, triangle.V0);
            const std::uint16_t b = FindOrAddVertex(data->Vertices, triangle.V1);
            const std::uint16_t c = FindOrAddVertex(data->Vertices, triangle.V2);

            data->Indices[index] = HullTriangle {a, b, c};

            tmpNeighbors[a].push_back(b);
            tmpNeighbors[a].push_back(c);

            tmpNeighbors[b].push_back(a);
            tmpNeighbors[b].push_back(c);

            tmpNeighbors[c].push_back(a);
            tmpNeighbors[c].push_back(b);
        }

        data->NeighborList.reserve(data->Vertices.size() * 6);

        for (std::size_t index = 0; index < data->Vertices.size(); ++index)
        {
            HullVector& element = data->Vertices[index];

            if (data->NeighborList.size() > std::numeric_limits<std::uint16_t>::max())
            {
                throw std::invalid_argument("The convex hull consists of too many neighbor entries (>65535).");
            }
            element.NeighborMinIndex = static_cast<std::uint16_t>(data->NeighborList.size());

            AddDistinct(tmpNeighbors[index], data->NeighborList);

            if (data->NeighborList.size() > std::numeric_limits<std::uint16_t>::max())
            {
                throw std::invalid_argument("The convex hull consists of too many neighbor entries (>65535).");
            }
            element.NeighborMaxIndex = static_cast<std::uint16_t>(data->NeighborList.size());
        }

        data_ = std::move(data);
    }

    std::uint16_t InternalSupportMap(
        const LinearMath::JVector& direction,
        LinearMath::JVector& result) const
    {
        const std::vector<HullVector>& vertices = data_->Vertices;
        const std::vector<std::uint16_t>& neighborList = data_->NeighborList;

        std::uint16_t current = 0;
        Real dotProduct = LinearMath::JVector::Dot(vertices[current].Vertex, direction);

        constexpr Real epsilonIncrement = static_cast<Real>(1e-12);

    main:
        bool needsVerify = false;
        LinearMath::JVector verifyDir = LinearMath::JVector::Arbitrary();

        std::uint16_t min = vertices[current].NeighborMinIndex;
        std::uint16_t max = vertices[current].NeighborMaxIndex;

        for (std::uint16_t index = min; index < max; ++index)
        {
            const std::uint16_t neighbor = neighborList[index];
            const Real neighborProduct = LinearMath::JVector::Dot(vertices[neighbor].Vertex, direction);

            if (std::abs(neighborProduct - dotProduct) < epsilonIncrement)
            {
                verifyDir = vertices[neighbor].Vertex - vertices[current].Vertex;
                needsVerify = true;
            }

            if (neighborProduct > dotProduct + epsilonIncrement)
            {
                dotProduct = neighborProduct;
                current = neighbor;
                goto main;
            }
        }

        if (needsVerify)
        {
            Real d0 = LinearMath::JVector::Dot(verifyDir, vertices[current].Vertex);

        secondary:
            min = vertices[current].NeighborMinIndex;
            max = vertices[current].NeighborMaxIndex;

            for (std::uint16_t index = min; index < max; ++index)
            {
                const std::uint16_t neighbor = neighborList[index];
                const Real neighborProduct = LinearMath::JVector::Dot(vertices[neighbor].Vertex, direction);

                if (neighborProduct > dotProduct + epsilonIncrement)
                {
                    dotProduct = neighborProduct;
                    current = neighbor;
                    goto main;
                }

                if (std::abs(neighborProduct - dotProduct) < epsilonIncrement)
                {
                    const Real d1 = LinearMath::JVector::Dot(verifyDir, vertices[neighbor].Vertex);

                    if (d1 > d0 + epsilonIncrement)
                    {
                        d0 = d1;
                        current = neighbor;
                        goto secondary;
                    }
                }
            }
        }

        result = vertices[current].Vertex + shift_;
        return current;
    }

    void CalculateMassInertia()
    {
        const std::vector<HullVector>& vertices = data_->Vertices;
        const std::vector<HullTriangle>& indices = data_->Indices;

        cachedCenter_ = LinearMath::JVector::Zero();
        cachedInertia_ = LinearMath::JMatrix::Zero();
        cachedMass_ = 0;

        constexpr Real a = static_cast<Real>(1.0 / 60.0);
        constexpr Real b = static_cast<Real>(1.0 / 120.0);
        const LinearMath::JMatrix canonicalInertia(a, b, b, b, a, b, b, b, a);

        LinearMath::JVector pointWithin = LinearMath::JVector::Zero();
        for (const HullVector& vertex : vertices)
        {
            pointWithin += vertex.Vertex;
        }
        pointWithin = pointWithin / static_cast<Real>(vertices.size()) + shift_;

        for (const HullTriangle& triangle : indices)
        {
            LinearMath::JVector column0 = vertices[triangle.A].Vertex + shift_;
            LinearMath::JVector column1 = vertices[triangle.B].Vertex + shift_;
            LinearMath::JVector column2 = vertices[triangle.C].Vertex + shift_;

            const LinearMath::JVector normal = LinearMath::JVector::Cross(column1 - column0, column2 - column0);
            if (LinearMath::JVector::Dot(normal, column0 - pointWithin) < static_cast<Real>(0))
            {
                std::swap(column0, column1);
            }

            const LinearMath::JMatrix transformation(
                column0.X, column1.X, column2.X,
                column0.Y, column1.Y, column2.Y,
                column0.Z, column1.Z, column2.Z);
            const Real determinant = transformation.Determinant();
            const LinearMath::JMatrix tetrahedronInertia =
                (transformation * canonicalInertia * LinearMath::JMatrix::Transpose(transformation)) * determinant;
            const LinearMath::JVector tetrahedronCenter =
                static_cast<Real>(0.25) * (column0 + column1 + column2);
            const Real tetrahedronMass = static_cast<Real>(1.0 / 6.0) * determinant;

            cachedInertia_ = cachedInertia_ + tetrahedronInertia;
            cachedCenter_ += tetrahedronMass * tetrahedronCenter;
            cachedMass_ += tetrahedronMass;
        }

        if (!std::isfinite(cachedMass_) || cachedMass_ <= static_cast<Real>(1e-12))
        {
            throw std::logic_error("Convex hull must define a non-degenerate volume.");
        }

        cachedInertia_ = LinearMath::JMatrix::Identity() * cachedInertia_.Trace() - cachedInertia_;
        cachedCenter_ /= cachedMass_;
    }

    void CalculateInitialBox()
    {
        LinearMath::JVector result;
        InternalSupportMap(LinearMath::JVector::UnitX(), result);
        cachedBoundingBox_.Max.X = result.X;
        InternalSupportMap(LinearMath::JVector::UnitY(), result);
        cachedBoundingBox_.Max.Y = result.Y;
        InternalSupportMap(LinearMath::JVector::UnitZ(), result);
        cachedBoundingBox_.Max.Z = result.Z;

        InternalSupportMap(-LinearMath::JVector::UnitX(), result);
        cachedBoundingBox_.Min.X = result.X;
        InternalSupportMap(-LinearMath::JVector::UnitY(), result);
        cachedBoundingBox_.Min.Y = result.Y;
        InternalSupportMap(-LinearMath::JVector::UnitZ(), result);
        cachedBoundingBox_.Min.Z = result.Z;
    }

    std::shared_ptr<HullData> data_;
    LinearMath::JVector shift_ = LinearMath::JVector::Zero();
    LinearMath::JBoundingBox cachedBoundingBox_;
    LinearMath::JMatrix cachedInertia_ = LinearMath::JMatrix::Identity();
    LinearMath::JVector cachedCenter_ = LinearMath::JVector::Zero();
    Real cachedMass_ = 0;
};

} // namespace Jitter2::Collision::Shapes
