#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Jitter2/Collision/Shapes/ICloneableShape.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Collision/Shapes/ShapeHelper.hpp>
#include <Jitter2/Collision/Shapes/VertexSupportMap.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a convex hull shape defined by a point cloud. Unlike ConvexHullShape,
// it is not necessary for the points to lie on the convex hull. For performance optimization,
// this shape should ideally be used for a small number of points (~300).
class PointCloudShape final : public RigidBodyShape, public ICloneableShape<PointCloudShape>
{
public:

    explicit PointCloudShape(const std::vector<LinearMath::JVector>& vertices)
        : supportMap_(vertices)
    {

    // Updates the shape's cached mass, inertia, and bounding box.
    // Thrown when the point cloud does not define a non-degenerate volume.

        UpdateShape();
        UpdateWorldBoundingBox();
    }

    // Initializes a new instance of the PointCloudShape class.
    // vertices: All vertices that define the convex hull.
    // Thrown when vertices is empty.
    // Thrown when vertices does not define a non-degenerate volume.
    explicit PointCloudShape(VertexSupportMap supportMap)
        : supportMap_(std::move(supportMap))
    {
        UpdateShape();
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] PointCloudShape Clone() const override
    {
        PointCloudShape result(CloneTag {}, supportMap_);
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

    // Recalculates the mass, center of mass, and inertia tensor.
    // Thrown when the point cloud does not define a non-degenerate volume.

        CalculateMassInertia();
        CalculateInitialBox();
    }

    void CalculateMassInertia()
    {
        ShapeHelper::CalculateMassInertia(*this, cachedInertia_, cachedCenter_, cachedMass_);

        if (!std::isfinite(cachedMass_) || cachedMass_ <= static_cast<Real>(1e-12))
        {
            throw std::logic_error("Point cloud must define a non-degenerate volume.");
        }
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        supportMap_.SupportMap(direction, result);
        result += shift_;
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

    PointCloudShape(CloneTag, VertexSupportMap supportMap)
        : supportMap_(std::move(supportMap))
    {
    }

    void CalculateInitialBox()
    {
        LinearMath::JVector result;
        SupportMap(LinearMath::JVector::UnitX(), result);
        cachedBoundingBox_.Max.X = result.X;
        SupportMap(LinearMath::JVector::UnitY(), result);
        cachedBoundingBox_.Max.Y = result.Y;
        SupportMap(LinearMath::JVector::UnitZ(), result);
        cachedBoundingBox_.Max.Z = result.Z;

        SupportMap(-LinearMath::JVector::UnitX(), result);
        cachedBoundingBox_.Min.X = result.X;
        SupportMap(-LinearMath::JVector::UnitY(), result);
        cachedBoundingBox_.Min.Y = result.Y;
        SupportMap(-LinearMath::JVector::UnitZ(), result);
        cachedBoundingBox_.Min.Z = result.Z;
    }

    VertexSupportMap supportMap_;
    LinearMath::JVector shift_ = LinearMath::JVector::Zero();
    LinearMath::JBoundingBox cachedBoundingBox_;
    LinearMath::JMatrix cachedInertia_ = LinearMath::JMatrix::Identity();
    LinearMath::JVector cachedCenter_ = LinearMath::JVector::Zero();
    Real cachedMass_ = 0;
};

} // namespace Jitter2::Collision::Shapes
