#pragma once

#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a cylinder shape defined by a height and radius.
class CylinderShape final : public RigidBodyShape
{
public:

    // Initializes a new instance of the CylinderShape class, creating a cylinder shape with the specified
    // height and radius. The symmetry axis of the cylinder is aligned along the y-axis.
    // height: The height of the cylinder.
    // radius: The radius of the cylinder at its base.
    // Thrown when height or radius is less than or equal to zero.

    CylinderShape(Real height, Real radius)
        : radius_(Positive(radius, "radius")),
          height_(Positive(height, "height"))
    {
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] Real Radius() const { return radius_; }
    [[nodiscard]] Real Height() const { return height_; }

    void Radius(Real value)
    {
        radius_ = Positive(value, "Radius");
        UpdateWorldBoundingBox();
    }

    void Height(Real value)
    {
        height_ = Positive(value, "Height");
        UpdateWorldBoundingBox();
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        const Real sigma = std::sqrt(direction.X * direction.X + direction.Z * direction.Z);
        if (sigma > static_cast<Real>(0))
        {
            result.X = direction.X / sigma * radius_;
            result.Z = direction.Z / sigma * radius_;
        }
        else
        {
            result.X = static_cast<Real>(0);
            result.Z = static_cast<Real>(0);
        }

        result.Y = (direction.Y > static_cast<Real>(0)
            ? static_cast<Real>(1)
            : (direction.Y < static_cast<Real>(0) ? static_cast<Real>(-1) : static_cast<Real>(0)))
            * height_ * static_cast<Real>(0.5);
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    void CalculateBoundingBox(
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const override
    {
        const LinearMath::JVector up = orientation.GetBasisY();

        const Real xx = up.X * up.X;
        const Real yy = up.Y * up.Y;
        const Real zz = up.Z * up.Z;

        const Real xExt = std::sqrt(yy + zz) * radius_;
        const Real yExt = std::sqrt(xx + zz) * radius_;
        const Real zExt = std::sqrt(xx + yy) * radius_;

        const LinearMath::JVector p1 = -static_cast<Real>(0.5) * height_ * up;
        const LinearMath::JVector p2 = +static_cast<Real>(0.5) * height_ * up;
        const LinearMath::JVector delta = LinearMath::JVector::Max(p1, p2)
            + LinearMath::JVector(xExt, yExt, zExt);

        box.Min = position - delta;
        box.Max = position + delta;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        mass = pi * radius_ * radius_ * height_;

        inertia = LinearMath::JMatrix::Identity();
        inertia.M11 = static_cast<Real>(1.0 / 4.0) * mass * radius_ * radius_
            + static_cast<Real>(1.0 / 12.0) * mass * height_ * height_;
        inertia.M22 = static_cast<Real>(1.0 / 2.0) * mass * radius_ * radius_;
        inertia.M33 = inertia.M11;
        centerOfMass = LinearMath::JVector::Zero();
    }

private:
    Real radius_;
    Real height_;
};

} // namespace Jitter2::Collision::Shapes
