#pragma once

#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a cone shape defined by a base radius and height.
class ConeShape final : public RigidBodyShape
{
public:
    ConeShape(Real radius = static_cast<Real>(0.5), Real height = static_cast<Real>(1))
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
        constexpr Real zeroEpsilon = static_cast<Real>(1e-12);
        LinearMath::JVector baseDirection(direction.X, 0, direction.Z);
        baseDirection = LinearMath::JVector::NormalizeSafe(baseDirection, zeroEpsilon) * radius_;
        baseDirection.Y = -static_cast<Real>(0.25) * height_;
        const LinearMath::JVector tip(0, static_cast<Real>(0.75) * height_, 0);
        result = LinearMath::JVector::Dot(direction, baseDirection)
                >= LinearMath::JVector::Dot(direction, tip)
            ? baseDirection
            : tip;
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

        const LinearMath::JVector p1 = -static_cast<Real>(0.25) * height_ * up;
        const LinearMath::JVector p2 = +static_cast<Real>(0.75) * height_ * up;

        box.Min = p1 - LinearMath::JVector(xExt, yExt, zExt);
        box.Max = p1 + LinearMath::JVector(xExt, yExt, zExt);
        LinearMath::JBoundingBox::AddPointInPlace(box, p2);
        box.Min += position;
        box.Max += position;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        mass = static_cast<Real>(1.0 / 3.0) * pi * radius_ * radius_ * height_;

        inertia = LinearMath::JMatrix::Identity();
        inertia.M11 = mass * (static_cast<Real>(3.0 / 20.0) * radius_ * radius_
            + static_cast<Real>(3.0 / 80.0) * height_ * height_);
        inertia.M22 = static_cast<Real>(3.0 / 10.0) * mass * radius_ * radius_;
        inertia.M33 = inertia.M11;
        centerOfMass = LinearMath::JVector::Zero();
    }

private:
    Real radius_;
    Real height_;
};

} // namespace Jitter2::Collision::Shapes
