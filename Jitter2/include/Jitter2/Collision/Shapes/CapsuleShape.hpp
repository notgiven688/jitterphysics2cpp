#pragma once

#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a capsule shape defined by a radius and the length of its cylindrical section.
class CapsuleShape final : public RigidBodyShape
{
public:
    CapsuleShape(Real radius = static_cast<Real>(0.5), Real length = static_cast<Real>(1))
        : radius_(Positive(radius, "radius")),
          halfLength_(static_cast<Real>(0.5) * NonNegative(length, "length"))
    {
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] Real Radius() const { return radius_; }
    [[nodiscard]] Real Length() const { return static_cast<Real>(2) * halfLength_; }

    void Radius(Real value)
    {
        radius_ = Positive(value, "Radius");
        UpdateWorldBoundingBox();
    }

    void Length(Real value)
    {
        halfLength_ = static_cast<Real>(0.5) * NonNegative(value, "Length");
        UpdateWorldBoundingBox();
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result = LinearMath::JVector::Normalize(direction) * radius_;
        result.Y += (direction.Y > static_cast<Real>(0)
            ? static_cast<Real>(1)
            : (direction.Y < static_cast<Real>(0) ? static_cast<Real>(-1) : static_cast<Real>(0)))
            * halfLength_;
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
        const LinearMath::JVector delta = orientation.GetBasisY() * halfLength_;
        box.Max.X = +radius_ + std::abs(delta.X);
        box.Max.Y = +radius_ + std::abs(delta.Y);
        box.Max.Z = +radius_ + std::abs(delta.Z);
        box.Min = -box.Max;
        box.Min += position;
        box.Max += position;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        const Real length = static_cast<Real>(2) * halfLength_;
        const Real massSphere = static_cast<Real>(4.0 / 3.0) * pi * radius_ * radius_ * radius_;
        const Real massCylinder = pi * radius_ * radius_ * length;

        inertia = LinearMath::JMatrix::Identity();
        inertia.M11 = massCylinder * (static_cast<Real>(1.0 / 12.0) * length * length
            + static_cast<Real>(1.0 / 4.0) * radius_ * radius_)
            + massSphere * (static_cast<Real>(2.0 / 5.0) * radius_ * radius_
            + static_cast<Real>(1.0 / 4.0) * length * length
            + static_cast<Real>(3.0 / 8.0) * length * radius_);
        inertia.M22 = static_cast<Real>(1.0 / 2.0) * massCylinder * radius_ * radius_
            + static_cast<Real>(2.0 / 5.0) * massSphere * radius_ * radius_;
        inertia.M33 = inertia.M11;

        mass = massCylinder + massSphere;
        centerOfMass = LinearMath::JVector::Zero();
    }

private:
    Real radius_;
    Real halfLength_;
};

} // namespace Jitter2::Collision::Shapes
