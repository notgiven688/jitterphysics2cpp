#pragma once

#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a sphere.
class SphereShape final : public RigidBodyShape
{
public:
    explicit SphereShape(Real radius = static_cast<Real>(1))
        : radius_(Positive(radius, "radius"))
    {
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] Real Radius() const { return radius_; }

    void Radius(Real value)
    {
        radius_ = Positive(value, "Radius");
        UpdateWorldBoundingBox();
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result = LinearMath::JVector::Normalize(direction) * radius_;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    void CalculateBoundingBox(
        const LinearMath::JQuaternion&,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const override
    {
        box.Min = position + LinearMath::JVector(-radius_);
        box.Max = position + LinearMath::JVector(+radius_);
    }

    bool LocalRayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        normal = LinearMath::JVector::Zero();
        lambda = 0;

        const Real disq = static_cast<Real>(1) / direction.LengthSquared();
        const Real p = LinearMath::JVector::Dot(direction, origin) * disq;
        const Real d = p * p - (origin.LengthSquared() - radius_ * radius_) * disq;
        if (d < static_cast<Real>(0))
        {
            return false;
        }

        const Real sqrtd = std::sqrt(d);
        const Real t0 = -p - sqrtd;
        const Real t1 = -p + sqrtd;

        if (t0 >= static_cast<Real>(0))
        {
            lambda = t0;
            normal = LinearMath::JVector::Normalize(origin + direction * t0);
            return true;
        }

        return t1 > static_cast<Real>(0);
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        mass = static_cast<Real>(4.0 / 3.0) * pi * radius_ * radius_ * radius_;

        inertia = LinearMath::JMatrix::Identity();
        inertia.M11 = static_cast<Real>(2.0 / 5.0) * mass * radius_ * radius_;
        inertia.M22 = inertia.M11;
        inertia.M33 = inertia.M11;
        centerOfMass = LinearMath::JVector::Zero();
    }

private:
    Real radius_;
};

} // namespace Jitter2::Collision::Shapes
