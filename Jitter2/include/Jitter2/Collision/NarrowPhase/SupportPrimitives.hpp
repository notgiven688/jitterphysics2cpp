#pragma once

#include <cmath>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::Collision::SupportPrimitives
{

inline Real Sign(Real value)
{
    return value > static_cast<Real>(0)
        ? static_cast<Real>(1)
        : (value < static_cast<Real>(0) ? static_cast<Real>(-1) : static_cast<Real>(0));
}

// Represents a point as a lightweight support-mapped query primitive.
struct Point : public ISupportMappable
{
    void SupportMap(const LinearMath::JVector&, LinearMath::JVector& result) const override
    {
        result = LinearMath::JVector::Zero();
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }
};

// Represents a sphere as a lightweight support-mapped query primitive.
struct Sphere : public ISupportMappable
{
    explicit Sphere(Real radius)
        : Radius(Shapes::Positive(radius, "radius"))
    {
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result = LinearMath::JVector::Normalize(direction) * Radius;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    Real Radius;
};

// Represents a box as a lightweight support-mapped query primitive.
struct Box : public ISupportMappable
{
    explicit Box(const LinearMath::JVector& halfExtents)
        : HalfExtents(Shapes::PositiveComponents(halfExtents, "halfExtents"))
    {
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result.X = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.X)) * HalfExtents.X;
        result.Y = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.Y)) * HalfExtents.Y;
        result.Z = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.Z)) * HalfExtents.Z;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    LinearMath::JVector HalfExtents;
};

// Represents a capsule as a lightweight support-mapped query primitive. The symmetry axis is the Y-axis.
struct Capsule : public ISupportMappable
{
    Capsule(Real radius, Real halfLength)
        : Radius(Shapes::Positive(radius, "radius")),
          HalfLength(Shapes::NonNegative(halfLength, "halfLength"))
    {
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result = LinearMath::JVector::Normalize(direction) * Radius;
        result.Y += Sign(direction.Y) * HalfLength;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    Real Radius;
    Real HalfLength;
};

// Represents a cylinder as a lightweight support-mapped query primitive. The symmetry axis is the Y-axis.
struct Cylinder : public ISupportMappable
{
    Cylinder(Real radius, Real halfHeight)
        : Radius(Shapes::Positive(radius, "radius")),
          HalfHeight(Shapes::Positive(halfHeight, "halfHeight"))
    {
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        const Real sigma = std::sqrt(direction.X * direction.X + direction.Z * direction.Z);
        if (sigma > static_cast<Real>(0))
        {
            result.X = direction.X / sigma * Radius;
            result.Z = direction.Z / sigma * Radius;
        }
        else
        {
            result.X = static_cast<Real>(0);
            result.Z = static_cast<Real>(0);
        }

        result.Y = Sign(direction.Y) * HalfHeight;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    Real Radius;
    Real HalfHeight;
};

// Represents a cone as a lightweight support-mapped query primitive. The symmetry axis is the Y-axis.
// The cone is centered at its centroid: the base sits at Y = -height/4 and the apex at Y = 3*height/4.
struct Cone : public ISupportMappable
{
    Cone(Real radius, Real height)
        : Radius(Shapes::Positive(radius, "radius")),
          Height(Shapes::Positive(height, "height"))
    {
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        constexpr Real zeroEpsilon = static_cast<Real>(1e-12);
        LinearMath::JVector baseDirection(direction.X, 0, direction.Z);
        baseDirection = LinearMath::JVector::NormalizeSafe(baseDirection, zeroEpsilon) * Radius;
        baseDirection.Y = -static_cast<Real>(0.25) * Height;
        const LinearMath::JVector tip(0, static_cast<Real>(0.75) * Height, 0);
        result = LinearMath::JVector::Dot(direction, baseDirection)
                >= LinearMath::JVector::Dot(direction, tip)
            ? baseDirection
            : tip;
    }

    void GetCenter(LinearMath::JVector& point) const override
    {
        point = LinearMath::JVector::Zero();
    }

    Real Radius;
    Real Height;
};

// Creates a point support primitive at the origin.
inline Point CreatePoint()
{
    return Point {};
}

inline Sphere CreateSphere(Real radius)
{
    return Sphere(radius);
}

inline Box CreateBox(const LinearMath::JVector& halfExtents)
{
    return Box(halfExtents);
}

inline Capsule CreateCapsule(Real radius, Real halfLength)
{
    return Capsule(radius, halfLength);
}

inline Cylinder CreateCylinder(Real radius, Real halfHeight)
{
    return Cylinder(radius, halfHeight);
}

inline Cone CreateCone(Real radius, Real height)
{
    return Cone(radius, height);
}

} // namespace Jitter2::Collision::SupportPrimitives
