#pragma once

#include <cmath>
#include <limits>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a three-dimensional box shape.
class BoxShape final : public RigidBodyShape
{
public:

    // Creates a box shape with specified dimensions.
    // size: The dimensions of the box.
    // Thrown when any component of size is less than or equal to zero.
    explicit BoxShape(const LinearMath::JVector& size)
        : halfSize_(static_cast<Real>(0.5) * PositiveComponents(size, "size"))
    {
        UpdateWorldBoundingBox();
    }

    // Creates a cube shape with sides of equal length.
    // size: The length of each side.
    // Thrown when size is less than or equal to zero.
    explicit BoxShape(Real size)
        : halfSize_(Positive(size, "size") * static_cast<Real>(0.5))
    {
        UpdateWorldBoundingBox();
    }

    // Creates a box shape with the specified length, height, and width.
    // length: The length of the box.
    // height: The height of the box.
    // width: The width of the box.
    // Thrown when length, height, or width is less than
    // or equal to zero.

    BoxShape(Real width, Real height, Real length)
        : halfSize_(static_cast<Real>(0.5) * LinearMath::JVector(
            Positive(width, "width"),
            Positive(height, "height"),
            Positive(length, "length")))
    {
        UpdateWorldBoundingBox();
    }

    [[nodiscard]] LinearMath::JVector Size() const
    {
        return static_cast<Real>(2) * halfSize_;
    }

    void Size(const LinearMath::JVector& value)
    {
        halfSize_ = PositiveComponents(value, "Size") * static_cast<Real>(0.5);
        UpdateWorldBoundingBox();
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        result.X = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.X)) * halfSize_.X;
        result.Y = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.Y)) * halfSize_.Y;
        result.Z = static_cast<Real>(LinearMath::MathHelper::SignBit(direction.Z)) * halfSize_.Z;
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
        const LinearMath::JMatrix absolute = LinearMath::JMatrix::Absolute(
            LinearMath::JMatrix::CreateFromQuaternion(orientation));
        const LinearMath::JVector transformedHalfSize = absolute * halfSize_;
        box.Min = position - transformedHalfSize;
        box.Max = position + transformedHalfSize;
    }

    bool LocalRayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        constexpr Real epsilon = static_cast<Real>(1e-22);
        const LinearMath::JVector min = -halfSize_;
        const LinearMath::JVector max = halfSize_;

        normal = LinearMath::JVector::Zero();
        lambda = 0;
        Real exit = std::numeric_limits<Real>::infinity();

        if (std::abs(direction.X) > epsilon)
        {
            const Real ix = static_cast<Real>(1) / direction.X;
            Real t0 = (min.X - origin.X) * ix;
            Real t1 = (max.X - origin.X) * ix;

            if (t0 > t1)
            {
                std::swap(t0, t1);
            }

            if (t0 > exit || t1 < lambda)
            {
                return false;
            }

            if (t0 > lambda)
            {
                lambda = t0;
                normal = direction.X < static_cast<Real>(0) ? LinearMath::JVector::UnitX() : -LinearMath::JVector::UnitX();
            }

            if (t1 < exit)
            {
                exit = t1;
            }
        }
        else if (origin.X < min.X || origin.X > max.X)
        {
            return false;
        }

        if (std::abs(direction.Y) > epsilon)
        {
            const Real iy = static_cast<Real>(1) / direction.Y;
            Real t0 = (min.Y - origin.Y) * iy;
            Real t1 = (max.Y - origin.Y) * iy;

            if (t0 > t1)
            {
                std::swap(t0, t1);
            }

            if (t0 > exit || t1 < lambda)
            {
                return false;
            }

            if (t0 > lambda)
            {
                lambda = t0;
                normal = direction.Y < static_cast<Real>(0) ? LinearMath::JVector::UnitY() : -LinearMath::JVector::UnitY();
            }

            if (t1 < exit)
            {
                exit = t1;
            }
        }
        else if (origin.Y < min.Y || origin.Y > max.Y)
        {
            return false;
        }

        if (std::abs(direction.Z) > epsilon)
        {
            const Real iz = static_cast<Real>(1) / direction.Z;
            Real t0 = (min.Z - origin.Z) * iz;
            Real t1 = (max.Z - origin.Z) * iz;

            if (t0 > t1)
            {
                std::swap(t0, t1);
            }

            if (t0 > exit || t1 < lambda)
            {
                return false;
            }

            if (t0 > lambda)
            {
                lambda = t0;
                normal = direction.Z < static_cast<Real>(0) ? LinearMath::JVector::UnitZ() : -LinearMath::JVector::UnitZ();
            }
        }
        else if (origin.Z < min.Z || origin.Z > max.Z)
        {
            return false;
        }

        return true;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        const LinearMath::JVector size = halfSize_ * static_cast<Real>(2);
        mass = size.X * size.Y * size.Z;

        inertia = LinearMath::JMatrix::Identity();
        inertia.M11 = static_cast<Real>(1.0 / 12.0) * mass * (size.Y * size.Y + size.Z * size.Z);
        inertia.M22 = static_cast<Real>(1.0 / 12.0) * mass * (size.X * size.X + size.Z * size.Z);
        inertia.M33 = static_cast<Real>(1.0 / 12.0) * mass * (size.X * size.X + size.Y * size.Y);
        centerOfMass = LinearMath::JVector::Zero();
    }

private:
    LinearMath::JVector halfSize_;
};

} // namespace Jitter2::Collision::Shapes
