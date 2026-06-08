#pragma once

#include <cmath>

#include <Jitter2/Precision.hpp>

namespace Jitter2::LinearMath
{

struct JAngle
{
    Real Radian = static_cast<Real>(0);

    [[nodiscard]] Real Degree() const
    {
        return Radian / Pi() * static_cast<Real>(180);
    }

    void Degree(Real value)
    {
        Radian = value / static_cast<Real>(180) * Pi();
    }

    // Creates a JAngle from a value in radians.
    [[nodiscard]] static JAngle FromRadian(Real radians)
    {
        return JAngle {radians};
    }

    // Creates a JAngle from a value in degrees.
    [[nodiscard]] static JAngle FromDegree(Real degrees)
    {
        JAngle angle;
        angle.Degree(degrees);
        return angle;
    }

    explicit operator Real() const
    {
        return Radian;
    }

    [[nodiscard]] JAngle operator-() const
    {
        return FromRadian(-Radian);
    }

    friend JAngle operator+(JAngle left, JAngle right)
    {
        return FromRadian(left.Radian + right.Radian);
    }

    friend JAngle operator-(JAngle left, JAngle right)
    {
        return FromRadian(left.Radian - right.Radian);
    }

    friend bool operator==(JAngle left, JAngle right)
    {
        return left.Radian == right.Radian;
    }

    friend bool operator!=(JAngle left, JAngle right)
    {
        return !(left == right);
    }

    friend bool operator<(JAngle left, JAngle right)
    {
        return left.Radian < right.Radian;
    }

    friend bool operator>(JAngle left, JAngle right)
    {
        return right < left;
    }

    friend bool operator<=(JAngle left, JAngle right)
    {
        return !(right < left);
    }

    friend bool operator>=(JAngle left, JAngle right)
    {
        return !(left < right);
    }

private:
    [[nodiscard]] static constexpr Real Pi()
    {
        return static_cast<Real>(3.141592653589793238462643383279502884L);
    }
};

} // namespace Jitter2::LinearMath
