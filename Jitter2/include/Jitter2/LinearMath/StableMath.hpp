#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <Jitter2/Precision.hpp>

// Internal trigonometric helpers for bit-identical results on different platforms (not guaranteed by Math and MathF).
// Until scalar trig in the BCL is both deterministic and fixed by managed source across targets,
// this helper keeps its own tiny approximation pipeline so the physics engine owns the behavior.

namespace Jitter2::StableMath
{

inline constexpr Real Pi = static_cast<Real>(3.141592653589793238462643383279502884L);
inline constexpr Real HalfPi = static_cast<Real>(1.570796326794896619231321691639751442L);
inline constexpr Real QuarterPi = static_cast<Real>(0.785398163397448309615660845819875721L);
inline constexpr Real TwoPi = static_cast<Real>(6.283185307179586476925286766559005768L);

inline constexpr Real TanPiOver8 = static_cast<Real>(0.414213562373095048801688724209698079L);

inline int FloorToInt(Real value)
{
    const int integer = static_cast<int>(value);
    return value < static_cast<Real>(integer) ? integer - 1 : integer;
}

inline Real ReduceAngle(Real angle)
{
    const int periods = FloorToInt((angle + Pi) / TwoPi);
    angle -= static_cast<Real>(periods) * TwoPi;

    if (angle > Pi)
    {
        angle -= TwoPi;
    }
    else if (angle <= -Pi)
    {
        angle += TwoPi;
    }

    return angle;
}

inline void ReduceToQuadrant(Real angle, int& quadrant, Real& reduced)
{
    angle = ReduceAngle(angle);
    const int nearestQuarterTurn = FloorToInt((angle + QuarterPi) / HalfPi);
    reduced = angle - static_cast<Real>(nearestQuarterTurn) * HalfPi;
    quadrant = nearestQuarterTurn & 3;
}

inline Real SinPolynomial(Real x)
{
    const Real x2 = x * x;
    Real polynomial = -static_cast<Real>(1.0 / 6227020800.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 39916800.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 362880.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 5040.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 120.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 6.0);
    polynomial = polynomial * x2 - static_cast<Real>(1);
    return -x * polynomial;
}

inline Real CosPolynomial(Real x)
{
    const Real x2 = x * x;
    Real polynomial = -static_cast<Real>(1.0 / 479001600.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 3628800.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 40320.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 720.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 24.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 2.0);
    polynomial = polynomial * x2 - static_cast<Real>(1);
    return -polynomial;
}

inline std::pair<Real, Real> ApplyQuadrant(int quadrant, Real sin, Real cos)
{
    switch (quadrant)
    {
    case 0: return {sin, cos};
    case 1: return {cos, -sin};
    case 2: return {-sin, -cos};
    default: return {-cos, sin};
    }
}

inline std::pair<Real, Real> SinCos(Real angle)
{
    if (angle >= -QuarterPi && angle <= QuarterPi)
    {
        return {SinPolynomial(angle), CosPolynomial(angle)};
    }

    int quadrant = 0;
    Real reduced = 0;
    ReduceToQuadrant(angle, quadrant, reduced);

    const Real sin = SinPolynomial(reduced);
    const Real cos = CosPolynomial(reduced);
    return ApplyQuadrant(quadrant, sin, cos);
}

inline Real Sin(Real angle)
{
    if (angle >= -QuarterPi && angle <= QuarterPi)
    {
        return SinPolynomial(angle);
    }

    int quadrant = 0;
    Real reduced = 0;
    ReduceToQuadrant(angle, quadrant, reduced);

    switch (quadrant)
    {
    case 0: return SinPolynomial(reduced);
    case 1: return CosPolynomial(reduced);
    case 2: return -SinPolynomial(reduced);
    default: return -CosPolynomial(reduced);
    }
}

inline Real Cos(Real angle)
{
    if (angle >= -QuarterPi && angle <= QuarterPi)
    {
        return CosPolynomial(angle);
    }

    int quadrant = 0;
    Real reduced = 0;
    ReduceToQuadrant(angle, quadrant, reduced);

    switch (quadrant)
    {
    case 0: return CosPolynomial(reduced);
    case 1: return -SinPolynomial(reduced);
    case 2: return -CosPolynomial(reduced);
    default: return SinPolynomial(reduced);
    }
}

inline Real AtanTaylor(Real value)
{
    const Real x2 = value * value;
    Real polynomial = static_cast<Real>(1.0 / 17.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 15.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 13.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 11.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 9.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 7.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 5.0);
    polynomial = polynomial * x2 - static_cast<Real>(1.0 / 3.0);
    polynomial = polynomial * x2 + static_cast<Real>(1);
    return value * polynomial;
}

inline Real Atan(Real value)
{
    if (value < static_cast<Real>(0))
    {
        return -Atan(-value);
    }

    if (value > static_cast<Real>(1))
    {
        return HalfPi - Atan(static_cast<Real>(1) / value);
    }

    if (value > TanPiOver8)
    {
        const Real reduced = (value - static_cast<Real>(1)) / (value + static_cast<Real>(1));
        return QuarterPi + AtanTaylor(reduced);
    }

    return AtanTaylor(value);
}

inline Real Atan2(Real y, Real x)
{
    if (x > static_cast<Real>(0))
    {
        return Atan(y / x);
    }

    if (x < static_cast<Real>(0))
    {
        return y >= static_cast<Real>(0)
            ? Atan(y / x) + Pi
            : Atan(y / x) - Pi;
    }

    if (y > static_cast<Real>(0))
    {
        return HalfPi;
    }

    if (y < static_cast<Real>(0))
    {
        return -HalfPi;
    }

    return static_cast<Real>(0);
}

inline Real AsinTaylor(Real value)
{
    const Real x2 = value * value;
    Real polynomial = static_cast<Real>(143.0 / 10240.0);
    polynomial = polynomial * x2 + static_cast<Real>(231.0 / 13312.0);
    polynomial = polynomial * x2 + static_cast<Real>(63.0 / 2816.0);
    polynomial = polynomial * x2 + static_cast<Real>(35.0 / 1152.0);
    polynomial = polynomial * x2 + static_cast<Real>(5.0 / 112.0);
    polynomial = polynomial * x2 + static_cast<Real>(3.0 / 40.0);
    polynomial = polynomial * x2 + static_cast<Real>(1.0 / 6.0);
    polynomial = polynomial * x2 + static_cast<Real>(1);
    return value * polynomial;
}

inline Real Acos(Real value)
{
    value = std::clamp(value, static_cast<Real>(-1), static_cast<Real>(1));

    if (value > static_cast<Real>(0.5))
    {
        const Real reduced = std::sqrt(std::max(static_cast<Real>(0), (static_cast<Real>(1) - value) * static_cast<Real>(0.5)));
        return static_cast<Real>(2) * AsinTaylor(reduced);
    }

    if (value < static_cast<Real>(-0.5))
    {
        const Real reduced = std::sqrt(std::max(static_cast<Real>(0), (static_cast<Real>(1) + value) * static_cast<Real>(0.5)));
        return Pi - static_cast<Real>(2) * AsinTaylor(reduced);
    }

    return HalfPi - AsinTaylor(value);
}

inline Real Asin(Real value)
{
    value = std::clamp(value, static_cast<Real>(-1), static_cast<Real>(1));
    const Real absValue = std::abs(value);

    if (absValue <= static_cast<Real>(0.5))
    {
        return AsinTaylor(value);
    }

    const Real reduced = std::sqrt(std::max(static_cast<Real>(0), (static_cast<Real>(1) - absValue) * static_cast<Real>(0.5)));
    const Real angle = HalfPi - static_cast<Real>(2) * AsinTaylor(reduced);
    return value < static_cast<Real>(0) ? -angle : angle;
}

inline Real Sqrt(Real value)
{
    return std::sqrt(value);
}

inline Real Abs(Real value)
{
    return std::abs(value);
}

} // namespace Jitter2::StableMath
