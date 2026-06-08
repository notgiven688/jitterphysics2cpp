#pragma once

#include <limits>

#include <Jitter2/Precision.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Represents an angular limit defined by a minimum and maximum angle.
// Used by constraints to restrict rotational motion within a specified range.
// from: The minimum angle of the limit.
// to: The maximum angle of the limit.
struct AngularLimit
{
    Real From = static_cast<Real>(0);
    Real To = static_cast<Real>(0);

    constexpr AngularLimit() = default;
    constexpr AngularLimit(Real from, Real to)
        : From(from),
          To(to)
    {
    }

    static constexpr AngularLimit Full()
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        return AngularLimit(-pi, pi);
    }

    static constexpr AngularLimit Fixed()
    {
        return AngularLimit(static_cast<Real>(1e-6), static_cast<Real>(-1e-6));
    }

    // Creates an angular limit from degree values.
    // min: The minimum angle in degrees.
    // max: The maximum angle in degrees.
    // Returns: A new AngularLimit instance.
    static constexpr AngularLimit FromDegree(Real min, Real max)
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        return AngularLimit(
            min / static_cast<Real>(180) * pi,
            max / static_cast<Real>(180) * pi);
    }
};

// Represents a linear limit defined by a minimum and maximum distance.
// Used by constraints to restrict translational motion within a specified range.
// from: The minimum distance of the limit.
// to: The maximum distance of the limit.
struct LinearLimit
{
    Real From = static_cast<Real>(0);
    Real To = static_cast<Real>(0);

    constexpr LinearLimit() = default;
    constexpr LinearLimit(Real from, Real to)
        : From(from),
          To(to)
    {
    }

    static constexpr LinearLimit Fixed()
    {
        return LinearLimit(static_cast<Real>(1e-6), static_cast<Real>(-1e-6));
    }

    static constexpr LinearLimit Full()
    {
        return LinearLimit(
            -std::numeric_limits<Real>::infinity(),
            std::numeric_limits<Real>::infinity());
    }

    // Creates a linear limit from minimum and maximum values.
    // min: The minimum distance.
    // max: The maximum distance.
    // Returns: A new LinearLimit instance.
    static constexpr LinearLimit FromMinMax(Real min, Real max)
    {
        return LinearLimit(min, max);
    }
};

} // namespace Jitter2::Dynamics::Constraints
