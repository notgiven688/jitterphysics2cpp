#pragma once

#include <Jitter2/ArgumentCheck.hpp>

namespace Jitter2
{

// Runs additional input sanity checks in Debug builds.
// Calls to this helper are compiled only when the DEBUG symbol is defined. It is used for
// runtime state mutation paths where invalid values should be caught during development without
// adding Release-build overhead.
class DebugCheck
{
public:
    static void IsFinite(Real value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Finite(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsFinite(LinearMath::JAngle value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Finite(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsNotNaN(Real value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::NotNaN(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsNotNaN(LinearMath::JAngle value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::NotNaN(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsFinite(const LinearMath::JVector& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Finite(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsFinite(const LinearMath::JQuaternion& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Finite(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsFinite(const LinearMath::JMatrix& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Finite(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsNonNegative(Real value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::NonNegative(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsPositive(Real value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::Positive(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsInRange(Real value, Real min, Real max, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::InRange(value, min, max, paramName);
#else
        (void)value;
        (void)min;
        (void)max;
        (void)paramName;
#endif
    }

    static void IsNonZero(const LinearMath::JVector& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::NonZero(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsUnitVector(const LinearMath::JVector& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::UnitVector(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }

    static void IsUnitQuaternion(const LinearMath::JQuaternion& value, const char* paramName)
    {
#ifndef NDEBUG
        ArgumentCheck::UnitQuaternion(value, paramName);
#else
        (void)value;
        (void)paramName;
#endif
    }
};

} // namespace Jitter2
