#pragma once

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Jitter2/LinearMath/JAngle.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Precision.hpp>

namespace Jitter2
{

// Validates public API inputs that must be checked in all build configurations.
// Use this helper for construction, setup, and configuration values that would otherwise leave
// Jitter2 in an invalid persistent state. For Debug-only sanity checks on runtime mutation paths,
// use DebugCheck.
class ArgumentCheck
{
public:
    static constexpr Real NonZeroLengthSquared = static_cast<Real>(1e-12);
    static constexpr Real UnitLengthSquaredTolerance = static_cast<Real>(1e-3);

    static Real Finite(Real value, const char* paramName)
    {
        if (!IsFiniteCore(value))
        {
            throw std::invalid_argument(Message(paramName, "Value must be finite."));
        }
        return value;
    }

    static LinearMath::JAngle Finite(LinearMath::JAngle value, const char* paramName)
    {
        Finite(static_cast<Real>(value), paramName);
        return value;
    }

    static Real NotNaN(Real value, const char* paramName)
    {
        if (std::isnan(value))
        {
            throw std::invalid_argument(Message(paramName, "Value must not be NaN."));
        }
        return value;
    }

    static LinearMath::JAngle NotNaN(LinearMath::JAngle value, const char* paramName)
    {
        NotNaN(static_cast<Real>(value), paramName);
        return value;
    }

    static LinearMath::JVector Finite(const LinearMath::JVector& value, const char* paramName)
    {
        if (!IsFiniteCore(value))
        {
            throw std::invalid_argument(Message(paramName, "Vector components must be finite."));
        }
        return value;
    }

    static LinearMath::JQuaternion Finite(const LinearMath::JQuaternion& value, const char* paramName)
    {
        if (!IsFiniteCore(value))
        {
            throw std::invalid_argument(Message(paramName, "Quaternion components must be finite."));
        }
        return value;
    }

    static LinearMath::JMatrix Finite(const LinearMath::JMatrix& value, const char* paramName)
    {
        if (!IsFiniteCore(value))
        {
            throw std::invalid_argument(Message(paramName, "Matrix components must be finite."));
        }
        return value;
    }

    static Real NonNegative(Real value, const char* paramName)
    {
        if (!IsFiniteCore(value) || value < static_cast<Real>(0))
        {
            throw std::out_of_range(Message(paramName, "Value must be finite and non-negative."));
        }
        return value;
    }

    static Real Positive(Real value, const char* paramName)
    {
        if (!IsFiniteCore(value) || value <= static_cast<Real>(0))
        {
            throw std::out_of_range(Message(paramName, "Value must be finite and positive."));
        }
        return value;
    }

    static Real InRange(Real value, Real min, Real max, const char* paramName)
    {
        if (!IsFiniteCore(value) || value < min || value > max)
        {
            std::ostringstream message;
            message << "Value must be finite and in the range [" << min << ", " << max << "].";
            throw std::out_of_range(Message(paramName, message.str().c_str()));
        }
        return value;
    }

    static LinearMath::JVector NonZero(const LinearMath::JVector& value, const char* paramName)
    {
        if (!IsFiniteCore(value) || value.LengthSquared() <= NonZeroLengthSquared)
        {
            throw std::invalid_argument(Message(paramName, "Vector must be finite and non-zero."));
        }
        return value;
    }

    static LinearMath::JVector PositiveComponents(const LinearMath::JVector& value, const char* paramName)
    {
        Positive(value.X, paramName);
        Positive(value.Y, paramName);
        Positive(value.Z, paramName);
        return value;
    }

    static LinearMath::JVector UnitVector(const LinearMath::JVector& value, const char* paramName)
    {
        const Real lengthSquared = value.LengthSquared();
        if (!IsFiniteCore(value)
            || std::abs(lengthSquared - static_cast<Real>(1)) > UnitLengthSquaredTolerance)
        {
            throw std::invalid_argument(Message(paramName, "Vector must be finite and normalized."));
        }
        return value;
    }

    static LinearMath::JQuaternion UnitQuaternion(const LinearMath::JQuaternion& value, const char* paramName)
    {
        const Real lengthSquared = value.LengthSquared();
        if (!IsFiniteCore(value)
            || std::abs(lengthSquared - static_cast<Real>(1)) > UnitLengthSquaredTolerance)
        {
            throw std::invalid_argument(Message(paramName, "Quaternion must be finite and normalized."));
        }
        return value;
    }

private:
    static bool IsFiniteCore(Real value)
    {
        return std::isfinite(value);
    }

    static bool IsFiniteCore(const LinearMath::JVector& value)
    {
        return IsFiniteCore(value.X) && IsFiniteCore(value.Y) && IsFiniteCore(value.Z);
    }

    static bool IsFiniteCore(const LinearMath::JQuaternion& value)
    {
        return IsFiniteCore(value.X) && IsFiniteCore(value.Y)
            && IsFiniteCore(value.Z) && IsFiniteCore(value.W);
    }

    static bool IsFiniteCore(const LinearMath::JMatrix& value)
    {
        return IsFiniteCore(value.M11) && IsFiniteCore(value.M12) && IsFiniteCore(value.M13)
            && IsFiniteCore(value.M21) && IsFiniteCore(value.M22) && IsFiniteCore(value.M23)
            && IsFiniteCore(value.M31) && IsFiniteCore(value.M32) && IsFiniteCore(value.M33);
    }

    static std::string Message(const char* paramName, const char* text)
    {
        return std::string(paramName) + ": " + text;
    }
};

} // namespace Jitter2
