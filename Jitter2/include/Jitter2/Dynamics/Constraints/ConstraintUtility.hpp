#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <Jitter2/ArgumentCheck.hpp>
#include <Jitter2/DebugCheck.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Precision.hpp>

namespace Jitter2::Dynamics::Constraints::Detail
{

inline void CheckFinite(const LinearMath::JVector& value, const char* name)
{
    ArgumentCheck::Finite(value, name);
}

inline void CheckFinite(Real value, const char* name)
{
    ArgumentCheck::Finite(value, name);
}

inline void CheckNonZero(const LinearMath::JVector& value, const char* name)
{
    ArgumentCheck::NonZero(value, name);
}

inline void CheckUnitVector(const LinearMath::JVector& value, const char* name)
{
    ArgumentCheck::UnitVector(value, name);
}

inline void CheckNonNegative(Real value, const char* name)
{
    ArgumentCheck::NonNegative(value, name);
}

inline LinearMath::JMatrix ProjectMultiplyLeftRight(
    const LinearMath::JQuaternion& left,
    const LinearMath::JQuaternion& right)
{
    return LinearMath::JMatrix(
        -left.X * right.X + left.W * right.W + left.Z * right.Z + left.Y * right.Y,
        -left.X * right.Y + left.W * right.Z - left.Z * right.W - left.Y * right.X,
        -left.X * right.Z - left.W * right.Y - left.Z * right.X + left.Y * right.W,
        -left.Y * right.X + left.Z * right.W - left.W * right.Z - left.X * right.Y,
        -left.Y * right.Y + left.Z * right.Z + left.W * right.W + left.X * right.X,
        -left.Y * right.Z - left.Z * right.Y + left.W * right.X - left.X * right.W,
        -left.Z * right.X - left.Y * right.W - left.X * right.Z + left.W * right.Y,
        -left.Z * right.Y - left.Y * right.Z + left.X * right.W - left.W * right.X,
        -left.Z * right.Z + left.Y * right.Y + left.X * right.X + left.W * right.W);
}

} // namespace Jitter2::Dynamics::Constraints::Detail
