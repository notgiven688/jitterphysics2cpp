#pragma once

#include <bit>
#include <cstdint>
#include <cmath>

#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::LinearMath::MathHelper
{

// Gets the sign of value purely from its IEEE-754 sign bit.
// value: The number to test.
// Returns: +1 when the sign bit is clear (positive, +0, or a positive-sign NaN),
// -1 when the sign bit is set (negative, −0, or a negative-sign NaN).
// Never returns 0, unlike Math.Sign(float).
inline int SignBit(float value)
{
    return 1 | (std::bit_cast<std::int32_t>(value) >> 31);
}


inline int SignBit(double value)
{
    return 1 | static_cast<int>(std::bit_cast<std::int64_t>(value) >> 63);
}

inline bool IsZero(Real value, Real epsilon = static_cast<Real>(1e-6))
{
    return std::abs(value) < epsilon;
}

inline bool IsZero(const JVector& vector, Real epsilon = static_cast<Real>(1e-6))
{
    return !(std::abs(vector.X) >= epsilon)
        && !(std::abs(vector.Y) >= epsilon)
        && !(std::abs(vector.Z) >= epsilon);
}

inline bool UnsafeIsZero(const JMatrix& matrix, Real epsilon = static_cast<Real>(1e-6))
{
    return IsZero(matrix.GetColumn(0), epsilon)
        && IsZero(matrix.GetColumn(1), epsilon)
        && IsZero(matrix.GetColumn(2), epsilon);
}

inline bool IsZero(const JMatrix& matrix, Real epsilon = static_cast<Real>(1e-6))
{

    // Checks if all entries of a matrix are close to zero.
    // matrix: The matrix to check.
    // epsilon: The tolerance for each element.
    // Returns: true if all elements are within epsilon of zero; otherwise, false.
    return UnsafeIsZero(matrix, epsilon);
}

inline bool CloseToZero(const JVector& vector, Real epsilonSq = static_cast<Real>(1e-16))
{
    return vector.LengthSquared() < epsilonSq;
}

inline bool IsRotationMatrix(const JMatrix& matrix, Real epsilon = static_cast<Real>(1e-6))
{
    const JMatrix delta = JMatrix::MultiplyTransposed(matrix, matrix) - JMatrix::Identity();
    if (!UnsafeIsZero(delta, epsilon))
    {
        return false;
    }

    return std::abs(matrix.Determinant() - static_cast<Real>(1)) < epsilon;
}

inline JMatrix InverseSquareRoot(JMatrix matrix, int sweeps = 2)
{
    JMatrix rotation = JMatrix::Identity();

    for (int i = 0; i < sweeps; ++i)
    {
        Real phi;
        Real cp;
        Real sp;
        JMatrix r;

        if (std::abs(matrix.M23) > static_cast<Real>(1e-6))
        {
            phi = StableMath::Atan2(
                static_cast<Real>(1),
                (matrix.M33 - matrix.M22) / (static_cast<Real>(2) * matrix.M23)) / static_cast<Real>(2);
            auto sinCos = StableMath::SinCos(phi);
            sp = sinCos.first;
            cp = sinCos.second;
            r = JMatrix(1, 0, 0, 0, cp, sp, 0, -sp, cp);
            JMatrix::Multiply(matrix, r, matrix);
            JMatrix::TransposedMultiply(r, matrix, matrix);
            JMatrix::Multiply(rotation, r, rotation);
        }

        if (std::abs(matrix.M21) > static_cast<Real>(1e-6))
        {
            phi = StableMath::Atan2(
                static_cast<Real>(1),
                (matrix.M22 - matrix.M11) / (static_cast<Real>(2) * matrix.M21)) / static_cast<Real>(2);
            auto sinCos = StableMath::SinCos(phi);
            sp = sinCos.first;
            cp = sinCos.second;
            r = JMatrix(cp, sp, 0, -sp, cp, 0, 0, 0, 1);
            JMatrix::Multiply(matrix, r, matrix);
            JMatrix::TransposedMultiply(r, matrix, matrix);
            JMatrix::Multiply(rotation, r, rotation);
        }

        if (std::abs(matrix.M31) > static_cast<Real>(1e-6))
        {
            phi = StableMath::Atan2(
                static_cast<Real>(1),
                (matrix.M33 - matrix.M11) / (static_cast<Real>(2) * matrix.M31)) / static_cast<Real>(2);
            auto sinCos = StableMath::SinCos(phi);
            sp = sinCos.first;
            cp = sinCos.second;
            r = JMatrix(cp, 0, sp, 0, 1, 0, -sp, 0, cp);
            JMatrix::Multiply(matrix, r, matrix);
            JMatrix::TransposedMultiply(r, matrix, matrix);
            JMatrix::Multiply(rotation, r, rotation);
        }
    }

    const JMatrix d(
        static_cast<Real>(1) / StableMath::Sqrt(matrix.M11), 0, 0,
        0, static_cast<Real>(1) / StableMath::Sqrt(matrix.M22), 0,
        0, 0, static_cast<Real>(1) / StableMath::Sqrt(matrix.M33));

    return rotation * d * JMatrix::Transpose(rotation);
}

inline bool CheckOrthonormalBasis(const JMatrix& matrix, Real epsilon = static_cast<Real>(1e-6))
{
    const JMatrix delta = JMatrix::MultiplyTransposed(matrix, matrix) - JMatrix::Identity();
    return UnsafeIsZero(delta, epsilon);
}

// Calculates an orthonormal vector to the given vector.
// The input vector must be non-zero. Debug builds assert this condition.
// vec: The input vector (must be non-zero, does not need to be normalized).
// Returns: A unit vector orthogonal to the input.
inline JVector CreateOrthonormal(const JVector& vector)
{
    const Real ax = std::abs(vector.X);
    const Real ay = std::abs(vector.Y);
    const Real az = std::abs(vector.Z);

    JVector result;
    if (ax <= ay && ax <= az)
    {
        result = JVector(0, vector.Z, -vector.Y);
    }
    else if (ay <= ax && ay <= az)
    {
        result = JVector(-vector.Z, 0, vector.X);
    }
    else
    {
        result = JVector(vector.Y, -vector.X, 0);
    }

    result.Normalize();
    return result;
}

// Calculates the rotation quaternion corresponding to the given angular velocity using
// deterministic trigonometric approximations.
// omega: The angular velocity vector in radians per second.
// dt: The time step in seconds.
// Returns: A unit quaternion representing the rotation.
inline JQuaternion RotationQuaternion(const JVector& omega, Real dt)
{
    const Real angle = omega.Length();
    const Real theta = angle * dt;

    if (theta < static_cast<Real>(1e-3))
    {
        const Real dt3 = dt * dt * dt;
        const Real angle2 = angle * angle;
        const Real scale = static_cast<Real>(0.5) * dt
            - (static_cast<Real>(1) / static_cast<Real>(48)) * dt3 * angle2;
        const JVector axis = omega * scale;
        const Real c = static_cast<Real>(1) - (static_cast<Real>(1) / static_cast<Real>(8)) * theta * theta;
        return JQuaternion(axis.X, axis.Y, axis.Z, c);
    }

    const Real halfAngleDt = static_cast<Real>(0.5) * angle * dt;
    const auto [s, c] = StableMath::SinCos(halfAngleDt);
    const Real scale = s / angle;
    const JVector axis = omega * scale;
    return JQuaternion(axis.X, axis.Y, axis.Z, c);
}

} // namespace Jitter2::LinearMath::MathHelper
