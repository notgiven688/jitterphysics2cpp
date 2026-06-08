#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>

#include <Jitter2/Precision.hpp>

namespace Jitter2::LinearMath
{

struct JVector
{
    Real X;
    Real Y;
    Real Z;

    constexpr JVector() : X(0), Y(0), Z(0) {}

    // Creates a vector with all components set to the same value.
    // xyz: The value for X, Y, and Z components.
    constexpr explicit JVector(Real xyz) : X(xyz), Y(xyz), Z(xyz) {}

    // Represents a three-dimensional vector with components of type Real.
    constexpr JVector(Real x, Real y, Real z) : X(x), Y(y), Z(z) {}

    void Set(Real x, Real y, Real z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    static constexpr JVector Zero() { return JVector(0, 0, 0); }
    static constexpr JVector One() { return JVector(1, 1, 1); }
    static constexpr JVector Arbitrary() { return JVector(1, 1, 1); }
    static constexpr JVector UnitX() { return JVector(1, 0, 0); }
    static constexpr JVector UnitY() { return JVector(0, 1, 0); }
    static constexpr JVector UnitZ() { return JVector(0, 0, 1); }
    static constexpr JVector MinValue()
    {
        return JVector(std::numeric_limits<Real>::lowest());
    }
    static constexpr JVector MaxValue()
    {
        return JVector(std::numeric_limits<Real>::max());
    }

    Real& operator[](int index) { return (&X)[index]; }
    const Real& operator[](int index) const { return (&X)[index]; }

    // Returns a string representation of the JVector.
    std::string ToString() const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(6);
        stream << "X=" << X << ", Y=" << Y << ", Z=" << Z;
        return stream.str();
    }

    // Calculates the squared length of the vector (‖v‖²).
    Real LengthSquared() const { return X * X + Y * Y + Z * Z; }

    // Calculates the length of the vector (‖v‖).
    Real Length() const { return std::sqrt(LengthSquared()); }

    void Normalize()
    {
        const Real inverseLength = static_cast<Real>(1) / std::sqrt(LengthSquared());
        *this *= inverseLength;
    }

    // Returns a normalized unit vector.
    // value: The vector to normalize.
    // Returns: The normalized unit vector.
    static JVector Normalize(JVector value)
    {
        value.Normalize();
        return value;
    }

    // Normalizes the vector in-place.
    // toNormalize: The vector to normalize.
    static void NormalizeInPlace(JVector& value)
    {
        value.Normalize();
    }

    static JVector NormalizeSafe(const JVector& value, Real epsilonSquared = static_cast<Real>(1e-16))
    {
        const Real lengthSquared = value.LengthSquared();
        if (lengthSquared < epsilonSquared)
        {
            return Zero();
        }

        const Real inverseLength = static_cast<Real>(1) / std::sqrt(lengthSquared);
        return JVector(value.X * inverseLength, value.Y * inverseLength, value.Z * inverseLength);
    }

    // Calculates the dot product of two vectors (u · v).
    // vector1: The first vector.
    // vector2: The second vector.
    // Returns: The dot product.
    static constexpr Real Dot(const JVector& left, const JVector& right)
    {
        return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
    }

    // Calculates the cross product of two vectors (u × v).
    // vector1: The first vector.
    // vector2: The second vector.
    // Returns: The cross product.
    static constexpr JVector Cross(const JVector& left, const JVector& right)
    {
        return JVector(
            left.Y * right.Z - left.Z * right.Y,
            left.Z * right.X - left.X * right.Z,
            left.X * right.Y - left.Y * right.X);
    }

    // Returns a vector containing the minimum components of the specified vectors.
    // value1: The first vector.
    // value2: The second vector.
    // Returns: A vector with the minimum of each component.
    static constexpr JVector Min(const JVector& left, const JVector& right)
    {
        return JVector(
            left.X < right.X ? left.X : right.X,
            left.Y < right.Y ? left.Y : right.Y,
            left.Z < right.Z ? left.Z : right.Z);
    }

    // Returns a vector containing the maximum components of the specified vectors.
    // value1: The first vector.
    // value2: The second vector.
    // Returns: A vector with the maximum of each component.
    static constexpr JVector Max(const JVector& left, const JVector& right)
    {
        return JVector(
            left.X > right.X ? left.X : right.X,
            left.Y > right.Y ? left.Y : right.Y,
            left.Z > right.Z ? left.Z : right.Z);
    }

    // Returns a vector containing the absolute values of the components of the specified vector.
    // value1: The input vector.
    // Returns: A vector with absolute values of each component.
    static JVector Abs(const JVector& value)
    {
        return JVector(std::abs(value.X), std::abs(value.Y), std::abs(value.Z));
    }

    // Returns the maximum absolute value among the vector's components.
    // value1: The input vector.
    // Returns: The maximum of |X|, |Y|, and |Z|.
    static Real MaxAbs(const JVector& value)
    {
        const JVector abs = Abs(value);
        return std::max({abs.X, abs.Y, abs.Z});
    }

    // Adds two vectors.
    // value1: The first vector.
    // value2: The second vector.
    // Returns: The sum of the two vectors.
    static constexpr JVector Add(const JVector& left, const JVector& right)
    {
        return JVector(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
    }

    // Adds two vectors.
    // value1: The first vector.
    // value2: The second vector.
    // result: Output: The sum of the two vectors.
    static constexpr void Add(const JVector& left, const JVector& right, JVector& result)
    {
        result = Add(left, right);
    }

    // Subtracts the second vector from the first.
    // value1: The first vector.
    // value2: The second vector.
    // Returns: The difference of the two vectors.
    static constexpr JVector Subtract(const JVector& left, const JVector& right)
    {
        return JVector(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
    }

    // Subtracts the second vector from the first.
    // value1: The first vector.
    // value2: The second vector.
    // result: Output: The difference of the two vectors.
    static constexpr void Subtract(const JVector& left, const JVector& right, JVector& result)
    {
        result = Subtract(left, right);
    }

    // Multiplies a vector by a scalar.
    // value1: The vector.
    // scaleFactor: The scalar factor.
    // Returns: The scaled vector.
    static constexpr JVector Multiply(const JVector& value, Real scale)
    {
        return JVector(value.X * scale, value.Y * scale, value.Z * scale);
    }

    // Multiplies a vector by a scalar.
    // value1: The vector.
    // scaleFactor: The scalar factor.
    // result: Output: The scaled vector.
    static constexpr void Multiply(const JVector& value, Real scale, JVector& result)
    {
        result = Multiply(value, scale);
    }

    void Negate()
    {
        X = -X;
        Y = -Y;
        Z = -Z;
    }

    // Negates the vector in-place.
    // vector: The vector to negate.
    static void NegateInPlace(JVector& value)
    {
        value.Negate();
    }

    // Returns a negated copy of the vector.
    // value: The vector to negate.
    // Returns: The negated vector.
    static constexpr JVector Negate(const JVector& value)
    {
        return JVector(-value.X, -value.Y, -value.Z);
    }

    // Returns a negated copy of the vector.
    // value: The vector to negate.
    // result: Output: The negated vector.
    static constexpr void Negate(const JVector& value, JVector& result)
    {
        result = Negate(value);
    }

    // Swaps the values of two vectors.
    static void Swap(JVector& left, JVector& right)
    {
        std::swap(left, right);
    }

    constexpr JVector& operator+=(const JVector& other)
    {
        X += other.X;
        Y += other.Y;
        Z += other.Z;
        return *this;
    }

    constexpr JVector& operator-=(const JVector& other)
    {
        X -= other.X;
        Y -= other.Y;
        Z -= other.Z;
        return *this;
    }

    constexpr JVector& operator*=(Real scale)
    {
        X *= scale;
        Y *= scale;
        Z *= scale;
        return *this;
    }

    constexpr JVector& operator/=(Real scale)
    {
        X /= scale;
        Y /= scale;
        Z /= scale;
        return *this;
    }
};

static_assert(sizeof(JVector) == 3 * sizeof(Real));

constexpr bool operator==(const JVector& left, const JVector& right)
{
    return left.X == right.X && left.Y == right.Y && left.Z == right.Z;
}

constexpr bool operator!=(const JVector& left, const JVector& right)
{
    return !(left == right);
}

// Returns the vector itself.
constexpr JVector operator+(JVector left, const JVector& right)
{
    left += right;
    return left;
}

// Subtracts the second vector from the first.
constexpr JVector operator-(JVector left, const JVector& right)
{
    left -= right;
    return left;
}

// Negates the vector.
constexpr JVector operator-(const JVector& value)
{
    return JVector(-value.X, -value.Y, -value.Z);
}

// Adds two vectors.
constexpr JVector operator+(const JVector& value)
{
    return value;
}

// Calculates the cross product of two vectors (u × v).
constexpr JVector operator%(const JVector& left, const JVector& right)
{
    return JVector::Cross(left, right);
}

// Calculates the dot product of two vectors (u · v).
constexpr Real operator*(const JVector& left, const JVector& right)
{
    return JVector::Dot(left, right);
}

// Multiplies a vector by a scalar.
constexpr JVector operator*(JVector value, Real scale)
{
    value *= scale;
    return value;
}

// Multiplies a vector by a scalar.
constexpr JVector operator*(Real scale, JVector value)
{
    value *= scale;
    return value;
}

constexpr JVector operator/(JVector value, Real scale)
{
    value /= scale;
    return value;
}

inline std::ostream& operator<<(std::ostream& stream, const JVector& value)
{
    return stream << value.ToString();
}

} // namespace Jitter2::LinearMath
