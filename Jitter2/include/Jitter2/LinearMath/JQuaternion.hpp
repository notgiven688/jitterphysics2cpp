#pragma once

#include <algorithm>
#include <cmath>

#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>

namespace Jitter2::LinearMath
{

struct JMatrix;

struct JQuaternion
{
    Real X;
    Real Y;
    Real Z;
    Real W;

    constexpr JQuaternion() : X(0), Y(0), Z(0), W(0) {}

    // A structure representing a Quaternion: Q = x*i + y*j + z*k + w.
    // Uses the Hamilton convention where i² = j² = k² = ijk = -1.
    constexpr JQuaternion(Real x, Real y, Real z, Real w) : X(x), Y(y), Z(z), W(w) {}

    // Initializes a new instance of the JQuaternion struct.
    // w: The scalar (W) component.
    // v: The vector (X, Y, Z) component.
    constexpr JQuaternion(Real w, const JVector& vector) : X(vector.X), Y(vector.Y), Z(vector.Z), W(w) {}

    static constexpr JQuaternion Identity() { return JQuaternion(0, 0, 0, 1); }

    constexpr JVector Vector() const { return JVector(X, Y, Z); }
    constexpr Real Scalar() const { return W; }

    // Calculates the squared Euclidean length of the quaternion.
    // Returns: The squared length.
    Real LengthSquared() const { return X * X + Y * Y + Z * Z + W * W; }

    // Calculates the Euclidean length of the quaternion.
    // Returns: The length (magnitude).
    Real Length() const { return std::sqrt(LengthSquared()); }

    void Normalize()
    {
        const Real inverseLength = static_cast<Real>(1) / std::sqrt(LengthSquared());
        X *= inverseLength;
        Y *= inverseLength;
        Z *= inverseLength;
        W *= inverseLength;
    }


    static JQuaternion Normalize(JQuaternion value)
    {
        value.Normalize();
        return value;
    }

    // Returns a normalized version of the quaternion.
    // value: The source quaternion.
    // result: Output: The normalized unit quaternion.
    static void Normalize(const JQuaternion& value, JQuaternion& result)
    {
        result = Normalize(value);
    }

    // Normalizes the provided quaternion structure in place.
    // quaternion: The quaternion to normalize.
    static void NormalizeInPlace(JQuaternion& value)
    {
        value.Normalize();
    }

    // Calculates the dot product of two quaternions.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // Returns: The dot product.
    static constexpr Real Dot(const JQuaternion& left, const JQuaternion& right)
    {
        return left.X * right.X + left.Y * right.Y + left.Z * right.Z + left.W * right.W;
    }

    // Adds two quaternions component-wise.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // Returns: The sum of the two quaternions.
    static constexpr JQuaternion Add(const JQuaternion& left, const JQuaternion& right)
    {
        return JQuaternion(left.X + right.X, left.Y + right.Y, left.Z + right.Z, left.W + right.W);
    }

    // Adds two quaternions component-wise.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // result: Output: The sum of the two quaternions.
    static constexpr void Add(const JQuaternion& left, const JQuaternion& right, JQuaternion& result)
    {
        result = Add(left, right);
    }

    // Subtracts the second quaternion from the first component-wise.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // Returns: The difference of the two quaternions.
    static constexpr JQuaternion Subtract(const JQuaternion& left, const JQuaternion& right)
    {
        return JQuaternion(left.X - right.X, left.Y - right.Y, left.Z - right.Z, left.W - right.W);
    }

    // Subtracts the second quaternion from the first component-wise.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // result: Output: The difference of the two quaternions.
    static constexpr void Subtract(const JQuaternion& left, const JQuaternion& right, JQuaternion& result)
    {
        result = Subtract(left, right);
    }

    // Returns the conjugate of a quaternion.
    // The conjugate is defined as (-x, -y, -z, w).
    // For unit quaternions, the conjugate is equivalent to the inverse.
    // value: The quaternion to conjugate.
    // Returns: The conjugate of the quaternion.
    static constexpr JQuaternion Conjugate(const JQuaternion& value)
    {
        return JQuaternion(-value.X, -value.Y, -value.Z, value.W);
    }

    // Returns the conjugate of the quaternion.
    // The conjugate is defined as (-x, -y, -z, w).
    // Returns: The conjugate of the quaternion.
    constexpr JQuaternion Conjugate() const
    {
        return Conjugate(*this);
    }

    // Multiplies two quaternions (Hamilton Product).
    // Non-commutative. Q1 * Q2 represents the rotation Q2 followed by Q1 (local frame) or Q1 followed by Q2 (global frame).
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // Returns: The product of the two quaternions.
    static constexpr JQuaternion Multiply(const JQuaternion& left, const JQuaternion& right)
    {
        return JQuaternion(
            left.W * right.X + left.X * right.W + left.Y * right.Z - left.Z * right.Y,
            left.W * right.Y - left.X * right.Z + left.Y * right.W + left.Z * right.X,
            left.W * right.Z + left.X * right.Y - left.Y * right.X + left.Z * right.W,
            left.W * right.W - left.X * right.X - left.Y * right.Y - left.Z * right.Z);
    }

    // Multiplies two quaternions.
    // quaternion1: The first quaternion.
    // quaternion2: The second quaternion.
    // result: Output: The product of the two quaternions.
    static constexpr void Multiply(const JQuaternion& left, const JQuaternion& right, JQuaternion& result)
    {

        // Multiplies a quaternion by a scalar factor.
        // quaternion1: The quaternion.
        // scaleFactor: The scalar factor.
        // Returns: The scaled quaternion.
        result = Multiply(left, right);
    }


    static constexpr JQuaternion ConjugateMultiply(const JQuaternion& left, const JQuaternion& right)
    {
        const Real r1 = left.W;
        const Real i1 = -left.X;
        const Real j1 = -left.Y;
        const Real k1 = -left.Z;

        const Real r2 = right.W;
        const Real i2 = right.X;
        const Real j2 = right.Y;
        const Real k2 = right.Z;

        return JQuaternion(
            r1 * i2 + r2 * i1 + j1 * k2 - k1 * j2,
            r1 * j2 + r2 * j1 + k1 * i2 - i1 * k2,
            r1 * k2 + r2 * k1 + i1 * j2 - j1 * i2,
            r1 * r2 - (i1 * i2 + j1 * j2 + k1 * k2));
    }


    static constexpr JQuaternion MultiplyConjugate(const JQuaternion& left, const JQuaternion& right)
    {
        const Real r1 = left.W;
        const Real i1 = left.X;
        const Real j1 = left.Y;
        const Real k1 = left.Z;

        const Real r2 = right.W;
        const Real i2 = -right.X;
        const Real j2 = -right.Y;
        const Real k2 = -right.Z;

        return JQuaternion(
            r1 * i2 + r2 * i1 + j1 * k2 - k1 * j2,
            r1 * j2 + r2 * j1 + k1 * i2 - i1 * k2,
            r1 * k2 + r2 * k1 + i1 * j2 - j1 * i2,
            r1 * r2 - (i1 * i2 + j1 * j2 + k1 * k2));
    }

    static constexpr JQuaternion Multiply(const JQuaternion& value, Real scale)
    {
        return JQuaternion(value.X * scale, value.Y * scale, value.Z * scale, value.W * scale);
    }

    // Multiplies a quaternion by a scalar factor.
    // quaternion1: The quaternion.
    // scaleFactor: The scalar factor.
    // result: Output: The scaled quaternion.
    static constexpr void Multiply(const JQuaternion& value, Real scale, JQuaternion& result)
    {
        result = Multiply(value, scale);
    }

    static JVector Transform(const JVector& vector, const JQuaternion& quaternion)
    {
        const JVector qv(quaternion.X, quaternion.Y, quaternion.Z);
        const JVector t = static_cast<Real>(2) * JVector::Cross(qv, vector);
        return vector + quaternion.W * t + JVector::Cross(qv, t);
    }

    static JVector ConjugatedTransform(const JVector& vector, const JQuaternion& quaternion)
    {
        return Transform(vector, quaternion.Conjugate());
    }

    // Creates a quaternion that rotates the unit vector from into
    // the unit vector to.
    // This calculation relies on the half-angle formula.
    // If the vectors are parallel (dot ≈ 1), Identity is returned.
    // If the vectors are opposite (dot ≈ -1), a rotation of 180° (π radians)
    // around an arbitrary orthogonal axis is returned.
    // from: Source direction (must be unit length).
    // to: Target direction (must be unit length).
    // Returns: A unit quaternion representing the shortest rotation.
    static JQuaternion CreateFromToRotation(JVector from, JVector to)
    {
        constexpr Real epsilon = static_cast<Real>(1e-6);
        const Real dot = JVector::Dot(from, to);
        if (dot < -static_cast<Real>(1) + epsilon)
        {
            const Real ax = std::abs(from.X);
            const Real ay = std::abs(from.Y);
            const Real az = std::abs(from.Z);

            JVector axis;
            if (ax <= ay && ax <= az)
            {
                axis = JVector(0, from.Z, -from.Y);
            }
            else if (ay <= ax && ay <= az)
            {
                axis = JVector(-from.Z, 0, from.X);
            }
            else
            {
                axis = JVector(from.Y, -from.X, 0);
            }

            axis.Normalize();
            return JQuaternion(axis.X, axis.Y, axis.Z, 0);
        }

        const Real s = std::sqrt((static_cast<Real>(1) + dot) * static_cast<Real>(2));
        const Real invS = static_cast<Real>(1) / s;
        const JVector c = JVector::Cross(from, to);
        return JQuaternion(s * static_cast<Real>(0.5), c * invS);
    }

    // Creates a quaternion from a unit axis and an angle.
    // The axis must be normalized.
    // axis: The unit vector to rotate around (must be normalized).
    // angle: The angle of rotation in radians.
    // Returns: A unit quaternion representing the rotation.
    static JQuaternion CreateFromAxisAngle(const JVector& axis, Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians * static_cast<Real>(0.5));
        return JQuaternion(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    // Decomposes a unit quaternion into an axis and an angle.
    // Assumes quaternion is normalized.
    // Returns the shortest arc (angle in [0, π]).
    // quaternion: The unit quaternion to decompose.
    // axis: Output: The unit rotation axis.
    // angle: Output: The rotation angle (radians).
    static void ToAxisAngle(const JQuaternion& quaternion, JVector& axis, Real& angle)
    {
        const Real s = std::sqrt(std::max(static_cast<Real>(0), static_cast<Real>(1) - quaternion.W * quaternion.W));
        constexpr Real epsilonSingularity = static_cast<Real>(1e-6);

        if (s < epsilonSingularity)
        {
            angle = static_cast<Real>(0);
            axis = JVector::UnitX();
            return;
        }

        const Real invS = static_cast<Real>(1) / s;
        axis = JVector(quaternion.X * invS, quaternion.Y * invS, quaternion.Z * invS);
        angle = static_cast<Real>(2) * StableMath::Acos(quaternion.W);

        if (angle > StableMath::Pi)
        {
            angle = static_cast<Real>(2) * StableMath::Pi - angle;
            axis = -axis;
        }
    }

    // Returns the inverse of a quaternion.
    // Unlike Conjugate(in JQuaternion), this handles non-unit quaternions correctly
    // by dividing by the squared length.
    static JQuaternion Inverse(const JQuaternion& value)
    {
        const Real lengthSquared = value.LengthSquared();
        if (lengthSquared < static_cast<Real>(1e-12))
        {
            return Identity();
        }

        const Real inverseLengthSquared = static_cast<Real>(1) / lengthSquared;
        return JQuaternion(
            -value.X * inverseLengthSquared,
            -value.Y * inverseLengthSquared,
            -value.Z * inverseLengthSquared,
            value.W * inverseLengthSquared);
    }

    // Linearly interpolates between two quaternions and normalizes the result.
    // quaternion1: Source quaternion.
    // quaternion2: Target quaternion.
    // amount: Weight of the interpolation.
    // Returns: The interpolated unit quaternion.
    static JQuaternion Lerp(const JQuaternion& left, const JQuaternion& right, Real amount)
    {
        const Real inverse = static_cast<Real>(1) - amount;
        JQuaternion result;

        if (Dot(left, right) >= static_cast<Real>(0))
        {
            result.X = inverse * left.X + amount * right.X;
            result.Y = inverse * left.Y + amount * right.Y;
            result.Z = inverse * left.Z + amount * right.Z;
            result.W = inverse * left.W + amount * right.W;
        }
        else
        {
            result.X = inverse * left.X - amount * right.X;
            result.Y = inverse * left.Y - amount * right.Y;
            result.Z = inverse * left.Z - amount * right.Z;
            result.W = inverse * left.W - amount * right.W;
        }

        return Normalize(result);
    }

    // Interpolates between two quaternions using Spherical Linear Interpolation (SLERP).
    // quaternion1: Source quaternion.
    // quaternion2: Target quaternion.
    // amount: Weight of the interpolation.
    // Returns: The interpolated quaternion.
    static JQuaternion Slerp(const JQuaternion& left, const JQuaternion& right, Real amount)
    {
        Real dot = Dot(left, right);
        JQuaternion target = right;

        if (dot < static_cast<Real>(0))
        {
            dot = -dot;
            target = JQuaternion(-target.X, -target.Y, -target.Z, -target.W);
        }

        constexpr Real epsilon = static_cast<Real>(1e-6);
        Real scale0;
        Real scale1;

        if (dot > static_cast<Real>(1) - epsilon)
        {
            scale0 = static_cast<Real>(1) - amount;
            scale1 = amount;
        }
        else
        {
            const Real omega = StableMath::Acos(dot);
            const Real inverseSinOmega = static_cast<Real>(1) / StableMath::Sin(omega);
            scale0 = StableMath::Sin((static_cast<Real>(1) - amount) * omega) * inverseSinOmega;
            scale1 = StableMath::Sin(amount * omega) * inverseSinOmega;
        }

        return JQuaternion(
            scale0 * left.X + scale1 * target.X,
            scale0 * left.Y + scale1 * target.Y,
            scale0 * left.Z + scale1 * target.Z,
            scale0 * left.W + scale1 * target.W);
    }

    // Calculates the transformation of the X-axis (1, 0, 0) by this quaternion.
    // Mathematically equivalent to q · (1,0,0) · q⁻¹.
    // Result: [1 - 2(y² + z²), 2(xy + zw), 2(xz - yw)]
    // Returns: The transformed vector.
    JVector GetBasisX() const
    {
        return JVector(
            static_cast<Real>(1) - static_cast<Real>(2) * (Y * Y + Z * Z),
            static_cast<Real>(2) * (X * Y + Z * W),
            static_cast<Real>(2) * (X * Z - Y * W));
    }

    // Calculates the transformation of the Y-axis (0, 1, 0) by this quaternion.
    // Mathematically equivalent to q · (0,1,0) · q⁻¹.
    // Result: [2(xy - zw), 1 - 2(x² + z²), 2(yz + xw)]
    // Returns: The transformed vector.
    JVector GetBasisY() const
    {
        return JVector(
            static_cast<Real>(2) * (X * Y - Z * W),
            static_cast<Real>(1) - static_cast<Real>(2) * (X * X + Z * Z),
            static_cast<Real>(2) * (Y * Z + X * W));
    }

    // Calculates the transformation of the Z-axis (0, 0, 1) by this quaternion.
    // Mathematically equivalent to q · (0,0,1) · q⁻¹.
    // Result: [2(xz + yw), 2(yz - xw), 1 - 2(x² + y²)]
    // Returns: The transformed vector.
    JVector GetBasisZ() const
    {
        return JVector(
            static_cast<Real>(2) * (X * Z + Y * W),
            static_cast<Real>(2) * (Y * Z - X * W),
            static_cast<Real>(1) - static_cast<Real>(2) * (X * X + Y * Y));
    }

    // Creates a quaternion representing a rotation around the X-axis.
    // radians: The angle of rotation in radians.
    // Returns: The resulting quaternion.
    static JQuaternion CreateRotationX(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians * static_cast<Real>(0.5));
        return JQuaternion(s, 0, 0, c);
    }

    // Creates a quaternion representing a rotation around the Y-axis.
    // radians: The angle of rotation in radians.
    // Returns: The resulting quaternion.
    static JQuaternion CreateRotationY(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians * static_cast<Real>(0.5));
        return JQuaternion(0, s, 0, c);
    }

    // Creates a quaternion representing a rotation around the Z-axis.
    // radians: The angle of rotation in radians.
    // Returns: The resulting quaternion.
    static JQuaternion CreateRotationZ(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians * static_cast<Real>(0.5));
        return JQuaternion(0, 0, s, c);
    }

    // Creates a quaternion from a rotation matrix.
    // matrix: The rotation matrix.
    // Returns: The quaternion representing the rotation.
    static JQuaternion CreateFromMatrix(const JMatrix& matrix);

    // Creates a quaternion from a rotation matrix.
    // matrix: The rotation matrix.
    // result: Output: The quaternion representing the rotation.
    static void CreateFromMatrix(const JMatrix& matrix, JQuaternion& result);
};

static_assert(sizeof(JQuaternion) == 4 * sizeof(Real));

constexpr bool operator==(const JQuaternion& left, const JQuaternion& right)
{
    return left.X == right.X && left.Y == right.Y && left.Z == right.Z && left.W == right.W;
}

constexpr bool operator!=(const JQuaternion& left, const JQuaternion& right)
{
    return !(left == right);
}

// Adds two quaternions component-wise.
constexpr JQuaternion operator+(const JQuaternion& left, const JQuaternion& right)
{
    return JQuaternion::Add(left, right);
}

// Flips the sign of each component of the quaternion.
constexpr JQuaternion operator-(const JQuaternion& left, const JQuaternion& right)
{
    return JQuaternion::Subtract(left, right);
}

// Subtracts the second quaternion from the first component-wise.
constexpr JQuaternion operator-(const JQuaternion& value)
{
    return JQuaternion(-value.X, -value.Y, -value.Z, -value.W);
}

// Multiplies two quaternions (Hamilton Product).
constexpr JQuaternion operator*(const JQuaternion& left, const JQuaternion& right)
{
    return JQuaternion::Multiply(left, right);
}

// Scales a quaternion by a factor.
constexpr JQuaternion operator*(JQuaternion value, Real scale)
{
    return JQuaternion::Multiply(value, scale);
}

// Scales a quaternion by a factor.
constexpr JQuaternion operator*(Real scale, JQuaternion value)
{
    return value * scale;
}

} // namespace Jitter2::LinearMath
