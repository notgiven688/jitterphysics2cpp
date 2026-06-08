#pragma once

#include <cmath>
#include <limits>

#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>

namespace Jitter2::LinearMath
{

struct JMatrix
{
    Real M11;
    Real M21;
    Real M31;
    Real M12;
    Real M22;
    Real M32;
    Real M13;
    Real M23;
    Real M33;

    constexpr JMatrix()
        : M11(0), M21(0), M31(0),
          M12(0), M22(0), M32(0),
          M13(0), M23(0), M33(0)
    {
    }

    constexpr JMatrix(
        Real m11, Real m12, Real m13,
        Real m21, Real m22, Real m23,
        Real m31, Real m32, Real m33)
        : M11(m11), M21(m21), M31(m31),
          M12(m12), M22(m22), M32(m32),
          M13(m13), M23(m23), M33(m33)
    {
    }

    static constexpr JMatrix Zero() { return JMatrix(); }

    static constexpr JMatrix Identity()
    {
        return JMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    // Creates a matrix from three column vectors.
    // col1: The first column vector.
    // col2: The second column vector.
    // col3: The third column vector.
    static constexpr JMatrix FromColumns(const JVector& col1, const JVector& col2, const JVector& col3)
    {
        return JMatrix(
            col1.X, col2.X, col3.X,
            col1.Y, col2.Y, col3.Y,
            col1.Z, col2.Z, col3.Z);
    }

    // Gets a column vector by index.
    // index: The zero-based index of the column (0, 1, or 2).
    JVector GetColumn(int index) const
    {
        const Real* data = &M11;
        const int offset = index * 3;
        return JVector(data[offset], data[offset + 1], data[offset + 2]);
    }

    // Calculates the determinant of the matrix.
    // Returns: The determinant.
    Real Determinant() const
    {
        return M11 * (M22 * M33 - M23 * M32)
             - M12 * (M21 * M33 - M23 * M31)
             + M13 * (M21 * M32 - M22 * M31);
    }

    // Calculates the trace (sum of diagonal elements) of the matrix.
    // Returns: The trace of the matrix.
    Real Trace() const
    {
        return M11 + M22 + M33;
    }

    // Calculates the inverse of the matrix.
    // matrix: The matrix to invert.
    // result: Output: The inverted matrix, or a zero matrix if the determinant is zero.
    // Returns: true if the matrix can be inverted; otherwise, false.
    static bool Inverse(const JMatrix& matrix, JMatrix& result)
    {
        const Real determinant = matrix.Determinant();
        const Real inverseDeterminant = static_cast<Real>(1) / determinant;
        if (!std::isnormal(inverseDeterminant))
        {
            result = JMatrix::Zero();
            return false;
        }

        result.M11 = (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * inverseDeterminant;
        result.M12 = (matrix.M13 * matrix.M32 - matrix.M12 * matrix.M33) * inverseDeterminant;
        result.M13 = (matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13) * inverseDeterminant;

        result.M21 = (matrix.M23 * matrix.M31 - matrix.M33 * matrix.M21) * inverseDeterminant;
        result.M22 = (matrix.M11 * matrix.M33 - matrix.M31 * matrix.M13) * inverseDeterminant;
        result.M23 = (matrix.M13 * matrix.M21 - matrix.M23 * matrix.M11) * inverseDeterminant;

        result.M31 = (matrix.M21 * matrix.M32 - matrix.M31 * matrix.M22) * inverseDeterminant;
        result.M32 = (matrix.M12 * matrix.M31 - matrix.M32 * matrix.M11) * inverseDeterminant;
        result.M33 = (matrix.M11 * matrix.M22 - matrix.M21 * matrix.M12) * inverseDeterminant;

        return true;
    }

    // Multiplies two matrices.
    // matrix1: The first matrix.
    // matrix2: The second matrix.
    // Returns: The product of the two matrices.
    static JMatrix Multiply(const JMatrix& matrix1, const JMatrix& matrix2)
    {
        return JMatrix(
            matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M21 + matrix1.M13 * matrix2.M31,
            matrix1.M11 * matrix2.M12 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M32,
            matrix1.M11 * matrix2.M13 + matrix1.M12 * matrix2.M23 + matrix1.M13 * matrix2.M33,
            matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M23 * matrix2.M31,
            matrix1.M21 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M32,
            matrix1.M21 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M23 * matrix2.M33,
            matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M21 + matrix1.M33 * matrix2.M31,
            matrix1.M31 * matrix2.M12 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M32,
            matrix1.M31 * matrix2.M13 + matrix1.M32 * matrix2.M23 + matrix1.M33 * matrix2.M33);
    }

    // Multiplies two matrices.
    // matrix1: The first matrix.
    // matrix2: The second matrix.
    // result: Output: The product of the two matrices.
    static void Multiply(const JMatrix& matrix1, const JMatrix& matrix2, JMatrix& result)
    {

        // Multiplies a matrix by a scalar factor.
        // matrix1: The matrix.
        // scaleFactor: The scalar factor.
        // Returns: The scaled matrix.
        result = Multiply(matrix1, matrix2);
    }

    // Adds two matrices.
    // matrix1: The first matrix.
    // matrix2: The second matrix.
    // Returns: The sum of the two matrices.
    static JMatrix Add(const JMatrix& matrix1, const JMatrix& matrix2)
    {
        return JMatrix(
            matrix1.M11 + matrix2.M11, matrix1.M12 + matrix2.M12, matrix1.M13 + matrix2.M13,
            matrix1.M21 + matrix2.M21, matrix1.M22 + matrix2.M22, matrix1.M23 + matrix2.M23,
            matrix1.M31 + matrix2.M31, matrix1.M32 + matrix2.M32, matrix1.M33 + matrix2.M33);
    }

    // Adds two matrices component-wise.
    // matrix1: The first matrix.
    // matrix2: The second matrix.
    // result: Output: The sum of the two matrices.
    static void Add(const JMatrix& matrix1, const JMatrix& matrix2, JMatrix& result)
    {
        result = Add(matrix1, matrix2);
    }

    static JMatrix Subtract(const JMatrix& matrix1, const JMatrix& matrix2)
    {
        return JMatrix(
            matrix1.M11 - matrix2.M11, matrix1.M12 - matrix2.M12, matrix1.M13 - matrix2.M13,
            matrix1.M21 - matrix2.M21, matrix1.M22 - matrix2.M22, matrix1.M23 - matrix2.M23,
            matrix1.M31 - matrix2.M31, matrix1.M32 - matrix2.M32, matrix1.M33 - matrix2.M33);
    }

    // Subtracts the second matrix from the first component-wise.
    // matrix1: The first matrix.
    // matrix2: The second matrix.
    // result: Output: The difference of the two matrices.
    static void Subtract(const JMatrix& matrix1, const JMatrix& matrix2, JMatrix& result)
    {
        result = Subtract(matrix1, matrix2);
    }

    static JMatrix Multiply(const JMatrix& matrix, Real scale)
    {
        return JMatrix(
            matrix.M11 * scale, matrix.M12 * scale, matrix.M13 * scale,
            matrix.M21 * scale, matrix.M22 * scale, matrix.M23 * scale,
            matrix.M31 * scale, matrix.M32 * scale, matrix.M33 * scale);
    }

    // Multiplies a matrix by a scalar factor.
    // matrix1: The matrix.
    // scaleFactor: The scalar factor.
    // result: Output: The scaled matrix.
    static void Multiply(const JMatrix& matrix, Real scale, JMatrix& result)
    {
        result = Multiply(matrix, scale);
    }

    // Calculates matrix1 * transpose(matrix2).
    // matrix1: The first matrix.
    // matrix2: The second matrix (which will be transposed during multiplication).
    // Returns: The result of the multiplication.
    static JMatrix MultiplyTransposed(const JMatrix& matrix1, const JMatrix& matrix2)
    {
        return JMatrix(
            matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M12 + matrix1.M13 * matrix2.M13,
            matrix1.M11 * matrix2.M21 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M23,
            matrix1.M11 * matrix2.M31 + matrix1.M12 * matrix2.M32 + matrix1.M13 * matrix2.M33,
            matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M12 + matrix1.M23 * matrix2.M13,
            matrix1.M21 * matrix2.M21 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M23,
            matrix1.M21 * matrix2.M31 + matrix1.M22 * matrix2.M32 + matrix1.M23 * matrix2.M33,
            matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M12 + matrix1.M33 * matrix2.M13,
            matrix1.M31 * matrix2.M21 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M23,
            matrix1.M31 * matrix2.M31 + matrix1.M32 * matrix2.M32 + matrix1.M33 * matrix2.M33);
    }

    // Calculates matrix1 * matrix2ᵀ (multiplying matrix1 by the transpose of matrix2).
    // matrix1: The first matrix.
    // matrix2: The second matrix (transposed during operation).
    // result: Output: The result of the multiplication.
    static void MultiplyTransposed(const JMatrix& matrix1, const JMatrix& matrix2, JMatrix& result)
    {
        result = MultiplyTransposed(matrix1, matrix2);
    }

    // Calculates transpose(matrix1) * matrix2.
    // matrix1: The first matrix (which will be transposed during multiplication).
    // matrix2: The second matrix.
    // Returns: The result of the multiplication.
    static JMatrix TransposedMultiply(const JMatrix& matrix1, const JMatrix& matrix2)
    {
        return JMatrix(
            matrix1.M11 * matrix2.M11 + matrix1.M21 * matrix2.M21 + matrix1.M31 * matrix2.M31,
            matrix1.M11 * matrix2.M12 + matrix1.M21 * matrix2.M22 + matrix1.M31 * matrix2.M32,
            matrix1.M11 * matrix2.M13 + matrix1.M21 * matrix2.M23 + matrix1.M31 * matrix2.M33,
            matrix1.M12 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M32 * matrix2.M31,
            matrix1.M12 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M32 * matrix2.M32,
            matrix1.M12 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M32 * matrix2.M33,
            matrix1.M13 * matrix2.M11 + matrix1.M23 * matrix2.M21 + matrix1.M33 * matrix2.M31,
            matrix1.M13 * matrix2.M12 + matrix1.M23 * matrix2.M22 + matrix1.M33 * matrix2.M32,
            matrix1.M13 * matrix2.M13 + matrix1.M23 * matrix2.M23 + matrix1.M33 * matrix2.M33);
    }

    // Calculates matrix1ᵀ * matrix2 (multiplying the transpose of matrix1 by matrix2).
    // matrix1: The first matrix (transposed during operation).
    // matrix2: The second matrix.
    // result: Output: The result of the multiplication.
    static void TransposedMultiply(const JMatrix& matrix1, const JMatrix& matrix2, JMatrix& result)
    {
        result = TransposedMultiply(matrix1, matrix2);
    }

    // Transposes a matrix.
    // matrix: The matrix to transpose.
    // Returns: The transposed matrix.
    static JMatrix Transpose(const JMatrix& matrix)
    {
        return JMatrix(
            matrix.M11, matrix.M21, matrix.M31,
            matrix.M12, matrix.M22, matrix.M32,
            matrix.M13, matrix.M23, matrix.M33);
    }

    // Creates a skew-symmetric matrix from a vector, representing the cross product operation.
    // Result is equivalent to: [ 0 -z y ]
    // [ z 0 -x ]
    // [ -y x 0 ]
    // vec: The vector.
    // Returns: The skew-symmetric matrix.
    static JMatrix CreateCrossProduct(const JVector& vector)
    {
        return JMatrix(
            0, -vector.Z, vector.Y,
            vector.Z, 0, -vector.X,
            -vector.Y, vector.X, 0);
    }

    // Creates a rotation matrix from an axis and an angle.
    // axis: The axis to rotate around.
    // angle: The angle of rotation in radians.
    // Returns: The rotation matrix.
    static JMatrix CreateRotationMatrix(JVector axis, Real angle)
    {
        const auto [s, c] = StableMath::SinCos(angle / static_cast<Real>(2));
        axis *= s;

        // Creates a rotation matrix from a quaternion.
        // quaternion: The quaternion representing the rotation.
        // Returns: The rotation matrix.
        return CreateFromQuaternion(JQuaternion(axis.X, axis.Y, axis.Z, c));
    }

    // Creates a scaling matrix.
    // scale: The scaling vector.
    // Returns: The scaling matrix.
    static JMatrix CreateScale(const JVector& scale)
    {
        JMatrix result = Zero();
        result.M11 = scale.X;
        result.M22 = scale.Y;
        result.M33 = scale.Z;
        return result;
    }

    // Creates a scaling matrix.
    // x: Scaling factor on the X-axis.
    // y: Scaling factor on the Y-axis.
    // z: Scaling factor on the Z-axis.
    // Returns: The scaling matrix.
    static JMatrix CreateScale(Real x, Real y, Real z)
    {
        return CreateScale(JVector(x, y, z));
    }

    static JVector Transform(const JVector& vector, const JMatrix& matrix)
    {
        return JVector(
            vector.X * matrix.M11 + vector.Y * matrix.M12 + vector.Z * matrix.M13,
            vector.X * matrix.M21 + vector.Y * matrix.M22 + vector.Z * matrix.M23,
            vector.X * matrix.M31 + vector.Y * matrix.M32 + vector.Z * matrix.M33);
    }

    static JMatrix Outer(const JVector& left, const JVector& right)
    {
        return JMatrix(
            left.X * right.X, left.X * right.Y, left.X * right.Z,
            left.Y * right.X, left.Y * right.Y, left.Y * right.Z,
            left.Z * right.X, left.Z * right.Y, left.Z * right.Z);
    }

    static JVector TransposedTransform(const JVector& vector, const JMatrix& matrix)
    {
        return JVector(
            vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31,
            vector.X * matrix.M12 + vector.Y * matrix.M22 + vector.Z * matrix.M32,
            vector.X * matrix.M13 + vector.Y * matrix.M23 + vector.Z * matrix.M33);
    }

    static JMatrix Absolute(const JMatrix& matrix)
    {
        return JMatrix(
            std::abs(matrix.M11), std::abs(matrix.M12), std::abs(matrix.M13),
            std::abs(matrix.M21), std::abs(matrix.M22), std::abs(matrix.M23),
            std::abs(matrix.M31), std::abs(matrix.M32), std::abs(matrix.M33));
    }

    // Creates a matrix where each component is the absolute value of the input matrix component.
    // matrix: The input matrix.
    // result: Output: The absolute matrix.
    static void Absolute(const JMatrix& matrix, JMatrix& result)
    {
        result = Absolute(matrix);
    }

    // Creates a rotation matrix around the X-axis.
    // radians: The angle of rotation in radians.
    // Returns: The rotation matrix.
    static JMatrix CreateRotationX(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians);
        return JMatrix(1, 0, 0, 0, c, -s, 0, s, c);
    }

    // Creates a rotation matrix around the Y-axis.
    // radians: The angle of rotation in radians.
    // Returns: The rotation matrix.
    static JMatrix CreateRotationY(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians);
        return JMatrix(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    // Creates a rotation matrix around the Z-axis.
    // radians: The angle of rotation in radians.
    // Returns: The rotation matrix.
    static JMatrix CreateRotationZ(Real radians)
    {
        const auto [s, c] = StableMath::SinCos(radians);
        return JMatrix(c, -s, 0, s, c, 0, 0, 0, 1);
    }

    static JMatrix CreateFromQuaternion(const JQuaternion& quaternion)
    {
        const Real xx = quaternion.X * quaternion.X;
        const Real yy = quaternion.Y * quaternion.Y;
        const Real zz = quaternion.Z * quaternion.Z;
        const Real xy = quaternion.X * quaternion.Y;
        const Real zw = quaternion.Z * quaternion.W;
        const Real zx = quaternion.Z * quaternion.X;
        const Real yw = quaternion.Y * quaternion.W;
        const Real yz = quaternion.Y * quaternion.Z;
        const Real xw = quaternion.X * quaternion.W;

        return JMatrix(
            static_cast<Real>(1) - static_cast<Real>(2) * (yy + zz),
            static_cast<Real>(2) * (xy - zw),
            static_cast<Real>(2) * (zx + yw),
            static_cast<Real>(2) * (xy + zw),
            static_cast<Real>(1) - static_cast<Real>(2) * (zz + xx),
            static_cast<Real>(2) * (yz - xw),
            static_cast<Real>(2) * (zx - yw),
            static_cast<Real>(2) * (yz + xw),
            static_cast<Real>(1) - static_cast<Real>(2) * (yy + xx));
    }

    // Creates a rotation matrix from a quaternion.
    // quaternion: The quaternion representing the rotation.
    // result: Output: The rotation matrix.
    static void CreateFromQuaternion(const JQuaternion& quaternion, JMatrix& result)
    {
        result = CreateFromQuaternion(quaternion);
    }
};

static_assert(sizeof(JMatrix) == 9 * sizeof(Real));

constexpr bool operator==(const JMatrix& left, const JMatrix& right)
{
    return left.M11 == right.M11 && left.M12 == right.M12 && left.M13 == right.M13
        && left.M21 == right.M21 && left.M22 == right.M22 && left.M23 == right.M23
        && left.M31 == right.M31 && left.M32 == right.M32 && left.M33 == right.M33;
}

constexpr bool operator!=(const JMatrix& left, const JMatrix& right)
{
    return !(left == right);
}

// Adds two matrices.
inline JMatrix operator+(const JMatrix& left, const JMatrix& right)
{
    return JMatrix::Add(left, right);
}

// Subtracts the second matrix from the first.
inline JMatrix operator-(const JMatrix& left, const JMatrix& right)
{
    return JMatrix::Subtract(left, right);
}

// Multiplies two matrices.
inline JMatrix operator*(const JMatrix& left, const JMatrix& right)
{
    return JMatrix::Multiply(left, right);
}

// Scales a matrix by a factor.
inline JVector operator*(const JMatrix& matrix, const JVector& vector)
{
    return JVector(
        matrix.M11 * vector.X + matrix.M12 * vector.Y + matrix.M13 * vector.Z,
        matrix.M21 * vector.X + matrix.M22 * vector.Y + matrix.M23 * vector.Z,
        matrix.M31 * vector.X + matrix.M32 * vector.Y + matrix.M33 * vector.Z);
}

// Scales a matrix by a factor.
inline JMatrix operator*(const JMatrix& matrix, Real scale)
{
    return JMatrix::Multiply(matrix, scale);
}

inline JMatrix operator*(Real scale, const JMatrix& matrix)
{
    return matrix * scale;
}

inline JQuaternion JQuaternion::CreateFromMatrix(const JMatrix& matrix)
{
    Real t;
    JQuaternion result;

    if (matrix.M33 < static_cast<Real>(0))
    {
        if (matrix.M11 > matrix.M22)
        {
            t = static_cast<Real>(1) + matrix.M11 - matrix.M22 - matrix.M33;
            result = JQuaternion(t, matrix.M21 + matrix.M12, matrix.M31 + matrix.M13, matrix.M32 - matrix.M23);
        }
        else
        {
            t = static_cast<Real>(1) - matrix.M11 + matrix.M22 - matrix.M33;
            result = JQuaternion(matrix.M21 + matrix.M12, t, matrix.M32 + matrix.M23, matrix.M13 - matrix.M31);
        }
    }
    else
    {
        if (matrix.M11 < -matrix.M22)
        {
            t = static_cast<Real>(1) - matrix.M11 - matrix.M22 + matrix.M33;
            result = JQuaternion(matrix.M13 + matrix.M31, matrix.M32 + matrix.M23, t, matrix.M21 - matrix.M12);
        }
        else
        {
            t = static_cast<Real>(1) + matrix.M11 + matrix.M22 + matrix.M33;
            result = JQuaternion(matrix.M32 - matrix.M23, matrix.M13 - matrix.M31, matrix.M21 - matrix.M12, t);
        }
    }

    t = static_cast<Real>(0.5) / std::sqrt(t);
    result.X *= t;
    result.Y *= t;
    result.Z *= t;
    result.W *= t;
    return result;
}

inline void JQuaternion::CreateFromMatrix(const JMatrix& matrix, JQuaternion& result)
{
    result = CreateFromMatrix(matrix);
}

} // namespace Jitter2::LinearMath
