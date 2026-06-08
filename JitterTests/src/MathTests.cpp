#include <cmath>

#include <Jitter2/Jitter2.hpp>
#include "TestSupport.hpp"

using Jitter2::Real;
using Jitter2::LinearMath::JAngle;
using Jitter2::LinearMath::JMatrix;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JTriangle;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

void VectorBasics()
{
    const JVector a(1, 2, 3);
    const JVector b(4, -5, 6);

    Require(a + b == JVector(5, -3, 9), "vector addition");
    Require(a - b == JVector(-3, 7, -3), "vector subtraction");
    Require(JVector::Cross(a, b) == JVector(27, 6, -13), "cross product");
    RequireClose(JVector::Dot(a, b), static_cast<Real>(12), static_cast<Real>(1e-6), "dot product");
    RequireClose(a.LengthSquared(), static_cast<Real>(14), static_cast<Real>(1e-6), "length squared");
}

void MatrixBasics()
{
    const JMatrix identity = JMatrix::Identity();
    const JVector value(2, -3, 5);
    Require(identity * value == value, "identity matrix transform");

    const JMatrix m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Require(m.GetColumn(0) == JVector(1, 4, 7), "matrix column 0");
    Require(m.GetColumn(1) == JVector(2, 5, 8), "matrix column 1");
    RequireClose(identity.Determinant(), static_cast<Real>(1), static_cast<Real>(1e-6), "identity determinant");
    RequireClose(m.Trace(), static_cast<Real>(15), static_cast<Real>(1e-6), "matrix trace");

    const JMatrix cross = JMatrix::CreateCrossProduct(JVector(1, 2, 3));
    Require(cross * JVector(4, 5, 6) == JVector(-3, 6, -3), "cross product matrix");

    const JMatrix outer = JMatrix::Outer(JVector(1, 2, 3), JVector(4, 5, 6));
    Require(outer.GetColumn(0) == JVector(4, 8, 12), "outer product column");

    const JVector transposed = JMatrix::TransposedTransform(JVector(1, 0, 0), m);
    Require(transposed == JVector(1, 2, 3), "transposed transform");
}

void MatrixInverse()
{
    const JMatrix matrix(
        static_cast<Real>(4), static_cast<Real>(7), static_cast<Real>(2),
        static_cast<Real>(3), static_cast<Real>(6), static_cast<Real>(1),
        static_cast<Real>(2), static_cast<Real>(5), static_cast<Real>(3));

    JMatrix inverse;
    Require(JMatrix::Inverse(matrix, inverse), "matrix inverse succeeds");
    const JMatrix product = matrix * inverse;

    RequireClose(product.M11, static_cast<Real>(1), static_cast<Real>(1e-5), "inverse product m11");
    RequireClose(product.M22, static_cast<Real>(1), static_cast<Real>(1e-5), "inverse product m22");
    RequireClose(product.M33, static_cast<Real>(1), static_cast<Real>(1e-5), "inverse product m33");
    RequireClose(product.M12, static_cast<Real>(0), static_cast<Real>(1e-5), "inverse product m12");
    RequireClose(product.M23, static_cast<Real>(0), static_cast<Real>(1e-5), "inverse product m23");

    JMatrix singularInverse;
    Require(!JMatrix::Inverse(JMatrix::Zero(), singularInverse), "singular inverse fails");
    Require(singularInverse == JMatrix::Zero(), "singular inverse result zero");
}

void QuaternionBasics()
{
    const JQuaternion identity = JQuaternion::Identity();
    Require(identity == JQuaternion(0, 0, 0, 1), "identity quaternion");

    const JQuaternion rot = JQuaternion::CreateRotationZ(static_cast<Real>(3.14159265358979323846 / 2.0));
    const JMatrix matrix = JMatrix::CreateFromQuaternion(rot);
    const JVector x = matrix * JVector::UnitX();

    RequireClose(x.X, static_cast<Real>(0), static_cast<Real>(1e-5), "rotation x component");
    RequireClose(x.Y, static_cast<Real>(1), static_cast<Real>(1e-5), "rotation y component");
    RequireClose(x.Z, static_cast<Real>(0), static_cast<Real>(1e-5), "rotation z component");

    const JVector transformed = JQuaternion::Transform(JVector::UnitX(), rot);
    RequireClose(transformed.X, static_cast<Real>(0), static_cast<Real>(1e-5), "quaternion transform x");
    RequireClose(transformed.Y, static_cast<Real>(1), static_cast<Real>(1e-5), "quaternion transform y");
    RequireClose(transformed.Z, static_cast<Real>(0), static_cast<Real>(1e-5), "quaternion transform z");

    const JVector restored = JQuaternion::ConjugatedTransform(transformed, rot);
    RequireClose(restored.X, static_cast<Real>(1), static_cast<Real>(1e-5), "quaternion conjugated transform x");
    RequireClose(restored.Y, static_cast<Real>(0), static_cast<Real>(1e-5), "quaternion conjugated transform y");
    RequireClose(restored.Z, static_cast<Real>(0), static_cast<Real>(1e-5), "quaternion conjugated transform z");
}

void TriangleBasics()
{
    const JTriangle triangle(JVector(0, 0, 0), JVector(1, 0, 0), JVector(0, 1, 0));

    RequireClose(triangle.GetArea(), static_cast<Real>(0.5), static_cast<Real>(1e-6), "triangle area");
    Require(triangle.GetCenter() == JVector(static_cast<Real>(1.0 / 3.0), static_cast<Real>(1.0 / 3.0), 0),
        "triangle center");
    Require(triangle.GetBoundingBox() == Jitter2::LinearMath::JBoundingBox(JVector(0), JVector(1, 1, 0)),
        "triangle bounds");

    JVector normal;
    Real lambda = 0;
    Require(triangle.RayIntersect(
        JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), 1),
        JVector(0, 0, -1),
        JTriangle::CullMode::None,
        normal,
        lambda),
        "triangle front ray hit");
    RequireClose(lambda, static_cast<Real>(1), static_cast<Real>(1e-6), "triangle ray lambda");
    Require(normal == JVector::UnitZ(), "triangle ray normal");

    Require(!triangle.RayIntersect(
        JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), -1),
        JVector(0, 0, 1),
        JTriangle::CullMode::BackFacing,
        normal,
        lambda),
        "triangle back face culled");

    const JVector closest = triangle.ClosestPoint(JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), 2));
    RequireClose(closest.X, static_cast<Real>(0.25), static_cast<Real>(1e-6), "triangle closest x");
    RequireClose(closest.Y, static_cast<Real>(0.25), static_cast<Real>(1e-6), "triangle closest y");
    RequireClose(closest.Z, static_cast<Real>(0), static_cast<Real>(1e-6), "triangle closest z");
}

void AngleBasics()
{
    const JAngle radians = JAngle::FromRadian(static_cast<Real>(0.5));
    RequireClose(radians.Radian, static_cast<Real>(0.5), static_cast<Real>(1e-6), "angle radians");

    JAngle degrees = JAngle::FromDegree(static_cast<Real>(180));
    RequireClose(degrees.Radian, static_cast<Real>(3.14159265358979323846), static_cast<Real>(1e-6), "angle from degrees");
    RequireClose(degrees.Degree(), static_cast<Real>(180), static_cast<Real>(1e-6), "angle degree getter");

    degrees.Degree(static_cast<Real>(90));
    RequireClose(degrees.Radian, static_cast<Real>(1.57079632679489661923), static_cast<Real>(1e-6), "angle degree setter");
    Require(JAngle::FromRadian(static_cast<Real>(1)) + JAngle::FromRadian(static_cast<Real>(2))
        == JAngle::FromRadian(static_cast<Real>(3)), "angle addition");
    Require(JAngle::FromRadian(static_cast<Real>(3)) - JAngle::FromRadian(static_cast<Real>(2))
        == JAngle::FromRadian(static_cast<Real>(1)), "angle subtraction");
    Require(-JAngle::FromRadian(static_cast<Real>(3)) == JAngle::FromRadian(static_cast<Real>(-3)), "angle negation");
    Require(JAngle::FromRadian(static_cast<Real>(1)) < JAngle::FromRadian(static_cast<Real>(2)), "angle comparison");
}

void WorldSmoke()
{
    Jitter2::World world;
    world.Step(static_cast<Real>(0.01), false);
    world.Step(static_cast<Real>(0.02), true);

    Require(world.StepCount() == 2, "world step count");
    RequireClose(world.Time(), static_cast<Real>(0.03), static_cast<Real>(1e-6), "world time");
    world.Clear();
    Require(world.StepCount() == 0, "world clear step count");
}

} // namespace

JITTER_TEST_CASE("JVector basics")
{
    VectorBasics();
}

JITTER_TEST_CASE("JMatrix basics")
{
    MatrixBasics();
}

JITTER_TEST_CASE("JMatrix inverse")
{
    MatrixInverse();
}

JITTER_TEST_CASE("JQuaternion basics")
{
    QuaternionBasics();
}

JITTER_TEST_CASE("JTriangle basics")
{
    TriangleBasics();
}

JITTER_TEST_CASE("JAngle basics")
{
    AngleBasics();
}

JITTER_TEST_CASE("World smoke")
{
    WorldSmoke();
}
