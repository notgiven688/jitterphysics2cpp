#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::Real;
using Jitter2::Collision::SimplexSolver;
using Jitter2::Collision::SimplexSolverAB;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::Collision::Shapes::SphereShape;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

void RequireCloseVector(
    const JVector& actual,
    const JVector& expected,
    Real epsilonSq,
    const char* message)
{
    const JVector delta = actual - expected;
    if (delta.LengthSquared() >= epsilonSq)
    {
        throw std::runtime_error(std::string(message) + ": actual=("
            + std::to_string(actual.X) + ", " + std::to_string(actual.Y) + ", " + std::to_string(actual.Z)
            + ") expected=("
            + std::to_string(expected.X) + ", " + std::to_string(expected.Y) + ", " + std::to_string(expected.Z)
            + ")");
    }
}

void SupportPrimitiveBasics()
{
    auto sphere = Jitter2::Collision::SupportPrimitives::CreateSphere(static_cast<Real>(2));
    JVector support;
    sphere.SupportMap(JVector(3, 0, 0), support);
    Require(support == JVector(2, 0, 0), "support sphere");

    auto box = Jitter2::Collision::SupportPrimitives::CreateBox(JVector(1, 2, 3));
    box.SupportMap(JVector(-4, 5, -6), support);
    Require(support == JVector(-1, 2, -3), "support box");

    auto capsule = Jitter2::Collision::SupportPrimitives::CreateCapsule(static_cast<Real>(0.5), static_cast<Real>(2));
    capsule.SupportMap(JVector(0, 3, 0), support);
    Require(support == JVector(0, static_cast<Real>(2.5), 0), "support capsule");

    auto cylinder = Jitter2::Collision::SupportPrimitives::CreateCylinder(static_cast<Real>(2), static_cast<Real>(3));
    cylinder.SupportMap(JVector(4, 1, 0), support);
    Require(support == JVector(2, 3, 0), "support cylinder");

    auto cone = Jitter2::Collision::SupportPrimitives::CreateCone(static_cast<Real>(2), static_cast<Real>(4));
    cone.SupportMap(JVector(0, 3, 0), support);
    Require(support == JVector(0, 3, 0), "support cone");
}

void MinkowskiSupportUsesBodyBTransform()
{
    auto point = Jitter2::Collision::SupportPrimitives::CreatePoint();
    auto sphere = Jitter2::Collision::SupportPrimitives::CreateSphere(static_cast<Real>(1));

    Jitter2::Collision::MinkowskiDifference::Vertex vertex;
    Jitter2::Collision::MinkowskiDifference::Support(
        point,
        sphere,
        JQuaternion::Identity(),
        JVector(3, 0, 0),
        JVector(1, 0, 0),
        vertex);

    Require(vertex.A == JVector::Zero(), "minkowski point support");
    Require(vertex.B == JVector(2, 0, 0), "minkowski sphere support");
    Require(vertex.V == JVector(-2, 0, 0), "minkowski difference");
}

void SimplexSolverReducesSegment()
{
    SimplexSolver solver;
    solver.Reset();

    JVector closest;
    Require(solver.AddVertex(JVector(-1, 0, 0), closest), "simplex first point");
    Require(closest == JVector(-1, 0, 0), "simplex first closest");
    Require(solver.AddVertex(JVector(1, 0, 0), closest), "simplex segment");
    RequireClose(closest.X, static_cast<Real>(0), static_cast<Real>(1e-6), "simplex segment closest x");
    RequireClose(closest.Y, static_cast<Real>(0), static_cast<Real>(1e-6), "simplex segment closest y");
    RequireClose(closest.Z, static_cast<Real>(0), static_cast<Real>(1e-6), "simplex segment closest z");
}

void SimplexSolverABReconstructsClosestPoints()
{
    SimplexSolverAB solver;
    solver.Reset();

    SimplexSolverAB::Vertex first;
    first.A = JVector(0, 0, 0);
    first.B = JVector(1, 0, 0);
    first.V = first.A - first.B;

    SimplexSolverAB::Vertex second;
    second.A = JVector(2, 0, 0);
    second.B = JVector(1, 0, 0);
    second.V = second.A - second.B;

    JVector closest;
    solver.AddVertex(first, closest);
    solver.AddVertex(second, closest);

    JVector pointA;
    JVector pointB;
    solver.GetClosest(pointA, pointB);

    RequireClose(closest.X, static_cast<Real>(0), static_cast<Real>(1e-6), "simplex ab closest");
    RequireClose(pointA.X, static_cast<Real>(1), static_cast<Real>(1e-6), "simplex ab point a");
    RequireClose(pointB.X, static_cast<Real>(1), static_cast<Real>(1e-6), "simplex ab point b");
}

void SupportMapDistanceMatchesReferenceScenario()
{
    BoxShape box(static_cast<Real>(1));
    SphereShape sphere(static_cast<Real>(1));

    const JQuaternion orientationA = JQuaternion::Identity();
    const JQuaternion orientationB = JQuaternion::CreateRotationX(static_cast<Real>(0.2));
    const JVector positionA = JVector::Zero();
    const JVector positionB = JVector::UnitY() * static_cast<Real>(3);

    Require(!Jitter2::Collision::NarrowPhase::Overlap(
        box, sphere, orientationA, orientationB, positionA, positionB), "box sphere separated");

    JVector pointA;
    JVector pointB;
    JVector normal;
    Real distance;

    Require(Jitter2::Collision::NarrowPhase::Distance(
        box, sphere, orientationA, orientationB, positionA, positionB,
        pointA, pointB, normal, distance), "box sphere distance");

    RequireClose(distance, static_cast<Real>(1.5), static_cast<Real>(1e-4), "box sphere distance value");
    RequireCloseVector(pointA, JVector(0, static_cast<Real>(0.5), 0), static_cast<Real>(1e-4), "box sphere point a");
    RequireCloseVector(pointB, JVector(0, static_cast<Real>(2), 0), static_cast<Real>(1e-4), "box sphere point b");
    RequireClose(normal.X, static_cast<Real>(0), static_cast<Real>(1e-4), "box sphere normal x");
    RequireClose(normal.Y, static_cast<Real>(1), static_cast<Real>(1e-4), "box sphere normal y");
    RequireClose(normal.Z, static_cast<Real>(0), static_cast<Real>(1e-4), "box sphere normal z");
}

void SupportMapOverlapMatchesReferenceScenario()
{
    BoxShape box(static_cast<Real>(1));
    SphereShape sphere(static_cast<Real>(1));

    const JQuaternion orientationA = JQuaternion::Identity();
    const JQuaternion orientationB = JQuaternion::CreateRotationX(static_cast<Real>(0.2));
    const JVector positionA = JVector::Zero();
    const JVector positionB = JVector::UnitY() * static_cast<Real>(0.5);

    Require(Jitter2::Collision::NarrowPhase::Overlap(
        box, sphere, orientationA, orientationB, positionA, positionB), "box sphere overlap");

    JVector pointA;
    JVector pointB;
    JVector normal;
    Real distance;
    Require(!Jitter2::Collision::NarrowPhase::Distance(
        box, sphere, orientationA, orientationB, positionA, positionB,
        pointA, pointB, normal, distance), "overlapping box sphere has no distance");
    RequireClose(distance, static_cast<Real>(0), static_cast<Real>(1e-6), "overlap distance");
}

void SphereDistanceReconstructsClosestPoints()
{
    SphereShape sphereA(static_cast<Real>(1));
    SphereShape sphereB(static_cast<Real>(1));

    const JVector delta(10, 13, -22);
    const JVector expectedNormal = JVector::Normalize(delta);

    JVector pointA;
    JVector pointB;
    JVector normal;
    Real distance;

    Require(Jitter2::Collision::NarrowPhase::Distance(
        sphereA,
        sphereB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector::Zero(),
        delta,
        pointA,
        pointB,
        normal,
        distance), "sphere sphere distance");

    RequireClose(distance, delta.Length() - static_cast<Real>(2), static_cast<Real>(1e-4), "sphere distance value");
    RequireCloseVector(pointA, expectedNormal, static_cast<Real>(1e-4), "sphere point a");
    RequireCloseVector(pointB, delta - expectedNormal, static_cast<Real>(1e-4), "sphere point b");
    RequireClose(normal.X, expectedNormal.X, static_cast<Real>(1e-4), "sphere normal x");
    RequireClose(normal.Y, expectedNormal.Y, static_cast<Real>(1e-4), "sphere normal y");
    RequireClose(normal.Z, expectedNormal.Z, static_cast<Real>(1e-4), "sphere normal z");
}

void PointRayAndSweepQueries()
{
    SphereShape sphere(static_cast<Real>(1));

    Require(Jitter2::Collision::NarrowPhase::PointTest(sphere, JVector(0, 0, 0)), "point inside sphere");
    Require(!Jitter2::Collision::NarrowPhase::PointTest(sphere, JVector(2, 0, 0)), "point outside sphere");

    Real lambda;
    JVector normal;
    Require(Jitter2::Collision::NarrowPhase::RayCast(
        sphere, JVector(-3, 0, 0), JVector(1, 0, 0), lambda, normal), "ray hits sphere");
    RequireClose(lambda, static_cast<Real>(2), static_cast<Real>(1e-4), "ray lambda");
    RequireClose(normal.X, static_cast<Real>(-1), static_cast<Real>(1e-4), "ray normal x");

    SphereShape moving(static_cast<Real>(1));
    JVector pointA;
    JVector pointB;
    JVector sweepNormal;
    Real sweepLambda;
    Require(Jitter2::Collision::NarrowPhase::Sweep(
        sphere,
        moving,
        JQuaternion::Identity(),
        JVector(4, 0, 0),
        JVector(-4, 0, 0),
        pointA,
        pointB,
        sweepNormal,
        sweepLambda), "linear sweep hits sphere");
    RequireClose(sweepLambda, static_cast<Real>(0.5), static_cast<Real>(1e-4), "sweep lambda");
}

void MprEpaAndCollisionNormalDirection()
{
    SphereShape sphereA(static_cast<Real>(0.5));
    SphereShape sphereB(static_cast<Real>(0.5));

    JVector pointA;
    JVector pointB;
    JVector normal;
    Real penetration;

    Require(Jitter2::Collision::NarrowPhase::MprEpa(
        sphereA,
        sphereB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector(static_cast<Real>(-0.25), 0, 0),
        JVector(static_cast<Real>(0.25), 0, 0),
        pointA,
        pointB,
        normal,
        penetration), "mpr epa sphere hit");

    Require(pointA.X > static_cast<Real>(0), "mpr epa sphere point a side");
    Require(pointB.X < static_cast<Real>(0), "mpr epa sphere point b side");
    Require(normal.X > static_cast<Real>(0), "mpr epa sphere normal");
    Require(penetration > static_cast<Real>(0), "mpr epa sphere penetration");

    Require(Jitter2::Collision::NarrowPhase::Collision(
        sphereA,
        sphereB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector(static_cast<Real>(-0.25), 0, 0),
        JVector(static_cast<Real>(0.25), 0, 0),
        pointA,
        pointB,
        normal,
        penetration), "epa sphere hit");

    Require(pointA.X > static_cast<Real>(0), "epa sphere point a side");
    Require(pointB.X < static_cast<Real>(0), "epa sphere point b side");
    Require(normal.X > static_cast<Real>(0), "epa sphere normal");
    Require(penetration > static_cast<Real>(0), "epa sphere penetration");

    BoxShape boxA(static_cast<Real>(1));
    BoxShape boxB(static_cast<Real>(1));

    Require(Jitter2::Collision::NarrowPhase::MprEpa(
        boxA,
        boxB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector(static_cast<Real>(-0.25), static_cast<Real>(0.1), 0),
        JVector(static_cast<Real>(0.25), static_cast<Real>(-0.1), 0),
        pointA,
        pointB,
        normal,
        penetration), "mpr epa box hit");

    Require(pointA.X > static_cast<Real>(0), "mpr epa box point a side");
    Require(pointB.X < static_cast<Real>(0), "mpr epa box point b side");
    Require(normal.X > static_cast<Real>(0), "mpr epa box normal");
    Require(penetration > static_cast<Real>(0), "mpr epa box penetration");

    Require(Jitter2::Collision::NarrowPhase::Collision(
        boxA,
        boxB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector(static_cast<Real>(-2.25), 0, 0),
        JVector(static_cast<Real>(2.25), 0, 0),
        pointA,
        pointB,
        normal,
        penetration), "epa separated boxes");

    Require(normal.X > static_cast<Real>(0), "epa separated box normal");
    Require(penetration < static_cast<Real>(0), "epa separated box penetration");
}

void EqualFaceBoxesProduceFourManifoldContacts()
{
    BoxShape shapeA(JVector(2, 2, 2));
    BoxShape shapeB(JVector(2, 2, 2));

    JVector pointA;
    JVector pointB;
    JVector normal;
    Real penetration;

    Require(Jitter2::Collision::NarrowPhase::Collision(
        shapeA,
        shapeB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector::Zero(),
        JVector(0, static_cast<Real>(1.9), 0),
        pointA,
        pointB,
        normal,
        penetration), "face boxes collision");

    Jitter2::Collision::CollisionManifold manifold;
    manifold.BuildManifold(
        shapeA,
        shapeB,
        JQuaternion::Identity(),
        JQuaternion::Identity(),
        JVector::Zero(),
        JVector(0, static_cast<Real>(1.9), 0),
        pointA,
        pointB,
        normal);

    constexpr Real epsilon = static_cast<Real>(1e-4);

    Require(manifold.Count() == 4, "face boxes manifold count");

    for (int i = 0; i < manifold.Count(); ++i)
    {
        const JVector manifoldA = manifold.ManifoldA(i);
        const JVector manifoldB = manifold.ManifoldB(i);

        RequireClose(std::abs(manifoldA.Y - static_cast<Real>(1)), static_cast<Real>(0), epsilon, "manifold a y");
        RequireClose(std::abs(manifoldB.Y - static_cast<Real>(0.9)), static_cast<Real>(0), epsilon, "manifold b y");
        RequireClose(std::abs(std::abs(manifoldA.X) - static_cast<Real>(1)), static_cast<Real>(0), epsilon, "manifold a x");
        RequireClose(std::abs(std::abs(manifoldA.Z) - static_cast<Real>(1)), static_cast<Real>(0), epsilon, "manifold a z");
        RequireClose(
            std::abs(JVector::Dot(manifoldA - manifoldB, normal) - penetration),
            static_cast<Real>(0),
            epsilon,
            "manifold penetration");

        for (int j = i + 1; j < manifold.Count(); ++j)
        {
            Require(
                (manifoldA - manifold.ManifoldA(j)).LengthSquared() > epsilon * epsilon,
                "manifold contacts unique");
        }
    }
}

} // namespace

JITTER_TEST_CASE("Support primitives basics")
{
    SupportPrimitiveBasics();
}

JITTER_TEST_CASE("Minkowski support transforms shape B")
{
    MinkowskiSupportUsesBodyBTransform();
}

JITTER_TEST_CASE("Simplex solver reduces segment")
{
    SimplexSolverReducesSegment();
}

JITTER_TEST_CASE("Simplex solver AB reconstructs closest points")
{
    SimplexSolverABReconstructsClosestPoints();
}

JITTER_TEST_CASE("Support map distance matches reference scenario")
{
    SupportMapDistanceMatchesReferenceScenario();
}

JITTER_TEST_CASE("Support map overlap matches reference scenario")
{
    SupportMapOverlapMatchesReferenceScenario();
}

JITTER_TEST_CASE("Sphere distance reconstructs closest points")
{
    SphereDistanceReconstructsClosestPoints();
}

JITTER_TEST_CASE("Point, ray, and sweep narrowphase queries")
{
    PointRayAndSweepQueries();
}

JITTER_TEST_CASE("MPR EPA and Collision normal direction")
{
    MprEpaAndCollisionNormalDirection();
}

JITTER_TEST_CASE("Equal face boxes produce four manifold contacts")
{
    EqualFaceBoxesProduceFourManifoldContacts();
}
