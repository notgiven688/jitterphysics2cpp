#include <algorithm>
#include <cmath>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::MotionType;
using Jitter2::Real;
using Jitter2::RigidBody;
using Jitter2::World;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::LinearMath::JVector;
using Jitter2::SoftBodies::SoftBody;
using Jitter2::SoftBodies::SoftBodyTetrahedron;
using Jitter2::SoftBodies::SoftBodyTriangle;
using Jitter2::SoftBodies::SpringConstraint;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

RigidBody& CreateVertex(World& world, const JVector& position)
{
    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(position);
    body.SetMassInertia(static_cast<Real>(1));
    return body;
}

void SpringConstraintConvergesToTargetDistance()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& bodyA = CreateVertex(world, JVector::Zero());
    RigidBody& bodyB = CreateVertex(world, JVector(4, 0, 0));

    SpringConstraint& spring = world.CreateConstraint<SpringConstraint>(bodyA, bodyB);
    spring.Initialize(bodyA.Position(), bodyB.Position());
    spring.TargetDistance(static_cast<Real>(2));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    RequireClose(spring.Distance(), static_cast<Real>(2), static_cast<Real>(0.08), "spring target distance");
    Require(bodyA.Position().X > static_cast<Real>(0.9), "spring moved body a");
    Require(bodyB.Position().X < static_cast<Real>(3.1), "spring moved body b");
}

void SoftBodyTriangleSupportBoundsAndClosest()
{
    World world;
    SoftBody softBody(world);

    RigidBody& v1 = CreateVertex(world, JVector(-1, 0, 0));
    RigidBody& v2 = CreateVertex(world, JVector(2, 0, 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, 1, 0));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);

    SoftBodyTriangle triangle(softBody, v1, v2, v3);
    triangle.Thickness(static_cast<Real>(0.2));
    softBody.AddShape(triangle);

    JVector support;
    triangle.SupportMap(JVector::UnitX(), support);

    Require(&triangle.GetClosest(JVector(static_cast<Real>(1.8), 0, 0)) == &v2, "triangle closest vertex");
    RequireClose(support.X, static_cast<Real>(2.1), static_cast<Real>(1e-5), "triangle support thickness");
    Require(triangle.WorldBoundingBox().Min.X < static_cast<Real>(-1), "triangle bounds min");
    Require(triangle.WorldBoundingBox().Max.X > static_cast<Real>(2), "triangle bounds max");
    Require(world.DynamicTree().Count() == 1, "soft triangle registered in dynamic tree");
}

void SoftBodyTetrahedronSupportBoundsAndClosest()
{
    World world;
    SoftBody softBody(world);

    RigidBody& v1 = CreateVertex(world, JVector(-1, 0, 0));
    RigidBody& v2 = CreateVertex(world, JVector(2, 0, 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, 1, 0));
    RigidBody& v4 = CreateVertex(world, JVector(0, 0, 3));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);
    softBody.AddVertex(v4);

    SoftBodyTetrahedron tetrahedron(softBody, v1, v2, v3, v4);
    softBody.AddShape(tetrahedron);

    JVector support;
    tetrahedron.SupportMap(JVector::UnitZ(), support);

    Require(&tetrahedron.GetClosest(JVector(0, 0, static_cast<Real>(2.7))) == &v4, "tetrahedron closest vertex");
    Require(support == v4.Position(), "tetrahedron support vertex");
    Require(tetrahedron.WorldBoundingBox().Max.Z > static_cast<Real>(3), "tetrahedron bounds max");
}

void SoftBodyDestroyRemovesComponents()
{
    World world;
    SoftBody softBody(world);

    RigidBody& v1 = CreateVertex(world, JVector::Zero());
    RigidBody& v2 = CreateVertex(world, JVector(1, 0, 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, 1, 0));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);

    SpringConstraint& spring = world.CreateConstraint<SpringConstraint>(v1, v2);
    spring.Initialize(v1.Position(), v2.Position());
    softBody.AddSpring(spring);

    SoftBodyTriangle triangle(softBody, v1, v2, v3);
    softBody.AddShape(triangle);

    softBody.Destroy();

    Require(world.DynamicTree().Count() == 0, "soft body destroy removes shape proxy");
    Require(world.Constraints().empty(), "soft body destroy removes springs");
    Require(world.RigidBodies().size() == 1, "soft body destroy removes vertices");
}

void SoftBodyTriangleCollidesWithRigidShape()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;
    world.DynamicTree().Filter(Jitter2::SoftBodies::DynamicTreeCollisionFilter::Filter);
    Jitter2::SoftBodies::BroadPhaseCollisionFilter broadPhaseFilter(world);
    world.BroadPhaseFilter = &broadPhaseFilter;

    RigidBody& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    floor.Position(JVector(0, static_cast<Real>(-0.5), 0));
    BoxShape floorShape(JVector(10, 1, 10));
    floor.AddShape(floorShape);

    SoftBody softBody(world);
    RigidBody& v1 = CreateVertex(world, JVector(static_cast<Real>(-0.25), static_cast<Real>(0.02), 0));
    RigidBody& v2 = CreateVertex(world, JVector(static_cast<Real>(0.25), static_cast<Real>(0.02), 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, static_cast<Real>(0.02), static_cast<Real>(0.25)));
    v1.Velocity(JVector(0, static_cast<Real>(-1), 0));
    v2.Velocity(JVector(0, static_cast<Real>(-1), 0));
    v3.Velocity(JVector(0, static_cast<Real>(-1), 0));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);

    SoftBodyTriangle triangle(softBody, v1, v2, v3);
    triangle.Thickness(static_cast<Real>(0.2));
    softBody.AddShape(triangle);

    world.Step(static_cast<Real>(0.1), false);

    const Real highestVertex = std::max({v1.Position().Y, v2.Position().Y, v3.Position().Y});
    Require(highestVertex > static_cast<Real>(-0.08), "soft body contact corrected a vertex");
}

void SoftBodyTriangleWithoutBroadPhaseFilterThrows()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    floor.Position(JVector(0, static_cast<Real>(-0.5), 0));
    BoxShape floorShape(JVector(10, 1, 10));
    floor.AddShape(floorShape);

    SoftBody softBody(world);
    RigidBody& v1 = CreateVertex(world, JVector(static_cast<Real>(-0.25), static_cast<Real>(0.02), 0));
    RigidBody& v2 = CreateVertex(world, JVector(static_cast<Real>(0.25), static_cast<Real>(0.02), 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, static_cast<Real>(0.02), static_cast<Real>(0.25)));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);

    SoftBodyTriangle triangle(softBody, v1, v2, v3);
    triangle.Thickness(static_cast<Real>(0.2));
    softBody.AddShape(triangle);

    bool threw = false;
    try
    {
        world.Step(static_cast<Real>(0.1), false);
    }
    catch (const World::InvalidCollisionTypeException&)
    {
        threw = true;
    }

    Require(threw, "unhandled soft body proxy pair throws like C# World.Detect");
}

void SoftBodyDynamicTreeCollisionFilterMatchesCSharpRules()
{
    World world;

    SoftBody first(world);
    RigidBody& a0 = CreateVertex(world, JVector(0, 0, 0));
    RigidBody& a1 = CreateVertex(world, JVector(1, 0, 0));
    RigidBody& a2 = CreateVertex(world, JVector(0, 1, 0));
    RigidBody& a3 = CreateVertex(world, JVector(0, 0, 1));
    first.AddVertex(a0);
    first.AddVertex(a1);
    first.AddVertex(a2);
    first.AddVertex(a3);

    SoftBodyTriangle firstTriangle(first, a0, a1, a2);
    SoftBodyTriangle secondFirstTriangle(first, a0, a2, a3);

    SoftBody second(world);
    RigidBody& b0 = CreateVertex(world, JVector(3, 0, 0));
    RigidBody& b1 = CreateVertex(world, JVector(4, 0, 0));
    RigidBody& b2 = CreateVertex(world, JVector(3, 1, 0));
    second.AddVertex(b0);
    second.AddVertex(b1);
    second.AddVertex(b2);
    SoftBodyTriangle secondTriangle(second, b0, b1, b2);

    Require(
        !Jitter2::SoftBodies::DynamicTreeCollisionFilter::Filter(firstTriangle, secondFirstTriangle),
        "soft body filter rejects same soft body");
    Require(
        Jitter2::SoftBodies::DynamicTreeCollisionFilter::Filter(firstTriangle, secondTriangle),
        "soft body filter accepts different soft bodies");

    BoxShape sharedShape(JVector(1));
    a0.AddShape(sharedShape);
    BoxShape otherShape(JVector(1));
    b0.AddShape(otherShape);

    Require(
        !Jitter2::SoftBodies::DynamicTreeCollisionFilter::Filter(sharedShape, sharedShape),
        "soft body filter rejects shapes on same rigid body");
    Require(
        Jitter2::SoftBodies::DynamicTreeCollisionFilter::Filter(sharedShape, otherShape),
        "soft body filter accepts shapes on different rigid bodies");
}

void SoftBodyPostStepTogglesShapeProxyActivation()
{
    World world;
    SoftBody softBody(world);

    RigidBody& v1 = CreateVertex(world, JVector(-1, 0, 0));
    RigidBody& v2 = CreateVertex(world, JVector(1, 0, 0));
    RigidBody& v3 = CreateVertex(world, JVector(0, 1, 0));
    softBody.AddVertex(v1);
    softBody.AddVertex(v2);
    softBody.AddVertex(v3);

    SoftBodyTriangle triangle(softBody, v1, v2, v3);
    softBody.AddShape(triangle);

    Require(world.DynamicTree().IsActive(triangle), "soft proxy starts active");

    v1.Data().IsActive(false);
    world.PostStep(static_cast<Real>(0));
    Require(!world.DynamicTree().IsActive(triangle), "soft proxy deactivates after post step");

    v1.Data().IsActive(true);
    world.PostStep(static_cast<Real>(0));
    Require(world.DynamicTree().IsActive(triangle), "soft proxy reactivates after post step");
}

} // namespace

JITTER_TEST_CASE("SpringConstraint converges to target distance")
{
    SpringConstraintConvergesToTargetDistance();
}

JITTER_TEST_CASE("SoftBodyTriangle support bounds and closest")
{
    SoftBodyTriangleSupportBoundsAndClosest();
}

JITTER_TEST_CASE("SoftBodyTetrahedron support bounds and closest")
{
    SoftBodyTetrahedronSupportBoundsAndClosest();
}

JITTER_TEST_CASE("SoftBody destroy removes components")
{
    SoftBodyDestroyRemovesComponents();
}

JITTER_TEST_CASE("SoftBodyTriangle collides with rigid shape")
{
    SoftBodyTriangleCollidesWithRigidShape();
}

JITTER_TEST_CASE("SoftBodyTriangle requires broadphase filter")
{
    SoftBodyTriangleWithoutBroadPhaseFilterThrows();
}

JITTER_TEST_CASE("SoftBody collision filters match C# rules")
{
    SoftBodyDynamicTreeCollisionFilterMatchesCSharpRules();
}

JITTER_TEST_CASE("SoftBody post step toggles shape proxy activation")
{
    SoftBodyPostStepTogglesShapeProxyActivation();
}
