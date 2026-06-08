#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::IDebugDrawer;
using Jitter2::MotionType;
using Jitter2::Real;
using Jitter2::RigidBody;
using Jitter2::World;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::Dynamics::Constraints::AngularLimit;
using Jitter2::Dynamics::Constraints::AngularMotor;
using Jitter2::Dynamics::Constraints::BallSocket;
using Jitter2::Dynamics::Constraints::ConeLimit;
using Jitter2::Dynamics::Constraints::DistanceLimit;
using Jitter2::Dynamics::Constraints::FixedAngle;
using Jitter2::Dynamics::Constraints::HingeAngle;
using Jitter2::Dynamics::Constraints::HingeJoint;
using Jitter2::Dynamics::Constraints::LinearMotor;
using Jitter2::Dynamics::Constraints::PointOnLine;
using Jitter2::Dynamics::Constraints::PointOnPlane;
using Jitter2::Dynamics::Constraints::TwistAngle;
using Jitter2::LinearMath::JVector;
using Jitter2::SoftBodies::SpringConstraint;
using JitterTests::Require;

namespace
{

class CountingDebugDrawer : public IDebugDrawer
{
public:
    int Segments = 0;
    int Triangles = 0;
    int Points = 0;

    void DrawSegment(const JVector&, const JVector&) override
    {
        ++Segments;
    }

    void DrawTriangle(const JVector&, const JVector&, const JVector&) override
    {
        ++Triangles;
    }

    void DrawPoint(const JVector&) override
    {
        ++Points;
    }
};

RigidBody& CreateBody(World& world, const JVector& position)
{
    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Position(position);
    body.SetMassInertia(static_cast<Real>(1));
    return body;
}

void RequireCounts(
    CountingDebugDrawer& drawer,
    int segments,
    int triangles,
    int points,
    const char* message)
{
    Require(drawer.Segments == segments, message);
    Require(drawer.Triangles == triangles, message);
    Require(drawer.Points == points, message);
}

void ConstraintDebugDrawsMatchCSharpPrimitiveCounts()
{
    World world;
    RigidBody& body1 = CreateBody(world, JVector(0, 0, 0));
    RigidBody& body2 = CreateBody(world, JVector(2, 0, 0));

    CountingDebugDrawer drawer;

    BallSocket& ballSocket = world.CreateConstraint<BallSocket>(body1, body2);
    ballSocket.Initialize(JVector(1, 0, 0));
    ballSocket.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 2, "BallSocket debug draw count");

    drawer = {};
    DistanceLimit& distanceLimit = world.CreateConstraint<DistanceLimit>(body1, body2);
    distanceLimit.Initialize(body1.Position(), body2.Position());
    distanceLimit.DebugDraw(drawer);
    RequireCounts(drawer, 3, 0, 2, "DistanceLimit debug draw count");

    drawer = {};
    AngularMotor& angularMotor = world.CreateConstraint<AngularMotor>(body1, body2);
    angularMotor.Initialize(JVector::UnitY());
    angularMotor.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 0, "AngularMotor debug draw count");

    drawer = {};
    LinearMotor& linearMotor = world.CreateConstraint<LinearMotor>(body1, body2);
    linearMotor.Initialize(JVector::UnitX());
    linearMotor.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 0, "LinearMotor debug draw count");

    drawer = {};
    FixedAngle& fixedAngle = world.CreateConstraint<FixedAngle>(body1, body2);
    fixedAngle.Initialize();
    fixedAngle.DebugDraw(drawer);
    RequireCounts(drawer, 6, 0, 0, "FixedAngle debug draw count");

    drawer = {};
    PointOnPlane& pointOnPlane = world.CreateConstraint<PointOnPlane>(body1, body2);
    pointOnPlane.Initialize(JVector::UnitY(), body1.Position(), body2.Position());
    pointOnPlane.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 2, "PointOnPlane debug draw count");

    drawer = {};
    PointOnLine& pointOnLine = world.CreateConstraint<PointOnLine>(body1, body2);
    pointOnLine.Initialize(JVector::UnitX(), body1.Position(), body2.Position());
    pointOnLine.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 2, "PointOnLine debug draw count");

    drawer = {};
    HingeAngle& hingeAngle = world.CreateConstraint<HingeAngle>(body1, body2);
    hingeAngle.Initialize(JVector::UnitZ(), AngularLimit(-static_cast<Real>(0.25), static_cast<Real>(0.25)));
    hingeAngle.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 0, "HingeAngle debug draw count");

    drawer = {};
    TwistAngle& twistAngle = world.CreateConstraint<TwistAngle>(body1, body2);
    twistAngle.Initialize(JVector::UnitX(), JVector::UnitX(), AngularLimit(-static_cast<Real>(0.25), static_cast<Real>(0.25)));
    twistAngle.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 0, "TwistAngle debug draw count");

    drawer = {};
    ConeLimit& coneLimit = world.CreateConstraint<ConeLimit>(body1, body2);
    coneLimit.Initialize(JVector::UnitY(), AngularLimit(static_cast<Real>(0), static_cast<Real>(0.5)));
    coneLimit.DebugDraw(drawer);
    RequireCounts(drawer, 2, 0, 0, "ConeLimit debug draw count");
}

void SpringConstraintDebugDrawMatchesCSharpPrimitiveCounts()
{
    World world;
    RigidBody& body1 = CreateBody(world, JVector(0, 0, 0));
    RigidBody& body2 = CreateBody(world, JVector(2, 0, 0));

    SpringConstraint& spring = world.CreateConstraint<SpringConstraint>(body1, body2);
    spring.Initialize(body1.Position(), body2.Position());

    CountingDebugDrawer drawer;
    spring.DebugDraw(drawer);
    RequireCounts(drawer, 3, 0, 2, "SpringConstraint debug draw count");
}

void JointDebugDrawForwardsToRegisteredConstraints()
{
    World world;
    RigidBody& body1 = CreateBody(world, JVector(0, 0, 0));
    RigidBody& body2 = CreateBody(world, JVector(2, 0, 0));

    HingeJoint joint(world, body1, body2, JVector(1, 0, 0), JVector::UnitY());

    CountingDebugDrawer drawer;
    joint.DebugDraw(drawer);
    RequireCounts(drawer, 4, 0, 2, "Joint debug draw forwards constraints");
}

void RigidBodyDebugDrawTessellatesAttachedShapes()
{
    World world;
    RigidBody& body = CreateBody(world, JVector(3, 0, 0));
    BoxShape shape(JVector(1, 2, 3));
    body.AddShape(shape);

    CountingDebugDrawer drawer;
    body.DebugDraw(drawer);

    Require(drawer.Triangles > 0, "RigidBody debug draw emits tessellated triangles");
    Require(drawer.Segments == 0, "RigidBody debug draw does not emit segments");
    Require(drawer.Points == 0, "RigidBody debug draw does not emit points");
}

} // namespace

JITTER_TEST_CASE("Constraint debug draw primitive counts match CSharp")
{
    ConstraintDebugDrawsMatchCSharpPrimitiveCounts();
}

JITTER_TEST_CASE("SpringConstraint debug draw primitive counts match CSharp")
{
    SpringConstraintDebugDrawMatchesCSharpPrimitiveCounts();
}

JITTER_TEST_CASE("Joint debug draw forwards to registered constraints")
{
    JointDebugDrawForwardsToRegisteredConstraints();
}

JITTER_TEST_CASE("RigidBody debug draw tessellates attached shapes")
{
    RigidBodyDebugDrawTessellatesAttachedShapes();
}
