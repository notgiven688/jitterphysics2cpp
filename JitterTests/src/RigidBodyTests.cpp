#include <algorithm>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::MotionType;
using Jitter2::MassInertiaUpdateMode;
using Jitter2::Real;
using Jitter2::ContactData;
using Jitter2::RigidBody;
using Jitter2::World;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::Collision::Shapes::SphereShape;
using Jitter2::Collision::Shapes::TriangleMesh;
using Jitter2::Collision::Shapes::TriangleShape;
using Jitter2::LinearMath::JTriangle;
using Jitter2::Dynamics::Constraints::AngularLimit;
using Jitter2::Dynamics::Constraints::AngularMotor;
using Jitter2::Dynamics::Constraints::BallSocket;
using Jitter2::Dynamics::Constraints::ConeLimit;
using Jitter2::Dynamics::Constraints::DistanceLimit;
using Jitter2::Dynamics::Constraints::FixedAngle;
using Jitter2::Dynamics::Constraints::HingeAngle;
using Jitter2::Dynamics::Constraints::HingeJoint;
using Jitter2::Dynamics::Constraints::LinearLimit;
using Jitter2::Dynamics::Constraints::LinearMotor;
using Jitter2::Dynamics::Constraints::PointOnLine;
using Jitter2::Dynamics::Constraints::PointOnPlane;
using Jitter2::Dynamics::Constraints::PrismaticJoint;
using Jitter2::Dynamics::Constraints::TwistAngle;
using Jitter2::Dynamics::Constraints::UniversalJoint;
using Jitter2::Dynamics::Constraints::WeldJoint;
using Jitter2::LinearMath::JAngle;
using Jitter2::LinearMath::JMatrix;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

class RejectAllBroadPhaseFilter final : public Jitter2::Collision::IBroadPhaseFilter
{
public:
    bool Filter(
        Jitter2::Collision::IDynamicTreeProxy&,
        Jitter2::Collision::IDynamicTreeProxy&) override
    {
        return false;
    }
};

class RejectAllNarrowPhaseFilter final : public Jitter2::Collision::INarrowPhaseFilter
{
public:
    bool Filter(
        const Jitter2::Collision::Shapes::RigidBodyShape&,
        const Jitter2::Collision::Shapes::RigidBodyShape&,
        JVector&,
        JVector&,
        JVector&,
        Real&) override
    {
        return false;
    }
};

void CreatedBodyDefaults()
{
    World world;
    RigidBody& body = world.CreateRigidBody();

    Require(body.Position() == JVector::Zero(), "default position");
    Require(body.Orientation() == JQuaternion::Identity(), "default orientation");
    Require(body.Velocity() == JVector::Zero(), "default velocity");
    Require(body.AngularVelocity() == JVector::Zero(), "default angular velocity");
    Require(body.MotionTypeValue() == MotionType::Dynamic, "default motion type");
    Require(body.Force() == JVector::Zero(), "default force");
    Require(body.Torque() == JVector::Zero(), "default torque");
    Require(body.AffectedByGravity(), "default affected by gravity");
    Require(!body.EnableSpeculativeContacts(), "default speculative contacts");
    Require(&body.GetWorld() == &world, "body belongs to world");
}

void PropertyRoundTrips()
{
    World world;
    RigidBody& body = world.CreateRigidBody();

    body.Position(JVector(1, 2, 3));
    body.Orientation(JQuaternion::CreateRotationY(static_cast<Real>(0.25)));
    body.Velocity(JVector(4, 5, 6));
    body.AngularVelocity(JVector(0, 1, 0));
    body.Force(JVector(10, 0, 0));
    body.Torque(JVector(0, 5, 0));
    body.Friction(static_cast<Real>(0.7));
    body.Restitution(static_cast<Real>(0.5));

    Require(body.Position() == JVector(1, 2, 3), "position round trip");
    Require(body.Velocity() == JVector(4, 5, 6), "velocity round trip");
    Require(body.AngularVelocity() == JVector(0, 1, 0), "angular velocity round trip");
    Require(body.Force() == JVector(10, 0, 0), "force round trip");
    Require(body.Torque() == JVector(0, 5, 0), "torque round trip");
    RequireClose(body.Friction(), static_cast<Real>(0.7), static_cast<Real>(1e-6), "friction round trip");
    RequireClose(body.Restitution(), static_cast<Real>(0.5), static_cast<Real>(1e-6), "restitution round trip");
}

void PredictPoseMatchesPositionAndOrientationHelpers()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.Position(JVector(1, 2, 3));
    body.Orientation(JQuaternion::Identity());
    body.Velocity(JVector(4, 0, -2));
    body.AngularVelocity(JVector(0, 1, 0));

    JVector predictedPosition;
    JQuaternion predictedOrientation;
    body.PredictPose(static_cast<Real>(0.5), predictedPosition, predictedOrientation);

    Require(predictedPosition == body.PredictPosition(static_cast<Real>(0.5)), "predict pose position matches helper");
    Require(predictedPosition == JVector(3, 2, 2), "predict pose extrapolates position");
    Require(predictedOrientation == body.PredictOrientation(static_cast<Real>(0.5)),
        "predict pose orientation matches helper");
}

void CreatedBodiesHaveUniqueIds()
{
    World world;
    std::unordered_set<std::uint64_t> ids;
    for (int i = 0; i < 100; ++i)
    {
        ids.insert(world.CreateRigidBody().RigidBodyId());
    }

    Require(ids.size() == 100, "created body ids unique");
}

void RemoveBodyUpdatesWorldList()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    const std::size_t initialCount = world.RigidBodies().size();

    world.Remove(body);

    Require(world.RigidBodies().size() == initialCount - 1, "remove body updates list");
    for (RigidBody* candidate : world.RigidBodies())
    {
        Require(candidate != &body, "removed body absent from list");
    }
}

void RemoveForeignBodyThrows()
{
    World world;
    World other;
    RigidBody& foreign = other.CreateRigidBody();

    bool threw = false;
    try
    {
        world.Remove(foreign);
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }

    Require(threw, "removing foreign body throws");
}

void StaticBodyRejectsNonZeroVelocity()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.MotionTypeValue(MotionType::Static);

    bool velocityThrew = false;
    bool angularThrew = false;
    try
    {
        body.Velocity(JVector(1, 0, 0));
    }
    catch (const std::logic_error&)
    {
        velocityThrew = true;
    }

    try
    {
        body.AngularVelocity(JVector(0, 1, 0));
    }
    catch (const std::logic_error&)
    {
        angularThrew = true;
    }

    Require(velocityThrew, "static body rejects velocity");
    Require(angularThrew, "static body rejects angular velocity");
    Require(body.Data().InverseMass == static_cast<Real>(0), "static inverse mass zero");
    Require(body.Data().InverseInertiaWorld == JMatrix::Zero(), "static inverse inertia zero");
}

void WorldClearKeepsNullBodyOnly()
{
    World world;
    world.CreateRigidBody();
    world.CreateRigidBody();

    world.Clear();

    Require(world.RigidBodies().size() == 1, "clear keeps null body only");
    Require(&world.NullBody() == world.RigidBodies().front(), "remaining body is null body");
    Require(world.StepCount() == 0, "clear resets step count");
    Require(world.Time() == static_cast<Real>(0), "clear resets time");
}

void WorldStepRejectsInvalidTime()
{
    World world;

    bool negativeThrew = false;
    try
    {
        world.Step(static_cast<Real>(-0.01), false);
    }
    catch (const std::invalid_argument&)
    {
        negativeThrew = true;
    }

    Require(negativeThrew, "negative step throws");
    world.Step(static_cast<Real>(0), false);
    Require(world.StepCount() == 0, "zero step does not advance");
}

void RequestIdRangeSharesCounter()
{
    const auto [minId, maxId] = World::RequestId(3);
    const std::uint64_t nextId = World::RequestId();

    Require(maxId == minId + 3, "request id range is upper-exclusive");
    Require(nextId == maxId, "request id range shares counter");
}

void WorldStepCallbacksFire()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SubstepCount = 3;

    int preStepCount = 0;
    int postStepCount = 0;
    int multicastPreStepCount = 0;
    int preSubStepCount = 0;
    int postSubStepCount = 0;
    Real preStepDt = 0;
    Real postStepDt = 0;
    Real preSubStepDt = 0;
    Real postSubStepDt = 0;

    world.PreStep = [&](Real dt)
    {
        ++preStepCount;
        preStepDt = dt;
    };
    const auto removedToken = world.PreStep.Add([&](Real)
    {
        multicastPreStepCount += 100;
    });
    world.PreStep.Remove(removedToken);
    world.PreStep += [&](Real)
    {
        ++multicastPreStepCount;
    };
    world.PostStep = [&](Real dt)
    {
        ++postStepCount;
        postStepDt = dt;
    };
    world.PreSubStep = [&](Real dt)
    {
        ++preSubStepCount;
        preSubStepDt += dt;
    };
    world.PostSubStep = [&](Real dt)
    {
        ++postSubStepCount;
        postSubStepDt += dt;
    };

    world.Step(static_cast<Real>(0.3), false);

    Require(preStepCount == 1, "pre-step callback count");
    Require(multicastPreStepCount == 1, "multicast pre-step callback count");
    Require(postStepCount == 1, "post-step callback count");
    Require(preSubStepCount == 3, "pre-substep callback count");
    Require(postSubStepCount == 3, "post-substep callback count");
    RequireClose(preStepDt, static_cast<Real>(0.3), static_cast<Real>(1e-6), "pre-step dt");
    RequireClose(postStepDt, static_cast<Real>(0.3), static_cast<Real>(1e-6), "post-step dt");
    RequireClose(preSubStepDt, static_cast<Real>(0.3), static_cast<Real>(1e-6), "pre-substep dt sum");
    RequireClose(postSubStepDt, static_cast<Real>(0.3), static_cast<Real>(1e-6), "post-substep dt sum");
}

void WorldTwoBodyLocksMatchDynamicBodyRules()
{
    World world;
    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();

    Require(World::TryLockTwoBody(bodyA.Data(), bodyB.Data()), "dynamic two-body lock succeeds");
    Require(!World::TryLockTwoBody(bodyA.Data(), bodyB.Data()), "locked dynamic bodies reject try lock");
    World::UnlockTwoBody(bodyA.Data(), bodyB.Data());
    Require(bodyA.Data()._lockFlag == 0, "body a lock released");
    Require(bodyB.Data()._lockFlag == 0, "body b lock released");

    bodyB.MotionTypeValue(MotionType::Static);
    Require(World::TryLockTwoBody(bodyA.Data(), bodyB.Data()), "dynamic-static lock succeeds");
    Require(bodyA.Data()._lockFlag == 1, "dynamic body locked");
    Require(bodyB.Data()._lockFlag == 0, "static body not locked");
    World::UnlockTwoBody(bodyA.Data(), bodyB.Data());
    Require(bodyA.Data()._lockFlag == 0, "dynamic body released");
    Require(bodyB.Data()._lockFlag == 0, "static body remains unlocked");
}

void WorldSolverIterationsRoundTripAndValidation()
{
    World world;

    const auto defaults = world.SolverIterations();
    Require(defaults.first == 6, "default solver iterations");
    Require(defaults.second == 4, "default relaxation iterations");

    world.SolverIterations(10, 3);
    const auto updated = world.SolverIterations();
    Require(updated.first == 10, "solver iterations round trip");
    Require(updated.second == 3, "relaxation iterations round trip");

    bool solverThrew = false;
    try
    {
        world.SolverIterations(0, 3);
    }
    catch (const std::out_of_range&)
    {
        solverThrew = true;
    }

    bool relaxationThrew = false;
    try
    {
        world.SolverIterations(1, -1);
    }
    catch (const std::out_of_range&)
    {
        relaxationThrew = true;
    }

    Require(solverThrew, "solver iterations below one throws");
    Require(relaxationThrew, "relaxation iterations below zero throws");
}

void WorldContactSettingsRoundTrip()
{
    World world;

    Require(world.EnableAuxiliaryContactPoints, "auxiliary contact points default true");
    Require(world.PersistentContactManifold, "persistent contact manifold default true");
    RequireClose(world.SpeculativeRelaxationFactor, static_cast<Real>(0.9), static_cast<Real>(1e-6), "speculative relaxation default");
    RequireClose(world.SpeculativeVelocityThreshold, static_cast<Real>(10), static_cast<Real>(1e-6), "speculative threshold default");

    world.EnableAuxiliaryContactPoints = false;
    world.PersistentContactManifold = false;
    world.SpeculativeRelaxationFactor = static_cast<Real>(0.5);
    world.SpeculativeVelocityThreshold = static_cast<Real>(25);

    Require(!world.EnableAuxiliaryContactPoints, "auxiliary contact points round trip");
    Require(!world.PersistentContactManifold, "persistent contact manifold round trip");
    RequireClose(world.SpeculativeRelaxationFactor, static_cast<Real>(0.5), static_cast<Real>(1e-6), "speculative relaxation round trip");
    RequireClose(world.SpeculativeVelocityThreshold, static_cast<Real>(25), static_cast<Real>(1e-6), "speculative threshold round trip");
}

void SpeculativeContactsSlowFastBodyBeforeOverlap()
{
    World world;
    world.Gravity = JVector::Zero();
    world.AllowDeactivation = false;

    RigidBody& wall = world.CreateRigidBody();
    wall.MotionTypeValue(MotionType::Static);
    wall.Position(JVector(2, 0, 0));
    BoxShape wallShape(JVector(static_cast<Real>(0.1), 10, 10));
    wall.AddShape(wallShape);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.EnableSpeculativeContacts(true);
    body.Position(JVector::Zero());
    body.Velocity(JVector(200, 0, 0));
    BoxShape bodyShape(JVector(1, 1, 1));
    body.AddShape(bodyShape);

    world.DynamicTree().Update(false, static_cast<Real>(0.01));
    Require(world.DynamicTree().PairCount() == 1, "swept broadphase sees speculative pair");

    world.Step(static_cast<Real>(0.01), false);

    Require(body.Position().X < static_cast<Real>(1), "speculative contact prevents first-step crossing");
    Require(body.Velocity().X < static_cast<Real>(200), "speculative contact slows body");
}

void BeginCollideFiresOnceWhenBodiesStartTouching()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    SphereShape shapeA(static_cast<Real>(1));
    bodyA.AddShape(shapeA);

    RigidBody& bodyB = world.CreateRigidBody();
    SphereShape shapeB(static_cast<Real>(1));
    bodyB.AddShape(shapeB);
    bodyB.Position(JVector(static_cast<Real>(1.5), 0, 0));

    int beginA = 0;
    int beginB = 0;
    bodyA.BeginCollide = [&](Jitter2::Arbiter&)
    {
        ++beginA;
    };
    bodyB.BeginCollide = [&](Jitter2::Arbiter&)
    {
        ++beginB;
    };

    world.Step(static_cast<Real>(1.0 / 60.0), false);
    world.Step(static_cast<Real>(1.0 / 60.0), false);

    Require(beginA == 1, "body a begin collide count");
    Require(beginB == 1, "body b begin collide count");
    Require(bodyA.InternalContacts.size() == 1, "body a contact count");
    Require(bodyB.InternalContacts.size() == 1, "body b contact count");
    Require(std::find(bodyA.InternalConnections.begin(), bodyA.InternalConnections.end(), &bodyB)
        != bodyA.InternalConnections.end(), "body a connected to body b");
    Require(std::find(bodyB.InternalConnections.begin(), bodyB.InternalConnections.end(), &bodyA)
        != bodyB.InternalConnections.end(), "body b connected to body a");
}

void EndCollideFiresOnceWhenBodiesSeparate()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    SphereShape shapeA(static_cast<Real>(1));
    bodyA.AddShape(shapeA);

    RigidBody& bodyB = world.CreateRigidBody();
    SphereShape shapeB(static_cast<Real>(1));
    bodyB.AddShape(shapeB);
    bodyB.Position(JVector(static_cast<Real>(1.5), 0, 0));

    int endA = 0;
    int endB = 0;
    bodyA.EndCollide = [&](Jitter2::Arbiter&)
    {
        ++endA;
    };
    bodyB.EndCollide = [&](Jitter2::Arbiter&)
    {
        ++endB;
    };

    world.Step(static_cast<Real>(1.0 / 60.0), false);
    bodyB.Position(JVector(5, 0, 0));
    world.Step(static_cast<Real>(1.0 / 60.0), false);
    world.Step(static_cast<Real>(1.0 / 60.0), false);

    Require(endA == 1, "body a end collide count");
    Require(endB == 1, "body b end collide count");
    Require(bodyA.InternalContacts.empty(), "body a contacts empty after separation");
    Require(bodyB.InternalContacts.empty(), "body b contacts empty after separation");
    Require(std::find(bodyA.InternalConnections.begin(), bodyA.InternalConnections.end(), &bodyB)
        == bodyA.InternalConnections.end(), "body a disconnected from body b");
    Require(std::find(bodyB.InternalConnections.begin(), bodyB.InternalConnections.end(), &bodyA)
        == bodyB.InternalConnections.end(), "body b disconnected from body a");
}

void RemovingBodyInContactCleansOtherBodyContactsAndConnections()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    SphereShape shapeA(static_cast<Real>(1));
    bodyA.AddShape(shapeA);

    RigidBody& bodyB = world.CreateRigidBody();
    SphereShape shapeB(static_cast<Real>(1));
    bodyB.AddShape(shapeB);
    bodyB.Position(JVector(static_cast<Real>(1.5), 0, 0));

    world.Step(static_cast<Real>(1.0 / 60.0), false);
    Require(bodyA.Contacts().size() == 1, "body a has contact before remove");

    world.Remove(bodyB);

    Require(bodyA.Contacts().empty(), "body a contacts empty after body remove");
    Require(bodyA.Connections().empty(), "body a connections empty after body remove");
}

void RegisterContactDefersBeginCollideUntilStep()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);

    int beginA = 0;
    int beginB = 0;
    bodyA.BeginCollide = [&](Jitter2::Arbiter&)
    {
        ++beginA;
    };
    bodyB.BeginCollide = [&](Jitter2::Arbiter&)
    {
        ++beginB;
    };

    world.RegisterContact(
        10,
        20,
        bodyA,
        bodyB,
        JVector(static_cast<Real>(0.5), 0, 0),
        JVector(static_cast<Real>(-0.5), 0, 0),
        JVector::UnitX());

    Jitter2::Arbiter* arbiter = nullptr;
    Require(world.GetArbiter(10, 20, arbiter), "registered contact creates ordered arbiter");
    Require(!world.GetArbiter(20, 10, arbiter), "arbiter key order matters");
    Require(bodyA.Contacts().empty(), "deferred arbiter not yet in body contacts");
    Require(beginA == 0 && beginB == 0, "begin collide deferred before step");

    world.Step(static_cast<Real>(1.0 / 60.0), false);

    Require(beginA == 1, "body a begin collide after step");
    Require(beginB == 1, "body b begin collide after step");
    Require(bodyA.Contacts().size() == 1, "body a has registered contact");
    Require(bodyB.Contacts().size() == 1, "body b has registered contact");
}

void ContactDataBodyHandlesSurviveRigidBodyBufferResize()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Position(JVector(1, 2, 3));

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Position(JVector(4, 5, 6));

    world.RegisterContact(
        30,
        40,
        bodyA,
        bodyB,
        JVector(static_cast<Real>(0.5), 0, 0),
        JVector(static_cast<Real>(-0.5), 0, 0),
        JVector::UnitX());

    Jitter2::Arbiter* arbiter = nullptr;
    Require(world.GetArbiter(30, 40, arbiter) && arbiter != nullptr, "registered arbiter exists");
    Require(!arbiter->Handle().IsZero(), "arbiter has contact-data handle");
    Require(&arbiter->Data().Body1.Data() == &bodyA.Data(), "contact body1 handle points to body a before resize");
    Require(&arbiter->Data().Body2.Data() == &bodyB.Data(), "contact body2 handle points to body b before resize");

    for (int i = 0; i < 1100; ++i)
    {
        world.CreateRigidBody();
    }

    bodyA.Position(JVector(7, 8, 9));
    bodyB.Position(JVector(10, 11, 12));

    Require(&arbiter->Data().Body1.Data() == &bodyA.Data(), "contact body1 handle points to body a after resize");
    Require(&arbiter->Data().Body2.Data() == &bodyB.Data(), "contact body2 handle points to body b after resize");
    Require(arbiter->Data().Body1.Data().Position == bodyA.Position(), "contact body1 data follows body a after resize");
    Require(arbiter->Data().Body2.Data().Position == bodyB.Position(), "contact body2 data follows body b after resize");
}

void RemovedArbitersAreReusedFromPool()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);

    world.RegisterContact(
        50,
        60,
        bodyA,
        bodyB,
        JVector(static_cast<Real>(0.5), 0, 0),
        JVector(static_cast<Real>(-0.5), 0, 0),
        JVector::UnitX());

    Jitter2::Arbiter* first = nullptr;
    Require(world.GetArbiter(50, 60, first) && first != nullptr, "first arbiter exists");
    world.Step(static_cast<Real>(1.0 / 60.0), false);

    const Jitter2::Arbiter* firstAddress = first;
    world.Remove(*first);

    world.RegisterContact(
        70,
        80,
        bodyA,
        bodyB,
        JVector(static_cast<Real>(0.5), 0, 0),
        JVector(static_cast<Real>(-0.5), 0, 0),
        JVector::UnitX());

    Jitter2::Arbiter* second = nullptr;
    Require(world.GetArbiter(70, 80, second) && second != nullptr, "second arbiter exists");
    Require(second == firstAddress, "arbiter reused from pool");
    Require(!second->Handle().IsZero(), "reused arbiter has fresh contact handle");
    Require(&second->Body1() == &bodyA, "reused arbiter body1 reset");
    Require(&second->Body2() == &bodyB, "reused arbiter body2 reset");
}

void MultithreadedContactBatchesStepOverlappingPairs()
{
    std::vector<std::unique_ptr<SphereShape>> shapes;
    shapes.reserve(96 * 2);

    World world;
    world.Gravity = JVector::Zero();
    world.AllowDeactivation = false;

    constexpr int PairCount = 96;
    std::vector<RigidBody*> bodies;
    bodies.reserve(PairCount * 2);

    for (int i = 0; i < PairCount; ++i)
    {
        const Real x = static_cast<Real>(i * 5);

        RigidBody& bodyA = world.CreateRigidBody();
        bodyA.AffectedByGravity(false);
        bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
        bodyA.Position(JVector(x, 0, 0));
        auto shapeA = std::make_unique<SphereShape>(static_cast<Real>(1));
        bodyA.AddShape(*shapeA);
        shapes.push_back(std::move(shapeA));
        bodies.push_back(&bodyA);

        RigidBody& bodyB = world.CreateRigidBody();
        bodyB.AffectedByGravity(false);
        bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
        bodyB.Position(JVector(x + static_cast<Real>(1.5), 0, 0));
        auto shapeB = std::make_unique<SphereShape>(static_cast<Real>(1));
        bodyB.AddShape(*shapeB);
        shapes.push_back(std::move(shapeB));
        bodies.push_back(&bodyB);
    }

    world.Step(static_cast<Real>(1.0 / 60.0), true);
    world.Step(static_cast<Real>(1.0 / 60.0), true);

    std::size_t contactReferences = 0;
    for (RigidBody* body : bodies)
    {
        contactReferences += body->Contacts().size();
    }

    Require(contactReferences >= static_cast<std::size_t>(PairCount * 2), "multithread contact batches create contacts");
    Require(world.StepCount() == 2, "multithread contact batches advance steps");
}

void MultithreadedConstraintBatchesStepDistanceLimits()
{
    World world;
    world.Gravity = JVector::Zero();
    world.AllowDeactivation = false;

    constexpr int PairCount = 96;
    std::vector<RigidBody*> bodies;
    bodies.reserve(PairCount * 2);

    for (int i = 0; i < PairCount; ++i)
    {
        const Real x = static_cast<Real>(i * 4);

        RigidBody& bodyA = world.CreateRigidBody();
        bodyA.AffectedByGravity(false);
        bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
        bodyA.Position(JVector(x, 0, 0));
        bodies.push_back(&bodyA);

        RigidBody& bodyB = world.CreateRigidBody();
        bodyB.AffectedByGravity(false);
        bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
        bodyB.Position(JVector(x + static_cast<Real>(2), 0, 0));
        bodyB.Velocity(JVector(static_cast<Real>(1), 0, 0));
        bodies.push_back(&bodyB);

        DistanceLimit& limit = world.CreateConstraint<DistanceLimit>(bodyA, bodyB);
        limit.Initialize(bodyA.Position(), bodyB.Position());
    }

    world.Step(static_cast<Real>(1.0 / 60.0), true);
    world.Step(static_cast<Real>(1.0 / 60.0), true);

    Require(world.Constraints().size() == static_cast<std::size_t>(PairCount), "multithread constraint count");
    Require(world.StepCount() == 2, "multithread constraint batches advance steps");

    for (RigidBody* body : bodies)
    {
        Require(body->Data()._lockFlag == 0, "multithread constraint body lock released");
    }
}

void MovingBodyClearsCachedContactState()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    SphereShape shapeA(static_cast<Real>(1));
    bodyA.AddShape(shapeA);

    RigidBody& bodyB = world.CreateRigidBody();
    SphereShape shapeB(static_cast<Real>(1));
    bodyB.AddShape(shapeB);
    bodyB.Position(JVector(static_cast<Real>(1.5), 0, 0));

    world.Step(static_cast<Real>(1.0 / 60.0), false);

    Require(bodyA.Contacts().size() == 1, "body a has cached contact");
    Jitter2::Arbiter* arbiter = *bodyA.Contacts().begin();
    Require((arbiter->Data().UsageMask & ContactData::MaskContactAll) != 0, "contact cache active before move");

    bodyB.Position(JVector(static_cast<Real>(1.4), 0, 0));

    Require((arbiter->Data().UsageMask & ContactData::MaskContactAll) == 0, "contact cache cleared by move");

    world.Step(static_cast<Real>(1.0 / 60.0), false);

    Require(bodyA.Contacts().size() == 1, "body a keeps one contact after cache rebuild");
    Require((arbiter->Data().UsageMask & ContactData::MaskContactAll) != 0, "contact cache rebuilt after step");
}

void DynamicBodyIntegratesGravity()
{
    World world;
    world.AllowDeactivation = false;
    RigidBody& body = world.CreateRigidBody();
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    world.Step(static_cast<Real>(1), false);

    RequireClose(body.Velocity().X, static_cast<Real>(0), static_cast<Real>(1e-6), "gravity first-step velocity x");
    RequireClose(body.Velocity().Y, static_cast<Real>(0), static_cast<Real>(1e-6), "gravity first-step velocity y");
    RequireClose(body.Position().Y, static_cast<Real>(0), static_cast<Real>(1e-6), "gravity first-step position y");
    Require(world.StepCount() == 1, "gravity first step count");

    world.Step(static_cast<Real>(1), false);

    RequireClose(body.Velocity().X, static_cast<Real>(0), static_cast<Real>(1e-6), "gravity velocity x");
    RequireClose(body.Velocity().Y, static_cast<Real>(-9.81), static_cast<Real>(1e-5), "gravity velocity y");
    RequireClose(body.Position().Y, static_cast<Real>(-9.81), static_cast<Real>(1e-5), "gravity position y");
    Require(world.StepCount() == 2, "gravity step count");
}

void DynamicBodyAppliesForceAndClearsIt()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    body.AddForce(JVector(2, 0, 0));
    world.Step(static_cast<Real>(0.5), false);

    RequireClose(body.Velocity().X, static_cast<Real>(0), static_cast<Real>(1e-6), "force first-step velocity x");
    RequireClose(body.Position().X, static_cast<Real>(0), static_cast<Real>(1e-6), "force first-step position x");
    Require(body.Force() == JVector::Zero(), "force cleared after preparation");
    Require(body.Torque() == JVector::Zero(), "torque cleared after preparation");

    world.Step(static_cast<Real>(0.5), false);

    RequireClose(body.Velocity().X, static_cast<Real>(1), static_cast<Real>(1e-6), "force velocity x");
    RequireClose(body.Position().X, static_cast<Real>(0.5), static_cast<Real>(1e-6), "force position x");
    Require(body.Force() == JVector::Zero(), "force cleared");
    Require(body.Torque() == JVector::Zero(), "torque cleared");
}

void DynamicBodyAppliesImpulse()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);

    body.ApplyImpulse(JVector(3, 0, 0));
    body.ApplyImpulse(JVector(1, 0, 0), JVector(0, 1, 0));

    RequireClose(body.Velocity().X, static_cast<Real>(4), static_cast<Real>(1e-6), "impulse velocity x");
    RequireClose(body.AngularVelocity().Z, static_cast<Real>(-1), static_cast<Real>(1e-6), "impulse angular z");
}

void BodyDeactivatesAfterThreshold()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.DeactivationTime(static_cast<Real>(0.05));

    Require(body.IsActive(), "new body starts active");

    for (int i = 0; i < 10; ++i)
    {
        world.Step(static_cast<Real>(0.01), false);
    }

    Require(!body.IsActive(), "resting body deactivates after threshold");
}

void SetActivationStateReactivatesOnNextStep()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.DeactivationTime(static_cast<Real>(0.05));

    for (int i = 0; i < 10; ++i)
    {
        world.Step(static_cast<Real>(0.01), false);
    }

    Require(!body.IsActive(), "body inactive before activation request");

    body.SetActivationState(true);
    Require(!body.IsActive(), "activation request is delayed");

    world.Step(static_cast<Real>(0.01), false);
    Require(body.IsActive(), "body active after next step");
}

void SleepingBodyForceWakeFlagsMatchCSharp()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.DeactivationTime(static_cast<Real>(0.05));

    for (int i = 0; i < 10; ++i)
    {
        world.Step(static_cast<Real>(0.01), false);
    }

    Require(!body.IsActive(), "body inactive before force");

    body.AddForce(JVector(10, 0, 0), false);
    Require(body.Force() == JVector::Zero(), "wakeup false ignores force on sleeping body");
    Require(!body.IsActive(), "wakeup false keeps body inactive");

    body.AddForce(JVector(10, 0, 0), true);
    RequireClose(body.Force().X, static_cast<Real>(10), static_cast<Real>(1e-6), "wakeup true queues force");
    Require(!body.IsActive(), "wakeup true activation is delayed");

    world.Step(static_cast<Real>(0.01), false);
    Require(body.IsActive(), "force wake activates on next step");
}

void SleepingBodyImpulseWakeFlagsMatchCSharp()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.DeactivationTime(static_cast<Real>(0.05));

    for (int i = 0; i < 10; ++i)
    {
        world.Step(static_cast<Real>(0.01), false);
    }

    Require(!body.IsActive(), "body inactive before impulse");

    body.ApplyImpulse(JVector(10, 0, 0), false);
    Require(body.Velocity() == JVector::Zero(), "wakeup false ignores impulse on sleeping body");
    Require(!body.IsActive(), "wakeup false keeps body inactive after impulse");

    body.ApplyImpulse(JVector(10, 0, 0), true);
    Require(body.Velocity().X > static_cast<Real>(0), "wakeup true applies impulse immediately");
    Require(!body.IsActive(), "impulse wake activation is delayed");

    world.Step(static_cast<Real>(0.01), false);
    Require(body.IsActive(), "impulse wake activates on next step");
}

void SleepingBodyDeactivatesAndReactivatesShapeProxy()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.DeactivationTime(static_cast<Real>(0.05));
    BoxShape shape(JVector(1, 1, 1));
    body.AddShape(shape);

    for (int i = 0; i < 10; ++i)
    {
        world.Step(static_cast<Real>(0.01), false);
    }

    Require(!body.IsActive(), "body inactive after sleep");
    Require(!world.DynamicTree().IsActive(shape), "shape proxy inactive after sleep");

    body.SetActivationState(true);
    world.Step(static_cast<Real>(0.01), false);

    Require(body.IsActive(), "body reactivated");
    Require(world.DynamicTree().IsActive(shape), "shape proxy reactivated with body");
}

void ForceSleepIslandClearsMotionAndDeactivates()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Velocity(JVector(1, 2, 3));
    body.AngularVelocity(JVector(4, 5, 6));
    body.Force(JVector(7, 8, 9));
    body.Torque(JVector(10, 11, 12));

    int deactivated = 0;
    world.IslandDeactivated = [&](Jitter2::Collision::Island& island)
    {
        ++deactivated;
        Require(!world.Islands().IsActive(island), "island inactive during deactivation callback");
    };

    world.ForceSleepIsland(body.Island());

    Require(deactivated == 1, "force sleep raises deactivation callback");
    Require(!body.IsActive(), "force sleep deactivates body");
    Require(body.Velocity() == JVector::Zero(), "force sleep clears velocity");
    Require(body.AngularVelocity() == JVector::Zero(), "force sleep clears angular velocity");
    Require(body.Force() == JVector::Zero(), "force sleep clears force");
    Require(body.Torque() == JVector::Zero(), "force sleep clears torque");
}

void KinematicBodyIntegratesVelocityOnly()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.MotionTypeValue(MotionType::Kinematic);
    body.Velocity(JVector(2, 0, 0));
    body.AngularVelocity(JVector(0, 0, static_cast<Real>(3.14159265358979323846)));

    world.Step(static_cast<Real>(0.25), false);

    RequireClose(body.Position().X, static_cast<Real>(0.5), static_cast<Real>(1e-6), "kinematic position x");
    RequireClose(body.Velocity().X, static_cast<Real>(2), static_cast<Real>(1e-6), "kinematic velocity unchanged");
    RequireClose(body.Orientation().Length(), static_cast<Real>(1), static_cast<Real>(1e-5), "kinematic orientation normalized");
}

void StepUpdatesAttachedShapeBounds()
{
    World world;
    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    BoxShape box(JVector(1, 1, 1));
    body.AddShape(box);
    body.Velocity(JVector(2, 0, 0));

    world.Step(static_cast<Real>(0.5), false);

    RequireClose(box.Position.X, static_cast<Real>(1), static_cast<Real>(1e-6), "shape position x");
    RequireClose(box.WorldBoundingBox().Min.X, static_cast<Real>(0.5), static_cast<Real>(1e-6), "shape bounds min x");
    RequireClose(box.WorldBoundingBox().Max.X, static_cast<Real>(1.5), static_cast<Real>(1e-6), "shape bounds max x");
}

void DynamicBodyCollidesWithStaticFloor()
{
    World world;

    RigidBody& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    floor.Position(JVector(0, static_cast<Real>(-0.5), 0));
    BoxShape floorShape(JVector(10, 1, 10));
    floor.AddShape(floorShape);

    RigidBody& body = world.CreateRigidBody();
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(JVector(0, 2, 0));
    BoxShape bodyShape(JVector(1, 1, 1));
    body.AddShape(bodyShape);

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(body.Position().Y > static_cast<Real>(0.489), "body stays within allowed contact penetration");
    Require(std::abs(body.Velocity().Y) < static_cast<Real>(0.25), "body vertical velocity resolved");
}

void ContactDataSolvesApproachingBodies()
{
    World world;

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyA.Position(JVector(static_cast<Real>(-0.25), 0, 0));
    bodyA.Velocity(JVector(1, 0, 0));

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyB.Position(JVector(static_cast<Real>(0.25), 0, 0));
    bodyB.Velocity(JVector(-1, 0, 0));

    ContactData contactData;
    contactData.Init(bodyA, bodyB);
    contactData.ResetMode();
    contactData.AddContact(
        JVector(static_cast<Real>(0.25), 0, 0),
        JVector(static_cast<Real>(-0.25), 0, 0),
        JVector::UnitX());

    contactData.PrepareForIteration(static_cast<Real>(60));
    for (int i = 0; i < 8; ++i)
    {
        contactData.Iterate(true);
    }

    Require(bodyA.Velocity().X < static_cast<Real>(1), "contact slows body a");
    Require(bodyB.Velocity().X > static_cast<Real>(-1), "contact slows body b");
    Require((contactData.UsageMask & ContactData::MaskContact0) != 0, "contact slot active");
    Require(contactData.Contacts[0].AccumulatedNormalImpulse > static_cast<Real>(0), "contact impulse accumulated");
}

void WorldBroadPhaseFilterSuppressesContacts()
{
    World world;
    world.Gravity = JVector::Zero();
    RejectAllBroadPhaseFilter filter;
    world.BroadPhaseFilter = &filter;

    RigidBody& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    BoxShape floorShape(JVector(10, 1, 10));
    floor.AddShape(floorShape);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(JVector(0, static_cast<Real>(0.25), 0));
    body.Velocity(JVector(0, static_cast<Real>(-1), 0));
    BoxShape bodyShape(JVector(1, 1, 1));
    body.AddShape(bodyShape);

    world.Step(static_cast<Real>(0.1), false);

    Require(body.Position().Y < static_cast<Real>(0.25), "broadphase filter allowed unresolved movement");
    Require(body.Velocity().Y < static_cast<Real>(0), "broadphase filter suppressed contact impulse");
}

void WorldDefaultDynamicTreeFilterMatchesCSharp()
{
    World world;

    RigidBody& bodyA = world.CreateRigidBody();
    BoxShape shapeA(JVector(1, 1, 1));
    BoxShape sameBodyShape(JVector(1, 1, 1));
    bodyA.AddShape(shapeA);
    bodyA.AddShape(sameBodyShape);

    RigidBody& bodyB = world.CreateRigidBody();
    BoxShape shapeB(JVector(1, 1, 1));
    bodyB.AddShape(shapeB);

    Require(!World::DefaultDynamicTreeFilter(shapeA, sameBodyShape), "default tree filter rejects same body");
    Require(World::DefaultDynamicTreeFilter(shapeA, shapeB), "default tree filter accepts different bodies");
}

void WorldNarrowPhaseFilterSuppressesContacts()
{
    World world;
    world.Gravity = JVector::Zero();
    RejectAllNarrowPhaseFilter filter;
    world.NarrowPhaseFilter = &filter;

    RigidBody& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    BoxShape floorShape(JVector(10, 1, 10));
    floor.AddShape(floorShape);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(JVector(0, static_cast<Real>(0.25), 0));
    body.Velocity(JVector(0, static_cast<Real>(-1), 0));
    BoxShape bodyShape(JVector(1, 1, 1));
    body.AddShape(bodyShape);

    world.Step(static_cast<Real>(0.1), false);

    Require(body.Position().Y < static_cast<Real>(0.25), "narrowphase filter allowed unresolved movement");
    Require(body.Velocity().Y < static_cast<Real>(0), "narrowphase filter suppressed contact impulse");
}

void WorldDefaultNarrowPhaseFilterRejectsTriangleBackFace()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& triangleBody = world.CreateRigidBody();
    triangleBody.MotionTypeValue(MotionType::Static);
    const std::vector<JTriangle> soup {
        JTriangle(JVector(0, 0, 0), JVector(1, 0, 0), JVector(0, 1, 0)),
    };
    TriangleMesh mesh(soup);
    TriangleShape triangle(mesh, 0);
    triangleBody.AddShape(triangle, MassInertiaUpdateMode::Preserve);

    RigidBody& sphereBody = world.CreateRigidBody();
    SphereShape sphere(static_cast<Real>(0.25));
    sphereBody.AddShape(sphere);

    JVector pointA(static_cast<Real>(0.25), static_cast<Real>(0.25), 0);
    JVector pointB = pointA;
    JVector normal(0, 0, static_cast<Real>(-1));
    Real penetration = static_cast<Real>(0.1);

    Require(!World::DefaultNarrowPhaseFilter(
        triangle,
        sphere,
        pointA,
        pointB,
        normal,
        penetration), "default narrowphase filter rejects triangle back-face");

    Require(world.NarrowPhaseFilter != nullptr, "world default narrowphase filter assigned");
    pointA = JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), 0);
    pointB = pointA;
    normal = JVector(0, 0, static_cast<Real>(-1));
    penetration = static_cast<Real>(0.1);

    Require(!world.NarrowPhaseFilter->Filter(
        triangle,
        sphere,
        pointA,
        pointB,
        normal,
        penetration), "world default narrowphase filter rejects triangle back-face");

    Jitter2::Collision::TriangleEdgeCollisionFilter filter;
    RequireClose(filter.EdgeThreshold(), static_cast<Real>(0.01), static_cast<Real>(1e-6), "triangle filter edge threshold default");
    RequireClose(filter.ProjectionThreshold(), static_cast<Real>(0.5), static_cast<Real>(1e-6), "triangle filter projection threshold default");
    filter.EdgeThreshold(static_cast<Real>(0.02));
    filter.ProjectionThreshold(static_cast<Real>(0.25));
    filter.AngleThreshold(JAngle::FromRadian(static_cast<Real>(0.1)));
    RequireClose(filter.EdgeThreshold(), static_cast<Real>(0.02), static_cast<Real>(1e-6), "triangle filter edge threshold set");
    RequireClose(filter.ProjectionThreshold(), static_cast<Real>(0.25), static_cast<Real>(1e-6), "triangle filter projection threshold set");
    RequireClose(filter.AngleThreshold().Radian, static_cast<Real>(0.1), static_cast<Real>(1e-6), "triangle filter angle threshold set");

    pointA = JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), 0);
    pointB = pointA;
    normal = JVector(0, 0, static_cast<Real>(-1));
    penetration = static_cast<Real>(0.1);

    Require(!filter.Filter(
        triangle,
        sphere,
        pointA,
        pointB,
        normal,
        penetration), "triangle edge filter rejects triangle back-face");
}

void WorldCreatesAndRemovesConstraints()
{
    World world;
    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();

    BallSocket& socket = world.CreateConstraint<BallSocket>(bodyA, bodyB);
    socket.Initialize(JVector::Zero());

    Require(world.Constraints().size() == 1, "constraint registered");
    Require(&socket.Body1() == &bodyA, "constraint body1");
    Require(&socket.Body2() == &bodyB, "constraint body2");
    Require(socket.IsEnabled(), "constraint enabled");
    Require(socket.ConstraintId() != 0, "constraint id assigned");
    Require(!socket.Handle().IsZero(), "constraint handle assigned");
    Require(socket.SmallHandle().IsZero(), "constraint small handle zero");
    Require(socket.Handle().Data().IsEnabled(), "constraint data enabled");
    Require(socket.Handle().Data().ConstraintId == socket.ConstraintId(), "constraint data id");
    Require(&socket.Handle().Data().Body1.Data() == &bodyA.Data(), "constraint data body1 handle");
    Require(&socket.Handle().Data().Body2.Data() == &bodyB.Data(), "constraint data body2 handle");

    world.Remove(socket);

    Require(world.Constraints().empty(), "constraint removed");
}

void WorldRejectsInvalidConstraints()
{
    World world;
    World other;
    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();
    RigidBody& foreign = other.CreateRigidBody();

    bool sameBodyThrew = false;
    try
    {
        (void)world.CreateConstraint<BallSocket>(bodyA, bodyA);
    }
    catch (const std::invalid_argument&)
    {
        sameBodyThrew = true;
    }

    bool foreignThrew = false;
    try
    {
        (void)world.CreateConstraint<BallSocket>(bodyB, foreign);
    }
    catch (const std::invalid_argument&)
    {
        foreignThrew = true;
    }

    Require(sameBodyThrew, "same-body constraint throws");
    Require(foreignThrew, "foreign-body constraint throws");
}

void MotionTypeStaticSplitsAndDynamicRebuildsConstraintIsland()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();

    BallSocket& socket = world.CreateConstraint<BallSocket>(bodyA, bodyB);
    (void)socket;

    Require(&bodyA.Island() == &bodyB.Island(), "constraint merges dynamic bodies into one island");

    bodyB.MotionTypeValue(MotionType::Static);
    Require(&bodyA.Island() != &bodyB.Island(), "switching constrained body to static splits island");

    bodyB.MotionTypeValue(MotionType::Dynamic);
    Require(&bodyA.Island() == &bodyB.Island(), "switching back to dynamic rebuilds connection");
}

void BallSocketAlignsAnchors()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyA.Position(JVector::Zero());

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyB.Position(JVector(2, 0, 0));

    BallSocket& socket = world.CreateConstraint<BallSocket>(bodyA, bodyB);
    socket.Initialize(JVector::Zero());
    bodyB.Position(JVector(3, 0, 0));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    const Real anchorError = (socket.Anchor2() - socket.Anchor1()).Length();
    Require(anchorError < static_cast<Real>(0.05), "ball socket aligns anchors");
    Require(socket.Impulse().LengthSquared() > static_cast<Real>(0), "ball socket impulse accumulated");
}

void DistanceLimitConvergesToTargetDistance()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyA.Position(JVector::Zero());

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyB.Position(JVector(4, 0, 0));

    DistanceLimit& limit = world.CreateConstraint<DistanceLimit>(bodyA, bodyB);
    limit.Initialize(bodyA.Position(), bodyB.Position());
    limit.TargetDistance(static_cast<Real>(2));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    RequireClose(limit.Distance(), static_cast<Real>(2), static_cast<Real>(0.08), "distance limit target");
    Require(bodyA.Position().X > static_cast<Real>(0.9), "distance limit moved body a");
    Require(bodyB.Position().X < static_cast<Real>(3.1), "distance limit moved body b");
}

void StabilizeSolvesConstraintsWithoutIntegrating()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyA.Position(JVector::Zero());

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));
    bodyB.Position(JVector(4, 0, 0));

    DistanceLimit& limit = world.CreateConstraint<DistanceLimit>(bodyA, bodyB);
    limit.Initialize(bodyA.Position(), bodyB.Position());
    limit.TargetDistance(static_cast<Real>(2));

    world.Stabilize(static_cast<Real>(1.0 / 60.0), 16, 2, false);

    Require(bodyA.Position() == JVector::Zero(), "stabilize does not integrate body a");
    Require(bodyB.Position() == JVector(4, 0, 0), "stabilize does not integrate body b");
    Require(bodyA.Velocity().X > static_cast<Real>(0), "stabilize changes body a velocity");
    Require(bodyB.Velocity().X < static_cast<Real>(0), "stabilize changes body b velocity");
}

void FixedAngleAlignsOrientations()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));

    FixedAngle& fixed = world.CreateConstraint<FixedAngle>(bodyA, bodyB);
    fixed.Initialize();
    bodyB.Orientation(JQuaternion::CreateRotationY(static_cast<Real>(0.8)));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(JVector::Dot(bodyA.Orientation().GetBasisX(), bodyB.Orientation().GetBasisX())
        > static_cast<Real>(0.99), "fixed angle aligns x basis");
    Require(fixed.Impulse().LengthSquared() > static_cast<Real>(0), "fixed angle impulse accumulated");
}

void PointOnPlaneMovesAnchorToPlane()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& planeBody = world.CreateRigidBody();
    planeBody.MotionTypeValue(MotionType::Static);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(JVector(0, 2, 0));

    PointOnPlane& plane = world.CreateConstraint<PointOnPlane>(planeBody, body);
    plane.Initialize(JVector::UnitY(), JVector::Zero(), body.Position());

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(std::abs(plane.Distance()) < static_cast<Real>(0.05), "point on plane distance");
    Require(body.Position().Y < static_cast<Real>(0.1), "point on plane moved body");
}

void PointOnLineMovesAnchorToLine()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& lineBody = world.CreateRigidBody();
    lineBody.MotionTypeValue(MotionType::Static);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));
    body.Position(JVector(0, 2, 3));

    PointOnLine& line = world.CreateConstraint<PointOnLine>(lineBody, body);
    line.Initialize(JVector::UnitX(), JVector::Zero(), body.Position(), LinearLimit::Full());

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(std::abs(body.Position().Y) < static_cast<Real>(0.05), "point on line y distance");
    Require(std::abs(body.Position().Z) < static_cast<Real>(0.05), "point on line z distance");
}

void LinearMotorDrivesRelativeVelocity()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));

    LinearMotor& motor = world.CreateConstraint<LinearMotor>(bodyA, bodyB);
    motor.Initialize(JVector::UnitX());
    motor.TargetVelocity(static_cast<Real>(2));
    motor.MaximumForce(static_cast<Real>(1000));

    world.Step(static_cast<Real>(1.0 / 60.0), false);

    const Real relativeVelocity = -bodyA.Velocity().X + bodyB.Velocity().X;
    RequireClose(relativeVelocity, static_cast<Real>(2), static_cast<Real>(1e-5), "linear motor relative velocity");
}

void AngularMotorDrivesRelativeVelocity()
{
    World world;
    world.Gravity = JVector::Zero();

    RigidBody& bodyA = world.CreateRigidBody();
    bodyA.AffectedByGravity(false);
    bodyA.Damping(static_cast<Real>(0), static_cast<Real>(0));

    RigidBody& bodyB = world.CreateRigidBody();
    bodyB.AffectedByGravity(false);
    bodyB.Damping(static_cast<Real>(0), static_cast<Real>(0));

    AngularMotor& motor = world.CreateConstraint<AngularMotor>(bodyA, bodyB);
    motor.Initialize(JVector::UnitY());
    motor.TargetVelocity(static_cast<Real>(2));
    motor.MaximumForce(static_cast<Real>(1000));

    world.Step(static_cast<Real>(1.0 / 60.0), false);

    const Real relativeVelocity = -bodyA.AngularVelocity().Y + bodyB.AngularVelocity().Y;
    RequireClose(relativeVelocity, static_cast<Real>(2), static_cast<Real>(1e-5), "angular motor relative velocity");
}

void HingeAngleRemovesOffAxisRotation()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& reference = world.CreateRigidBody();
    reference.MotionTypeValue(MotionType::Static);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    HingeAngle& hinge = world.CreateConstraint<HingeAngle>(reference, body);
    hinge.Initialize(JVector::UnitY(), AngularLimit::Full());
    body.Orientation(JQuaternion::CreateRotationZ(static_cast<Real>(0.7)));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(JVector::Dot(body.Orientation().GetBasisY(), JVector::UnitY())
        > static_cast<Real>(0.99), "hinge removes off-axis rotation");
}

void TwistAngleEnforcesFixedTwist()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& reference = world.CreateRigidBody();
    reference.MotionTypeValue(MotionType::Static);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    TwistAngle& twist = world.CreateConstraint<TwistAngle>(reference, body);
    twist.Initialize(JVector::UnitX(), JVector::UnitX());
    body.Orientation(JQuaternion::CreateRotationX(static_cast<Real>(0.7)));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(std::abs(twist.Angle()) < static_cast<Real>(0.05), "twist angle fixed");
}

void ConeLimitKeepsAxesInsideCone()
{
    World world;
    world.Gravity = JVector::Zero();
    world.SolverVelocityIterations = 16;

    RigidBody& reference = world.CreateRigidBody();
    reference.MotionTypeValue(MotionType::Static);

    RigidBody& body = world.CreateRigidBody();
    body.AffectedByGravity(false);
    body.Damping(static_cast<Real>(0), static_cast<Real>(0));

    ConeLimit& cone = world.CreateConstraint<ConeLimit>(reference, body);
    cone.Initialize(JVector::UnitY(), AngularLimit(static_cast<Real>(0), static_cast<Real>(0.1)));
    body.Orientation(JQuaternion::CreateRotationZ(static_cast<Real>(0.8)));

    for (int step = 0; step < 180; ++step)
    {
        world.Step(static_cast<Real>(1.0 / 60.0), false);
    }

    Require(cone.Angle() < static_cast<Real>(0.15), "cone limit angle");
}

void JointEnableDisableAndRemove()
{
    World world;
    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();

    WeldJoint weld(world, bodyA, bodyB, JVector::Zero());

    Require(world.Constraints().size() == 2, "weld joint creates constraints");
    Require(weld.Constraints().size() == 2, "weld joint registers constraints");

    weld.Disable();
    Require(!weld.FixedAngleConstraint().IsEnabled(), "weld fixed disabled");
    Require(!weld.BallSocketConstraint().IsEnabled(), "weld socket disabled");

    weld.Enable();
    Require(weld.FixedAngleConstraint().IsEnabled(), "weld fixed enabled");
    Require(weld.BallSocketConstraint().IsEnabled(), "weld socket enabled");

    weld.Remove();
    Require(world.Constraints().empty(), "weld remove clears world constraints");
    Require(weld.Constraints().empty(), "weld remove clears joint constraints");
}

void JointWrappersCreateExpectedConstraints()
{
    World world;
    RigidBody& bodyA = world.CreateRigidBody();
    RigidBody& bodyB = world.CreateRigidBody();
    RigidBody& bodyC = world.CreateRigidBody();
    RigidBody& bodyD = world.CreateRigidBody();
    RigidBody& bodyE = world.CreateRigidBody();
    RigidBody& bodyF = world.CreateRigidBody();

    HingeJoint hinge(world, bodyA, bodyB, JVector::Zero(), JVector::UnitY(), true);
    UniversalJoint universal(world, bodyC, bodyD, JVector::Zero(), JVector::UnitX(), JVector::UnitZ(), true);
    PrismaticJoint prismatic(world, bodyE, bodyF, JVector::Zero(), JVector::UnitX(), false, true);

    Require(hinge.Constraints().size() == 3, "hinge creates motorized constraints");
    Require(hinge.Motor() != nullptr, "hinge motor created");
    Require(universal.Constraints().size() == 3, "universal creates motorized constraints");
    Require(universal.Motor() != nullptr, "universal motor created");
    Require(prismatic.Constraints().size() == 3, "prismatic creates motorized constraints");
    Require(prismatic.FixedAngleConstraint() == nullptr, "unpinned prismatic has no fixed angle");
    Require(prismatic.HingeAngleConstraint() != nullptr, "unpinned prismatic has hinge angle");
    Require(prismatic.Motor() != nullptr, "prismatic motor created");
    Require(world.Constraints().size() == 9, "joint wrappers registered constraints");
}

} // namespace

JITTER_TEST_CASE("RigidBody creation defaults")
{
    CreatedBodyDefaults();
}

JITTER_TEST_CASE("RigidBody property round trips")
{
    PropertyRoundTrips();
}

JITTER_TEST_CASE("RigidBody predicts pose")
{
    PredictPoseMatchesPositionAndOrientationHelpers();
}

JITTER_TEST_CASE("RigidBody ids are unique")
{
    CreatedBodiesHaveUniqueIds();
}

JITTER_TEST_CASE("World removes bodies")
{
    RemoveBodyUpdatesWorldList();
}

JITTER_TEST_CASE("World rejects foreign body removal")
{
    RemoveForeignBodyThrows();
}

JITTER_TEST_CASE("Static body rejects non-zero velocity")
{
    StaticBodyRejectsNonZeroVelocity();
}

JITTER_TEST_CASE("World clear keeps null body")
{
    WorldClearKeepsNullBodyOnly();
}

JITTER_TEST_CASE("World step rejects invalid time")
{
    WorldStepRejectsInvalidTime();
}

JITTER_TEST_CASE("World request id range shares counter")
{
    RequestIdRangeSharesCounter();
}

JITTER_TEST_CASE("World step callbacks fire")
{
    WorldStepCallbacksFire();
}

JITTER_TEST_CASE("World two-body locks match dynamic body rules")
{
    WorldTwoBodyLocksMatchDynamicBodyRules();
}

JITTER_TEST_CASE("World solver iterations round trip and validation")
{
    WorldSolverIterationsRoundTripAndValidation();
}

JITTER_TEST_CASE("World contact settings round trip")
{
    WorldContactSettingsRoundTrip();
}

JITTER_TEST_CASE("Speculative contacts slow fast body before overlap")
{
    SpeculativeContactsSlowFastBodyBeforeOverlap();
}

JITTER_TEST_CASE("BeginCollide fires once when bodies start touching")
{
    BeginCollideFiresOnceWhenBodiesStartTouching();
}

JITTER_TEST_CASE("EndCollide fires once when bodies separate")
{
    EndCollideFiresOnceWhenBodiesSeparate();
}

JITTER_TEST_CASE("Removing body in contact cleans other body contacts and connections")
{
    RemovingBodyInContactCleansOtherBodyContactsAndConnections();
}

JITTER_TEST_CASE("RegisterContact defers BeginCollide until step")
{
    RegisterContactDefersBeginCollideUntilStep();
}

JITTER_TEST_CASE("ContactData body handles survive rigid body buffer resize")
{
    ContactDataBodyHandlesSurviveRigidBodyBufferResize();
}

JITTER_TEST_CASE("Removed arbiters are reused from pool")
{
    RemovedArbitersAreReusedFromPool();
}

JITTER_TEST_CASE("Multithreaded contact batches step overlapping pairs")
{
    MultithreadedContactBatchesStepOverlappingPairs();
}

JITTER_TEST_CASE("Multithreaded constraint batches step distance limits")
{
    MultithreadedConstraintBatchesStepDistanceLimits();
}

JITTER_TEST_CASE("Moving body clears cached contact state")
{
    MovingBodyClearsCachedContactState();
}

JITTER_TEST_CASE("Dynamic body integrates gravity")
{
    DynamicBodyIntegratesGravity();
}

JITTER_TEST_CASE("Dynamic body applies force")
{
    DynamicBodyAppliesForceAndClearsIt();
}

JITTER_TEST_CASE("Dynamic body applies impulse")
{
    DynamicBodyAppliesImpulse();
}

JITTER_TEST_CASE("Body deactivates after threshold")
{
    BodyDeactivatesAfterThreshold();
}

JITTER_TEST_CASE("SetActivationState reactivates on next step")
{
    SetActivationStateReactivatesOnNextStep();
}

JITTER_TEST_CASE("Sleeping body force wake flags match CSharp")
{
    SleepingBodyForceWakeFlagsMatchCSharp();
}

JITTER_TEST_CASE("Sleeping body impulse wake flags match CSharp")
{
    SleepingBodyImpulseWakeFlagsMatchCSharp();
}

JITTER_TEST_CASE("Sleeping body deactivates and reactivates shape proxy")
{
    SleepingBodyDeactivatesAndReactivatesShapeProxy();
}

JITTER_TEST_CASE("ForceSleepIsland clears motion and deactivates")
{
    ForceSleepIslandClearsMotionAndDeactivates();
}

JITTER_TEST_CASE("Kinematic body integrates velocity")
{
    KinematicBodyIntegratesVelocityOnly();
}

JITTER_TEST_CASE("World step updates attached shape bounds")
{
    StepUpdatesAttachedShapeBounds();
}

JITTER_TEST_CASE("Dynamic body collides with static floor")
{
    DynamicBodyCollidesWithStaticFloor();
}

JITTER_TEST_CASE("ContactData solves approaching bodies")
{
    ContactDataSolvesApproachingBodies();
}

JITTER_TEST_CASE("World broadphase filter suppresses contacts")
{
    WorldBroadPhaseFilterSuppressesContacts();
}

JITTER_TEST_CASE("World default dynamic tree filter matches CSharp")
{
    WorldDefaultDynamicTreeFilterMatchesCSharp();
}

JITTER_TEST_CASE("World narrowphase filter suppresses contacts")
{
    WorldNarrowPhaseFilterSuppressesContacts();
}

JITTER_TEST_CASE("World default narrowphase filter rejects triangle back-face")
{
    WorldDefaultNarrowPhaseFilterRejectsTriangleBackFace();
}

JITTER_TEST_CASE("World creates and removes constraints")
{
    WorldCreatesAndRemovesConstraints();
}

JITTER_TEST_CASE("World rejects invalid constraints")
{
    WorldRejectsInvalidConstraints();
}

JITTER_TEST_CASE("MotionType static splits and dynamic rebuilds constraint island")
{
    MotionTypeStaticSplitsAndDynamicRebuildsConstraintIsland();
}

JITTER_TEST_CASE("BallSocket aligns anchors")
{
    BallSocketAlignsAnchors();
}

JITTER_TEST_CASE("DistanceLimit converges to target distance")
{
    DistanceLimitConvergesToTargetDistance();
}

JITTER_TEST_CASE("World stabilize solves constraints without integrating")
{
    StabilizeSolvesConstraintsWithoutIntegrating();
}

JITTER_TEST_CASE("FixedAngle aligns orientations")
{
    FixedAngleAlignsOrientations();
}

JITTER_TEST_CASE("PointOnPlane moves anchor to plane")
{
    PointOnPlaneMovesAnchorToPlane();
}

JITTER_TEST_CASE("PointOnLine moves anchor to line")
{
    PointOnLineMovesAnchorToLine();
}

JITTER_TEST_CASE("LinearMotor drives relative velocity")
{
    LinearMotorDrivesRelativeVelocity();
}

JITTER_TEST_CASE("AngularMotor drives relative velocity")
{
    AngularMotorDrivesRelativeVelocity();
}

JITTER_TEST_CASE("HingeAngle removes off-axis rotation")
{
    HingeAngleRemovesOffAxisRotation();
}

JITTER_TEST_CASE("TwistAngle enforces fixed twist")
{
    TwistAngleEnforcesFixedTwist();
}

JITTER_TEST_CASE("ConeLimit keeps axes inside cone")
{
    ConeLimitKeepsAxesInsideCone();
}

JITTER_TEST_CASE("Joint enable disable and remove")
{
    JointEnableDisableAndRemove();
}

JITTER_TEST_CASE("Joint wrappers create expected constraints")
{
    JointWrappersCreateExpectedConstraints();
}
