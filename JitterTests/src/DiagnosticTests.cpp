#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::ArgumentCheck;
using Jitter2::DebugCheck;
using Jitter2::Logger;
using Jitter2::TraceCategory;
using Jitter2::TraceName;
using Jitter2::Tracer;
using Jitter2::World;
using Jitter2::LinearMath::JMatrix;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;

namespace
{

template<typename TAction>
bool Throws(TAction&& action)
{
    try
    {
        action();
    }
    catch (const std::exception&)
    {
        return true;
    }

    return false;
}

void LoggerDispatchesLevelsAndCSharpStyleFormatting()
{
    std::vector<std::pair<Logger::LogLevel, std::string>> entries;
    Logger::Listener = [&entries](Logger::LogLevel level, const std::string& message)
    {
        entries.emplace_back(level, message);
    };

    Logger::Information("Creating new world.");
    Logger::Warning("{0}: Resizing from {1} to {2} elements.", "PairHashSet", 0, 16384);
    Logger::Error("{0}: EPA failed after {1} iterations.", "NarrowPhase", 85);

    Require(entries.size() == 3, "logger listener receives all messages");
    Require(entries[0].first == Logger::LogLevel::Information, "logger information level");
    Require(entries[0].second == "Creating new world.", "logger information message");
    Require(entries[1].first == Logger::LogLevel::Warning, "logger warning level");
    Require(entries[1].second == "PairHashSet: Resizing from 0 to 16384 elements.", "logger warning formatting");
    Require(entries[2].first == Logger::LogLevel::Error, "logger error level");
    Require(entries[2].second == "NarrowPhase: EPA failed after 85 iterations.", "logger error formatting");

    Logger::Listener = {};
    Logger::Warning("ignored");
    Require(entries.size() == 3, "logger is no-op without listener");
}

void TracerNoOpsAndWritesTraceArray()
{
    Tracer::ProfileBegin(TraceName::Step);
    Tracer::ProfileEvent(TraceName::BroadPhase, TraceCategory::General);
    Tracer::ProfileScopeBegin();
    Tracer::ProfileScopeEnd(TraceName::Queue, TraceCategory::General, 0.0);
    Tracer::ProfileEnd(TraceName::Step);

    const char* path = "/tmp/jitter_tracer_diagnostics.json";
    Tracer::WriteToFile(path, true);

    std::ifstream reader(path);
    const std::string contents(
        (std::istreambuf_iterator<char>(reader)),
        std::istreambuf_iterator<char>());

    Require(!contents.empty(), "tracer writes a file");
    Require(contents.front() == '[', "tracer output starts as chrome trace array");
    Require(contents.back() == ']', "tracer output ends as chrome trace array");
}

void ArgumentCheckMatchesCSharpValidationSurface()
{
    const JVector finiteVector(1, 2, 3);
    const JQuaternion finiteQuaternion = JQuaternion::Identity();
    const JMatrix finiteMatrix = JMatrix::Identity();

    Require(ArgumentCheck::Finite(static_cast<Jitter2::Real>(1), "value") == static_cast<Jitter2::Real>(1), "finite scalar returns value");
    Require(ArgumentCheck::Finite(finiteVector, "vector") == finiteVector, "finite vector returns value");
    Require(ArgumentCheck::Finite(finiteQuaternion, "quaternion") == finiteQuaternion, "finite quaternion returns value");
    Require(ArgumentCheck::Finite(finiteMatrix, "matrix").M11 == finiteMatrix.M11, "finite matrix returns value");
    Require(ArgumentCheck::NonNegative(static_cast<Jitter2::Real>(0), "value") == static_cast<Jitter2::Real>(0), "non-negative allows zero");
    Require(ArgumentCheck::Positive(static_cast<Jitter2::Real>(1), "value") == static_cast<Jitter2::Real>(1), "positive allows positive");
    Require(ArgumentCheck::InRange(static_cast<Jitter2::Real>(2), static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(3), "value")
        == static_cast<Jitter2::Real>(2), "range allows value");
    Require(ArgumentCheck::NonZero(JVector::UnitX(), "vector") == JVector::UnitX(), "non-zero vector returns value");
    Require(ArgumentCheck::UnitVector(JVector::UnitY(), "vector") == JVector::UnitY(), "unit vector returns value");
    Require(ArgumentCheck::UnitQuaternion(JQuaternion::Identity(), "quaternion") == JQuaternion::Identity(), "unit quaternion returns value");

    Require(Throws([] { (void)ArgumentCheck::Finite(std::numeric_limits<Jitter2::Real>::infinity(), "value"); }), "finite rejects infinity");
    Require(Throws([] { (void)ArgumentCheck::NotNaN(std::numeric_limits<Jitter2::Real>::quiet_NaN(), "value"); }), "not-NaN rejects NaN");
    Require(Throws([] { (void)ArgumentCheck::Finite(JVector(1, std::numeric_limits<Jitter2::Real>::infinity(), 3), "vector"); }), "finite vector rejects infinity");
    Require(Throws([] { (void)ArgumentCheck::NonNegative(static_cast<Jitter2::Real>(-1), "value"); }), "non-negative rejects negative");
    Require(Throws([] { (void)ArgumentCheck::Positive(static_cast<Jitter2::Real>(0), "value"); }), "positive rejects zero");
    Require(Throws([] { (void)ArgumentCheck::InRange(static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(3), "value"); }), "range rejects outside value");
    Require(Throws([] { (void)ArgumentCheck::NonZero(JVector::Zero(), "vector"); }), "non-zero rejects zero vector");
    Require(Throws([] { (void)ArgumentCheck::UnitVector(JVector(2, 0, 0), "vector"); }), "unit vector rejects non-unit vector");
    Require(Throws([] { (void)ArgumentCheck::UnitQuaternion(JQuaternion(0, 0, 0, 2), "quaternion"); }), "unit quaternion rejects non-unit quaternion");

    DebugCheck::IsFinite(static_cast<Jitter2::Real>(1), "value");
    DebugCheck::IsFinite(finiteVector, "vector");
    DebugCheck::IsFinite(finiteQuaternion, "quaternion");
    DebugCheck::IsFinite(finiteMatrix, "matrix");
    DebugCheck::IsNonNegative(static_cast<Jitter2::Real>(0), "value");
    DebugCheck::IsPositive(static_cast<Jitter2::Real>(1), "value");
    DebugCheck::IsInRange(static_cast<Jitter2::Real>(2), static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(3), "value");
    DebugCheck::IsNonZero(JVector::UnitX(), "vector");
    DebugCheck::IsUnitVector(JVector::UnitY(), "vector");
    DebugCheck::IsUnitQuaternion(JQuaternion::Identity(), "quaternion");
}

void WorldDebugTimingsExposeCSharpBuckets()
{
    World world;
    world.Step(static_cast<Jitter2::Real>(1.0 / 60.0), false);

    const std::span<const double> timings = world.DebugTimings();
    Require(
        timings.size() == static_cast<std::size_t>(World::Timings::Last),
        "world debug timings exposes one value per timing bucket");

    for (double timing : timings)
    {
        Require(timing >= 0.0, "world debug timing is non-negative");
    }

    Require(
        std::string(World::TimingName(World::Timings::BroadPhase)) == "BroadPhase",
        "world timing name matches CSharp enum name");
}

void WorldRawCountsExposeDemoObjectCounters()
{
    World world;
    auto& bodyA = world.CreateRigidBody();
    auto& bodyB = world.CreateRigidBody();
    Jitter2::Collision::Shapes::SphereShape shapeA(1);
    Jitter2::Collision::Shapes::SphereShape shapeB(1);
    bodyA.AddShape(shapeA);
    bodyB.AddShape(shapeB);
    (void)world.CreateConstraint<Jitter2::Dynamics::Constraints::BallSocket>(bodyA, bodyB);

    Require(world.RigidBodyDataCount() == world.RigidBodies().size(), "world exposes total rigid body data count");
    Require(world.ActiveRigidBodyCount() <= world.RigidBodyDataCount(), "world exposes active rigid body data count");
    Require(world.ConstraintDataCount() == 1, "world exposes full constraint data count");
    Require(world.ActiveConstraintDataCount() == 1, "world exposes active full constraint data count");
    Require(world.SmallConstraintDataCount() == 0, "world exposes small constraint data count");
    Require(world.ActiveSmallConstraintDataCount() == 0, "world exposes active small constraint data count");
    Require(world.DynamicTree().Count() == 2, "dynamic tree exposes total proxy count");
    Require(world.DynamicTree().ActiveCount() == 2, "dynamic tree exposes active proxy count");
}

} // namespace

JITTER_TEST_CASE("Logger dispatches levels and CSharp style formatting")
{
    LoggerDispatchesLevelsAndCSharpStyleFormatting();
}

JITTER_TEST_CASE("Tracer no-ops and writes chrome trace array")
{
    TracerNoOpsAndWritesTraceArray();
}

JITTER_TEST_CASE("ArgumentCheck and DebugCheck expose CSharp validation surface")
{
    ArgumentCheckMatchesCSharpValidationSurface();
}

JITTER_TEST_CASE("World debug timings expose CSharp buckets")
{
    WorldDebugTimingsExposeCSharpBuckets();
}

JITTER_TEST_CASE("World raw counts expose demo object counters")
{
    WorldRawCountsExposeDemoObjectCounters();
}
