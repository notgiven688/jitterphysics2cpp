#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::MotionType;
using Jitter2::Real;
using Jitter2::SolveMode;
using Jitter2::ThreadModelType;
using Jitter2::World;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::LinearMath::JVector;
using Jitter2::Parallelization::Batch;
using Jitter2::Parallelization::ForBatch;
using Jitter2::Parallelization::GetBounds;
using Jitter2::Parallelization::ReaderWriterLock;
using Jitter2::Parallelization::ThreadPool;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

void GetBoundsMatchesCSharpExample()
{
    const int expectedStarts[] = {0, 4, 8, 11};
    const int expectedEnds[] = {4, 8, 11, 14};

    for (int part = 0; part < 4; ++part)
    {
        int start = 0;
        int end = 0;
        GetBounds(14, 4, part, start, end);
        Require(start == expectedStarts[part], "batch start matches C# example");
        Require(end == expectedEnds[part], "batch end matches C# example");
    }
}

void ForBatchVisitsEachElementOnce()
{
    ThreadPool& pool = ThreadPool::Instance();
    const int originalThreadCount = pool.ThreadCount();

#if JITTER_ENABLE_MULTITHREADING
    pool.ChangeThreadCount(std::max(2, std::min(4, ThreadPool::ThreadCountSuggestion())));
#else
    pool.ChangeThreadCount(1);
#endif

    std::vector<std::atomic<int>> visits(100);
    ForBatch(
        0,
        static_cast<int>(visits.size()),
        7,
        [&visits](Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                visits[static_cast<std::size_t>(i)].fetch_add(1, std::memory_order_relaxed);
            }
        });

    for (const std::atomic<int>& visit : visits)
    {
        Require(visit.load(std::memory_order_relaxed) == 1, "parallel batch visits element once");
    }

    pool.ChangeThreadCount(originalThreadCount);
    pool.PauseWorkers();
}

void ThreadModelControlsWorkerPauseState()
{
    ThreadPool& pool = ThreadPool::Instance();
    const int originalThreadCount = pool.ThreadCount();

#if JITTER_ENABLE_MULTITHREADING
    pool.ChangeThreadCount(2);

    World world;
    world.Gravity = JVector::Zero();
    world.AllowDeactivation = false;
    world.CreateRigidBody().AffectedByGravity(false);

    world.ThreadModel = ThreadModelType::Regular;
    world.Step(static_cast<Real>(1.0 / 60.0), true);
    Require(pool.IsPaused(), "regular thread model pauses workers after step");

    world.ThreadModel = ThreadModelType::Persistent;
    world.Step(static_cast<Real>(1.0 / 60.0), true);
    Require(!pool.IsPaused(), "persistent thread model leaves workers resumed after step");

    world.Stabilize(static_cast<Real>(1.0 / 60.0), 1, 0, true);
    Require(!pool.IsPaused(), "persistent thread model leaves workers resumed after stabilize");
#else
    World world;
    world.Step(static_cast<Real>(1.0 / 60.0), true);
    Require(pool.ThreadCount() == 1, "disabled multithreading keeps one worker");
#endif

    pool.ChangeThreadCount(originalThreadCount);
    pool.PauseWorkers();
}

void ReaderWriterLockExcludesWritersAndReaders()
{
    ReaderWriterLock lock;

    lock.EnterReadLock();
    std::atomic<bool> writerEntered = false;
    std::thread writer([&]
    {
        lock.EnterWriteLock();
        writerEntered.store(true, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        lock.ExitWriteLock();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    Require(!writerEntered.load(std::memory_order_acquire), "writer waits for active reader");
    lock.ExitReadLock();

    while (!writerEntered.load(std::memory_order_acquire))
    {
        std::this_thread::yield();
    }

    std::atomic<bool> readerEntered = false;
    std::thread reader([&]
    {
        lock.EnterReadLock();
        readerEntered.store(true, std::memory_order_release);
        lock.ExitReadLock();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    Require(!readerEntered.load(std::memory_order_acquire), "reader waits for active writer");

    writer.join();
    reader.join();
    Require(readerEntered.load(std::memory_order_acquire), "reader enters after writer exits");
}

void FillIndependentBodies(World& world)
{
    world.Gravity = JVector(0, static_cast<Real>(-10), 0);
    world.AllowDeactivation = false;
    world.SubstepCount = 2;

    for (int i = 0; i < 512; ++i)
    {
        auto& body = world.CreateRigidBody();
        body.AffectedByGravity(true);
        body.Damping(static_cast<Real>(0), static_cast<Real>(0));
        body.Position(JVector(static_cast<Real>(i * 4), static_cast<Real>(i % 7), static_cast<Real>(i % 11)));
        body.Velocity(JVector(
            static_cast<Real>(0.01 * i),
            static_cast<Real>(0.02 * (i % 5)),
            static_cast<Real>(-0.015 * (i % 3))));
        body.AddForce(JVector(
            static_cast<Real>(i % 13),
            static_cast<Real>(2 * (i % 17)),
            static_cast<Real>(-(i % 19))));
    }
}

void MultithreadStepMatchesSingleThreadForIndependentBodies()
{
    ThreadPool& pool = ThreadPool::Instance();
    const int originalThreadCount = pool.ThreadCount();

#if JITTER_ENABLE_MULTITHREADING
    pool.ChangeThreadCount(std::max(2, std::min(4, ThreadPool::ThreadCountSuggestion())));
#endif

    World singleThreaded;
    World multiThreaded;
    FillIndependentBodies(singleThreaded);
    FillIndependentBodies(multiThreaded);

    singleThreaded.Step(static_cast<Real>(1.0 / 30.0), false);
    multiThreaded.Step(static_cast<Real>(1.0 / 30.0), true);

    Require(singleThreaded.RigidBodies().size() == multiThreaded.RigidBodies().size(), "world body counts match");
    Require(singleThreaded.StepCount() == multiThreaded.StepCount(), "world step counts match");

    const Real tolerance = static_cast<Real>(1e-5);
    for (std::size_t i = 0; i < singleThreaded.RigidBodies().size(); ++i)
    {
        const auto& singleBody = *singleThreaded.RigidBodies()[i];
        const auto& multiBody = *multiThreaded.RigidBodies()[i];
        Require(singleBody.MotionTypeValue() == multiBody.MotionTypeValue(), "motion type matches");
        RequireClose(singleBody.Position().X, multiBody.Position().X, tolerance, "position x matches");
        RequireClose(singleBody.Position().Y, multiBody.Position().Y, tolerance, "position y matches");
        RequireClose(singleBody.Position().Z, multiBody.Position().Z, tolerance, "position z matches");
        RequireClose(singleBody.Velocity().X, multiBody.Velocity().X, tolerance, "velocity x matches");
        RequireClose(singleBody.Velocity().Y, multiBody.Velocity().Y, tolerance, "velocity y matches");
        RequireClose(singleBody.Velocity().Z, multiBody.Velocity().Z, tolerance, "velocity z matches");
    }

    pool.ChangeThreadCount(originalThreadCount);
    pool.PauseWorkers();
}

void FillDeterministicContactWorld(World& world, std::vector<std::unique_ptr<BoxShape>>& shapes)
{
    world.SolverMode = SolveMode::Deterministic;
    world.AllowDeactivation = false;
    world.SubstepCount = 2;
    world.SolverIterations(4, 2);

    auto& floor = world.CreateRigidBody();
    floor.MotionTypeValue(MotionType::Static);
    floor.Position(JVector(0, -1, 0));
    shapes.push_back(std::make_unique<BoxShape>(JVector(20, 1, 20)));
    floor.AddShape(*shapes.back());

    for (int i = 0; i < 16; ++i)
    {
        auto& body = world.CreateRigidBody();
        body.AffectedByGravity(true);
        body.Damping(static_cast<Real>(0), static_cast<Real>(0));
        body.Position(JVector(
            static_cast<Real>((i % 4) * 3 - 4.5),
            static_cast<Real>(0.45),
            static_cast<Real>((i / 4) * 3 - 4.5)));
        body.Velocity(JVector(
            static_cast<Real>(0.01 * (i % 3)),
            static_cast<Real>(-0.02 * (i % 5)),
            static_cast<Real>(-0.01 * (i % 7))));
        shapes.push_back(std::make_unique<BoxShape>(JVector(
            static_cast<Real>(0.5),
            static_cast<Real>(0.5),
            static_cast<Real>(0.5))));
        body.AddShape(*shapes.back());
    }
}

void DeterministicMultithreadedStepMatchesSingleThreadedContacts()
{
    ThreadPool& pool = ThreadPool::Instance();
    const int originalThreadCount = pool.ThreadCount();

#if JITTER_ENABLE_MULTITHREADING
    pool.ChangeThreadCount(std::max(2, std::min(4, ThreadPool::ThreadCountSuggestion())));
#endif

    World singleThreaded;
    World multiThreaded;
    std::vector<std::unique_ptr<BoxShape>> singleShapes;
    std::vector<std::unique_ptr<BoxShape>> multiShapes;
    FillDeterministicContactWorld(singleThreaded, singleShapes);
    FillDeterministicContactWorld(multiThreaded, multiShapes);

    for (int i = 0; i < 3; ++i)
    {
        singleThreaded.Step(static_cast<Real>(1.0 / 60.0), false);
        multiThreaded.Step(static_cast<Real>(1.0 / 60.0), true);
    }

    Require(singleThreaded.RigidBodies().size() == multiThreaded.RigidBodies().size(), "deterministic body counts match");
    Require(singleThreaded.StepCount() == multiThreaded.StepCount(), "deterministic step counts match");

    const Real tolerance = static_cast<Real>(1e-5);
    for (std::size_t i = 0; i < singleThreaded.RigidBodies().size(); ++i)
    {
        const auto& singleBody = *singleThreaded.RigidBodies()[i];
        const auto& multiBody = *multiThreaded.RigidBodies()[i];
        Require(singleBody.MotionTypeValue() == multiBody.MotionTypeValue(), "deterministic motion type matches");
        RequireClose(singleBody.Position().X, multiBody.Position().X, tolerance, "deterministic position x matches");
        RequireClose(singleBody.Position().Y, multiBody.Position().Y, tolerance, "deterministic position y matches");
        RequireClose(singleBody.Position().Z, multiBody.Position().Z, tolerance, "deterministic position z matches");
        RequireClose(singleBody.Velocity().X, multiBody.Velocity().X, tolerance, "deterministic velocity x matches");
        RequireClose(singleBody.Velocity().Y, multiBody.Velocity().Y, tolerance, "deterministic velocity y matches");
        RequireClose(singleBody.Velocity().Z, multiBody.Velocity().Z, tolerance, "deterministic velocity z matches");
    }

    pool.ChangeThreadCount(originalThreadCount);
    pool.PauseWorkers();
}

} // namespace

JITTER_TEST_CASE("Parallel GetBounds matches C# example")
{
    GetBoundsMatchesCSharpExample();
}

JITTER_TEST_CASE("Parallel ForBatch visits each element once")
{
    ForBatchVisitsEachElementOnce();
}

JITTER_TEST_CASE("World ThreadModel controls worker pause state")
{
    ThreadModelControlsWorkerPauseState();
}

JITTER_TEST_CASE("ReaderWriterLock excludes writers and readers")
{
    ReaderWriterLockExcludesWritersAndReaders();
}

JITTER_TEST_CASE("Multithreaded step matches single threaded independent bodies")
{
    MultithreadStepMatchesSingleThreadForIndependentBodies();
}

JITTER_TEST_CASE("Deterministic multithreaded step matches single threaded contacts")
{
    DeterministicMultithreadedStepMatchesSingleThreadedContacts();
}
