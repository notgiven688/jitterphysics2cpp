#include <Jitter2/Dynamics/World.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <type_traits>

#include <Jitter2/Collision/IslandHelper.hpp>
#include <Jitter2/Collision/CollisionFilter/TriangleEdgeCollisionFilter.hpp>
#include <Jitter2/Collision/NarrowPhase/CollisionManifold.hpp>
#include <Jitter2/Collision/NarrowPhase/NarrowPhase.hpp>
#include <Jitter2/Collision/Shapes/TriangleShape.hpp>
#include <Jitter2/ArgumentCheck.hpp>
#include <Jitter2/Dynamics/Contact.hpp>
#include <Jitter2/Logger.hpp>
#include <Jitter2/Parallelization/ThreadPool.hpp>
#include <Jitter2/Tracer.hpp>

namespace Jitter2
{

namespace
{

using LinearMath::JVector;

std::atomic_uint64_t RequestIdCounter {0};
std::mutex MultiThreadStepLock;
constexpr int BodyTaskThreshold = 256;
constexpr int ContactTaskThreshold = 64;
constexpr int ConstraintTaskThreshold = 64;

struct ContactCandidate
{
    RigidBody* BodyA = nullptr;
    RigidBody* BodyB = nullptr;
    std::array<JVector, Collision::CollisionManifold::SolverContactLimit> PointsA {};
    std::array<JVector, Collision::CollisionManifold::SolverContactLimit> PointsB {};
    int PointCount = 0;
    JVector Normal = JVector::Zero();
    Real Penetration = 0;
    ContactData::SolveMode RemoveFlags = ContactData::SolveMode::None;
};

struct ThreadPoolPauseScope
{
    bool Enabled = false;

    ~ThreadPoolPauseScope()
    {
        if (Enabled && Parallelization::ThreadPool::InstanceInitialized())
        {
            Parallelization::ThreadPool::Instance().PauseWorkers();
        }
    }
};

bool AddressGreaterThan(RigidBodyData& body1, RigidBodyData& body2)
{
    return std::less<RigidBodyData*> {}(std::addressof(body2), std::addressof(body1));
}

bool TryLockBody(RigidBodyData& body)
{
    std::atomic_ref<int> lockFlag(body._lockFlag);
    int expected = 0;
    return lockFlag.compare_exchange_strong(
        expected,
        1,
        std::memory_order_acquire,
        std::memory_order_relaxed);
}

void SpinWait()
{
    constexpr int spinCount = 10;
    for (int i = 0; i < spinCount; ++i)
    {
        std::atomic_signal_fence(std::memory_order_seq_cst);
    }
}

void ReleaseBodyLock(RigidBodyData& body)
{
    std::atomic_ref<int> lockFlag(body._lockFlag);
    lockFlag.store(0, std::memory_order_release);
}

void DecrementBodyLock(RigidBodyData& body)
{
    std::atomic_ref<int> lockFlag(body._lockFlag);
    lockFlag.fetch_sub(1, std::memory_order_release);
}

void ValidateStepSettings(const World& world)
{
    ArgumentCheck::Finite(world.Gravity, "Gravity");

    if (world.SubstepCount < 1)
    {
        throw std::invalid_argument("The number of substeps has to be larger than zero.");
    }

    if (world.SolverVelocityIterations < 1)
    {
        throw std::invalid_argument("Solver iterations can not be smaller than one.");
    }

    if (world.SolverRelaxationIterations < 0)
    {
        throw std::invalid_argument("Relaxation iterations can not be smaller than zero.");
    }

    switch (world.SolverMode)
    {
    case SolveMode::Regular:
    case SolveMode::Deterministic:
        break;
    default:
        throw std::out_of_range("The solve mode must be a defined enum value.");
    }
}

} // namespace

World::World()
    : rigidBodyData_(1024, true),
      arbiters_(Parallelization::ThreadPool::ThreadCountSuggestion())
{
    Logger::Information("Creating new world.");

    dynamicTree_.Filter(DefaultDynamicTreeFilter);
    ResetNarrowPhaseFilter();

    nullBody_ = &CreateRigidBody();
    nullBody_->MotionTypeValue(MotionType::Static);
}

void World::Clear()
{
    // Snapshot the current body list because Remove mutates the live view.
    std::vector<RigidBody*> bodyStack = bodyView_;
    for (RigidBody* body : bodyStack)
    {
        if (body != nullBody_)
        {
            Remove(*body);
        }
    }

    dynamicTree_.Clear();
    // Return every arbiter to the pool after freeing its contact data.
    arbiters_.ForEach([this](const ArbiterKey&, std::unique_ptr<Arbiter>& arbiter)
    {
        contactData_.Free(arbiter->Handle());
        Arbiter::ReturnToPool(std::move(arbiter));
    });
    arbiters_.Clear();
    deferredArbiters_.Clear();
    brokenArbiters_.Clear();
    lastVisited_ = Unmanaged::JHandle<RigidBodyData>::Zero();
    sortCounter_ = 0;
    for (auto& constraint : constraints_)
    {
        if (constraint->IsSmallConstraint())
        {
            smallConstraintData_.Free(constraint->SmallHandle());
        }
        else
        {
            constraintData_.Free(constraint->Handle());
        }
        constraint->Detach();
    }
    constraints_.clear();

    RebuildBodyView();
    time_ = 0;
    stepCount_ = 0;
}

void World::Step(Real dt, bool multithread)
{
#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (!multithread)
    {
        StepInternal(dt, false);
        return;
    }

    std::lock_guard lock(MultiThreadStepLock);
    StepInternal(dt, true);
}

void World::StepInternal(Real dt, bool multithread)
{
    // Very small steps do not move the simulation in a meaningful way.
    if (!std::isfinite(dt))
    {
        throw std::invalid_argument("Time step must be finite.");
    }

    if (dt < static_cast<Real>(0))
    {
        throw std::invalid_argument("Time step cannot be negative.");
    }

    if (dt < std::numeric_limits<Real>::epsilon())
    {
        return;
    }

    ValidateStepSettings(*this);

    struct StepTraceEnd
    {
        ~StepTraceEnd()
        {
            Tracer::ProfileEnd(TraceName::Step);
        }
    };

    Tracer::ProfileBegin(TraceName::Step);
    StepTraceEnd stepTraceEnd;

    using Clock = std::chrono::steady_clock;
    auto time = Clock::now();
    auto setTime = [this, &time](Timings timing)
    {
        const auto current = Clock::now();
        debugTimings_[static_cast<std::size_t>(timing)] =
            std::chrono::duration<double, std::milli>(current - time).count();
        time = current;
    };

    const int substeps = SubstepCount;
    const Real substepDt = dt / static_cast<Real>(substeps);
    const Real inverseDt = static_cast<Real>(1) / dt;
    const std::uint64_t stepMarker = stepCount_ + 1;
    ThreadPoolPauseScope pauseScope {ThreadModel == ThreadModelType::Regular || !multithread};

    if (multithread)
    {
        Parallelization::ThreadPool::Instance().ResumeWorkers();
    }

    Tracer::ProfileScopeBegin();
    if (PreStep)
    {
        PreStep(dt);
    }
    Tracer::ProfileScopeEnd(TraceName::PreStep);
    setTime(Timings::PreStep);

    Tracer::ProfileBegin(TraceName::NarrowPhase);
    ResolveNarrowPhaseContacts(dt, stepMarker, multithread);
    Tracer::ProfileEnd(TraceName::NarrowPhase);
    setTime(Timings::NarrowPhase);

    Tracer::ProfileBegin(TraceName::AddArbiter);
    HandleDeferredArbiters();
    Tracer::ProfileEnd(TraceName::AddArbiter);
    setTime(Timings::AddArbiter);

    Tracer::ProfileBegin(TraceName::CheckDeactivation);
    CheckDeactivation();
    Tracer::ProfileEnd(TraceName::CheckDeactivation);
    setTime(Timings::CheckDeactivation);

    Tracer::ProfileBegin(TraceName::ReorderContacts);
    if (SolverMode == SolveMode::Regular)
    {
        ReorderContacts();
    }
    else
    {
        PrepareIslandSolveOrder();
    }
    Tracer::ProfileEnd(TraceName::ReorderContacts);
    setTime(Timings::ReorderContacts);

    Tracer::ProfileBegin(TraceName::Solve);
    for (int index = 0; index < substeps; ++index)
    {
        if (PreSubStep)
        {
            PreSubStep(substepDt);
        }

        IntegrateForces(multithread);

        if (SolverMode == SolveMode::Deterministic)
        {
            SolveIslands(multithread, inverseDt, SolverVelocityIterations);
        }
        else
        {
            SolveVelocities(inverseDt, SolverVelocityIterations, multithread);
        }

        IntegrateVelocities(substepDt, multithread);

        if (SolverMode == SolveMode::Deterministic)
        {
            RelaxIslands(multithread, SolverRelaxationIterations);
        }
        else
        {
            RelaxVelocities(SolverRelaxationIterations, multithread);
        }

        if (PostSubStep)
        {
            PostSubStep(substepDt);
        }
    }
    Tracer::ProfileEnd(TraceName::Solve);
    setTime(Timings::Solve);

    Tracer::ProfileBegin(TraceName::RemoveArbiter);
    RemoveBrokenArbiters();
    Tracer::ProfileEnd(TraceName::RemoveArbiter);
    setTime(Timings::RemoveArbiter);

    Tracer::ProfileBegin(TraceName::UpdateContacts);
    UpdateStoredContacts(multithread);
    Tracer::ProfileEnd(TraceName::UpdateContacts);
    setTime(Timings::UpdateContacts);

    Tracer::ProfileBegin(TraceName::UpdateBodies);
    UpdateActiveBodies(dt, substepDt, multithread);
    Tracer::ProfileEnd(TraceName::UpdateBodies);
    setTime(Timings::UpdateBodies);

    Tracer::ProfileBegin(TraceName::BroadPhase);
    dynamicTree_.Update(multithread, dt);
    Tracer::ProfileEnd(TraceName::BroadPhase);
    setTime(Timings::BroadPhase);

    time_ += dt;
    ++stepCount_;

    Tracer::ProfileScopeBegin();
    if (PostStep)
    {
        PostStep(dt);
    }
    Tracer::ProfileScopeEnd(TraceName::PostStep);
    setTime(Timings::PostStep);
}

void World::Stabilize(Real dt, int solverIterations, int relaxationIterations, bool multithread)
{
#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (!multithread)
    {
        StabilizeInternal(dt, solverIterations, relaxationIterations, false);
        return;
    }

    std::lock_guard lock(MultiThreadStepLock);
    StabilizeInternal(dt, solverIterations, relaxationIterations, true);
}

void World::StabilizeInternal(Real dt, int solverIterations, int relaxationIterations, bool multithread)
{
    if (!std::isfinite(dt))
    {
        throw std::invalid_argument("Time step must be finite.");
    }

    if (dt < static_cast<Real>(0))
    {
        throw std::invalid_argument("Time step cannot be negative.");
    }

    if (dt < std::numeric_limits<Real>::epsilon())
    {
        return;
    }

    if (solverIterations < 1)
    {
        throw std::invalid_argument("Solver iterations can not be smaller than one.");
    }

    if (relaxationIterations < 0)
    {
        throw std::invalid_argument("Relaxation iterations can not be smaller than zero.");
    }

    if (SubstepCount < 1)
    {
        throw std::invalid_argument("The number of substeps has to be larger than zero.");
    }

    switch (SolverMode)
    {
    case SolveMode::Regular:
    case SolveMode::Deterministic:
        break;
    default:
        throw std::out_of_range("The solve mode must be a defined enum value.");
    }

    const int substeps = SubstepCount;
    const Real inverseDt = static_cast<Real>(1) / dt;
    ThreadPoolPauseScope pauseScope {ThreadModel == ThreadModelType::Regular || !multithread};

    if (multithread)
    {
        Parallelization::ThreadPool::Instance().ResumeWorkers();
    }

    CheckDeactivation();

    if (SolverMode == SolveMode::Deterministic)
    {
        PrepareIslandSolveOrder();
    }

    for (int index = 0; index < substeps; ++index)
    {
        if (SolverMode == SolveMode::Deterministic)
        {
            SolveIslands(multithread, inverseDt, solverIterations);
            RelaxIslands(multithread, relaxationIterations);
        }
        else
        {
            SolveVelocities(inverseDt, solverIterations, multithread);
            RelaxVelocities(relaxationIterations, multithread);
        }
    }
}

RigidBody& World::CreateRigidBody()
{
    auto handle = rigidBodyData_.Allocate(true, true);
    handle.Data().IsActive(true);

    auto body = std::unique_ptr<RigidBody>(new RigidBody(handle, *this, RequestId()));
    RigidBody& reference = *body;
    bodies_.push_back(std::move(body));
    bodiesSet_.Add(reference, true);
    Collision::IslandHelper::BodyAdded(islands_, reference);
    AddToActiveList(*reference.InternalIsland);
    RebuildBodyView();
    return reference;
}

void World::Remove(RigidBody& body)
{
    if (&body == nullBody_)
    {
        return;
    }

    if (&body.GetWorld() != this)
    {
        throw std::invalid_argument("The body does not belong to this world.");
    }

    auto iterator = std::find_if(bodies_.begin(), bodies_.end(),
        [&body](const std::unique_ptr<RigidBody>& candidate)
        {
            return candidate.get() == &body;
        });

    if (iterator == bodies_.end())
    {
        return;
    }

    // Snapshot the current attachments before removing them from the body.
    std::vector<Dynamics::Constraints::Constraint*> constraintsToRemove(
        body.InternalConstraints.begin(),
        body.InternalConstraints.end());
    for (Dynamics::Constraints::Constraint* constraint : constraintsToRemove)
    {
        Remove(*constraint);
    }

    std::vector<Arbiter*> contactsToRemove(body.InternalContacts.begin(), body.InternalContacts.end());
    for (Arbiter* arbiter : contactsToRemove)
    {
        Remove(*arbiter);
    }

    auto handle = body.Handle();
    // The body should be the only body left in its island at this point.
    Collision::IslandHelper::BodyRemoved(islands_, body);
    body.InternalIsland = nullptr;
    bodiesSet_.Remove(body);
    body.Invalidate();
    rigidBodyData_.Free(handle);
    bodies_.erase(iterator);
    RebuildBodyView();
}

std::uint64_t World::RequestId()
{
    return RequestIdCounter.fetch_add(1, std::memory_order_relaxed) + 1;
}

void World::SolverIterations(int solver, int relaxation)
{
    if (solver < 1)
    {
        throw std::out_of_range("Solver iterations can not be smaller than one.");
    }

    if (relaxation < 0)
    {
        throw std::out_of_range("Relaxation iterations can not be smaller than zero.");
    }

    SolverVelocityIterations = solver;
    SolverRelaxationIterations = relaxation;
}

std::pair<std::uint64_t, std::uint64_t> World::RequestId(int count)
{
    if (count < 1)
    {
        throw std::out_of_range("Count must be greater zero.");
    }

    const std::uint64_t count64 = static_cast<std::uint64_t>(count);
    const std::uint64_t min = RequestIdCounter.fetch_add(count64, std::memory_order_relaxed) + 1;
    return {min, min + count64};
}

bool World::TryLockTwoBody(RigidBodyData& body1, RigidBodyData& body2)
{
    if (AddressGreaterThan(body1, body2))
    {
        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            if (!TryLockBody(body1))
            {
                SpinWait();
                if (!TryLockBody(body1))
                {
                    return false;
                }
            }
        }

        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            if (!TryLockBody(body2))
            {
                SpinWait();
                if (!TryLockBody(body2))
                {
                    ReleaseBodyLock(body1);
                    return false;
                }
            }
        }
    }
    else
    {
        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            if (!TryLockBody(body2))
            {
                SpinWait();
                if (!TryLockBody(body2))
                {
                    return false;
                }
            }
        }

        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            if (!TryLockBody(body1))
            {
                SpinWait();
                if (!TryLockBody(body1))
                {
                    ReleaseBodyLock(body2);
                    return false;
                }
            }
        }
    }

    return true;
}

void World::LockTwoBody(RigidBodyData& body1, RigidBodyData& body2)
{
    if (AddressGreaterThan(body1, body2))
    {
        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            while (!TryLockBody(body1))
            {
                SpinWait();
            }
        }

        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            while (!TryLockBody(body2))
            {
                SpinWait();
            }
        }
    }
    else
    {
        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            while (!TryLockBody(body2))
            {
                SpinWait();
            }
        }

        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            while (!TryLockBody(body1))
            {
                SpinWait();
            }
        }
    }
}

void World::UnlockTwoBody(RigidBodyData& body1, RigidBodyData& body2)
{
    if (AddressGreaterThan(body1, body2))
    {
        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            DecrementBodyLock(body2);
        }

        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            DecrementBodyLock(body1);
        }
    }
    else
    {
        if (body1.MotionTypeValue() == MotionType::Dynamic)
        {
            DecrementBodyLock(body1);
        }

        if (body2.MotionTypeValue() == MotionType::Dynamic)
        {
            DecrementBodyLock(body2);
        }
    }
}

bool World::DefaultDynamicTreeFilter(
    const Collision::DynamicTree::Proxy& proxyA,
    const Collision::DynamicTree::Proxy& proxyB)
{
    const auto* shapeA = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyA);
    const auto* shapeB = dynamic_cast<const Collision::Shapes::RigidBodyShape*>(&proxyB);
    if (shapeA != nullptr && shapeB != nullptr)
    {
        return shapeA->GetRigidBody() != shapeB->GetRigidBody();
    }

    return true;
}

bool World::DefaultNarrowPhaseFilter(
    const Collision::Shapes::RigidBodyShape& shapeA,
    const Collision::Shapes::RigidBodyShape& shapeB,
    JVector& pointA,
    JVector& pointB,
    JVector& normal,
    Real& penetration)
{
    static Collision::TriangleEdgeCollisionFilter filter;
    return filter.Filter(shapeA, shapeB, pointA, pointB, normal, penetration);
}

void World::ResetNarrowPhaseFilter()
{
    defaultNarrowPhaseFilter_ = Collision::TriangleEdgeCollisionFilter();
    NarrowPhaseFilter = &defaultNarrowPhaseFilter_;
}

void World::Remove(Dynamics::Constraints::Constraint& constraint)
{
    if (&constraint.Body1().GetWorld() != this)
    {
        throw std::invalid_argument("The constraint does not belong to this world.");
    }

    ActivateBodyNextStep(constraint.Body1());
    ActivateBodyNextStep(constraint.Body2());

    auto iterator = std::find_if(
        constraints_.begin(),
        constraints_.end(),
        [&constraint](const std::unique_ptr<Dynamics::Constraints::Constraint>& candidate)
        {
            return candidate.get() == &constraint;
        });

    if (iterator != constraints_.end())
    {
        Collision::IslandHelper::ConstraintRemoved(islands_, constraint);
        if (constraint.IsSmallConstraint())
        {
            smallConstraintData_.Free(constraint.SmallHandle());
        }
        else
        {
            constraintData_.Free(constraint.Handle());
        }
        constraint.Detach();
        constraints_.erase(iterator);
    }
}

void World::Remove(Arbiter& arbiter)
{
    if (&arbiter.Body1().GetWorld() != this)
    {
        throw std::invalid_argument("The arbiter does not belong to this world.");
    }

    ActivateBodyNextStep(arbiter.Body1());
    ActivateBodyNextStep(arbiter.Body2());

    const ArbiterKey key = arbiter.Key();
    const Unmanaged::JHandle<ContactData> handle = arbiter.Handle();
    std::unique_ptr<Arbiter> removed;
    {
        std::lock_guard lock(arbiters_.GetLock(key));
        std::unique_ptr<Arbiter>* stored = arbiters_.TryGetValue(key);
        if (stored != nullptr && stored->get() == &arbiter)
        {
            deferredArbiters_.Remove(&arbiter);
            brokenArbiters_.Remove(handle);
            Collision::IslandHelper::ArbiterRemoved(islands_, *stored->get());
            contactData_.Free(handle);
            removed = arbiters_.Take(key);
        }
    }

    if (removed)
    {
        Arbiter::ReturnToPool(std::move(removed));
    }
}

void World::ForceSleepIsland(Collision::Island& island)
{
    if (!islands_.Contains(island))
    {
        throw std::invalid_argument("The island does not belong to this world.");
    }

    for (RigidBody* body : island.InternalBodies)
    {
        ClearMotionForSleep(*body);
    }

    island.MarkedAsActive = false;
    island.NeedsUpdate = false;

    if (!islands_.IsActive(island))
    {
        return;
    }

    bool deactivatedBody = false;
    for (RigidBody* body : island.InternalBodies)
    {
        deactivatedBody = DeactivateBodyForSleep(*body, false) || deactivatedBody;
    }

    islands_.MoveToInactive(island);

    if (deactivatedBody && IslandDeactivated)
    {
        IslandDeactivated(island);
    }
}

bool World::GetArbiter(std::uint64_t id0, std::uint64_t id1, Arbiter*& arbiter)
{
    const ArbiterKey key {id0, id1};
    std::lock_guard lock(arbiters_.GetLock(key));
    std::unique_ptr<Arbiter>* stored = arbiters_.TryGetValue(key);
    if (stored == nullptr)
    {
        arbiter = nullptr;
        return false;
    }

    arbiter = stored->get();
    return true;
}

void World::GetOrCreateArbiter(
    std::uint64_t id0,
    std::uint64_t id1,
    RigidBody& body1,
    RigidBody& body2,
    Arbiter*& arbiter)
{
    const ArbiterKey key {id0, id1};
    std::lock_guard lock(arbiters_.GetLock(key));
    std::unique_ptr<Arbiter>* stored = arbiters_.TryGetValue(key);
    if (stored == nullptr)
    {
        auto created = Arbiter::GetFromPool();
        Unmanaged::JHandle<ContactData> handle;
        {
            std::lock_guard contactLock(contactDataMutex_);
            handle = contactData_.Allocate(true);
        }
        created->Create(handle, body1, body2, key, SpeculativeRelaxationFactor);
        arbiter = created.get();
        deferredArbiters_.ConcurrentAdd(created.get());
        arbiters_.Add(key, std::move(created));
        return;
    }

    arbiter = stored->get();
}

void World::RegisterContact(
    Arbiter& arbiter,
    const LinearMath::JVector& point1,
    const LinearMath::JVector& point2,
    const LinearMath::JVector& normal,
    ContactData::SolveMode removeFlags)
{
    ContactData& data = arbiter.Data();
    if (!PersistentContactManifold)
    {
        data.UsageMask = 0;
    }

    data.AddContact(point1, point2, normal);
    data.ResetMode(removeFlags);
}

void World::RegisterContact(
    std::uint64_t id0,
    std::uint64_t id1,
    RigidBody& body1,
    RigidBody& body2,
    const LinearMath::JVector& normal,
    const Collision::CollisionManifold& manifold,
    ContactData::SolveMode removeFlags)
{
    Arbiter* arbiter = nullptr;
    GetOrCreateArbiter(id0, id1, body1, body2, arbiter);

    ContactData& data = arbiter->Data();
    if (!PersistentContactManifold)
    {
        data.UsageMask = 0;
    }

    data.ResetMode(removeFlags);

    for (int i = 0; i < manifold.Count(); ++i)
    {
        data.AddContact(manifold.ManifoldA(i), manifold.ManifoldB(i), normal);
    }
}

void World::RegisterContact(
    std::uint64_t id0,
    std::uint64_t id1,
    RigidBody& body1,
    RigidBody& body2,
    const LinearMath::JVector& point1,
    const LinearMath::JVector& point2,
    const LinearMath::JVector& normal,
    ContactData::SolveMode removeFlags)
{
    Arbiter* arbiter = nullptr;
    GetOrCreateArbiter(id0, id1, body1, body2, arbiter);
    RegisterContact(*arbiter, point1, point2, normal, removeFlags);
}

void World::RebuildBodyView()
{
    bodyView_.clear();
    bodyView_.reserve(bodies_.size());
    for (const auto& body : bodies_)
    {
        bodyView_.push_back(body.get());
    }
}

void World::AddToActiveList(Collision::Island& island)
{
    const bool wasActive = islands_.IsActive(island);
    island.MarkedAsActive = true;
    if (!wasActive)
    {
        island.NeedsUpdate = true;
    }

    islands_.MoveToActive(island);
}

void World::ActivateBodyNextStep(RigidBody& body, bool wakeUpStatic)
{
    body.InternalSleepTime = static_cast<Real>(0);

    if (body.IsActive())
    {
        return;
    }

    if (body.MotionTypeValue() == MotionType::Static && !wakeUpStatic)
    {
        return;
    }

    AddToActiveList(*body.InternalIsland);

    if (body.MotionTypeValue() == MotionType::Static)
    {
        for (Dynamics::Constraints::Constraint* constraint : body.InternalConstraints)
        {
            ActivateBodyNextStep(&constraint->Body1() == &body ? constraint->Body2() : constraint->Body1());
        }

        for (Arbiter* arbiter : body.InternalContacts)
        {
            ActivateBodyNextStep(&arbiter->Body1() == &body ? arbiter->Body2() : arbiter->Body1());
        }
    }

    body.InternalIsland->NeedsUpdate = true;
}

void World::DeactivateBodyNextStep(RigidBody& body)
{
    body.InternalSleepTime = std::numeric_limits<Real>::infinity();
}

void World::RemoveStaticStaticConstraints(RigidBody& body)
{
    std::vector<Dynamics::Constraints::Constraint*> constraintsToRemove(
        body.InternalConstraints.begin(),
        body.InternalConstraints.end());

    for (Dynamics::Constraints::Constraint* constraint : constraintsToRemove)
    {
        if (constraint->Body1().Data().MotionTypeValue() != MotionType::Dynamic
            && constraint->Body2().Data().MotionTypeValue() != MotionType::Dynamic)
        {
            Remove(*constraint);
        }
    }

    std::vector<Arbiter*> contactsToRemove(body.InternalContacts.begin(), body.InternalContacts.end());
    for (Arbiter* arbiter : contactsToRemove)
    {
        if (arbiter->Body1().Data().MotionTypeValue() != MotionType::Dynamic
            && arbiter->Body2().Data().MotionTypeValue() != MotionType::Dynamic)
        {
            Remove(*arbiter);
        }
    }
}

void World::BuildConnectionsFromExistingContacts(RigidBody& body)
{
    for (Dynamics::Constraints::Constraint* constraint : body.InternalConstraints)
    {
        Collision::IslandHelper::AddConnection(islands_, constraint->Body1(), constraint->Body2());
    }

    for (Arbiter* arbiter : body.InternalContacts)
    {
        Collision::IslandHelper::AddConnection(islands_, arbiter->Body1(), arbiter->Body2());
    }
}

void World::RemoveConnections(RigidBody& body)
{
    if (body.InternalConnections.empty())
    {
        return;
    }

    std::vector<RigidBody*> connections = body.InternalConnections;
    for (RigidBody* connected : connections)
    {
        Collision::IslandHelper::RemoveConnection(islands_, body, *connected);
    }
}

void World::ClearMotionForSleep(RigidBody& body)
{
    body.Data().Velocity = LinearMath::JVector::Zero();
    body.Data().AngularVelocity = LinearMath::JVector::Zero();
    body.Data().DeltaVelocity = LinearMath::JVector::Zero();
    body.Data().DeltaAngularVelocity = LinearMath::JVector::Zero();
    body.force_ = LinearMath::JVector::Zero();
    body.torque_ = LinearMath::JVector::Zero();
    body.InternalSleepTime = std::numeric_limits<Real>::infinity();
}

bool World::DeactivateBodyForSleep(RigidBody& body, bool clearMotion)
{
    if (clearMotion)
    {
        ClearMotionForSleep(body);
    }

    const bool wasActive = body.Data().IsActive();
    body.Data().IsActive(false);

    rigidBodyData_.MoveToInactive(body.Handle());
    bodiesSet_.MoveToInactive(body);

    const bool deactivatedBody = wasActive && body.MotionTypeValue() != MotionType::Static;

    if (body.MotionTypeValue() != MotionType::Static)
    {
        for (Arbiter* arbiter : body.InternalContacts)
        {
            contactData_.MoveToInactive(arbiter->Handle());
        }

        for (Dynamics::Constraints::Constraint* constraint : body.InternalConstraints)
        {
            if (constraint->IsSmallConstraint())
            {
                smallConstraintData_.MoveToInactive(constraint->SmallHandle());
            }
            else
            {
                constraintData_.MoveToInactive(constraint->Handle());
            }
        }
    }

    for (Collision::Shapes::RigidBodyShape* shape : body.Shapes())
    {
        dynamicTree_.DeactivateProxy(*shape);
    }

    return deactivatedBody;
}

void World::CheckDeactivation()
{
    std::vector<std::pair<Collision::Island*, bool>> inactivateIslands;

    for (std::size_t i = 0; i < islands_.ActiveCount(); ++i)
    {
        Collision::Island& island = islands_[i];

        bool deactivateIsland = !island.MarkedAsActive;
        if (!AllowDeactivation)
        {
            deactivateIsland = false;
        }

        island.MarkedAsActive = false;

        const bool needsUpdate = island.NeedsUpdate;
        island.NeedsUpdate = false;

        if (!deactivateIsland && !needsUpdate)
        {
            continue;
        }

        bool activatedBody = false;
        bool deactivatedBody = false;

        for (RigidBody* body : island.InternalBodies)
        {
            if (body->Data().IsActive() != deactivateIsland)
            {
                if (!needsUpdate)
                {
                    break;
                }
                continue;
            }

            if (deactivateIsland)
            {
                deactivatedBody = DeactivateBodyForSleep(*body, false) || deactivatedBody;
            }
            else
            {
                if (body->MotionTypeValue() == MotionType::Static)
                {
                    continue;
                }

                body->Data().IsActive(true);
                activatedBody = true;
                body->InternalSleepTime = static_cast<Real>(0);
                rigidBodyData_.MoveToActive(body->Handle());
                bodiesSet_.MoveToActive(*body);

                for (Arbiter* arbiter : body->InternalContacts)
                {
                    contactData_.MoveToActive(arbiter->Handle());
                }

                for (Dynamics::Constraints::Constraint* constraint : body->InternalConstraints)
                {
                    if (constraint->IsSmallConstraint())
                    {
                        smallConstraintData_.MoveToActive(constraint->SmallHandle());
                    }
                    else
                    {
                        constraintData_.MoveToActive(constraint->Handle());
                    }
                }

                for (Collision::Shapes::RigidBodyShape* shape : body->Shapes())
                {
                    dynamicTree_.ActivateProxy(*shape);
                }
            }
        }

        if (deactivateIsland)
        {
            inactivateIslands.emplace_back(&island, deactivatedBody);
        }
        else if (activatedBody && IslandActivated)
        {
            IslandActivated(island);
        }
    }

    for (auto iterator = inactivateIslands.rbegin(); iterator != inactivateIslands.rend(); ++iterator)
    {
        Collision::Island& island = *iterator->first;
        islands_.MoveToInactive(island);
        if (iterator->second && IslandDeactivated)
        {
            IslandDeactivated(island);
        }
    }
}

void World::UpdateActiveBodies(Real stepDt, Real substepDt, bool multithread)
{
    ExecuteBodyBatches(
        multithread,
        BodyTaskThreshold,
        [this, stepDt, substepDt](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                RigidBody& body = bodiesSet_[static_cast<std::size_t>(i)];
                if (&body != nullBody_)
                {
                    body.Update(stepDt, substepDt, Gravity);
                }
            }
        });
}

void World::IntegrateForces(bool multithread)
{
    ExecuteBodyBatches(
        multithread,
        BodyTaskThreshold,
        [this](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                RigidBody& body = bodiesSet_[static_cast<std::size_t>(i)];
                if (&body != nullBody_)
                {
                    body.IntegrateForces();
                }
            }
        });
}

void World::IntegrateVelocities(Real substepDt, bool multithread)
{
    ExecuteBodyBatches(
        multithread,
        BodyTaskThreshold,
        [substepDt, this](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                RigidBody& body = bodiesSet_[static_cast<std::size_t>(i)];
                if (&body != nullBody_)
                {
                    body.IntegrateVelocity(substepDt);
                }
            }
        });
}

void World::ExecuteBodyBatches(
    bool multithread,
    int taskThreshold,
    const std::function<void(Parallelization::Batch)>& action)
{
    const int count = static_cast<int>(bodiesSet_.ActiveCount());
    if (count == 0)
    {
        return;
    }

#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (!multithread)
    {
        action(Parallelization::Batch {0, count});
        return;
    }

    const int threshold = std::max(1, taskThreshold);
    int numTasks = count / threshold + 1;
    numTasks = std::min(numTasks, Parallelization::ThreadPool::Instance().ThreadCount());

    Parallelization::ForBatch(0, count, numTasks, action);
}

void World::ExecuteContactBatches(
    bool multithread,
    int taskThreshold,
    const std::function<void(Parallelization::Batch)>& action,
    bool execute)
{
    const int count = static_cast<int>(contactData_.ActiveCount());
    if (count == 0)
    {
        return;
    }

#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (!multithread)
    {
        action(Parallelization::Batch {0, count});
        return;
    }

    const int threshold = std::max(1, taskThreshold);
    int numTasks = count / threshold + 1;
    numTasks = std::min(numTasks, Parallelization::ThreadPool::Instance().ThreadCount());

    Parallelization::ForBatch(0, count, numTasks, action, execute);
}

void World::ExecuteConstraintBatches(
    bool multithread,
    int taskThreshold,
    int count,
    const std::function<void(Parallelization::Batch)>& action,
    bool execute)
{
    if (count == 0)
    {
        return;
    }

#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (!multithread)
    {
        action(Parallelization::Batch {0, count});
        return;
    }

    const int threshold = std::max(1, taskThreshold);
    int numTasks = count / threshold + 1;
    numTasks = std::min(numTasks, Parallelization::ThreadPool::Instance().ThreadCount());

    Parallelization::ForBatch(0, count, numTasks, action, execute);
}

void World::PrepareConstraints(Parallelization::Batch batch, Real inverseDt)
{
    thread_local std::queue<int> deferredConstraints;
    auto active = constraintData_.Active();

    for (int i = batch.Start; i < batch.End; ++i)
    {
        Dynamics::Constraints::ConstraintData& constraint = active[static_cast<std::size_t>(i)];

        if (!constraint.IsEnabled())
        {
            continue;
        }

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.PrepareForIteration(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredConstraints.empty())
    {
        const int i = deferredConstraints.front();
        deferredConstraints.pop();

        Dynamics::Constraints::ConstraintData& constraint = active[static_cast<std::size_t>(i)];

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.PrepareForIteration(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }
}

void World::IterateConstraints(Parallelization::Batch batch, Real inverseDt)
{
    thread_local std::queue<int> deferredConstraints;
    auto active = constraintData_.Active();

    for (int i = batch.Start; i < batch.End; ++i)
    {
        Dynamics::Constraints::ConstraintData& constraint = active[static_cast<std::size_t>(i)];

        if (!constraint.IsEnabled())
        {
            continue;
        }

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.Iterate(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredConstraints.empty())
    {
        const int i = deferredConstraints.front();
        deferredConstraints.pop();

        Dynamics::Constraints::ConstraintData& constraint = active[static_cast<std::size_t>(i)];

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.Iterate(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }
}

void World::PrepareSmallConstraints(Parallelization::Batch batch, Real inverseDt)
{
    thread_local std::queue<int> deferredConstraints;
    auto active = smallConstraintData_.Active();

    for (int i = batch.Start; i < batch.End; ++i)
    {
        Dynamics::Constraints::SmallConstraintData& constraint = active[static_cast<std::size_t>(i)];

        if (!constraint.IsEnabled())
        {
            continue;
        }

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.PrepareForIteration(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredConstraints.empty())
    {
        const int i = deferredConstraints.front();
        deferredConstraints.pop();

        Dynamics::Constraints::SmallConstraintData& constraint = active[static_cast<std::size_t>(i)];

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.PrepareForIteration(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }
}

void World::IterateSmallConstraints(Parallelization::Batch batch, Real inverseDt)
{
    thread_local std::queue<int> deferredConstraints;
    auto active = smallConstraintData_.Active();

    for (int i = batch.Start; i < batch.End; ++i)
    {
        Dynamics::Constraints::SmallConstraintData& constraint = active[static_cast<std::size_t>(i)];

        if (!constraint.IsEnabled())
        {
            continue;
        }

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.Iterate(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredConstraints.empty())
    {
        const int i = deferredConstraints.front();
        deferredConstraints.pop();

        Dynamics::Constraints::SmallConstraintData& constraint = active[static_cast<std::size_t>(i)];

        RigidBodyData& body1 = constraint.Body1.Data();
        RigidBodyData& body2 = constraint.Body2.Data();
        if (!TryLockTwoBody(body1, body2))
        {
            deferredConstraints.push(i);
            continue;
        }

        constraint.Iterate(constraint, inverseDt);
        UnlockTwoBody(body1, body2);
    }
}

void World::SolveConstraints(Real inverseDt, int solverIterations, bool multithread)
{
    ExecuteConstraintBatches(
        multithread,
        ConstraintTaskThreshold,
        static_cast<int>(constraintData_.ActiveCount()),
        [this, inverseDt](Parallelization::Batch batch)
        {
            PrepareConstraints(batch, inverseDt);
        });

    ExecuteConstraintBatches(
        multithread,
        ConstraintTaskThreshold,
        static_cast<int>(smallConstraintData_.ActiveCount()),
        [this, inverseDt](Parallelization::Batch batch)
        {
            PrepareSmallConstraints(batch, inverseDt);
        });

    for (int iteration = 0; iteration < solverIterations; ++iteration)
    {
        ExecuteConstraintBatches(
            multithread,
            ConstraintTaskThreshold,
            static_cast<int>(constraintData_.ActiveCount()),
            [this, inverseDt](Parallelization::Batch batch)
            {
                IterateConstraints(batch, inverseDt);
            });

        ExecuteConstraintBatches(
            multithread,
            ConstraintTaskThreshold,
            static_cast<int>(smallConstraintData_.ActiveCount()),
            [this, inverseDt](Parallelization::Batch batch)
            {
                IterateSmallConstraints(batch, inverseDt);
            });
    }
}

void World::PrepareContacts(Parallelization::Batch batch, Real inverseDt)
{
    auto active = contactData_.Active();
    thread_local std::queue<int> deferredContacts;

    for (int i = batch.Start; i < batch.End; ++i)
    {
        ContactData& data = active[static_cast<std::size_t>(i)];
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (!TryLockTwoBody(body1, body2))
        {
            deferredContacts.push(i);
            continue;
        }

        if ((data.UsageMask & ContactData::MaskContactAll) == 0)
        {
            UnlockTwoBody(body1, body2);
            continue;
        }

        data.PrepareForIteration(inverseDt);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredContacts.empty())
    {
        const int i = deferredContacts.front();
        deferredContacts.pop();

        ContactData& data = active[static_cast<std::size_t>(i)];
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (!TryLockTwoBody(body1, body2))
        {
            deferredContacts.push(i);
            continue;
        }

        if ((data.UsageMask & ContactData::MaskContactAll) != 0)
        {
            data.PrepareForIteration(inverseDt);
        }

        UnlockTwoBody(body1, body2);
    }
}

void World::IterateContacts(Parallelization::Batch batch, bool applyBias)
{
    auto active = contactData_.Active();
    thread_local std::queue<int> deferredContacts;

    for (int i = batch.Start; i < batch.End; ++i)
    {
        ContactData& data = active[static_cast<std::size_t>(i)];
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (!TryLockTwoBody(body1, body2))
        {
            deferredContacts.push(i);
            continue;
        }

        if ((data.UsageMask & ContactData::MaskContactAll) == 0)
        {
            UnlockTwoBody(body1, body2);
            continue;
        }

        data.Iterate(applyBias);
        UnlockTwoBody(body1, body2);
    }

    while (!deferredContacts.empty())
    {
        const int i = deferredContacts.front();
        deferredContacts.pop();

        ContactData& data = active[static_cast<std::size_t>(i)];
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (!TryLockTwoBody(body1, body2))
        {
            deferredContacts.push(i);
            continue;
        }

        if ((data.UsageMask & ContactData::MaskContactAll) != 0)
        {
            data.Iterate(applyBias);
        }

        UnlockTwoBody(body1, body2);
    }
}

void World::PrepareStoredContacts(Real inverseDt, bool multithread)
{
    ExecuteContactBatches(
        multithread,
        ContactTaskThreshold,
        [this, inverseDt](Parallelization::Batch batch)
        {
            PrepareContacts(batch, inverseDt);
        });
}

void World::IterateStoredContacts(bool applyBias, int iterations, bool multithread)
{
    for (int iteration = 0; iteration < std::max(0, iterations); ++iteration)
    {
        ExecuteContactBatches(
            multithread,
            ContactTaskThreshold,
            [this, applyBias](Parallelization::Batch batch)
            {
                IterateContacts(batch, applyBias);
            });
    }
}

void World::SolveStoredContacts(Real inverseDt, int solverIterations, int relaxationIterations, bool multithread)
{
    PrepareStoredContacts(inverseDt, multithread);
    IterateStoredContacts(true, solverIterations, multithread);
    IterateStoredContacts(false, relaxationIterations, multithread);
}

void World::SolveVelocities(Real inverseDt, int solverIterations, bool multithread)
{
#if !JITTER_ENABLE_MULTITHREADING
    multithread = false;
#endif

    if (multithread)
    {
        ExecuteContactBatches(
            true,
            ContactTaskThreshold,
            [this, inverseDt](Parallelization::Batch batch)
            {
                PrepareContacts(batch, inverseDt);
            },
            false);

        ExecuteConstraintBatches(
            true,
            ConstraintTaskThreshold,
            static_cast<int>(constraintData_.ActiveCount()),
            [this, inverseDt](Parallelization::Batch batch)
            {
                PrepareConstraints(batch, inverseDt);
            },
            false);

        ExecuteConstraintBatches(
            true,
            ConstraintTaskThreshold,
            static_cast<int>(smallConstraintData_.ActiveCount()),
            [this, inverseDt](Parallelization::Batch batch)
            {
                PrepareSmallConstraints(batch, inverseDt);
            },
            false);

        Parallelization::ThreadPool::Instance().Execute();

        for (int iteration = 0; iteration < solverIterations; ++iteration)
        {
            ExecuteContactBatches(
                true,
                ContactTaskThreshold,
                [this](Parallelization::Batch batch)
                {
                    IterateContacts(batch, true);
                },
                false);

            ExecuteConstraintBatches(
                true,
                ConstraintTaskThreshold,
                static_cast<int>(constraintData_.ActiveCount()),
                [this, inverseDt](Parallelization::Batch batch)
                {
                    IterateConstraints(batch, inverseDt);
                },
                false);

            ExecuteConstraintBatches(
                true,
                ConstraintTaskThreshold,
                static_cast<int>(smallConstraintData_.ActiveCount()),
                [this, inverseDt](Parallelization::Batch batch)
                {
                    IterateSmallConstraints(batch, inverseDt);
                },
                false);

            Parallelization::ThreadPool::Instance().Execute();
        }

        return;
    }

    Parallelization::Batch batchContacts {0, static_cast<int>(contactData_.ActiveCount())};
    Parallelization::Batch batchConstraints {0, static_cast<int>(constraintData_.ActiveCount())};
    Parallelization::Batch batchSmallConstraints {0, static_cast<int>(smallConstraintData_.ActiveCount())};

    PrepareContacts(batchContacts, inverseDt);
    PrepareConstraints(batchConstraints, inverseDt);
    PrepareSmallConstraints(batchSmallConstraints, inverseDt);

    for (int iteration = 0; iteration < solverIterations; ++iteration)
    {
        IterateContacts(batchContacts, true);
        IterateConstraints(batchConstraints, inverseDt);
        IterateSmallConstraints(batchSmallConstraints, inverseDt);
    }
}

void World::RelaxVelocities(int iterations, bool multithread)
{
    IterateStoredContacts(false, iterations, multithread);
}

void World::BuildIslandLookup()
{
    handleToIsland_.clear();

    const auto activeBodies = bodiesSet_.Active();
    for (RigidBody* body : activeBodies)
    {
        handleToIsland_.emplace(body->Data()._index, body->InternalIsland->SetIndex);
    }
}

int World::IslandOf(Unmanaged::JHandle<RigidBodyData> body1, Unmanaged::JHandle<RigidBodyData> body2) const
{
    return body1.Data().MotionTypeValue() != MotionType::Static
        ? handleToIsland_.at(body1.Data()._index)
        : handleToIsland_.at(body2.Data()._index);
}

void World::SortForIslands()
{
    sortedContacts_.clear();
    auto contacts = contactData_.Active();
    for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
    {
        ContactData& contact = contacts[static_cast<std::size_t>(i)];
        sortedContacts_.push_back(ContactEntry {
            i,
            IslandOf(contact.Body1, contact.Body2),
            contact.Key});
    }

    std::sort(
        sortedContacts_.begin(),
        sortedContacts_.end(),
        [](const ContactEntry& left, const ContactEntry& right)
        {
            if (left.IslandIndex != right.IslandIndex)
            {
                return left.IslandIndex < right.IslandIndex;
            }

            if (left.Key.Key1 != right.Key.Key1)
            {
                return left.Key.Key1 < right.Key.Key1;
            }

            return left.Key.Key2 < right.Key.Key2;
        });

    sortedSmallConstraints_.clear();
    auto smallConstraints = smallConstraintData_.Active();
    for (int i = 0; i < static_cast<int>(smallConstraints.size()); ++i)
    {
        Dynamics::Constraints::SmallConstraintData& constraint = smallConstraints[static_cast<std::size_t>(i)];
        Unmanaged::JHandle<Dynamics::Constraints::SmallConstraintData> handle =
            smallConstraintData_.GetHandle(constraint);

        sortedSmallConstraints_.push_back(SmallConstraintEntry {
            handle,
            IslandOf(handle.Data().Body1, handle.Data().Body2),
            handle.Data().ConstraintId});
    }

    sortedConstraints_.clear();
    auto constraints = constraintData_.Active();
    for (int i = 0; i < static_cast<int>(constraints.size()); ++i)
    {
        Dynamics::Constraints::ConstraintData& constraint = constraints[static_cast<std::size_t>(i)];
        Unmanaged::JHandle<Dynamics::Constraints::ConstraintData> handle = constraintData_.GetHandle(constraint);

        sortedConstraints_.push_back(ConstraintEntry {
            handle,
            IslandOf(handle.Data().Body1, handle.Data().Body2),
            handle.Data().ConstraintId});
    }

    std::sort(
        sortedSmallConstraints_.begin(),
        sortedSmallConstraints_.end(),
        [](const SmallConstraintEntry& left, const SmallConstraintEntry& right)
        {
            if (left.IslandIndex != right.IslandIndex)
            {
                return left.IslandIndex < right.IslandIndex;
            }

            return left.ConstraintId < right.ConstraintId;
        });

    std::sort(
        sortedConstraints_.begin(),
        sortedConstraints_.end(),
        [](const ConstraintEntry& left, const ConstraintEntry& right)
        {
            if (left.IslandIndex != right.IslandIndex)
            {
                return left.IslandIndex < right.IslandIndex;
            }

            return left.ConstraintId < right.ConstraintId;
        });
}

void World::BuildIslandRanges()
{
    islandRanges_.clear();

    int contactIndex = 0;
    int smallConstraintIndex = 0;
    int constraintIndex = 0;
    const int contactCount = static_cast<int>(sortedContacts_.size());
    const int smallConstraintCount = static_cast<int>(sortedSmallConstraints_.size());
    const int constraintCount = static_cast<int>(sortedConstraints_.size());

    while (contactIndex < contactCount
        || smallConstraintIndex < smallConstraintCount
        || constraintIndex < constraintCount)
    {
        int minIsland = std::numeric_limits<int>::max();
        if (contactIndex < contactCount)
        {
            minIsland = std::min(minIsland, sortedContacts_[static_cast<std::size_t>(contactIndex)].IslandIndex);
        }

        if (smallConstraintIndex < smallConstraintCount)
        {
            minIsland = std::min(
                minIsland,
                sortedSmallConstraints_[static_cast<std::size_t>(smallConstraintIndex)].IslandIndex);
        }

        if (constraintIndex < constraintCount)
        {
            minIsland = std::min(
                minIsland,
                sortedConstraints_[static_cast<std::size_t>(constraintIndex)].IslandIndex);
        }

        IslandRange range;
        range.ContactStart = contactIndex;
        while (contactIndex < contactCount
            && sortedContacts_[static_cast<std::size_t>(contactIndex)].IslandIndex == minIsland)
        {
            ++contactIndex;
        }
        range.ContactEnd = contactIndex;

        range.SmallStart = smallConstraintIndex;
        while (smallConstraintIndex < smallConstraintCount
            && sortedSmallConstraints_[static_cast<std::size_t>(smallConstraintIndex)].IslandIndex == minIsland)
        {
            ++smallConstraintIndex;
        }
        range.SmallEnd = smallConstraintIndex;

        range.ConstraintStart = constraintIndex;
        while (constraintIndex < constraintCount
            && sortedConstraints_[static_cast<std::size_t>(constraintIndex)].IslandIndex == minIsland)
        {
            ++constraintIndex;
        }
        range.ConstraintEnd = constraintIndex;

        islandRanges_.push_back(range);
    }
}

void World::PrepareIslandSolveOrder()
{
    BuildIslandLookup();
    SortForIslands();
    BuildIslandRanges();
}

void World::SolveIslandBatch(Parallelization::Batch batch, Real inverseDt)
{
    auto contacts = contactData_.Active();

    for (int index = batch.Start; index < batch.End; ++index)
    {
        const IslandRange& range = islandRanges_[static_cast<std::size_t>(index)];

        for (int contactIndex = range.ContactStart; contactIndex < range.ContactEnd; ++contactIndex)
        {
            ContactData& contact =
                contacts[static_cast<std::size_t>(sortedContacts_[static_cast<std::size_t>(contactIndex)].Index)];
            contact.PrepareForIteration(inverseDt);
        }

        for (int constraintIndex = range.ConstraintStart; constraintIndex < range.ConstraintEnd; ++constraintIndex)
        {
            Dynamics::Constraints::ConstraintData& constraint =
                sortedConstraints_[static_cast<std::size_t>(constraintIndex)].Handle.Data();
            if (constraint.IsEnabled())
            {
                constraint.PrepareForIteration(constraint, inverseDt);
            }
        }

        for (int constraintIndex = range.SmallStart; constraintIndex < range.SmallEnd; ++constraintIndex)
        {
            Dynamics::Constraints::SmallConstraintData& constraint =
                sortedSmallConstraints_[static_cast<std::size_t>(constraintIndex)].Handle.Data();
            if (constraint.IsEnabled())
            {
                constraint.PrepareForIteration(constraint, inverseDt);
            }
        }

        for (int iteration = 0; iteration < islandSolverIterations_; ++iteration)
        {
            for (int contactIndex = range.ContactStart; contactIndex < range.ContactEnd; ++contactIndex)
            {
                ContactData& contact =
                    contacts[static_cast<std::size_t>(sortedContacts_[static_cast<std::size_t>(contactIndex)].Index)];
                contact.Iterate(true);
            }

            for (int constraintIndex = range.ConstraintStart; constraintIndex < range.ConstraintEnd; ++constraintIndex)
            {
                Dynamics::Constraints::ConstraintData& constraint =
                    sortedConstraints_[static_cast<std::size_t>(constraintIndex)].Handle.Data();
                if (constraint.IsEnabled())
                {
                    constraint.Iterate(constraint, inverseDt);
                }
            }

            for (int constraintIndex = range.SmallStart; constraintIndex < range.SmallEnd; ++constraintIndex)
            {
                Dynamics::Constraints::SmallConstraintData& constraint =
                    sortedSmallConstraints_[static_cast<std::size_t>(constraintIndex)].Handle.Data();
                if (constraint.IsEnabled())
                {
                    constraint.Iterate(constraint, inverseDt);
                }
            }
        }
    }
}

void World::RelaxIslandBatch(Parallelization::Batch batch)
{
    auto contacts = contactData_.Active();

    for (int index = batch.Start; index < batch.End; ++index)
    {
        const IslandRange& range = islandRanges_[static_cast<std::size_t>(index)];

        for (int iteration = 0; iteration < islandRelaxationIterations_; ++iteration)
        {
            for (int contactIndex = range.ContactStart; contactIndex < range.ContactEnd; ++contactIndex)
            {
                ContactData& contact =
                    contacts[static_cast<std::size_t>(sortedContacts_[static_cast<std::size_t>(contactIndex)].Index)];
                contact.Iterate(false);
            }
        }
    }
}

void World::SolveIslands(bool multithread, Real inverseDt, int iterations)
{
    islandSolverIterations_ = iterations;

    const int islandCount = static_cast<int>(islandRanges_.size());
    if (islandCount == 0)
    {
        return;
    }

    if (!multithread)
    {
        SolveIslandBatch(Parallelization::Batch {0, islandCount}, inverseDt);
        return;
    }

    int numTasks = std::min(islandCount, Parallelization::ThreadPool::Instance().ThreadCount());
    if (numTasks <= 1)
    {
        SolveIslandBatch(Parallelization::Batch {0, islandCount}, inverseDt);
        return;
    }

    Parallelization::ForBatch(
        0,
        islandCount,
        numTasks,
        [this, inverseDt](Parallelization::Batch batch)
        {
            SolveIslandBatch(batch, inverseDt);
        });
}

void World::RelaxIslands(bool multithread, int iterations)
{
    islandRelaxationIterations_ = iterations;

    const int islandCount = static_cast<int>(islandRanges_.size());
    if (islandCount == 0)
    {
        return;
    }

    if (!multithread)
    {
        RelaxIslandBatch(Parallelization::Batch {0, islandCount});
        return;
    }

    int numTasks = std::min(islandCount, Parallelization::ThreadPool::Instance().ThreadCount());
    if (numTasks <= 1)
    {
        RelaxIslandBatch(Parallelization::Batch {0, islandCount});
        return;
    }

    Parallelization::ForBatch(
        0,
        islandCount,
        numTasks,
        [this](Parallelization::Batch batch)
        {
            RelaxIslandBatch(batch);
        });
}

void World::ReorderContacts()
{
    const int activeCount = static_cast<int>(contactData_.ActiveCount());

    if (activeCount < 1024)
    {
        return;
    }

    constexpr int fraction = 1024;
    const int iterations = activeCount / fraction;

    for (int iter = 0; iter < iterations; ++iter)
    {
        if (sortCounter_ > activeCount - 2)
        {
            sortCounter_ = 0;
        }

        auto active = contactData_.Active();
        ContactData& current = active[static_cast<std::size_t>(sortCounter_)];
        ContactData& next = active[static_cast<std::size_t>(sortCounter_ + 1)];

        if (current.Body1 == next.Body1 || current.Body1 == next.Body2)
        {
            lastVisited_ = current.Body1;
            sortCounter_ += 1;
            continue;
        }

        if (current.Body2 == next.Body1 || current.Body2 == next.Body2)
        {
            lastVisited_ = current.Body2;
            sortCounter_ += 1;
            continue;
        }

        int swaps = 0;
        std::unique_ptr<Arbiter>* stored = arbiters_.TryGetValue(current.Key);
        if (stored != nullptr && *stored)
        {
            Arbiter& arbiter = **stored;
            if (lastVisited_ != arbiter.Body1().Handle())
            {
                for (Arbiter* contact : arbiter.Body1().Contacts())
                {
                    const int index = static_cast<int>(contactData_.GetIndex(contact->Handle()));
                    if (index <= sortCounter_ || index >= activeCount)
                    {
                        continue;
                    }

                    ++swaps;
                    contactData_.Swap(static_cast<std::size_t>(sortCounter_ + swaps), static_cast<std::size_t>(index));
                }

                lastVisited_ = arbiter.Body1().Handle();
            }
            else
            {
                for (Arbiter* contact : arbiter.Body2().Contacts())
                {
                    const int index = static_cast<int>(contactData_.GetIndex(contact->Handle()));
                    if (index <= sortCounter_ || index >= activeCount)
                    {
                        continue;
                    }

                    ++swaps;
                    contactData_.Swap(static_cast<std::size_t>(sortCounter_ + swaps), static_cast<std::size_t>(index));
                }

                lastVisited_ = arbiter.Body2().Handle();
            }
        }

        sortCounter_ += std::max(1, swaps);
    }
}

void World::HandleDeferredArbiters()
{
    const int count = deferredArbiters_.Count();
    for (int i = 0; i < count; ++i)
    {
        Arbiter* arbiter = deferredArbiters_[i];
        Collision::IslandHelper::ArbiterCreated(islands_, *arbiter);
        AddToActiveList(*arbiter->Body1().InternalIsland);
        AddToActiveList(*arbiter->Body2().InternalIsland);
        arbiter->Body1().RaiseBeginCollide(*arbiter);
        arbiter->Body2().RaiseBeginCollide(*arbiter);
    }

    deferredArbiters_.Clear();
}

void World::RemoveBrokenArbiters()
{
    const int count = brokenArbiters_.Count();
    for (int i = 0; i < count; ++i)
    {
        Unmanaged::JHandle<ContactData> handle = brokenArbiters_[i];
        ContactData& data = handle.Data();
        if ((data.UsageMask & ContactData::MaskContactAll) != 0)
        {
            continue;
        }

        const ArbiterKey key = data.Key;
        Arbiter* arbiter = nullptr;
        {
            std::lock_guard lock(arbiters_.GetLock(key));
            std::unique_ptr<Arbiter>* stored = arbiters_.TryGetValue(key);
            if (stored == nullptr || !*stored)
            {
                continue;
            }

            arbiter = stored->get();
        }

        AddToActiveList(*arbiter->Body1().InternalIsland);
        AddToActiveList(*arbiter->Body2().InternalIsland);
        contactData_.Free(handle);
        Collision::IslandHelper::ArbiterRemoved(islands_, *arbiter);

        std::unique_ptr<Arbiter> removed;
        {
            std::lock_guard lock(arbiters_.GetLock(key));
            removed = arbiters_.Take(key);
        }

        if (!removed)
        {
            continue;
        }

        removed->Body1().RaiseEndCollide(*removed);
        removed->Body2().RaiseEndCollide(*removed);

        Arbiter::ReturnToPool(std::move(removed));
    }

    brokenArbiters_.Clear();
}

void World::UpdateContacts(Parallelization::Batch batch)
{
    auto active = contactData_.Active();
    for (int i = batch.Start; i < batch.End; ++i)
    {
        ContactData& data = active[static_cast<std::size_t>(i)];
        data.UpdatePosition();

        if ((data.UsageMask & ContactData::MaskContactAll) == 0)
        {
            brokenArbiters_.ConcurrentAdd(contactData_.GetHandle(data));
        }
    }
}

void World::UpdateStoredContacts(bool multithread)
{
    ExecuteContactBatches(
        multithread,
        BodyTaskThreshold,
        [this](Parallelization::Batch batch)
        {
            UpdateContacts(batch);
        });
}

void World::ResolveNarrowPhaseContacts(Real stepDt, std::uint64_t stepMarker, bool multithread)
{
    DynamicTree().EnumerateOverlaps(
        [this, stepDt, stepMarker](Collision::DynamicTree::Proxy& proxyA, Collision::DynamicTree::Proxy& proxyB)
        {
            if (BroadPhaseFilter != nullptr && !BroadPhaseFilter->Filter(proxyA, proxyB))
            {
                return;
            }

            auto finishContact =
                [this, stepMarker]<typename TShapeA, typename TShapeB>(
                    const TShapeA& shapeA,
                    const TShapeB& shapeB,
                    RigidBody& bodyA,
                    RigidBody& bodyB,
                    const LinearMath::JQuaternion& orientationA,
                    const LinearMath::JQuaternion& orientationB,
                    const LinearMath::JVector& positionA,
                    const LinearMath::JVector& positionB,
                    LinearMath::JVector pointA,
                    LinearMath::JVector pointB,
                    LinearMath::JVector normal,
                    Real penetration,
                    bool useAuxiliaryContacts,
                    ContactData::SolveMode removeFlags)
            {
                if (&bodyA == &bodyB)
                {
                    return;
                }

                if (bodyA.MotionTypeValue() != MotionType::Dynamic
                    && bodyB.MotionTypeValue() != MotionType::Dynamic)
                {
                    return;
                }

                if (!bodyA.IsActive() && !bodyB.IsActive())
                {
                    return;
                }

                if constexpr (std::is_same_v<std::decay_t<TShapeA>, Collision::Shapes::RigidBodyShape>
                    && std::is_same_v<std::decay_t<TShapeB>, Collision::Shapes::RigidBodyShape>)
                {
                    if (NarrowPhaseFilter != nullptr
                        && !NarrowPhaseFilter->Filter(shapeA, shapeB, pointA, pointB, normal, penetration))
                    {
                        return;
                    }
                }

                ContactCandidate contact;
                contact.BodyA = &bodyA;
                contact.BodyB = &bodyB;
                contact.Normal = normal;
                contact.Penetration = penetration;
                contact.RemoveFlags = removeFlags;

                if (useAuxiliaryContacts && EnableAuxiliaryContactPoints)
                {
                    Collision::CollisionManifold manifold;
                    manifold.BuildManifold(
                        shapeA,
                        shapeB,
                        orientationA,
                        orientationB,
                        positionA,
                        positionB,
                        pointA,
                        pointB,
                        normal);

                    contact.PointCount = std::min(
                        manifold.Count(),
                        Collision::CollisionManifold::SolverContactLimit);
                    for (int i = 0; i < contact.PointCount; ++i)
                    {
                        contact.PointsA[static_cast<std::size_t>(i)] = manifold.ManifoldA(i);
                        contact.PointsB[static_cast<std::size_t>(i)] = manifold.ManifoldB(i);
                    }
                }

                if (contact.PointCount == 0)
                {
                    contact.PointsA[0] = pointA;
                    contact.PointsB[0] = pointB;
                    contact.PointCount = 1;
                }

                const ArbiterKey key {
                    std::min(shapeA.ShapeId(), shapeB.ShapeId()),
                    std::max(shapeA.ShapeId(), shapeB.ShapeId())};

                Arbiter* arbiter = nullptr;
                GetOrCreateArbiter(key.Key1, key.Key2, bodyA, bodyB, arbiter);
                arbiter->lastSeenStep_ = stepMarker;

                ContactData& contactData = arbiter->Data();
                if (!PersistentContactManifold)
                {
                    contactData.UsageMask = 0;
                }

                contactData.ResetMode(removeFlags);
                for (int i = 0; i < contact.PointCount; ++i)
                {
                    contactData.AddContact(
                        contact.PointsA[static_cast<std::size_t>(i)],
                        contact.PointsB[static_cast<std::size_t>(i)],
                        contact.Normal);
                }

            };

            auto* rigidA = dynamic_cast<Collision::Shapes::RigidBodyShape*>(&proxyA);
            auto* rigidB = dynamic_cast<Collision::Shapes::RigidBodyShape*>(&proxyB);

            if (rigidA != nullptr && rigidB != nullptr)
            {
                if (rigidB->ShapeId() < rigidA->ShapeId())
                {
                    std::swap(rigidA, rigidB);
                }

                RigidBody* bodyA = rigidA->GetRigidBody();
                RigidBody* bodyB = rigidB->GetRigidBody();
                if (bodyA == nullptr || bodyB == nullptr)
                {
                    return;
                }

                if (!bodyA->IsActive() && !bodyB->IsActive())
                {
                    return;
                }

                if (bodyA->MotionTypeValue() != MotionType::Dynamic
                    && bodyB->MotionTypeValue() != MotionType::Dynamic)
                {
                    return;
                }

                JVector pointA;
                JVector pointB;
                JVector normal;
                Real penetration = static_cast<Real>(0);
                const bool colliding = Collision::NarrowPhase::MprEpa(
                        *rigidA,
                        *rigidB,
                        bodyA->Orientation(),
                        bodyB->Orientation(),
                        bodyA->Position(),
                        bodyB->Position(),
                        pointA,
                        pointB,
                        normal,
                        penetration);

                if (!colliding)
                {
                    const bool speculative =
                        bodyA->EnableSpeculativeContacts() || bodyB->EnableSpeculativeContacts();
                    if (!speculative)
                    {
                        return;
                    }

                    const JVector deltaVelocity = bodyB->Velocity() - bodyA->Velocity();
                    if (deltaVelocity.LengthSquared()
                        < SpeculativeVelocityThreshold * SpeculativeVelocityThreshold)
                    {
                        return;
                    }

                    Real timeOfImpact = static_cast<Real>(0);
                    const bool sweepHit = Collision::NarrowPhase::Sweep(
                        *rigidA,
                        *rigidB,
                        bodyA->Orientation(),
                        bodyB->Orientation(),
                        bodyA->Position(),
                        bodyB->Position(),
                        bodyA->Velocity(),
                        bodyB->Velocity(),
                        pointA,
                        pointB,
                        normal,
                        timeOfImpact);

                    if (!sweepHit || timeOfImpact > stepDt || timeOfImpact == static_cast<Real>(0))
                    {
                        return;
                    }

                    penetration = LinearMath::JVector::Dot(normal, pointA - pointB)
                        * SpeculativeRelaxationFactor;

                    if (NarrowPhaseFilter != nullptr
                        && !NarrowPhaseFilter->Filter(*rigidA, *rigidB, pointA, pointB, normal, penetration))
                    {
                        return;
                    }

                    finishContact(
                        *rigidA,
                        *rigidB,
                        *bodyA,
                        *bodyB,
                        bodyA->Orientation(),
                        bodyB->Orientation(),
                        bodyA->Position(),
                        bodyB->Position(),
                        pointA,
                        pointB,
                        normal,
                        penetration,
                        false,
                        ContactData::SolveMode::Angular);
                    return;
                }

                finishContact(
                    *rigidA,
                    *rigidB,
                    *bodyA,
                    *bodyB,
                    bodyA->Orientation(),
                    bodyB->Orientation(),
                    bodyA->Position(),
                    bodyB->Position(),
                    pointA,
                    pointB,
                    normal,
                    penetration,
                    true,
                    ContactData::SolveMode::None);
                return;
            }

            throw InvalidCollisionTypeException(typeid(proxyA), typeid(proxyB));
        },
        multithread);
}

void RigidBody::ClearContactCache()
{
    for (Arbiter* arbiter : InternalContacts)
    {
        arbiter->Data().UsageMask = 0;
    }
}

void RigidBody::RegisterShape(Collision::Shapes::RigidBodyShape& shape)
{
    world_->DynamicTree().AddProxy(shape, IsActive());
}

void RigidBody::MotionTypeValue(MotionType value)
{
    if (Data().MotionTypeValue() == value)
    {
        return;
    }

    switch (value)
    {
    case MotionType::Static:
        ClearQueuedForces();
        if (world_ != nullptr)
        {
            world_->RemoveConnections(*this);
        }
        Data().MotionTypeValue(MotionType::Static);
        if (world_ != nullptr)
        {
            world_->RemoveStaticStaticConstraints(*this);
            world_->DeactivateBodyNextStep(*this);
        }
        Data().Velocity = LinearMath::JVector::Zero();
        Data().AngularVelocity = LinearMath::JVector::Zero();
        UpdateWorldInertia();
        break;
    case MotionType::Kinematic:
        ClearQueuedForces();
        if (Data().MotionTypeValue() == MotionType::Static && world_ != nullptr)
        {
            Data().MotionTypeValue(MotionType::Kinematic);
            world_->BuildConnectionsFromExistingContacts(*this);
        }
        Data().MotionTypeValue(MotionType::Kinematic);
        if (world_ != nullptr)
        {
            world_->RemoveStaticStaticConstraints(*this);
            world_->ActivateBodyNextStep(*this, true);
        }
        UpdateWorldInertia();
        break;
    case MotionType::Dynamic:
        ClearQueuedForces();
        if (Data().MotionTypeValue() == MotionType::Static && world_ != nullptr)
        {
            Data().MotionTypeValue(MotionType::Dynamic);
            world_->BuildConnectionsFromExistingContacts(*this);
        }
        Data().MotionTypeValue(MotionType::Dynamic);
        if (world_ != nullptr)
        {
            world_->ActivateBodyNextStep(*this, true);
        }
        UpdateWorldInertia();
        break;
    default:
        throw std::out_of_range("MotionType is out of range.");
    }
}

void RigidBody::UnregisterShape(Collision::Shapes::RigidBodyShape& shape)
{
    if (world_ != nullptr)
    {
        std::vector<Arbiter*> contactsToRemove;
        contactsToRemove.reserve(InternalContacts.size());

        const std::uint64_t shapeId = shape.ShapeId();
        for (Arbiter* arbiter : InternalContacts)
        {
            const ArbiterKey& key = arbiter->Key();
            if (key.Key1 == shapeId || key.Key2 == shapeId)
            {
                contactsToRemove.push_back(arbiter);
            }
        }

        for (Arbiter* arbiter : contactsToRemove)
        {
            world_->Remove(*arbiter);
        }

        world_->DynamicTree().RemoveProxy(shape);
    }
}

void RigidBody::UpdateShapeProxy(Collision::Shapes::RigidBodyShape& shape)
{
    if (world_ != nullptr)
    {
        world_->DynamicTree().UpdateProxy(shape);
    }
}

void RigidBody::RequestActivation(bool wakeUpStatic)
{
    if (world_ != nullptr)
    {
        world_->ActivateBodyNextStep(*this, wakeUpStatic);
    }
}

void RigidBody::RequestDeactivation()
{
    if (world_ != nullptr)
    {
        world_->DeactivateBodyNextStep(*this);
    }
}

namespace Collision::Shapes
{

Shape::Shape()
    : shapeId_(World::RequestId())
{
}

LinearMath::JVector RigidBodyShape::Velocity() const
{
    return rigidBody_ != nullptr ? rigidBody_->Velocity() : LinearMath::JVector::Zero();
}

LinearMath::JVector RigidBodyShape::WorldPosition() const
{
    return rigidBody_ != nullptr ? rigidBody_->Position() : Position;
}

LinearMath::JQuaternion RigidBodyShape::WorldOrientation() const
{
    return rigidBody_ != nullptr ? rigidBody_->Orientation() : Orientation;
}

void RigidBodyShape::CalculateBoundingBox(
    const LinearMath::JQuaternion& orientation,
    const LinearMath::JVector& position,
    LinearMath::JBoundingBox& box) const
{
    ShapeHelper::CalculateBoundingBox(*this, orientation, position, box);
}

void RigidBodyShape::CalculateMassInertia(
    LinearMath::JMatrix& inertia,
    LinearMath::JVector& centerOfMass,
    Real& mass) const
{
    ShapeHelper::CalculateMassInertia(*this, inertia, centerOfMass, mass);
}

bool RigidBodyShape::LocalRayCast(
    const LinearMath::JVector& origin,
    const LinearMath::JVector& direction,
    LinearMath::JVector& normal,
    Real& lambda) const
{
    return Collision::NarrowPhase::RayCast(*this, origin, direction, lambda, normal);
}

bool RigidBodyShape::Sweep(
    const Collision::ISupportMappable& support,
    const LinearMath::JQuaternion& orientation,
    const LinearMath::JVector& position,
    const LinearMath::JVector& sweep,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& lambda) const
{
    if (rigidBody_ == nullptr)
    {
        const bool hit = Collision::NarrowPhase::Sweep(
            *this,
            support,
            orientation,
            position,
            sweep,
            pointB,
            pointA,
            normal,
            lambda);
        normal = -normal;
        return hit;
    }

    return Collision::NarrowPhase::Sweep(
        support,
        *this,
        orientation,
        rigidBody_->Orientation(),
        position,
        rigidBody_->Position(),
        sweep,
        LinearMath::JVector::Zero(),
        pointA,
        pointB,
        normal,
        lambda);
}

bool RigidBodyShape::Distance(
    const Collision::ISupportMappable& support,
    const LinearMath::JQuaternion& orientation,
    const LinearMath::JVector& position,
    LinearMath::JVector& pointA,
    LinearMath::JVector& pointB,
    LinearMath::JVector& normal,
    Real& distance) const
{
    if (rigidBody_ == nullptr)
    {
        return Collision::NarrowPhase::Distance(
            support,
            *this,
            orientation,
            LinearMath::JQuaternion::Identity(),
            position,
            LinearMath::JVector::Zero(),
            pointA,
            pointB,
            normal,
            distance);
    }

    return Collision::NarrowPhase::Distance(
        support,
        *this,
        orientation,
        rigidBody_->Orientation(),
        position,
        rigidBody_->Position(),
        pointA,
        pointB,
        normal,
        distance);
}

void RigidBodyShape::UpdateWorldBoundingBox(Real dt)
{
    CalculateBoundingBox(WorldOrientation(), WorldPosition(), worldBoundingBox_);
    if (rigidBody_ != nullptr && rigidBody_->EnableSpeculativeContacts())
    {
        SweptExpandBoundingBox(dt);
    }
}

} // namespace Collision::Shapes

} // namespace Jitter2
