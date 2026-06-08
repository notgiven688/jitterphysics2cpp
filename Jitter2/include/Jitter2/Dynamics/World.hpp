#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Jitter2/Collision/CollisionFilter/IBroadPhaseFilter.hpp>
#include <Jitter2/Collision/CollisionFilter/TriangleEdgeCollisionFilter.hpp>
#include <Jitter2/Collision/CollisionIsland.hpp>
#include <Jitter2/Collision/DynamicTree/DynamicTree.hpp>
#include <Jitter2/Collision/IslandHelper.hpp>
#include <Jitter2/Collision/NarrowPhase/CollisionManifold.hpp>
#include <Jitter2/DataStructures/PartitionedSet.hpp>
#include <Jitter2/DataStructures/ShardedDictionary.hpp>
#include <Jitter2/DataStructures/SlimBag.hpp>
#include <Jitter2/Dynamics/Arbiter.hpp>
#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Parallelization/Parallel.hpp>
#include <Jitter2/Precision.hpp>
#include <Jitter2/Unmanaged/PartitionedBuffer.hpp>

namespace Jitter2
{

enum class SolveMode
{

    // Standard parallel solver. Contacts and constraints are distributed across threads
    // in fixed-size batches. An incremental cache-friendly reorder pass keeps related
    // contacts adjacent in memory. Fast, but non-deterministic.

    Regular,

    // Island-based deterministic solver. Each simulation island is solved sequentially
    // as an independent task. Islands are distributed across threads when multithreading
    // is enabled. This mode can be significantly slower than Regular.

    Deterministic
};

// Controls how internal worker threads behave between calls to World::Step.
enum class ThreadModelType
{

    // Worker threads may yield when the engine is idle. Lower background CPU usage.

    Regular,

    // Worker threads remain active between steps to reduce wake-up latency,
    // at the cost of higher background CPU usage.

    Persistent
};

template<typename... Args>
class CallbackList
{
public:
    using Function = std::function<void(Args...)>;
    using Token = std::size_t;

    CallbackList() = default;

    CallbackList& operator=(Function function)
    {
        Clear();
        if (function)
        {
            Add(std::move(function));
        }
        return *this;
    }

    CallbackList& operator=(std::nullptr_t)
    {
        Clear();
        return *this;
    }

    CallbackList& operator+=(Function function)
    {
        Add(std::move(function));
        return *this;
    }

    Token Add(Function function)
    {
        if (!function)
        {
            return 0;
        }

        const Token token = nextToken_++;
        handlers_.push_back(Handler {token, std::move(function)});
        return token;
    }

    void Remove(Token token)
    {
        handlers_.erase(
            std::remove_if(
                handlers_.begin(),
                handlers_.end(),
                [token](const Handler& handler)
                {
                    return handler.TokenValue == token;
                }),
            handlers_.end());
    }

    void Clear()
    {
        handlers_.clear();
    }

    [[nodiscard]] explicit operator bool() const
    {
        return !handlers_.empty();
    }

    void operator()(Args... args) const
    {
        for (const Handler& handler : handlers_)
        {
            handler.FunctionValue(args...);
        }
    }

private:
    struct Handler
    {
        Token TokenValue = 0;
        Function FunctionValue;
    };

    std::vector<Handler> handlers_;
    Token nextToken_ = 1;
};

class SameBodyException : public std::invalid_argument
{
public:

    // Thrown when two body arguments are required to reference different bodies.

    SameBodyException()
        : std::invalid_argument("Body arguments must be different.")
    {
    }

    explicit SameBodyException(const std::string& message)
        : std::invalid_argument(message)
    {
    }
};

// Represents a simulation environment that holds and manages the state of all simulation objects.
// A single World instance must not be stepped or modified concurrently from multiple
// external threads. Different worlds may be stepped concurrently with multiThread: false.
// Calls using multiThread: true are internally serialized across all worlds because they share
// the process-wide Jitter worker pool.
class World
{
public:

    // Thrown when the narrow phase encounters a pair of proxy types it cannot process.
    // This typically indicates that non-RigidBodyShape proxies were inserted into the
    // world's DynamicTree. Use BroadPhaseFilter to filter such pairs,
    // or ensure only supported proxy types are added.
    class InvalidCollisionTypeException : public std::runtime_error
    {
    public:
        InvalidCollisionTypeException(const std::type_info& proxyA, const std::type_info& proxyB)
            : std::runtime_error(CreateMessage(proxyA, proxyB)),
              ProxyA(proxyA),
              ProxyB(proxyB)
        {
        }

        const std::type_info& ProxyA;
        const std::type_info& ProxyB;

    private:
        static std::string CreateMessage(const std::type_info& proxyA, const std::type_info& proxyB)
        {
            return "Don't know how to handle collision between " + std::string(proxyA.name())
                + " and " + std::string(proxyB.name())
                + ". Register a BroadPhaseFilter to handle and/or filter out these collision types.";
        }
    };

    // Profiling buckets for DebugTimings, representing stages of Step(Real, bool).
    enum class Timings
    {
    // Time spent in PreStep callbacks.
        PreStep = 0,
    // Time spent in narrow phase collision detection and contact generation.
        NarrowPhase = 1,
    // Time spent creating deferred arbiters.
        AddArbiter = 2,
    // Time spent reordering contacts (for cache optimization or deterministic results).
        ReorderContacts = 3,
    // Time spent evaluating body deactivation (sleeping).
        CheckDeactivation = 4,
    // Time spent in substeps: force integration, constraint solving, and velocity integration.
        Solve = 5,
    // Time spent removing broken arbiters.
        RemoveArbiter = 6,
    // Time spent updating contact state after solving.
        UpdateContacts = 7,
    // Time spent finalizing body state and broadphase proxy updates.
        UpdateBodies = 8,
    // Time spent updating the dynamic tree (broadphase).
        BroadPhase = 9,
    // Time spent in PostStep callbacks.
        PostStep = 10,
    // Sentinel value for array sizing. Not a real timing bucket.
        Last = 11
    };

    World();
    ~World() = default;

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) = delete;
    World& operator=(World&&) = delete;

    void Clear();

    // Performs a single simulation step.
    // dt: The duration of time to simulate in seconds. Should remain fixed and not exceed 1/60 s.
    // multithread: If true, uses the internal thread pool for parallel execution.
    // Set to false for single-threaded execution (useful for debugging or platforms without threading).
    // The step is divided into SubstepCount substeps for improved stability.
    // Each World must not be stepped or modified concurrently from multiple external threads.
    // Different worlds may be stepped concurrently with multithread set to false.
    // When multithread is true, the step uses a process-wide worker pool;
    // therefore multithreaded steps are internally serialized across all worlds.
    // Callbacks (PreStep, PostStep, etc.) are invoked on the calling thread.
    // When multithread is true, BroadPhaseFilter and
    // NarrowPhaseFilter may be called concurrently from worker threads.
    void Step(Real dt, bool multithread = true);

    // Solves the existing contacts and constraints at the velocity level without advancing body transforms.
    // dt: The reference timestep in seconds used to scale bias and softness terms.
    // solverIterations: The number of solver iterations to execute.
    // relaxationIterations: The number of relaxation iterations to execute after solving.
    // multithread: If true, uses the internal thread pool for parallel execution.
    // Multithreaded calls are internally serialized across all worlds because the worker pool is process-wide.
    // Unlike Step, this method does not perform broadphase or narrowphase collision detection,
    // does not integrate forces, and does not integrate positions or orientations. It only processes the
    // existing active contacts and constraints already present in the world at the velocity level.
    void Stabilize(
        Real dt,
        int solverIterations,
        int relaxationIterations = 0,
        bool multithread = true);

    RigidBody& CreateRigidBody();
    void Remove(RigidBody& body);
    template<typename TConstraint>
    TConstraint& CreateConstraint(RigidBody& body1, RigidBody& body2);
    void Remove(Dynamics::Constraints::Constraint& constraint);
    void Remove(Arbiter& arbiter);
    void ForceSleepIsland(Collision::Island& island);

    // Gets an existing Arbiter instance for the given pair of IDs.
    // id0: The first identifier (e.g., shape ID).
    // id1: The second identifier.
    // arbiter: The arbiter for the ordered ID pair, or nullptr if none exists.
    // Returns: true if an arbiter exists for the ordered ID pair; otherwise, false.
    // The order of id0 and id1 matters.
    // For arbiters created by the engine, id0 &lt; id1 holds
    // for RigidBodyShapes.
    [[nodiscard]] bool GetArbiter(std::uint64_t id0, std::uint64_t id1, Arbiter*& arbiter);

    // Gets an existing Arbiter instance for the given pair of IDs,
    // or creates a new one if none exists.
    // This method ensures there is a unique Arbiter for each ordered pair of IDs.
    // If an arbiter already exists, it is returned via the arbiter out parameter.
    // Otherwise, a new arbiter is allocated, initialized with the provided body1 and
    // body2, and registered internally. The body arguments are used only when a new
    // arbiter is created.
    // This method is thread-safe.
    // Note: The order of id0 and id1 does matter.
    void GetOrCreateArbiter(
        std::uint64_t id0,
        std::uint64_t id1,
        RigidBody& body1,
        RigidBody& body2,
        Arbiter*& arbiter);

    // Registers a single contact point into an existing Arbiter.
    // This method adds a contact point to the specified arbiter, using the provided contact
    // points and normal. All input vectors must be in world space. The normal vector must be
    // normalized. This method assumes that the arbiter is already valid and mapped to the
    // correct pair of bodies.
    void RegisterContact(
        Arbiter& arbiter,
        const LinearMath::JVector& point1,
        const LinearMath::JVector& point2,
        const LinearMath::JVector& normal,
        ContactData::SolveMode removeFlags = ContactData::SolveMode::None);

    // Registers one or more contact points between two rigid bodies using a CollisionManifold,
    // creating an Arbiter if one does not already exist.
    // This method ensures that contact information between the specified ID pair is tracked by an
    // Arbiter. If no arbiter exists for the given IDs, one is created using
    // body1 and body2.
    // This method is thread-safe.
    // Note: The order of id0 and id1 does matter.
    void RegisterContact(
        std::uint64_t id0,
        std::uint64_t id1,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& normal,
        const Collision::CollisionManifold& manifold,
        ContactData::SolveMode removeFlags = ContactData::SolveMode::None);

    // Registers a contact point between two rigid bodies, creating an Arbiter if one does not already exist.
    // This method ensures that contact information between the specified ID pair is tracked by an
    // Arbiter. If no arbiter exists for the given IDs, one is created using
    // body1 and body2. The provided contact points and normal must be in
    // world space. The normal vector must be normalized.
    // This method is thread-safe.
    // Note: The order of id0 and id1 does matter.
    void RegisterContact(
        std::uint64_t id0,
        std::uint64_t id1,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& point1,
        const LinearMath::JVector& point2,
        const LinearMath::JVector& normal,
        ContactData::SolveMode removeFlags = ContactData::SolveMode::None);

    [[nodiscard]] Collision::DynamicTree& DynamicTree() { return dynamicTree_; }
    [[nodiscard]] const Collision::DynamicTree& DynamicTree() const { return dynamicTree_; }
    [[nodiscard]] RigidBody& NullBody() const { return *nullBody_; }
    [[nodiscard]] const std::vector<RigidBody*>& RigidBodies() const { return bodyView_; }
    [[nodiscard]] const DataStructures::PartitionedSet<Collision::Island>& Islands() const { return islands_; }
    [[nodiscard]] const std::vector<std::unique_ptr<Dynamics::Constraints::Constraint>>& Constraints() const
    {
        return constraints_;
    }
    [[nodiscard]] std::span<ContactData> ActiveContacts() { return contactData_.Active(); }
    [[nodiscard]] std::span<const ContactData> ActiveContacts() const { return contactData_.Active(); }
    [[nodiscard]] std::size_t ContactCount() const { return contactData_.Count(); }
    [[nodiscard]] std::size_t ActiveContactCount() const { return contactData_.ActiveCount(); }
    [[nodiscard]] std::size_t ConstraintDataCount() const { return constraintData_.Count(); }
    [[nodiscard]] std::size_t ActiveConstraintDataCount() const { return constraintData_.ActiveCount(); }
    [[nodiscard]] std::size_t SmallConstraintDataCount() const { return smallConstraintData_.Count(); }
    [[nodiscard]] std::size_t ActiveSmallConstraintDataCount() const { return smallConstraintData_.ActiveCount(); }
    [[nodiscard]] std::size_t RigidBodyDataCount() const { return rigidBodyData_.Count(); }
    [[nodiscard]] std::size_t ActiveRigidBodyCount() const { return rigidBodyData_.ActiveCount(); }

    [[nodiscard]] Real Time() const { return time_; }
    [[nodiscard]] std::uint64_t StepCount() const { return stepCount_; }

    // Contains timings for the stages of the last call to World::Step.
    // Values are in milliseconds. Index using (int)Timings::XYZ.
    [[nodiscard]] std::span<const double> DebugTimings() const
    {
        return std::span<const double>(debugTimings_.data(), debugTimings_.size());
    }

    [[nodiscard]] static const char* TimingName(Timings timing)
    {
        switch (timing)
        {
        case Timings::PreStep: return "PreStep";
        case Timings::NarrowPhase: return "NarrowPhase";
        case Timings::AddArbiter: return "AddArbiter";
        case Timings::ReorderContacts: return "ReorderContacts";
        case Timings::CheckDeactivation: return "CheckDeactivation";
        case Timings::Solve: return "Solve";
        case Timings::RemoveArbiter: return "RemoveArbiter";
        case Timings::UpdateContacts: return "UpdateContacts";
        case Timings::UpdateBodies: return "UpdateBodies";
        case Timings::BroadPhase: return "BroadPhase";
        case Timings::PostStep: return "PostStep";
        case Timings::Last: return "Last";
        }
        return "Unknown";
    }

    [[nodiscard]] std::pair<int, int> SolverIterations() const
    {
        return {SolverVelocityIterations, SolverRelaxationIterations};
    }
    void SolverIterations(int solver, int relaxation);
    [[nodiscard]] static std::uint64_t RequestId();
    [[nodiscard]] static std::pair<std::uint64_t, std::uint64_t> RequestId(int count);
    [[nodiscard]] static bool TryLockTwoBody(RigidBodyData& body1, RigidBodyData& body2);
    static void LockTwoBody(RigidBodyData& body1, RigidBodyData& body2);
    static void UnlockTwoBody(RigidBodyData& body1, RigidBodyData& body2);
    [[nodiscard]] static bool DefaultDynamicTreeFilter(
        const Collision::DynamicTree::Proxy& proxyA,
        const Collision::DynamicTree::Proxy& proxyB);
    [[nodiscard]] static bool DefaultNarrowPhaseFilter(
        const Collision::Shapes::RigidBodyShape& shapeA,
        const Collision::Shapes::RigidBodyShape& shapeB,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& penetration);
    void ResetNarrowPhaseFilter();

    using WorldStepFunction = CallbackList<Real>;
    using IslandFunction = CallbackList<Collision::Island&>;

    // Hook into the broadphase collision detection pipeline. The default value is null.
    // Use this to intercept shape pairs before narrow-phase detection, implement custom collision layers,
    // or handle collisions for custom proxy types. When Step(Real, bool) is called
    // with multithread=true, this may be invoked concurrently. Implementations must be thread-safe.

    Collision::IBroadPhaseFilter* BroadPhaseFilter = nullptr;

    // Hook into the narrow-phase collision detection pipeline.
    // The default instance is of type TriangleEdgeCollisionFilter.
    // Use this to intercept collisions after contact generation, modify contact data,
    // or implement custom collision responses. When Step(Real, bool) is called
    // with multithread=true, this may be invoked concurrently. Implementations must be thread-safe.

    Collision::INarrowPhaseFilter* NarrowPhaseFilter = nullptr;

    // Raised at the beginning of a simulation step, before any collision detection,
    // constraint solving, or integration is performed.

    WorldStepFunction PreStep;

    // Raised at the end of a simulation step, after all substeps, collision handling,
    // and integration have completed.

    WorldStepFunction PostStep;

    // Raised at the beginning of each substep during a simulation step.

    WorldStepFunction PreSubStep;

    // Raised at the end of each substep during a simulation step.

    WorldStepFunction PostSubStep;

    // Raised when inactive non-static bodies in a simulation island are activated during a simulation step.

    IslandFunction IslandActivated;

    // Raised when active non-static bodies in a simulation island are deactivated during a simulation step.

    IslandFunction IslandDeactivated;

    // Default gravity, see also RigidBody::AffectedByGravity.

    LinearMath::JVector Gravity = LinearMath::JVector(0, static_cast<Real>(-9.81), 0);

    // Controls how internal worker threads behave between calls to Step.

    ThreadModelType ThreadModel = ThreadModelType::Regular;

    // The solver strategy used during Step. Defaults to Jitter2::SolveMode::Regular.
    // Jitter2::SolveMode::Deterministic can be significantly slower than
    // Jitter2::SolveMode::Regular.
    SolveMode SolverMode = SolveMode::Regular;

    // Enables automatic deactivation (sleeping) of islands that have remained below the
    // motion thresholds long enough.

    bool AllowDeactivation = true;

    // Enables the generation of additional contacts for flat surfaces that are in contact.
    // Traditionally, the collision system reports the deepest collision point between two objects.
    // A full contact manifold is then generated over several time steps using contact caching, which
    // can be unstable. This method attempts to build a fuller or complete contact manifold within a single time step.

    bool EnableAuxiliaryContactPoints = true;

    // When enabled (the default), contact points and their accumulated impulses are cached between
    // frames. This allows the solver to warm-start from the previous solution and enables the
    // manifold to grow over several steps, which improves stability for resting contacts.
    // When disabled, all contact data is discarded at the end of each frame and every contact is
    // treated as brand-new. Disabling this removes all frame-to-frame contact memory at the cost
    // of solver convergence speed.

    bool PersistentContactManifold = true;

    // A speculative contact slows a body down such that it does not penetrate or tunnel through
    // an obstacle within one frame. The SpeculativeRelaxationFactor scales the
    // slowdown, ranging from 0 (where the body stops immediately during this frame) to 1 (where the body and the
    // obstacle just touch after the next velocity integration). A value below 1 is preferred, as the leftover velocity
    // might be enough to trigger another speculative contact in the next frame.
    // Default value: 0.9.

    Real SpeculativeRelaxationFactor = static_cast<Real>(0.9);

    // Speculative contacts are generated when the relative velocity between two bodies exceeds
    // the threshold value. To prevent bodies with a diameter of D from tunneling through thin walls, this
    // threshold should be set to approximately D / timestep, e.g., 100 for a unit cube and a
    // timestep of 0.01.
    // Default value: 10.0.

    Real SpeculativeVelocityThreshold = static_cast<Real>(10);

    // Gets or sets the number of substeps performed per call to Step.

    int SubstepCount = 1;
    int SolverVelocityIterations = 6;
    int SolverRelaxationIterations = 4;

private:
    void StepInternal(Real dt, bool multithread);
    void StabilizeInternal(Real dt, int solverIterations, int relaxationIterations, bool multithread);
    void RebuildBodyView();
    void AddToActiveList(Collision::Island& island);
    void ActivateBodyNextStep(RigidBody& body, bool wakeUpStatic = false);
    void DeactivateBodyNextStep(RigidBody& body);
    void RemoveStaticStaticConstraints(RigidBody& body);
    void BuildConnectionsFromExistingContacts(RigidBody& body);
    void RemoveConnections(RigidBody& body);
    void ClearMotionForSleep(RigidBody& body);
    bool DeactivateBodyForSleep(RigidBody& body, bool clearMotion);
    void CheckDeactivation();
    void UpdateActiveBodies(Real stepDt, Real substepDt, bool multithread);
    void IntegrateForces(bool multithread);
    void IntegrateVelocities(Real substepDt, bool multithread);
    void ExecuteBodyBatches(
        bool multithread,
        int taskThreshold,
        const std::function<void(Parallelization::Batch)>& action);
    void ExecuteContactBatches(
        bool multithread,
        int taskThreshold,
        const std::function<void(Parallelization::Batch)>& action,
        bool execute = true);
    void ExecuteConstraintBatches(
        bool multithread,
        int taskThreshold,
        int count,
        const std::function<void(Parallelization::Batch)>& action,
        bool execute = true);
    void PrepareConstraints(Parallelization::Batch batch, Real inverseDt);
    void IterateConstraints(Parallelization::Batch batch, Real inverseDt);
    void PrepareSmallConstraints(Parallelization::Batch batch, Real inverseDt);
    void IterateSmallConstraints(Parallelization::Batch batch, Real inverseDt);
    void SolveConstraints(Real inverseDt, int solverIterations, bool multithread);
    void PrepareContacts(Parallelization::Batch batch, Real inverseDt);
    void IterateContacts(Parallelization::Batch batch, bool applyBias);
    void PrepareStoredContacts(Real inverseDt, bool multithread);
    void IterateStoredContacts(bool applyBias, int iterations, bool multithread);
    void SolveStoredContacts(Real inverseDt, int solverIterations, int relaxationIterations, bool multithread);
    void SolveVelocities(Real inverseDt, int solverIterations, bool multithread);
    void RelaxVelocities(int iterations, bool multithread);
    void ReorderContacts();
    void BuildIslandLookup();
    [[nodiscard]] int IslandOf(
        Unmanaged::JHandle<RigidBodyData> body1,
        Unmanaged::JHandle<RigidBodyData> body2) const;
    void SortForIslands();
    void BuildIslandRanges();
    void PrepareIslandSolveOrder();
    void SolveIslandBatch(Parallelization::Batch batch, Real inverseDt);
    void RelaxIslandBatch(Parallelization::Batch batch);
    void SolveIslands(bool multithread, Real inverseDt, int iterations);
    void RelaxIslands(bool multithread, int iterations);
    void HandleDeferredArbiters();
    void RemoveBrokenArbiters();
    void UpdateContacts(Parallelization::Batch batch);
    void UpdateStoredContacts(bool multithread);
    void ResolveNarrowPhaseContacts(Real stepDt, std::uint64_t stepMarker, bool multithread);

    Collision::DynamicTree dynamicTree_;
    Unmanaged::PartitionedBuffer<RigidBodyData> rigidBodyData_;
    Unmanaged::PartitionedBuffer<ContactData> contactData_;
    Unmanaged::PartitionedBuffer<Dynamics::Constraints::ConstraintData> constraintData_;
    Unmanaged::PartitionedBuffer<Dynamics::Constraints::SmallConstraintData> smallConstraintData_;
    std::vector<std::unique_ptr<RigidBody>> bodies_;
    DataStructures::PartitionedSet<RigidBody> bodiesSet_;
    DataStructures::PartitionedSet<Collision::Island> islands_;
    std::vector<std::unique_ptr<Dynamics::Constraints::Constraint>> constraints_;
    std::vector<RigidBody*> bodyView_;
    DataStructures::ShardedDictionary<ArbiterKey, std::unique_ptr<Arbiter>, ArbiterKeyHash> arbiters_;
    DataStructures::SlimBag<Arbiter*> deferredArbiters_;
    DataStructures::SlimBag<Unmanaged::JHandle<ContactData>> brokenArbiters_;
    struct IslandRange
    {
        int ContactStart = 0;
        int ContactEnd = 0;
        int SmallStart = 0;
        int SmallEnd = 0;
        int ConstraintStart = 0;
        int ConstraintEnd = 0;
    };

    struct ContactEntry
    {
        int Index = 0;
        int IslandIndex = 0;
        ArbiterKey Key {};
    };

    struct ConstraintEntry
    {
        Unmanaged::JHandle<Dynamics::Constraints::ConstraintData> Handle;
        int IslandIndex = 0;
        std::uint64_t ConstraintId = 0;
    };

    struct SmallConstraintEntry
    {
        Unmanaged::JHandle<Dynamics::Constraints::SmallConstraintData> Handle;
        int IslandIndex = 0;
        std::uint64_t ConstraintId = 0;
    };

    std::unordered_map<int, int> handleToIsland_;
    std::vector<IslandRange> islandRanges_;
    std::vector<ContactEntry> sortedContacts_;
    std::vector<SmallConstraintEntry> sortedSmallConstraints_;
    std::vector<ConstraintEntry> sortedConstraints_;
    int islandSolverIterations_ = 0;
    int islandRelaxationIterations_ = 0;
    Unmanaged::JHandle<RigidBodyData> lastVisited_;
    std::mutex contactDataMutex_;
    Collision::TriangleEdgeCollisionFilter defaultNarrowPhaseFilter_;
    RigidBody* nullBody_ = nullptr;
    Real time_ = 0;
    std::uint64_t stepCount_ = 0;
    std::array<double, static_cast<std::size_t>(Timings::Last)> debugTimings_ {};
    int sortCounter_ = 0;

    friend class RigidBody;
};

template<typename TConstraint>
TConstraint& World::CreateConstraint(RigidBody& body1, RigidBody& body2)
{
    static_assert(
        std::is_base_of_v<Dynamics::Constraints::Constraint, TConstraint>,
        "TConstraint must derive from Jitter2::Dynamics::Constraints::Constraint.");

    if (&body1.GetWorld() != this)
    {
        throw std::invalid_argument("The body does not belong to this world.");
    }

    if (&body2.GetWorld() != this)
    {
        throw std::invalid_argument("The body does not belong to this world.");
    }

    if (&body1 == &body2)
    {
        throw SameBodyException();
    }

    auto constraint = std::make_unique<TConstraint>();
    TConstraint& reference = *constraint;
    const std::uint64_t constraintId = RequestId();
    if (reference.IsSmallConstraint())
    {
        reference.Attach(smallConstraintData_.Allocate(true, true), body1, body2, constraintId);
    }
    else
    {
        reference.Attach(constraintData_.Allocate(true, true), body1, body2, constraintId);
    }
    constraints_.push_back(std::move(constraint));
    Collision::IslandHelper::ConstraintCreated(islands_, reference);
    AddToActiveList(*body1.InternalIsland);
    AddToActiveList(*body2.InternalIsland);
    return reference;
}

} // namespace Jitter2
