#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#if (!defined(JITTER_DOUBLE_PRECISION) || !JITTER_DOUBLE_PRECISION) \
    && (defined(__SSE__) || defined(_M_IX86_FP) || defined(_M_X64))
#define JITTER_TREEBOX_USE_SSE_FLOAT 1
#include <immintrin.h>
#else
#define JITTER_TREEBOX_USE_SSE_FLOAT 0
#endif

#include <Jitter2/Collision/DynamicTree/IDynamicTreeProxy.hpp>
#include <Jitter2/Collision/NarrowPhase/NarrowPhase.hpp>
#include <Jitter2/Collision/NarrowPhase/SupportPrimitives.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Collision/Shapes/ShapeHelper.hpp>
#include <Jitter2/DataStructures/PartitionedSet.hpp>
#include <Jitter2/DataStructures/SlimBag.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/Logger.hpp>
#include <Jitter2/Parallelization/ReaderWriterLock.hpp>
#include <Jitter2/Parallelization/ThreadPool.hpp>
#include <Jitter2/Precision.hpp>
#include <Jitter2/Tracer.hpp>

namespace Jitter2::Collision
{

// Represents an axis-aligned bounding box used for spatial partitioning in acceleration structures
// such as DynamicTree.
struct alignas(sizeof(Real) * 4) TreeBox
{
    // Small epsilon value used for ray-box intersection tests.
    static constexpr Real Epsilon = static_cast<Real>(1e-12);

    // The minimum corner of the bounding box.
    LinearMath::JVector Min = LinearMath::JVector::Zero();

    // Padding for SIMD-friendly four-lane loads.
    Real MinW = RealZero;

    // The maximum corner of the bounding box.
    LinearMath::JVector Max = LinearMath::JVector::Zero();

    // Padding for SIMD-friendly four-lane loads.
    Real MaxW = RealZero;

    TreeBox() = default;

    // Creates a new TreeBox from minimum and maximum corner vectors.
    TreeBox(const LinearMath::JVector& min, const LinearMath::JVector& max)
        : Min(min),
          MinW(RealZero),
          Max(max),
          MaxW(RealZero)
    {
    }

    // Creates a new TreeBox from an existing JBoundingBox.
    explicit TreeBox(const LinearMath::JBoundingBox& box)
        : Min(box.Min),
          MinW(RealZero),
          Max(box.Max),
          MaxW(RealZero)
    {
    }

    // Converts this TreeBox to a JBoundingBox.
    [[nodiscard]] LinearMath::JBoundingBox AsJBoundingBox() const
    {
        return LinearMath::JBoundingBox(Min, Max);
    }

    // Determines whether this box contains the specified point.
    [[nodiscard]] bool Contains(const LinearMath::JVector& point) const
    {
        return Min.X <= point.X && point.X <= Max.X
            && Min.Y <= point.Y && point.Y <= Max.Y
            && Min.Z <= point.Z && point.Z <= Max.Z;
    }

    // Determines whether this box completely encloses the specified box.
    [[nodiscard]] bool Contains(const LinearMath::JBoundingBox& box) const
    {
        return Min.X <= box.Min.X && Max.X >= box.Max.X
            && Min.Y <= box.Min.Y && Max.Y >= box.Max.Y
            && Min.Z <= box.Min.Z && Max.Z >= box.Max.Z;
    }

    // Determines whether this box completely encloses the specified box.
    [[nodiscard]] bool Contains(const TreeBox& box) const
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        const __m128 outerMin = _mm_load_ps(&Min.X);
        const __m128 innerMin = _mm_load_ps(&box.Min.X);
        const __m128 outerMax = _mm_load_ps(&Max.X);
        const __m128 innerMax = _mm_load_ps(&box.Max.X);
        const __m128 leMin = _mm_cmple_ps(outerMin, innerMin);
        const __m128 geMax = _mm_cmpge_ps(outerMax, innerMax);
        return (_mm_movemask_ps(_mm_and_ps(leMin, geMax)) & 0x0F) == 0x0F;
#else
        return Min.X <= box.Min.X && Max.X >= box.Max.X
            && Min.Y <= box.Min.Y && Max.Y >= box.Max.Y
            && Min.Z <= box.Min.Z && Max.Z >= box.Max.Z;
#endif
    }

    // Gets the center point of the bounding box.
    [[nodiscard]] LinearMath::JVector Center() const
    {
        return (Min + Max) * static_cast<Real>(0.5);
    }

    [[nodiscard]] double GetSurfaceArea() const
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        alignas(16) float extent[4];
        _mm_store_ps(extent, _mm_sub_ps(_mm_load_ps(&Max.X), _mm_load_ps(&Min.X)));
        return 2.0 * (extent[0] * extent[1] + extent[1] * extent[2] + extent[2] * extent[0]);
#else
        const LinearMath::JVector len = Max - Min;
        return static_cast<double>(static_cast<Real>(2) * (len.X * len.Y + len.Y * len.Z + len.Z * len.X));
#endif
    }

    // Checks if a ray intersects this bounding box.
    [[nodiscard]] bool RayIntersect(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        Real& enter) const
    {
        return AsJBoundingBox().RayIntersect(origin, direction, enter);
    }

    // Determines whether the two boxes are completely separated from each other.
    [[nodiscard]] static bool Disjoint(const TreeBox& left, const TreeBox& right)
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        const __m128 leftMax = _mm_load_ps(&left.Max.X);
        const __m128 rightMin = _mm_load_ps(&right.Min.X);
        const __m128 leftMin = _mm_load_ps(&left.Min.X);
        const __m128 rightMax = _mm_load_ps(&right.Max.X);
        const __m128 ltMin = _mm_cmplt_ps(leftMax, rightMin);
        const __m128 gtMax = _mm_cmpgt_ps(leftMin, rightMax);
        return (_mm_movemask_ps(_mm_or_ps(ltMin, gtMax)) & 0x0F) != 0;
#else
        return left.Max.X < right.Min.X || left.Min.X > right.Max.X
            || left.Max.Y < right.Min.Y || left.Min.Y > right.Max.Y
            || left.Max.Z < right.Min.Z || left.Min.Z > right.Max.Z;
#endif
    }

    // Creates a bounding box that encloses both specified boxes.
    static void CreateMerged(const TreeBox& left, const TreeBox& right, TreeBox& result)
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        _mm_store_ps(&result.Min.X, _mm_min_ps(_mm_load_ps(&left.Min.X), _mm_load_ps(&right.Min.X)));
        _mm_store_ps(&result.Max.X, _mm_max_ps(_mm_load_ps(&left.Max.X), _mm_load_ps(&right.Max.X)));
#else
        result.Min = LinearMath::JVector::Min(left.Min, right.Min);
        result.MinW = RealZero;
        result.Max = LinearMath::JVector::Max(left.Max, right.Max);
        result.MaxW = RealZero;
#endif
    }

    // Creates a bounding box that encloses both specified boxes.
    [[nodiscard]] static TreeBox CreateMerged(const TreeBox& left, const TreeBox& right)
    {
        TreeBox result;
        CreateMerged(left, right, result);
        return result;
    }

    [[nodiscard]] static double MergedSurface(const TreeBox& left, const TreeBox& right)
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        alignas(16) float extent[4];
        const __m128 min = _mm_min_ps(_mm_load_ps(&left.Min.X), _mm_load_ps(&right.Min.X));
        const __m128 max = _mm_max_ps(_mm_load_ps(&left.Max.X), _mm_load_ps(&right.Max.X));
        _mm_store_ps(extent, _mm_sub_ps(max, min));
        return 2.0 * (extent[0] * extent[1] + extent[0] * extent[2] + extent[1] * extent[2]);
#else
        return CreateMerged(left, right).GetSurfaceArea();
#endif
    }

    bool operator==(const TreeBox& other) const
    {
#if JITTER_TREEBOX_USE_SSE_FLOAT
        const __m128 eqMin = _mm_cmpeq_ps(_mm_load_ps(&Min.X), _mm_load_ps(&other.Min.X));
        const __m128 eqMax = _mm_cmpeq_ps(_mm_load_ps(&Max.X), _mm_load_ps(&other.Max.X));
        return (_mm_movemask_ps(_mm_and_ps(eqMin, eqMax)) & 0x0F) == 0x0F;
#else
        return Min == other.Min && Max == other.Max;
#endif
    }
};

static_assert(std::is_standard_layout_v<TreeBox>);
static_assert(offsetof(TreeBox, Min) == 0);
static_assert(offsetof(TreeBox, MinW) == 3 * sizeof(Real));
static_assert(offsetof(TreeBox, Max) == 4 * sizeof(Real));
static_assert(offsetof(TreeBox, MaxW) == 7 * sizeof(Real));

// Represents a dynamic AABB tree for broadphase collision detection.
// Uses a bounding volume hierarchy with Surface Area Heuristic (SAH) for O(log n)
// insertion and removal. Supports incremental updates for moving objects.
class DynamicTree
{
public:
    using Proxy = IDynamicTreeProxy;
    using FilterFunction = std::function<bool(const Proxy&, const Proxy&)>;
    using OverlapAction = std::function<void(Proxy&, Proxy&)>;
    using TreeBoxAction = std::function<void(const TreeBox&, int)>;
    using RayCastPreFilter = std::function<bool(const Proxy&)>;

    struct RayCastResult
    {
        Proxy& Entity;
        LinearMath::JVector Normal;
        Real Lambda;
    };

    using RayCastPostFilter = std::function<bool(const RayCastResult&)>;
    using FindNearestPreFilter = std::function<bool(const Proxy&)>;

    struct FindNearestResult
    {
        Proxy* Entity = nullptr;
        LinearMath::JVector PointA;
        LinearMath::JVector PointB;
        LinearMath::JVector Normal;
        Real Distance = std::numeric_limits<Real>::max();
    };

    using FindNearestPostFilter = std::function<bool(const FindNearestResult&)>;
    using SweepCastPreFilter = std::function<bool(const Proxy&)>;

    struct SweepCastResult
    {
        Proxy* Entity = nullptr;
        LinearMath::JVector PointA = LinearMath::JVector::Zero();
        LinearMath::JVector PointB = LinearMath::JVector::Zero();
        LinearMath::JVector Normal = LinearMath::JVector::Zero();
        Real Lambda = std::numeric_limits<Real>::max();
    };

    using SweepCastPostFilter = std::function<bool(const SweepCastResult&)>;

    // Profiling buckets for DebugTimings, representing stages of Update.
    enum class Timings
    {
    // Time spent removing stale pairs from the potential pairs set.
        PruneInvalidPairs = 0,
    // Time spent updating proxy bounding boxes.
        UpdateBoundingBoxes = 1,
    // Time spent scanning for proxies that moved outside their expanded boxes.
        ScanMoved = 2,
    // Time spent reinserting moved proxies into the tree.
        UpdateProxies = 3,
    // Time spent scanning for new overlapping pairs.
        ScanOverlaps = 4,
    // Sentinel value for array sizing. Not a real timing bucket.
        Last = 5
    };

    // Sentinel value indicating a null/invalid node index.
    static constexpr int NullNode = -1;

    // Initial capacity of the internal node array.
    static constexpr int InitialSize = 1024;

    // Fraction of the potential pairs hash set to scan per update for pruning invalid entries.
    // A value of 128 means 1/128th of the hash set is scanned each update.
    static constexpr int PruningFraction = 128;

    // Specifies the factor by which the bounding box in the dynamic tree structure is expanded.
    static constexpr Real ExpandFactor = static_cast<Real>(0.1);

    // Specifies a small additional expansion of the bounding box which is constant.
    static constexpr Real ExpandEps = static_cast<Real>(0.1);

    // Represents a node in the AABB tree.
    struct Node
    {
    // Index of the left child node, or NullNode if this is a leaf.
        int Left = NullNode;
    // Index of the right child node, or NullNode if this is a leaf.
        int Right = NullNode;
    // Index of the parent node, or NullNode if this is the root.
        int Parent = NullNode;

    // The expanded bounding box of this node, used for broadphase culling.
    // For leaf nodes, this is the proxy's bounding box expanded by velocity and a margin.
    // For internal nodes, this is the union of its children's boxes.

        TreeBox ExpandedBox;

    // The proxy associated with this node, or nullptr for internal nodes.

        Proxy* Entity = nullptr;

    // When set, forces the node to be updated in the next DynamicTree::Update call,
    // even if its bounding box hasn't changed.

        bool ForceUpdate = false;

    // Returns true if this is a leaf node (has an associated proxy).
        [[nodiscard]] bool IsLeaf() const { return Entity != nullptr; }
    };

    // Initializes a new instance of the DynamicTree class.
    // filter: A function that returns false to exclude a proxy pair from collision detection.
    // See Filter.
    explicit DynamicTree(FilterFunction filter = {})
        : filter_(std::move(filter))
    {
        nodes_.reserve(InitialSize);
        if (!filter_)
        {
            filter_ = [](const Proxy&, const Proxy&) { return true; };
        }
    }

    // Gets or sets the filter function used to exclude proxy pairs from collision detection.
    // The filter is called during overlap enumeration. Return false to exclude a pair.
    // In Jitter, this is typically used to exclude shapes belonging to the same rigid body.
    void Filter(FilterFunction filter)
    {
        if (!filter)
        {
            throw std::invalid_argument("DynamicTree filter cannot be empty.");
        }
        filter_ = std::move(filter);
    }

    // Gets or sets the filter function used to exclude proxy pairs from collision detection.
    [[nodiscard]] const FilterFunction& Filter() const
    {
        return filter_;
    }

    // Adds a proxy to the tree.
    // proxy: The proxy to add.
    // active: If true, the proxy is tracked for movement each update.
    void AddProxy(Proxy& proxy, bool active = true)
    {
        if (ContainsProxy(proxy))
        {
            throw std::invalid_argument("The proxy has already been added to this tree instance.");
        }

        const double surfaceArea = static_cast<double>(proxy.WorldBoundingBox().GetSurfaceArea());
        if (surfaceArea > 9.007e15)
        {
            throw std::out_of_range("The proxy's bounding box is too large for robust tree balancing.");
        }

        InternalAddProxy(proxy);
        OverlapCheckAdd(root_, proxy.NodePtr());
        proxies_.Add(proxy, active);
    }

    // Removes all proxies and resets tree bookkeeping.
    void Clear()
    {
        for (Proxy* proxy : proxies_.Elements())
        {
            proxy->NodePtr(NullNode);
        }
        proxies_.Clear();
        nodes_.clear();
        nodes_.reserve(InitialSize);
        freeNodes_.clear();
        potentialPairs_.Clear();
        movedProxies_.Clear();
        root_ = NullNode;
        updatedProxyCount_ = 0;
    }

    // Removes a proxy from the tree.
    // proxy: The proxy to remove.
    void RemoveProxy(Proxy& proxy)
    {
        if (!ContainsProxy(proxy))
        {
            throw std::logic_error("The proxy is not registered with this tree instance.");
        }

        const int node = proxy.NodePtr();

        OverlapCheckRemove(root_, node);
        InternalRemoveProxy(proxy);
        proxies_.Remove(proxy);
        proxy.NodePtr(NullNode);
    }

    // Forces an immediate update of a single proxy in the tree.
    // proxy: The proxy to update.
    // dt: The timestep in seconds, used for velocity-based bounding box expansion.
    void UpdateProxy(Proxy& proxy, Real dt = 0)
    {
        const int node = proxy.NodePtr();
        if (node == NullNode)
        {
            return;
        }

        if (auto* updatable = dynamic_cast<IUpdatableBoundingBox*>(&proxy))
        {
            updatable->UpdateWorldBoundingBox(dt);
        }
        OverlapCheckRemove(root_, node);
        const int parent = RemoveLeaf(node);
        nodes_[static_cast<std::size_t>(node)].ExpandedBox = TreeBox(proxy.WorldBoundingBox());
        nodes_[static_cast<std::size_t>(node)].ForceUpdate = false;
        InsertLeaf(node, parent);
        OverlapCheckAdd(root_, node);
    }

    // Updates all active proxies in the tree.
    // multiThread: If true, uses multithreading for the update.
    // dt: The timestep in seconds, used for velocity-based bounding box expansion.
    void Update(bool multiThread, Real dt)
    {
        using Clock = std::chrono::steady_clock;
        auto time = Clock::now();
        auto setTime = [this, &time](Timings timing)
        {
            const auto current = Clock::now();
            debugTimings_[static_cast<std::size_t>(timing)] =
                std::chrono::duration<double, std::milli>(current - time).count();
            time = current;
        };

        Tracer::ProfileBegin(TraceName::PruneInvalidPairs);
        PruneInvalidPairs();
        Tracer::ProfileEnd(TraceName::PruneInvalidPairs);
        setTime(Timings::PruneInvalidPairs);

        movedProxies_.Clear();

        const int proxyCount = static_cast<int>(proxies_.ActiveCount());
        Tracer::ProfileBegin(TraceName::UpdateBoundingBoxes);
        ExecuteBatches(multiThread, proxyCount, 256, [this, dt](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                Proxy& proxy = proxies_[static_cast<std::size_t>(i)];
                if (auto* updatable = dynamic_cast<IUpdatableBoundingBox*>(&proxy))
                {
                    updatable->UpdateWorldBoundingBox(dt);
                }
            }
        });
        Tracer::ProfileEnd(TraceName::UpdateBoundingBoxes);
        setTime(Timings::UpdateBoundingBoxes);

        Tracer::ProfileBegin(TraceName::ScanMoved);
        ExecuteBatches(multiThread, proxyCount, 24, [this](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                Proxy& proxy = proxies_[static_cast<std::size_t>(i)];
                Node& node = nodes_[static_cast<std::size_t>(proxy.NodePtr())];
                if (node.ForceUpdate || !node.ExpandedBox.Contains(proxy.WorldBoundingBox()))
                {
                    node.ForceUpdate = false;
                    movedProxies_.ConcurrentAdd(&proxy);
                }
            }
        });
        Tracer::ProfileEnd(TraceName::ScanMoved);
        setTime(Timings::ScanMoved);

        updatedProxyCount_ = movedProxies_.Count();
        Tracer::ProfileBegin(TraceName::UpdateProxies);
        for (int i = 0; i < movedProxies_.Count(); ++i)
        {
            InternalAddRemoveProxy(*movedProxies_[i], dt);
        }
        Tracer::ProfileEnd(TraceName::UpdateProxies);
        setTime(Timings::UpdateProxies);

        const int movedCount = movedProxies_.Count();
        Tracer::ProfileBegin(TraceName::ScanOverlaps);
        ExecuteBatches(multiThread, movedCount, 24, [this](Parallelization::Batch batch)
        {
            for (int i = batch.Start; i < batch.End; ++i)
            {
                OverlapCheckAdd(root_, movedProxies_[i]->NodePtr());
            }
        });
        Tracer::ProfileEnd(TraceName::ScanOverlaps);
        setTime(Timings::ScanOverlaps);

        movedProxies_.TrackAndNullOutOne();
    }

    void Optimize(int sweeps = 100, Real chance = static_cast<Real>(0.01), bool incremental = false)
    {
        Optimize([this]() { return random_.NextDouble(); }, sweeps, chance, incremental);
    }

    void Optimize(
        const std::function<double()>& getNextRandom,
        int sweeps,
        Real chance,
        bool incremental)
    {
        if (sweeps <= 0)
        {
            throw std::out_of_range("Sweeps must be greater than zero.");
        }

        if (chance < static_cast<Real>(0) || chance > static_cast<Real>(1))
        {
            throw std::out_of_range("Chance must be between 0 and 1.");
        }

        for (int e = 0; e < sweeps; ++e)
        {
            const bool takeAll = e == 0 && !incremental;

            for (int i = 0; i < static_cast<int>(proxies_.Count()); ++i)
            {
                if (!takeAll && getNextRandom() > static_cast<double>(chance))
                {
                    continue;
                }

                Proxy& proxy = proxies_[static_cast<std::size_t>(i)];
                tempList_.push_back(&proxy);
                OverlapCheckRemove(root_, proxy.NodePtr());
                InternalRemoveProxy(proxy);
            }

            const int n = static_cast<int>(tempList_.size());
            for (int i = n - 1; i > 0; --i)
            {
                const double scaledValue = getNextRandom() * static_cast<double>(i + 1);
                const int j = static_cast<int>(scaledValue);
                std::swap(tempList_[static_cast<std::size_t>(i)], tempList_[static_cast<std::size_t>(j)]);
            }

            for (Proxy* proxy : tempList_)
            {
                InternalAddProxy(*proxy);
                OverlapCheckAdd(root_, proxy->NodePtr());
            }

            tempList_.clear();
        }

        if (!incremental)
        {
            tempList_.shrink_to_fit();
        }
    }

    [[nodiscard]] bool IsActive(const Proxy& proxy) const
    {
        return proxy.NodePtr() != NullNode
            && proxies_.IsActive(proxy);
    }

    void ActivateProxy(Proxy& proxy)
    {
        if (proxies_.MoveToActive(proxy))
        {
            nodes_[static_cast<std::size_t>(proxy.NodePtr())].ForceUpdate = true;
        }
    }

    void DeactivateProxy(Proxy& proxy)
    {
        proxies_.MoveToInactive(proxy);
    }

    [[nodiscard]] std::span<Proxy* const> Proxies() const
    {
        return proxies_.Elements();
    }

    [[nodiscard]] std::size_t Count() const
    {
        return proxies_.Count();
    }

    [[nodiscard]] std::size_t ActiveCount() const
    {
        return proxies_.ActiveCount();
    }

    [[nodiscard]] std::span<const Node> Nodes() const
    {
        return std::span<const Node>(nodes_.data(), nodes_.size());
    }

    [[nodiscard]] int Root() const
    {
        return root_;
    }

    [[nodiscard]] std::size_t UpdatedProxyCount() const
    {
        return updatedProxyCount_;
    }

    [[nodiscard]] std::span<const double> DebugTimings() const
    {
        return std::span<const double>(debugTimings_.data(), debugTimings_.size());
    }

    [[nodiscard]] static const char* TimingName(Timings timing)
    {
        switch (timing)
        {
        case Timings::PruneInvalidPairs: return "PruneInvalidPairs";
        case Timings::UpdateBoundingBoxes: return "UpdateBoundingBoxes";
        case Timings::ScanMoved: return "ScanMoved";
        case Timings::UpdateProxies: return "UpdateProxies";
        case Timings::ScanOverlaps: return "ScanOverlaps";
        case Timings::Last: return "Last";
        }
        return "Unknown";
    }

    [[nodiscard]] std::pair<std::size_t, std::size_t> HashSetInfo() const
    {
        return {potentialPairs_.SlotCount(), potentialPairs_.Count()};
    }

    void EnumerateTreeBoxes(const TreeBoxAction& action) const
    {
        if (root_ == NullNode)
        {
            return;
        }

        EnumerateTreeBoxes(nodes_[static_cast<std::size_t>(root_)], action, 1);
    }

    [[nodiscard]] double CalculateCost() const
    {
        double cost = 0.0;
        if (root_ == NullNode)
        {
            return cost;
        }
        return Cost(root_);
    }

    std::vector<Proxy*> Query(const LinearMath::JBoundingBox& box) const
    {
        std::vector<Proxy*> result;
        Query(result, box);
        return result;
    }

    void Query(std::vector<Proxy*>& destination, const LinearMath::JBoundingBox& box) const
    {
        if (root_ == NullNode)
        {
            return;
        }

        const TreeBox sbox(box);
        std::vector<int> stack;
        stack.reserve(256);
        stack.push_back(root_);

        while (!stack.empty())
        {
            const int index = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<std::size_t>(index)];
            if (node.IsLeaf())
            {
                if (node.Entity != nullptr
                    && !LinearMath::JBoundingBox::Disjoint(node.Entity->WorldBoundingBox(), box))
                {
                    destination.push_back(node.Entity);
                }

                continue;
            }

            const int left = node.Left;
            const int right = node.Right;

            if (left != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(left)].ExpandedBox, sbox))
            {
                stack.push_back(left);
            }

            if (right != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(right)].ExpandedBox, sbox))
            {
                stack.push_back(right);
            }
        }
    }

    std::vector<Proxy*> Query(
        const LinearMath::JVector& rayOrigin,
        const LinearMath::JVector& rayDirection) const
    {
        std::vector<Proxy*> result;
        Query(result, rayOrigin, rayDirection);
        return result;
    }

    void Query(
        std::vector<Proxy*>& destination,
        const LinearMath::JVector& rayOrigin,
        const LinearMath::JVector& rayDirection) const
    {
        if (root_ == NullNode)
        {
            return;
        }

        std::vector<int> stack;
        stack.reserve(256);
        stack.push_back(root_);

        while (!stack.empty())
        {
            const int index = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<std::size_t>(index)];
            if (node.IsLeaf())
            {
                Real enter = 0;
                if (node.Entity != nullptr
                    && node.Entity->WorldBoundingBox().RayIntersect(rayOrigin, rayDirection, enter))
                {
                    destination.push_back(node.Entity);
                }

                continue;
            }

            const int left = node.Left;
            const int right = node.Right;

            Real enter = 0;
            if (left != NullNode
                && nodes_[static_cast<std::size_t>(left)].ExpandedBox.RayIntersect(rayOrigin, rayDirection, enter))
            {
                stack.push_back(left);
            }

            enter = 0;
            if (right != NullNode
                && nodes_[static_cast<std::size_t>(right)].ExpandedBox.RayIntersect(rayOrigin, rayDirection, enter))
            {
                stack.push_back(right);
            }
        }
    }

    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        Proxy*& proxy,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return RayCast(origin, direction, {}, {}, proxy, normal, lambda);
    }

    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        const RayCastPreFilter& preFilter,
        const RayCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return RayCast(
            origin,
            direction,
            std::numeric_limits<Real>::max(),
            preFilter,
            postFilter,
            proxy,
            normal,
            lambda);
    }

    bool RayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const RayCastPreFilter& preFilter,
        const RayCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        proxy = nullptr;
        normal = LinearMath::JVector::Zero();
        lambda = maxLambda;

        if (root_ == NullNode)
        {
            return false;
        }

        std::vector<int> stack;
        stack.reserve(256);
        stack.push_back(root_);

        bool globalHit = false;

        while (!stack.empty())
        {
            const int index = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<std::size_t>(index)];
            if (node.IsLeaf())
            {
                Proxy* candidate = node.Entity;
                if (candidate == nullptr)
                {
                    continue;
                }

                const auto* rayCastable = dynamic_cast<const IRayCastable*>(candidate);
                if (rayCastable == nullptr)
                {
                    continue;
                }

                if (preFilter && !preFilter(*candidate))
                {
                    continue;
                }

                LinearMath::JVector candidateNormal;
                Real candidateLambda = 0;
                const bool hit = rayCastable->RayCast(origin, direction, candidateNormal, candidateLambda);
                if (hit && candidateLambda < lambda)
                {
                    RayCastResult result {*candidate, candidateNormal, candidateLambda};
                    if (postFilter && !postFilter(result))
                    {
                        continue;
                    }

                    proxy = candidate;
                    normal = candidateNormal;
                    lambda = candidateLambda;
                    globalHit = true;
                }

                continue;
            }

            const int left = node.Left;
            const int right = node.Right;

            Real leftEnter = 0;
            Real rightEnter = 0;
            bool leftResult = left != NullNode
                && nodes_[static_cast<std::size_t>(left)].ExpandedBox.RayIntersect(origin, direction, leftEnter);
            bool rightResult = right != NullNode
                && nodes_[static_cast<std::size_t>(right)].ExpandedBox.RayIntersect(origin, direction, rightEnter);

            if (leftEnter > lambda)
            {
                leftResult = false;
            }

            if (rightEnter > lambda)
            {
                rightResult = false;
            }

            if (leftResult && rightResult)
            {
                if (leftEnter < rightEnter)
                {
                    stack.push_back(right);
                    stack.push_back(left);
                }
                else
                {
                    stack.push_back(left);
                    stack.push_back(right);
                }
            }
            else
            {
                if (leftResult)
                {
                    stack.push_back(left);
                }

                if (rightResult)
                {
                    stack.push_back(right);
                }
            }
        }

        return globalHit;
    }

    bool FindNearestPoint(
        const LinearMath::JVector& point,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearestPoint(point, {}, {}, proxy, pointA, pointB, normal, distance);
    }

    bool FindNearestPoint(
        const LinearMath::JVector& point,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearestPoint(point, std::numeric_limits<Real>::max(), preFilter, postFilter,
            proxy, pointA, pointB, normal, distance);
    }

    bool FindNearestPoint(
        const LinearMath::JVector& point,
        Real maxDistance,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearest(
            SupportPrimitives::CreatePoint(),
            LinearMath::JQuaternion::Identity(),
            point,
            maxDistance,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            distance);
    }

    bool FindNearestSphere(
        Real radius,
        const LinearMath::JVector& center,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearestSphere(radius, center, {}, {}, proxy, pointA, pointB, normal, distance);
    }

    bool FindNearestSphere(
        Real radius,
        const LinearMath::JVector& center,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearestSphere(radius, center, std::numeric_limits<Real>::max(), preFilter, postFilter,
            proxy, pointA, pointB, normal, distance);
    }

    bool FindNearestSphere(
        Real radius,
        const LinearMath::JVector& center,
        Real maxDistance,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearest(
            SupportPrimitives::CreateSphere(radius),
            LinearMath::JQuaternion::Identity(),
            center,
            maxDistance,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            distance);
    }

    template<typename TSupport>
    bool FindNearest(
        const TSupport& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        return FindNearest(
            support,
            orientation,
            position,
            std::numeric_limits<Real>::max(),
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            distance);
    }

    template<typename TSupport>
    bool FindNearest(
        const TSupport& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        Real maxDistance,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& distance) const
    {
        LinearMath::JBoundingBox box;
        Shapes::ShapeHelper::CalculateBoundingBox(support, orientation, position, box);

        FindNearestResult result;
        result.Distance = maxDistance;

        const bool hit = QueryDistance(support, box, orientation, position, preFilter, postFilter, result);
        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        distance = result.Distance;
        return hit;
    }

    bool SweepCastSphere(
        Real radius,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateSphere(radius),
            LinearMath::JQuaternion::Identity(),
            position,
            direction,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastSphere(
        Real radius,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateSphere(radius),
            LinearMath::JQuaternion::Identity(),
            position,
            direction,
            maxLambda,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastBox(
        const LinearMath::JVector& halfExtents,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateBox(halfExtents),
            orientation,
            position,
            direction,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastBox(
        const LinearMath::JVector& halfExtents,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateBox(halfExtents),
            orientation,
            position,
            direction,
            maxLambda,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastCapsule(
        Real radius,
        Real halfLength,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateCapsule(radius, halfLength),
            orientation,
            position,
            direction,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastCapsule(
        Real radius,
        Real halfLength,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateCapsule(radius, halfLength),
            orientation,
            position,
            direction,
            maxLambda,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastCylinder(
        Real radius,
        Real halfHeight,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateCylinder(radius, halfHeight),
            orientation,
            position,
            direction,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    bool SweepCastCylinder(
        Real radius,
        Real halfHeight,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            SupportPrimitives::CreateCylinder(radius, halfHeight),
            orientation,
            position,
            direction,
            maxLambda,
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    template<typename TSupport>
    bool SweepCast(
        const TSupport& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        return SweepCast(
            support,
            orientation,
            position,
            direction,
            std::numeric_limits<Real>::max(),
            preFilter,
            postFilter,
            proxy,
            pointA,
            pointB,
            normal,
            lambda);
    }

    template<typename TSupport>
    bool SweepCast(
        const TSupport& support,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const LinearMath::JVector& direction,
        Real maxLambda,
        const SweepCastPreFilter& preFilter,
        const SweepCastPostFilter& postFilter,
        Proxy*& proxy,
        LinearMath::JVector& pointA,
        LinearMath::JVector& pointB,
        LinearMath::JVector& normal,
        Real& lambda) const
    {
        LinearMath::JBoundingBox box;
        Shapes::ShapeHelper::CalculateBoundingBox(support, orientation, position, box);

        SweepCastResult result;
        result.Lambda = maxLambda;

        if (root_ == NullNode)
        {
            proxy = nullptr;
            pointA = result.PointA;
            pointB = result.PointB;
            normal = result.Normal;
            lambda = result.Lambda;
            return false;
        }

        std::vector<int> stack;
        stack.reserve(256);
        stack.push_back(root_);

        while (!stack.empty())
        {
            const int index = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<std::size_t>(index)];
            if (node.IsLeaf())
            {
                Proxy* candidate = node.Entity;
                if (candidate == nullptr)
                {
                    continue;
                }

                const auto* sweepTestable = dynamic_cast<const ISweepTestable*>(candidate);
                if (sweepTestable == nullptr)
                {
                    continue;
                }

                if (preFilter && !preFilter(*candidate))
                {
                    continue;
                }

                SweepCastResult candidateResult;
                candidateResult.Entity = candidate;
                const bool hit = sweepTestable->Sweep(
                    support,
                    orientation,
                    position,
                    direction,
                    candidateResult.PointA,
                    candidateResult.PointB,
                    candidateResult.Normal,
                    candidateResult.Lambda);

                if (!hit || candidateResult.Lambda > result.Lambda)
                {
                    continue;
                }

                if (postFilter && !postFilter(candidateResult))
                {
                    continue;
                }

                result = candidateResult;
                continue;
            }

            const int left = node.Left;
            const int right = node.Right;

            Real leftEnter = 0;
            Real rightEnter = 0;
            bool leftHit = left != NullNode
                && SweepBox(box, direction, nodes_[static_cast<std::size_t>(left)].ExpandedBox, leftEnter);
            bool rightHit = right != NullNode
                && SweepBox(box, direction, nodes_[static_cast<std::size_t>(right)].ExpandedBox, rightEnter);

            if (leftEnter > result.Lambda)
            {
                leftHit = false;
            }

            if (rightEnter > result.Lambda)
            {
                rightHit = false;
            }

            if (leftHit && rightHit)
            {
                if (leftEnter < rightEnter)
                {
                    stack.push_back(right);
                    stack.push_back(left);
                }
                else
                {
                    stack.push_back(left);
                    stack.push_back(right);
                }
            }
            else
            {
                if (leftHit)
                {
                    stack.push_back(left);
                }

                if (rightHit)
                {
                    stack.push_back(right);
                }
            }
        }

        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        lambda = result.Lambda;
        return result.Entity != nullptr;
    }

    void EnumerateOverlaps(const OverlapAction& action, bool multiThread = false) const
    {
        if (!action)
        {
            return;
        }

        const auto enumerateBatch = [this, &action](Parallelization::Batch batch)
        {
            const auto slots = potentialPairs_.Slots();
            for (int i = batch.Start; i < batch.End; ++i)
            {
                const auto& pair = slots[static_cast<std::size_t>(i)];
                if (pair.ID == 0)
                {
                    continue;
                }

                Proxy* first = nodes_[static_cast<std::size_t>(pair.ID1())].Entity;
                Proxy* second = nodes_[static_cast<std::size_t>(pair.ID2())].Entity;
                if (first == nullptr || second == nullptr)
                {
                    continue;
                }

                if (!filter_(*first, *second))
                {
                    continue;
                }

                if (!LinearMath::JBoundingBox::Disjoint(first->WorldBoundingBox(), second->WorldBoundingBox()))
                {
                    action(*first, *second);
                }
            }
        };

        const int slotsLength = static_cast<int>(potentialPairs_.SlotCount());
        if (multiThread)
        {
            const int taskMultiplier = 6;
            const int taskCount = Parallelization::ThreadPool::Instance().ThreadCount() * taskMultiplier;
            Parallelization::ForBatch(0, slotsLength, taskCount, enumerateBatch);
            return;
        }

        enumerateBatch(Parallelization::Batch {0, slotsLength});
    }

    [[nodiscard]] std::size_t PairCount() const
    {
        std::size_t count = 0;
        EnumerateOverlaps([&count](Proxy&, Proxy&) { ++count; });
        return count;
    }

private:
    void EnumerateTreeBoxes(const Node& node, const TreeBoxAction& action, int depth) const
    {
        action(node.ExpandedBox, depth);
        if (node.IsLeaf())
        {
            return;
        }

        EnumerateTreeBoxes(nodes_[static_cast<std::size_t>(node.Left)], action, depth + 1);
        EnumerateTreeBoxes(nodes_[static_cast<std::size_t>(node.Right)], action, depth + 1);
    }

    [[nodiscard]] bool ContainsProxy(const Proxy& proxy) const
    {
        return proxies_.Contains(proxy);
    }

    template<typename Action>
    static void ExecuteBatches(
        bool multiThread,
        int count,
        int taskThreshold,
        Action&& action)
    {
        if (count <= 0)
        {
            return;
        }

#if !JITTER_ENABLE_MULTITHREADING
        multiThread = false;
#endif

        if (!multiThread)
        {
            action(Parallelization::Batch {0, count});
            return;
        }

        const int threshold = std::max(1, taskThreshold);
        int numTasks = count / threshold + 1;
        numTasks = std::min(numTasks, Parallelization::ThreadPool::Instance().ThreadCount());

        Parallelization::ForBatch(
            0,
            count,
            numTasks,
            std::function<void(Parallelization::Batch)>(std::forward<Action>(action)));
    }

    // A hash set implementation which stores unordered pairs of int values.
    // The implementation is based on open addressing with power-of-two sizing.
    // Pairs are stored in a canonical form where the smaller ID comes first.
    // A pair with Pair::ID equal to zero is treated as an empty slot.
    // This class is not generally thread-safe. Only ConcurrentAdd may be called
    // concurrently from multiple threads. All other methods require external synchronization.
    class PairHashSet
    {
    public:
        // Represents an unordered pair of integer IDs stored in canonical form.
        struct Pair
        {
            // Combined 64-bit identifier for the pair.
            std::uint64_t ID = 0;

            Pair() = default;

            // Creates a pair from two IDs, storing them in canonical order.
            Pair(int id1, int id2)
            {
                int first = 0;
                int second = 0;
                if (id1 < id2)
                {
                    first = id1;
                    second = id2;
                }
                else
                {
                    first = id2;
                    second = id1;
                }

                ID = static_cast<std::uint64_t>(static_cast<std::uint32_t>(first))
                    | (static_cast<std::uint64_t>(static_cast<std::uint32_t>(second)) << 32U);
            }

            [[nodiscard]] int ID1() const
            {
                return static_cast<int>(static_cast<std::uint32_t>(ID));
            }

            [[nodiscard]] int ID2() const
            {
                return static_cast<int>(static_cast<std::uint32_t>(ID >> 32U));
            }

            [[nodiscard]] std::size_t GetHash() const
            {
                std::uint64_t hash = ID;
                hash ^= hash >> 33U;
                hash *= 0xff51afd7ed558ccdULL;
                hash ^= hash >> 33U;
                hash *= 0xc4ceb9fe1a85ec53ULL;
                hash ^= hash >> 33U;
                return static_cast<std::size_t>(hash);
            }
        };

        static_assert(sizeof(Pair) == sizeof(std::uint64_t));
        static_assert(alignof(Pair) == alignof(std::uint64_t));

        // Minimum number of slots in the hash set.
        static constexpr std::size_t MinimumSize = 16384;

        // Factor used to determine when to shrink the hash set.
        static constexpr std::size_t TrimFactor = 8;

        PairHashSet()
        {
            Resize(PickSize());
        }

        // The internal storage array for pairs. Empty slots have Pair::ID equal to zero.
        [[nodiscard]] std::span<const Pair> Slots() const
        {
            return std::span<const Pair>(slots_.data(), slots_.size());
        }

        [[nodiscard]] std::size_t SlotCount() const
        {
            return slots_.size();
        }

        // Gets the number of pairs in the hash set.
        [[nodiscard]] std::size_t Count() const
        {
            return count_.load(std::memory_order_acquire);
        }

        // Removes all pairs from the hash set.
        // This method is not thread-safe. Do not call concurrently with any other method.
        void Clear()
        {
            std::fill(slots_.begin(), slots_.end(), Pair {});
            count_.store(0, std::memory_order_release);
        }

        [[nodiscard]] bool Contains(Pair pair) const
        {
            const std::size_t hashIndex = FindSlot(slots_, pair.GetHash(), pair.ID);
            return slots_[hashIndex].ID != 0;
        }

        // Adds the specified pair to the set.
        // Returns: true if the pair was added; false if it was already present.
        bool Add(Pair pair)
        {
            const std::size_t hashIndex = FindSlot(slots_, pair.GetHash(), pair.ID);
            if (slots_[hashIndex].ID == 0)
            {
                slots_[hashIndex] = pair;
                const std::size_t count = count_.fetch_add(1, std::memory_order_acq_rel) + 1;

                if (slots_.size() < 2 * count)
                {
                    Resize(PickSize(slots_.size() * 2));
                }

                return true;
            }

            return false;
        }

        // Adds the specified pair to the set in a thread-safe manner.
        // This method may be called concurrently by multiple threads.
        bool ConcurrentAdd(Pair pair)
        {
            const std::size_t hash = pair.GetHash();
            std::size_t count = 0;

            Pair* snapshotData = slotsData_.load(std::memory_order_acquire);
            const std::size_t snapshotSize = slotsSize_.load(std::memory_order_acquire);
            if (snapshotData != nullptr && snapshotSize != 0)
            {
                const std::size_t fastPathIndex = FindSlotAtomic(snapshotData, snapshotSize, hash, pair.ID);
                if (AtomicLoadID(snapshotData[fastPathIndex]) == pair.ID)
                {
                    return false;
                }
            }

            {
                ReadLockScope lock(rwLock_);

                while (true)
                {
                    const std::size_t hashIndex = FindSlotAtomic(slots_, hash, pair.ID);
                    std::atomic_ref<std::uint64_t> slotId(slots_[hashIndex].ID);

                    std::uint64_t expected = pair.ID;
                    if (slotId.load(std::memory_order_acquire) == expected)
                    {
                        return false;
                    }

                    expected = 0;
                    if (!slotId.compare_exchange_strong(
                            expected,
                            pair.ID,
                            std::memory_order_acq_rel,
                            std::memory_order_acquire))
                    {
                        continue;
                    }

                    count = count_.fetch_add(1, std::memory_order_acq_rel) + 1;
                    break;
                }
            }

            if (slots_.size() < 2 * count)
            {
                WriteLockScope lock(rwLock_);

                const std::size_t currentCount = count_.load(std::memory_order_acquire);
                if (slots_.size() < 2 * currentCount)
                {
                    Resize(PickSize(slots_.size() * 2));
                }
            }

            return true;
        }

        // Removes the pair stored at the specified slot.
        // This method is not thread-safe. Do not call concurrently with any other method.
        bool Remove(std::size_t slot)
        {
            const std::size_t modder = slots_.size() - 1;
            if (slots_[slot].ID == 0)
            {
                return false;
            }

            std::size_t hashJ = slot;
            while (true)
            {
                hashJ = (hashJ + 1) & modder;

                if (slots_[hashJ].ID == 0)
                {
                    break;
                }

                const std::size_t hashK = slots_[hashJ].GetHash() & modder;
                if ((hashJ > slot && (hashK <= slot || hashK > hashJ))
                    || (hashJ < slot && hashK <= slot && hashK > hashJ))
                {
                    slots_[slot] = slots_[hashJ];
                    slot = hashJ;
                }
            }

            slots_[slot] = Pair {};
            const std::size_t count = count_.fetch_sub(1, std::memory_order_acq_rel) - 1;

            if (slots_.size() > MinimumSize && count * TrimFactor < slots_.size())
            {
                Resize(PickSize(count * 2));
            }

            return true;
        }

        // Removes the specified pair from the set.
        // This method is not thread-safe. Do not call concurrently with any other method.
        bool Remove(Pair pair)
        {
            const std::size_t hashIndex = FindSlot(slots_, pair.GetHash(), pair.ID);
            return Remove(hashIndex);
        }

    private:
        struct ReadLockScope
        {
            explicit ReadLockScope(Parallelization::ReaderWriterLock& lock)
                : Lock(lock)
            {
                Lock.EnterReadLock();
            }

            ~ReadLockScope()
            {
                Lock.ExitReadLock();
            }

            Parallelization::ReaderWriterLock& Lock;
        };

        struct WriteLockScope
        {
            explicit WriteLockScope(Parallelization::ReaderWriterLock& lock)
                : Lock(lock)
            {
                Lock.EnterWriteLock();
            }

            ~WriteLockScope()
            {
                Lock.ExitWriteLock();
            }

            Parallelization::ReaderWriterLock& Lock;
        };

        static std::size_t PickSize(std::size_t size = MinimumSize)
        {
            std::size_t p2 = MinimumSize;
            while (p2 < size)
            {
                p2 *= 2;
            }

            return p2;
        }

        void Resize(std::size_t size)
        {
            if (slots_.size() == size)
            {
                return;
            }

            Logger::Information(
                "{0}: Resizing from {1} to {2} elements.",
                "PairHashSet",
                slots_.size(),
                size);

            std::vector<Pair> newSlots(size);
            for (const Pair& pair : slots_)
            {
                if (pair.ID != 0)
                {
                    const std::size_t hashIndex = FindSlot(newSlots, pair.GetHash(), pair.ID);
                    newSlots[hashIndex] = pair;
                }
            }

            if (!slots_.empty())
            {
                // Lock-free fast-path readers may still hold a snapshot of this storage.
                retiredSlots_.push_back(std::move(slots_));
            }

            slots_ = std::move(newSlots);
            slotsData_.store(slots_.data(), std::memory_order_release);
            slotsSize_.store(slots_.size(), std::memory_order_release);
        }

        static std::size_t FindSlot(const std::vector<Pair>& slots, std::size_t hash, std::uint64_t id)
        {
            const std::size_t modder = slots.size() - 1;
            hash &= modder;

            while (true)
            {
                if (slots[hash].ID == 0 || slots[hash].ID == id)
                {
                    return hash;
                }

                hash = (hash + 1) & modder;
            }
        }

        static std::uint64_t AtomicLoadID(const Pair& pair)
        {
            auto& id = const_cast<std::uint64_t&>(pair.ID);
            return std::atomic_ref<std::uint64_t>(id).load(std::memory_order_acquire);
        }

        static std::size_t FindSlotAtomic(const std::vector<Pair>& slots, std::size_t hash, std::uint64_t id)
        {
            return FindSlotAtomic(slots.data(), slots.size(), hash, id);
        }

        static std::size_t FindSlotAtomic(const Pair* slots, std::size_t slotCount, std::size_t hash, std::uint64_t id)
        {
            const std::size_t modder = slotCount - 1;
            hash &= modder;

            while (true)
            {
                const std::uint64_t slotId = AtomicLoadID(slots[hash]);
                if (slotId == 0 || slotId == id)
                {
                    return hash;
                }

                hash = (hash + 1) & modder;
            }
        }

        std::vector<Pair> slots_;
        std::vector<std::vector<Pair>> retiredSlots_;
        std::atomic<Pair*> slotsData_ {nullptr};
        std::atomic<std::size_t> slotsSize_ {0};
        std::atomic<std::size_t> count_ {0};
        Parallelization::ReaderWriterLock rwLock_;
    };

    [[nodiscard]] int AllocateNode()
    {
        if (!freeNodes_.empty())
        {
            const int node = freeNodes_.back();
            freeNodes_.pop_back();
            return node;
        }

        const std::size_t capacityBefore = nodes_.capacity();
        nodes_.push_back(Node {});
        if (nodes_.capacity() != capacityBefore)
        {
            Logger::Information(
                "{0}: Resized array of tree to {1} elements.",
                "DynamicTree",
                nodes_.capacity());
        }
        return static_cast<int>(nodes_.size()) - 1;
    }

    void FreeNode(int node)
    {
        nodes_[static_cast<std::size_t>(node)] = Node {};
        freeNodes_.push_back(node);
    }

    [[nodiscard]] bool PairHasLiveEntities(int first, int second) const
    {
        if (first == second)
        {
            return false;
        }

        const Node& nodeA = nodes_[static_cast<std::size_t>(first)];
        const Node& nodeB = nodes_[static_cast<std::size_t>(second)];
        return nodeA.Entity != nullptr && nodeB.Entity != nullptr;
    }

    [[nodiscard]] bool PairPassesFilter(int first, int second) const
    {
        const Node& nodeA = nodes_[static_cast<std::size_t>(first)];
        const Node& nodeB = nodes_[static_cast<std::size_t>(second)];
        return filter_(*nodeA.Entity, *nodeB.Entity);
    }

    [[nodiscard]] bool PairExpandedBoxesOverlap(int first, int second) const
    {
        const Node& nodeA = nodes_[static_cast<std::size_t>(first)];
        const Node& nodeB = nodes_[static_cast<std::size_t>(second)];
        return !TreeBox::Disjoint(nodeA.ExpandedBox, nodeB.ExpandedBox);
    }

    [[nodiscard]] bool PairHasActiveProxy(int first, int second) const
    {
        const Node& nodeA = nodes_[static_cast<std::size_t>(first)];
        const Node& nodeB = nodes_[static_cast<std::size_t>(second)];
        return IsActive(*nodeA.Entity) || IsActive(*nodeB.Entity);
    }

    void AddPotentialPair(int first, int second)
    {
        if (!PairHasLiveEntities(first, second)
            || !PairPassesFilter(first, second)
            || !PairExpandedBoxesOverlap(first, second))
        {
            return;
        }

        potentialPairs_.ConcurrentAdd(PairHashSet::Pair(first, second));
    }

    void OverlapCheckAdd(int index, int node)
    {
        if (index == NullNode)
        {
            return;
        }

        const Node& current = nodes_[static_cast<std::size_t>(index)];
        if (current.IsLeaf())
        {
            AddPotentialPair(index, node);
            return;
        }

        const int left = current.Left;
        const int right = current.Right;
        const TreeBox& nodeBox = nodes_[static_cast<std::size_t>(node)].ExpandedBox;

        if (left != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(left)].ExpandedBox, nodeBox))
        {
            OverlapCheckAdd(left, node);
        }

        if (right != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(right)].ExpandedBox, nodeBox))
        {
            OverlapCheckAdd(right, node);
        }
    }

    void OverlapCheckRemove(int index, int node)
    {
        if (index == NullNode)
        {
            return;
        }

        const Node& current = nodes_[static_cast<std::size_t>(index)];
        if (current.IsLeaf())
        {
            if (PairHasLiveEntities(index, node) && PairPassesFilter(index, node))
            {
                potentialPairs_.Remove(PairHashSet::Pair(index, node));
            }
            return;
        }

        const int left = current.Left;
        const int right = current.Right;
        const TreeBox& nodeBox = nodes_[static_cast<std::size_t>(node)].ExpandedBox;

        if (left != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(left)].ExpandedBox, nodeBox))
        {
            OverlapCheckRemove(left, node);
        }

        if (right != NullNode && !TreeBox::Disjoint(nodes_[static_cast<std::size_t>(right)].ExpandedBox, nodeBox))
        {
            OverlapCheckRemove(right, node);
        }
    }

    void PruneInvalidPairs()
    {
        stepper_ += 1;

        for (std::ptrdiff_t i = 0;
             i < static_cast<std::ptrdiff_t>(potentialPairs_.SlotCount() / PruningFraction);
             ++i)
        {
            const std::size_t slotsLength = potentialPairs_.SlotCount();
            const std::size_t slot =
                (static_cast<std::size_t>(i) * PruningFraction + stepper_) % slotsLength;

            const auto pair = potentialPairs_.Slots()[slot];
            if (pair.ID == 0)
            {
                continue;
            }

            if (PairHasLiveEntities(pair.ID1(), pair.ID2())
                && PairExpandedBoxesOverlap(pair.ID1(), pair.ID2())
                && PairHasActiveProxy(pair.ID1(), pair.ID2()))
            {
                continue;
            }

            potentialPairs_.Remove(slot);
            i -= 1;
        }
    }

    static void ExpandBoundingBox(LinearMath::JBoundingBox& box, const LinearMath::JVector& direction)
    {
        if (direction.X < static_cast<Real>(0))
        {
            box.Min.X += direction.X;
        }
        else
        {
            box.Max.X += direction.X;
        }

        if (direction.Y < static_cast<Real>(0))
        {
            box.Min.Y += direction.Y;
        }
        else
        {
            box.Max.Y += direction.Y;
        }

        if (direction.Z < static_cast<Real>(0))
        {
            box.Min.Z += direction.Z;
        }
        else
        {
            box.Max.Z += direction.Z;
        }

        box.Min -= LinearMath::JVector(ExpandEps);
        box.Max += LinearMath::JVector(ExpandEps);
    }

    void InternalAddRemoveProxy(Proxy& proxy, Real)
    {
        const int node = proxy.NodePtr();
        const int parent = RemoveLeaf(node);

        LinearMath::JBoundingBox box = proxy.WorldBoundingBox();
        const Real pseudoRandomExt = static_cast<Real>(random_.NextDouble());
        ExpandBoundingBox(
            box,
            proxy.Velocity() * ExpandFactor * (static_cast<Real>(1) + pseudoRandomExt));

        nodes_[static_cast<std::size_t>(node)].ExpandedBox = TreeBox(box);
        nodes_[static_cast<std::size_t>(node)].Entity = &proxy;
        proxy.NodePtr(node);
        InsertLeaf(node, parent);
    }

    void InternalAddProxy(Proxy& proxy)
    {
        const int index = AllocateNode();
        nodes_[static_cast<std::size_t>(index)].Entity = &proxy;
        proxy.NodePtr(index);
        nodes_[static_cast<std::size_t>(index)].ExpandedBox = TreeBox(proxy.WorldBoundingBox());
        nodes_[static_cast<std::size_t>(index)].ForceUpdate = false;
        InsertLeaf(index, root_);
    }

    void InternalRemoveProxy(Proxy& proxy)
    {
        RemoveLeaf(proxy.NodePtr());
        FreeNode(proxy.NodePtr());
    }

    [[nodiscard]] double Cost(int node) const
    {
        const Node& current = nodes_[static_cast<std::size_t>(node)];
        if (current.IsLeaf())
        {
            return current.ExpandedBox.GetSurfaceArea();
        }

        return current.ExpandedBox.GetSurfaceArea() + Cost(current.Left) + Cost(current.Right);
    }

    int RemoveLeaf(int node)
    {
        if (node == root_)
        {
            root_ = NullNode;
            return NullNode;
        }

        const int parent = nodes_[static_cast<std::size_t>(node)].Parent;
        const int grandParent = nodes_[static_cast<std::size_t>(parent)].Parent;
        const int sibling = nodes_[static_cast<std::size_t>(parent)].Left == node
            ? nodes_[static_cast<std::size_t>(parent)].Right
            : nodes_[static_cast<std::size_t>(parent)].Left;

        if (grandParent == NullNode)
        {
            root_ = sibling;
            nodes_[static_cast<std::size_t>(sibling)].Parent = NullNode;
            FreeNode(parent);
            return root_;
        }

        if (nodes_[static_cast<std::size_t>(grandParent)].Left == parent)
        {
            nodes_[static_cast<std::size_t>(grandParent)].Left = sibling;
        }
        else
        {
            nodes_[static_cast<std::size_t>(grandParent)].Right = sibling;
        }

        nodes_[static_cast<std::size_t>(sibling)].Parent = grandParent;
        FreeNode(parent);

        int index = grandParent;
        while (index != NullNode)
        {
            const int left = nodes_[static_cast<std::size_t>(index)].Left;
            const int right = nodes_[static_cast<std::size_t>(index)].Right;

            TreeBox treeBoxBefore = nodes_[static_cast<std::size_t>(index)].ExpandedBox;
            TreeBox::CreateMerged(
                nodes_[static_cast<std::size_t>(left)].ExpandedBox,
                nodes_[static_cast<std::size_t>(right)].ExpandedBox,
                nodes_[static_cast<std::size_t>(index)].ExpandedBox);

            if (treeBoxBefore == nodes_[static_cast<std::size_t>(index)].ExpandedBox)
            {
                break;
            }

            index = nodes_[static_cast<std::size_t>(index)].Parent;
        }

        return grandParent;
    }

    int FindBest(int node, int where) const
    {
        struct QueueEntry
        {
            int Node = NullNode;
            double Cost = 0.0;
        };

        struct GreaterCost
        {
            bool operator()(const QueueEntry& left, const QueueEntry& right) const
            {
                return left.Cost > right.Cost;
            }
        };

        const Node& nb = nodes_[static_cast<std::size_t>(node)];
        const double rootMergedArea =
            TreeBox::MergedSurface(nodes_[static_cast<std::size_t>(where)].ExpandedBox, nb.ExpandedBox);

        double bestCost = std::numeric_limits<double>::max();
        int currentBest = where;

        std::priority_queue<QueueEntry, std::vector<QueueEntry>, GreaterCost> priorityQueue;
        priorityQueue.push(QueueEntry {where, rootMergedArea});

        while (!priorityQueue.empty())
        {
            const QueueEntry entry = priorityQueue.top();
            priorityQueue.pop();

            const int currentIndex = entry.Node;
            const double cost = entry.Cost;
            const Node& cn = nodes_[static_cast<std::size_t>(currentIndex)];

            const double mergedHere = TreeBox::MergedSurface(cn.ExpandedBox, nb.ExpandedBox);
            const double inheritedCostBeforeNode = cost - mergedHere;

            if (inheritedCostBeforeNode > bestCost)
            {
                continue;
            }

            if (cost < bestCost)
            {
                bestCost = cost;
                currentBest = currentIndex;
            }

            if (cn.IsLeaf())
            {
                continue;
            }

            const double oldSurface = cn.ExpandedBox.GetSurfaceArea();
            const double inheritedCostAfterNode = cost - oldSurface;

            const double leftMerged = TreeBox::MergedSurface(
                nodes_[static_cast<std::size_t>(cn.Left)].ExpandedBox,
                nb.ExpandedBox);
            const double rightMerged = TreeBox::MergedSurface(
                nodes_[static_cast<std::size_t>(cn.Right)].ExpandedBox,
                nb.ExpandedBox);

            const double leftCost = inheritedCostAfterNode + leftMerged;
            const double rightCost = inheritedCostAfterNode + rightMerged;

            priorityQueue.push(QueueEntry {cn.Left, leftCost});
            priorityQueue.push(QueueEntry {cn.Right, rightCost});
        }

        return currentBest;
    }

    int FindBestGreedy(int node, int where) const
    {
        const TreeBox& nodeTreeBox = nodes_[static_cast<std::size_t>(node)].ExpandedBox;

        double areaD = nodes_[static_cast<std::size_t>(node)].ExpandedBox.GetSurfaceArea();
        double areaBase = nodes_[static_cast<std::size_t>(where)].ExpandedBox.GetSurfaceArea();
        double directCost = TreeBox::MergedSurface(nodes_[static_cast<std::size_t>(where)].ExpandedBox, nodeTreeBox);
        double inheritedCost = 0.0;

        int bestSibling = where;
        double bestCost = directCost;

        while (!nodes_[static_cast<std::size_t>(where)].IsLeaf())
        {
            const int left = nodes_[static_cast<std::size_t>(where)].Left;
            const int right = nodes_[static_cast<std::size_t>(where)].Right;

            const double cost = directCost + inheritedCost;
            if (cost < bestCost)
            {
                bestSibling = where;
                bestCost = cost;
            }

            inheritedCost += directCost - areaBase;

            double lowerCostLeft = std::numeric_limits<double>::max();
            double directCostLeft = TreeBox::MergedSurface(nodes_[static_cast<std::size_t>(left)].ExpandedBox, nodeTreeBox);
            double areaLeft = 0.0;

            if (nodes_[static_cast<std::size_t>(left)].IsLeaf())
            {
                const double costLeft = directCostLeft + inheritedCost;
                if (costLeft < bestCost)
                {
                    bestSibling = left;
                    bestCost = costLeft;
                }
            }
            else
            {
                areaLeft = nodes_[static_cast<std::size_t>(left)].ExpandedBox.GetSurfaceArea();
                lowerCostLeft = inheritedCost + directCostLeft + std::min(areaD - areaLeft, 0.0);
            }

            double lowerCostRight = std::numeric_limits<double>::max();
            double directCostRight = TreeBox::MergedSurface(nodes_[static_cast<std::size_t>(right)].ExpandedBox, nodeTreeBox);
            double areaRight = 0.0;

            if (nodes_[static_cast<std::size_t>(right)].IsLeaf())
            {
                const double costRight = directCostRight + inheritedCost;
                if (costRight < bestCost)
                {
                    bestSibling = right;
                    bestCost = costRight;
                }
            }
            else
            {
                areaRight = nodes_[static_cast<std::size_t>(right)].ExpandedBox.GetSurfaceArea();
                lowerCostRight = inheritedCost + directCostRight + std::min(areaD - areaRight, 0.0);
            }

            if (bestCost <= lowerCostLeft && bestCost <= lowerCostRight)
            {
                break;
            }

            if (lowerCostLeft == lowerCostRight)
            {
                const LinearMath::JVector center = nodeTreeBox.Center();
                lowerCostLeft = (nodes_[static_cast<std::size_t>(left)].ExpandedBox.Center() - center).LengthSquared();
                lowerCostRight = (nodes_[static_cast<std::size_t>(right)].ExpandedBox.Center() - center).LengthSquared();
            }

            if (lowerCostLeft < lowerCostRight)
            {
                where = left;
                areaBase = areaLeft;
                directCost = directCostLeft;
            }
            else
            {
                where = right;
                areaBase = areaRight;
                directCost = directCostRight;
            }
        }

        return bestSibling;
    }

    void InsertLeaf(int node, int where)
    {
        if (root_ == NullNode)
        {
            root_ = node;
            nodes_[static_cast<std::size_t>(root_)].Parent = NullNode;
            return;
        }

        const TreeBox& nodeTreeBox = nodes_[static_cast<std::size_t>(node)].ExpandedBox;

        if (where == NullNode)
        {
            where = root_;
        }

        while (where != root_)
        {
            if (nodes_[static_cast<std::size_t>(where)].ExpandedBox.Contains(nodeTreeBox))
            {
                break;
            }

            where = nodes_[static_cast<std::size_t>(where)].Parent;
        }

        const int insertionParent = nodes_[static_cast<std::size_t>(where)].Parent;
        const int sibling = FindBestGreedy(node, where);
        const int oldParent = nodes_[static_cast<std::size_t>(sibling)].Parent;
        const int newParent = AllocateNode();

        nodes_[static_cast<std::size_t>(newParent)].Parent = oldParent;
        nodes_[static_cast<std::size_t>(newParent)].Entity = nullptr;
        nodes_[static_cast<std::size_t>(newParent)].Left = sibling;
        nodes_[static_cast<std::size_t>(newParent)].Right = node;
        TreeBox::CreateMerged(
            nodes_[static_cast<std::size_t>(sibling)].ExpandedBox,
            nodes_[static_cast<std::size_t>(node)].ExpandedBox,
            nodes_[static_cast<std::size_t>(newParent)].ExpandedBox);

        if (oldParent != NullNode)
        {
            if (nodes_[static_cast<std::size_t>(oldParent)].Left == sibling)
            {
                nodes_[static_cast<std::size_t>(oldParent)].Left = newParent;
            }
            else
            {
                nodes_[static_cast<std::size_t>(oldParent)].Right = newParent;
            }

            nodes_[static_cast<std::size_t>(sibling)].Parent = newParent;
            nodes_[static_cast<std::size_t>(node)].Parent = newParent;
        }
        else
        {
            nodes_[static_cast<std::size_t>(sibling)].Parent = newParent;
            nodes_[static_cast<std::size_t>(node)].Parent = newParent;
            root_ = newParent;
        }

        int index = nodes_[static_cast<std::size_t>(node)].Parent;
        while (index != insertionParent)
        {
            const int left = nodes_[static_cast<std::size_t>(index)].Left;
            const int right = nodes_[static_cast<std::size_t>(index)].Right;

            TreeBox::CreateMerged(
                nodes_[static_cast<std::size_t>(left)].ExpandedBox,
                nodes_[static_cast<std::size_t>(right)].ExpandedBox,
                nodes_[static_cast<std::size_t>(index)].ExpandedBox);

            index = nodes_[static_cast<std::size_t>(index)].Parent;
        }
    }

    static Real MinDistBox(const LinearMath::JBoundingBox& queryBox, const TreeBox& targetBox)
    {
        const LinearMath::JVector extents = (queryBox.Max - queryBox.Min) * static_cast<Real>(0.5);
        const LinearMath::JVector center = (queryBox.Max + queryBox.Min) * static_cast<Real>(0.5);

        const Real dx = std::max(
            std::max(targetBox.Min.X - extents.X - center.X, center.X - targetBox.Max.X - extents.X),
            static_cast<Real>(0));
        const Real dy = std::max(
            std::max(targetBox.Min.Y - extents.Y - center.Y, center.Y - targetBox.Max.Y - extents.Y),
            static_cast<Real>(0));
        const Real dz = std::max(
            std::max(targetBox.Min.Z - extents.Z - center.Z, center.Z - targetBox.Max.Z - extents.Z),
            static_cast<Real>(0));

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    template<typename TSupport>
    bool QueryDistance(
        const TSupport& support,
        const LinearMath::JBoundingBox& queryBox,
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        const FindNearestPreFilter& preFilter,
        const FindNearestPostFilter& postFilter,
        FindNearestResult& result) const
    {
        if (root_ == NullNode)
        {
            return false;
        }

        std::vector<int> stack;
        stack.reserve(256);
        stack.push_back(root_);

        while (!stack.empty())
        {
            const int index = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<std::size_t>(index)];
            if (node.IsLeaf())
            {
                Proxy* candidate = node.Entity;
                if (candidate == nullptr)
                {
                    continue;
                }

                const auto* distanceTestable = dynamic_cast<const IDistanceTestable*>(candidate);
                if (distanceTestable == nullptr)
                {
                    continue;
                }

                if (preFilter && !preFilter(*candidate))
                {
                    continue;
                }

                FindNearestResult candidateResult;
                candidateResult.Entity = candidate;
                const bool separated = distanceTestable->Distance(
                    support,
                    orientation,
                    position,
                    candidateResult.PointA,
                    candidateResult.PointB,
                    candidateResult.Normal,
                    candidateResult.Distance);

                if (!separated)
                {
                    candidateResult.Distance = static_cast<Real>(0);
                    candidateResult.Normal = LinearMath::JVector::Zero();
                    if (postFilter && !postFilter(candidateResult))
                    {
                        continue;
                    }

                    result = candidateResult;
                    return true;
                }

                if (candidateResult.Distance > result.Distance)
                {
                    continue;
                }

                if (postFilter && !postFilter(candidateResult))
                {
                    continue;
                }

                result = candidateResult;
                continue;
            }

            const int left = node.Left;
            const int right = node.Right;

            Real leftDistance = std::numeric_limits<Real>::max();
            Real rightDistance = std::numeric_limits<Real>::max();

            bool leftHit = left != NullNode;
            bool rightHit = right != NullNode;

            if (leftHit)
            {
                leftDistance = MinDistBox(queryBox, nodes_[static_cast<std::size_t>(left)].ExpandedBox);
                leftHit = leftDistance <= result.Distance;
            }

            if (rightHit)
            {
                rightDistance = MinDistBox(queryBox, nodes_[static_cast<std::size_t>(right)].ExpandedBox);
                rightHit = rightDistance <= result.Distance;
            }

            if (leftHit && rightHit)
            {
                if (leftDistance < rightDistance)
                {
                    stack.push_back(right);
                    stack.push_back(left);
                }
                else
                {
                    stack.push_back(left);
                    stack.push_back(right);
                }
            }
            else
            {
                if (leftHit)
                {
                    stack.push_back(left);
                }

                if (rightHit)
                {
                    stack.push_back(right);
                }
            }
        }

        return result.Entity != nullptr;
    }

    static bool SweepBox(
        const LinearMath::JBoundingBox& movingBox,
        const LinearMath::JVector& translation,
        const TreeBox& targetBox,
        Real& enter)
    {
        const LinearMath::JVector extents = (movingBox.Max - movingBox.Min) * static_cast<Real>(0.5);
        const LinearMath::JVector center = (movingBox.Max + movingBox.Min) * static_cast<Real>(0.5);
        const LinearMath::JBoundingBox expanded(targetBox.Min - extents, targetBox.Max + extents);
        return expanded.RayIntersect(center, translation, enter);
    }

    class DotNetRandom
    {
    public:
        explicit DotNetRandom(int seed)
        {
            int subtraction = seed == std::numeric_limits<int>::min()
                ? std::numeric_limits<int>::max()
                : (seed < 0 ? -seed : seed);
            int mj = MSeed - subtraction;
            if (mj < 0)
            {
                mj += MBig;
            }

            seedArray_[55] = mj;
            int mk = 1;

            for (int i = 1; i < 55; ++i)
            {
                const int ii = (21 * i) % 55;
                seedArray_[static_cast<std::size_t>(ii)] = mk;
                mk = mj - mk;
                if (mk < 0)
                {
                    mk += MBig;
                }
                mj = seedArray_[static_cast<std::size_t>(ii)];
            }

            for (int k = 1; k < 5; ++k)
            {
                for (int i = 1; i < 56; ++i)
                {
                    seedArray_[static_cast<std::size_t>(i)] -=
                        seedArray_[static_cast<std::size_t>(1 + (i + 30) % 55)];
                    if (seedArray_[static_cast<std::size_t>(i)] < 0)
                    {
                        seedArray_[static_cast<std::size_t>(i)] += MBig;
                    }
                }
            }
        }

        [[nodiscard]] double NextDouble()
        {
            return static_cast<double>(InternalSample()) * (1.0 / static_cast<double>(MBig));
        }

    private:
        int InternalSample()
        {
            int locINext = inext_;
            if (++locINext >= 56)
            {
                locINext = 1;
            }

            int locINextp = inextp_;
            if (++locINextp >= 56)
            {
                locINextp = 1;
            }

            int result = seedArray_[static_cast<std::size_t>(locINext)]
                - seedArray_[static_cast<std::size_t>(locINextp)];
            if (result == MBig)
            {
                --result;
            }
            if (result < 0)
            {
                result += MBig;
            }

            seedArray_[static_cast<std::size_t>(locINext)] = result;
            inext_ = locINext;
            inextp_ = locINextp;
            return result;
        }

        static constexpr int MBig = 2147483647;
        static constexpr int MSeed = 161803398;

        std::array<int, 56> seedArray_ {};
        int inext_ = 0;
        int inextp_ = 21;
    };

    std::vector<Node> nodes_;
    std::vector<int> freeNodes_;
    DataStructures::PartitionedSet<Proxy> proxies_;
    DataStructures::SlimBag<Proxy*> movedProxies_;
    std::vector<Proxy*> tempList_;
    PairHashSet potentialPairs_;
    FilterFunction filter_;
    DotNetRandom random_ {1234};
    int root_ = NullNode;
    std::size_t updatedProxyCount_ = 0;
    std::array<double, static_cast<std::size_t>(Timings::Last)> debugTimings_ {};
    std::size_t stepper_ = 0;
};

} // namespace Jitter2::Collision
