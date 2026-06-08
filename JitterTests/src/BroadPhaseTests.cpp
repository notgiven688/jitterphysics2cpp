#include <algorithm>
#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::MotionType;
using Jitter2::World;
using Jitter2::Collision::DynamicTree;
using Jitter2::Collision::TreeBox;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::Collision::Shapes::Shape;
using Jitter2::Collision::Shapes::SphereShape;
using Jitter2::LinearMath::JBoundingBox;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

bool Contains(const std::vector<DynamicTree::Proxy*>& hits, Shape& shape)
{
    return std::find(hits.begin(), hits.end(), &shape) != hits.end();
}

class FixedRayProxy final : public Jitter2::Collision::IDynamicTreeProxy,
                            public Jitter2::Collision::IRayCastable
{
public:
    explicit FixedRayProxy(const JBoundingBox& box)
        : box_(box)
    {
    }

    [[nodiscard]] int NodePtr() const override { return nodePtr_; }
    void NodePtr(int value) override { nodePtr_ = value; }
    [[nodiscard]] JVector Velocity() const override { return JVector::Zero(); }
    [[nodiscard]] const JBoundingBox& WorldBoundingBox() const override { return box_; }
    void SetWorldBoundingBox(const JBoundingBox& box) { box_ = box; }

    bool RayCast(
        const JVector& origin,
        const JVector& direction,
        JVector& normal,
        Jitter2::Real& lambda) const override
    {
        normal = JVector::Zero();
        return box_.RayIntersect(origin, direction, lambda);
    }

private:
    JBoundingBox box_;
    int nodePtr_ = DynamicTree::NullNode;
};

class FixedDistanceSweepProxy final : public Jitter2::Collision::IDynamicTreeProxy,
                                      public Jitter2::Collision::IDistanceTestable,
                                      public Jitter2::Collision::ISweepTestable
{
public:
    FixedDistanceSweepProxy(const JBoundingBox& box, Jitter2::Real distance, Jitter2::Real lambda)
        : box_(box),
          distance_(distance),
          lambda_(lambda)
    {
    }

    [[nodiscard]] int NodePtr() const override { return nodePtr_; }
    void NodePtr(int value) override { nodePtr_ = value; }
    [[nodiscard]] JVector Velocity() const override { return JVector::Zero(); }
    [[nodiscard]] const JBoundingBox& WorldBoundingBox() const override { return box_; }

    bool Distance(
        const Jitter2::Collision::ISupportMappable& support,
        const JQuaternion&,
        const JVector& position,
        JVector& pointA,
        JVector& pointB,
        JVector& normal,
        Jitter2::Real& distance) const override
    {
        JVector center;
        support.GetCenter(center);
        pointA = position + center;
        pointB = box_.Center();
        normal = JVector::UnitX();
        distance = distance_;
        return true;
    }

    bool Sweep(
        const Jitter2::Collision::ISupportMappable& support,
        const JQuaternion&,
        const JVector& position,
        const JVector&,
        JVector& pointA,
        JVector& pointB,
        JVector& normal,
        Jitter2::Real& lambda) const override
    {
        JVector center;
        support.GetCenter(center);
        pointA = position + center;
        pointB = box_.Center();
        normal = JVector::UnitY();
        lambda = lambda_;
        return true;
    }

private:
    JBoundingBox box_;
    Jitter2::Real distance_;
    Jitter2::Real lambda_;
    int nodePtr_ = DynamicTree::NullNode;
};

void DynamicTreeAcceptsGenericProxy()
{
    DynamicTree tree;
    FixedRayProxy proxy(JBoundingBox(JVector(-1, -1, -1), JVector(1, 1, 1)));

    tree.AddProxy(proxy, false);

    Require(tree.Count() == 1, "generic proxy added to tree");
    Require(proxy.NodePtr() != DynamicTree::NullNode, "generic proxy node pointer set");
    Require(!tree.IsActive(proxy), "generic proxy inactive when added inactive");

    std::vector<DynamicTree::Proxy*> hits;
    tree.Query(hits, JBoundingBox(JVector(-2, -2, -2), JVector(2, 2, 2)));
    Require(std::find(hits.begin(), hits.end(), &proxy) != hits.end(), "generic proxy queryable");

    DynamicTree::Proxy* hitProxy = nullptr;
    JVector normal;
    Jitter2::Real lambda = static_cast<Jitter2::Real>(0);
    const bool hit = tree.RayCast(JVector(-5, 0, 0), JVector::UnitX(), hitProxy, normal, lambda);

    Require(hit, "generic ray proxy ray cast hit");
    Require(hitProxy == &proxy, "generic ray proxy returned from ray cast");
    RequireClose(lambda, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "generic ray proxy lambda");
}

void DynamicTreeRejectsDuplicateAndForeignProxyRemoval()
{
    DynamicTree tree;
    FixedRayProxy proxy(JBoundingBox(JVector(-1, -1, -1), JVector(1, 1, 1)));
    FixedRayProxy foreign(JBoundingBox(JVector(2, 2, 2), JVector(3, 3, 3)));

    tree.AddProxy(proxy);

    bool duplicateThrew = false;
    try
    {
        tree.AddProxy(proxy);
    }
    catch (const std::invalid_argument&)
    {
        duplicateThrew = true;
    }

    Require(duplicateThrew, "duplicate proxy add throws");

    bool foreignRemoveThrew = false;
    try
    {
        tree.RemoveProxy(foreign);
    }
    catch (const std::logic_error&)
    {
        foreignRemoveThrew = true;
    }

    Require(foreignRemoveThrew, "foreign proxy removal throws");

    tree.RemoveProxy(proxy);
    Require(proxy.NodePtr() == DynamicTree::NullNode, "valid remove resets proxy node pointer");
}

void DynamicTreeNearestAndSweepUseProxyInterfaces()
{
    DynamicTree tree;
    FixedRayProxy rayOnly(JBoundingBox(JVector(20, -1, -1), JVector(22, 1, 1)));
    FixedDistanceSweepProxy queryable(
        JBoundingBox(JVector(4, -1, -1), JVector(6, 1, 1)),
        static_cast<Jitter2::Real>(3.5),
        static_cast<Jitter2::Real>(0.25));

    tree.AddProxy(rayOnly);
    tree.AddProxy(queryable);

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real distance = 0;
    bool hit = tree.FindNearestSphere(
        static_cast<Jitter2::Real>(0.5),
        JVector::Zero(),
        std::numeric_limits<Jitter2::Real>::max(),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        distance);

    Require(hit, "generic distance-testable proxy found");
    Require(proxy == &queryable, "nearest query returns non-shape distance-testable proxy");
    RequireClose(distance, static_cast<Jitter2::Real>(3.5), static_cast<Jitter2::Real>(1e-6), "generic proxy distance");
    Require(normal == JVector::UnitX(), "generic proxy distance normal");

    proxy = nullptr;
    Jitter2::Real lambda = 0;
    hit = tree.SweepCastSphere(
        static_cast<Jitter2::Real>(0.5),
        JVector::Zero(),
        JVector::UnitX(),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        lambda);

    Require(hit, "generic sweep-testable proxy found");
    Require(proxy == &queryable, "sweep query returns non-shape sweep-testable proxy");
    RequireClose(lambda, static_cast<Jitter2::Real>(0.25), static_cast<Jitter2::Real>(1e-6), "generic proxy sweep lambda");
    Require(normal == JVector::UnitY(), "generic proxy sweep normal");
}

void AddShapeRegistersProxy()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape shape(1);

    body.AddShape(shape);

    Require(world.DynamicTree().IsActive(shape), "attached shape is active in tree");
    Require(world.DynamicTree().Count() == 1, "tree proxy count after add");
    Require(shape.NodePtr() == 0, "shape node pointer set");
}

void MovingBodyUpdatesQueryImmediately()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape shape(1);
    body.AddShape(shape);

    std::vector<DynamicTree::Proxy*> hits;
    world.DynamicTree().Query(hits, JBoundingBox(JVector(-2, -2, -2), JVector(2, 2, 2)));
    Require(Contains(hits, shape), "initial query sees shape");

    body.Position(JVector(10, 0, 0));

    hits.clear();
    world.DynamicTree().Query(hits, JBoundingBox(JVector(-2, -2, -2), JVector(2, 2, 2)));
    Require(!Contains(hits, shape), "old query misses moved shape");

    hits.clear();
    world.DynamicTree().Query(hits, JBoundingBox(JVector(8, -2, -2), JVector(12, 2, 2)));
    Require(Contains(hits, shape), "new query sees moved shape");
}

void RotatingBodyUpdatesBoxBounds()
{
    World world;
    auto& body = world.CreateRigidBody();
    BoxShape box(4, 2, 2);
    body.AddShape(box);

    const auto before = box.WorldBoundingBox();
    const auto widthXBefore = before.Max.X - before.Min.X;
    const auto widthYBefore = before.Max.Y - before.Min.Y;

    body.Orientation(JQuaternion::CreateRotationZ(static_cast<Jitter2::Real>(3.14159265358979323846 / 2.0)));

    const auto after = box.WorldBoundingBox();
    const auto widthXAfter = after.Max.X - after.Min.X;
    const auto widthYAfter = after.Max.Y - after.Min.Y;

    Require(widthXBefore > widthYBefore, "box initially wider on x");
    Require(widthYAfter > widthXAfter, "box wider on y after z rotation");
}

void RemoveShapeUnregistersProxy()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape shape(1);
    body.AddShape(shape);

    body.RemoveShape(shape);

    Require(!world.DynamicTree().IsActive(shape), "removed shape inactive in tree");
    Require(world.DynamicTree().Count() == 0, "tree proxy count after shape remove");
    Require(shape.NodePtr() == DynamicTree::NullNode, "shape node pointer reset");
}

void RemoveBodyUnregistersShapes()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape sphere(1);
    BoxShape box(1);
    body.AddShape(sphere);
    body.AddShape(box);

    world.Remove(body);

    Require(world.DynamicTree().Count() == 0, "tree proxy count after body remove");
    Require(!world.DynamicTree().IsActive(sphere), "sphere inactive after body remove");
    Require(!world.DynamicTree().IsActive(box), "box inactive after body remove");
    Require(sphere.GetRigidBody() == nullptr, "sphere detached after body remove");
    Require(box.GetRigidBody() == nullptr, "box detached after body remove");
}

void ClearRemovesAllTreeProxies()
{
    World world;
    auto& nullBody = world.NullBody();
    SphereShape nullShape(1);
    nullBody.AddShape(nullShape);

    auto& body = world.CreateRigidBody();
    SphereShape dynamicShape(1);
    body.AddShape(dynamicShape);

    Require(world.DynamicTree().Count() == 2, "tree has null and dynamic shape before clear");

    world.Clear();

    Require(world.RigidBodies().size() == 1, "clear leaves null body only");
    Require(world.DynamicTree().Count() == 0, "clear removes all tree proxies");
    Require(!world.DynamicTree().IsActive(nullShape), "null body shape unregistered by clear");
    Require(nullShape.GetRigidBody() == &nullBody, "null body shape remains attached");
}

void EnumerateOverlapsUsesFilter()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();
    auto& third = world.CreateRigidBody();

    SphereShape a(1);
    SphereShape b(1);
    SphereShape c(1);
    first.AddShape(a);
    second.AddShape(b);
    third.AddShape(c);

    third.Position(JVector(10, 0, 0));

    std::size_t overlaps = 0;
    world.DynamicTree().EnumerateOverlaps([&overlaps](DynamicTree::Proxy&, DynamicTree::Proxy&) { ++overlaps; });
    Require(overlaps == 1, "one overlapping pair before filter");

    world.DynamicTree().Filter([&a](const DynamicTree::Proxy& left, const DynamicTree::Proxy& right)
    {
        return &left != &a && &right != &a;
    });

    overlaps = 0;
    world.DynamicTree().EnumerateOverlaps([&overlaps](DynamicTree::Proxy&, DynamicTree::Proxy&) { ++overlaps; });
    Require(overlaps == 0, "filter suppresses overlapping pair");
}

void EnumerateOverlapsSupportsMultithreadedSlotScan()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();
    auto& third = world.CreateRigidBody();

    SphereShape a(1);
    SphereShape b(1);
    SphereShape c(1);
    first.AddShape(a);
    second.AddShape(b);
    third.AddShape(c);
    third.Position(JVector(10, 0, 0));

    std::atomic<std::size_t> overlaps {0};
    world.DynamicTree().EnumerateOverlaps(
        [&overlaps](DynamicTree::Proxy&, DynamicTree::Proxy&)
        {
            overlaps.fetch_add(1, std::memory_order_relaxed);
        },
        true);

    Require(overlaps.load(std::memory_order_relaxed) == 1, "multithreaded overlap enumeration count");
}

void DynamicTreeMultithreadedUpdateStoresMovedOverlapPairs()
{
    constexpr int ProxyCount = 64;

    DynamicTree tree;
    std::vector<FixedRayProxy> proxies;
    proxies.reserve(ProxyCount);

    for (int i = 0; i < ProxyCount; ++i)
    {
        const Jitter2::Real x = static_cast<Jitter2::Real>(i * 10);
        proxies.emplace_back(JBoundingBox(
            JVector(x - static_cast<Jitter2::Real>(0.25), -static_cast<Jitter2::Real>(0.25), -static_cast<Jitter2::Real>(0.25)),
            JVector(x + static_cast<Jitter2::Real>(0.25), static_cast<Jitter2::Real>(0.25), static_cast<Jitter2::Real>(0.25))));
        tree.AddProxy(proxies.back());
    }

    Require(tree.PairCount() == 0, "separated proxies start without pairs");

    const JBoundingBox overlapBox(
        JVector(-static_cast<Jitter2::Real>(1), -static_cast<Jitter2::Real>(1), -static_cast<Jitter2::Real>(1)),
        JVector(static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1)));

    for (FixedRayProxy& proxy : proxies)
    {
        proxy.SetWorldBoundingBox(overlapBox);
    }

    tree.Update(true, static_cast<Jitter2::Real>(0));

    constexpr std::size_t ExpectedPairs = static_cast<std::size_t>(ProxyCount * (ProxyCount - 1) / 2);
    Require(tree.UpdatedProxyCount() == static_cast<std::size_t>(ProxyCount), "all moved proxies tracked");
    Require(tree.PairCount() == ExpectedPairs, "multithreaded moved overlap scan stores all pairs");

    std::atomic<std::size_t> overlaps {0};
    tree.EnumerateOverlaps(
        [&overlaps](DynamicTree::Proxy&, DynamicTree::Proxy&)
        {
            overlaps.fetch_add(1, std::memory_order_relaxed);
        },
        true);

    Require(overlaps.load(std::memory_order_relaxed) == ExpectedPairs, "multithreaded moved overlaps enumerate all pairs");
}

void DynamicTreeProxyActivationControlsActivePartition()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();

    SphereShape a(1);
    SphereShape b(1);
    first.AddShape(a);
    second.AddShape(b);

    Require(world.DynamicTree().IsActive(a), "first shape starts active");
    Require(world.DynamicTree().IsActive(b), "second shape starts active");
    Require(world.DynamicTree().PairCount() == 1, "active overlapping pair counted");

    world.DynamicTree().DeactivateProxy(a);
    Require(!world.DynamicTree().IsActive(a), "first shape deactivated");
    Require(world.DynamicTree().PairCount() == 1, "active-inactive overlapping pair counted");

    std::vector<DynamicTree::Proxy*> hits;
    world.DynamicTree().Query(hits, JBoundingBox(JVector(-2, -2, -2), JVector(2, 2, 2)));
    Require(Contains(hits, a), "inactive proxy remains queryable");

    world.DynamicTree().DeactivateProxy(b);
    Require(!world.DynamicTree().IsActive(b), "second shape deactivated");
    Require(world.DynamicTree().PairCount() == 1, "inactive-inactive pair remains stored until pruning");

    std::size_t overlapCount = 0;
    world.DynamicTree().EnumerateOverlaps(
        [&overlapCount](DynamicTree::Proxy&, DynamicTree::Proxy&)
        {
            ++overlapCount;
        });
    Require(overlapCount == 1, "inactive-inactive overlap enumeration before pruning");

    for (int i = 0; i < DynamicTree::PruningFraction; ++i)
    {
        world.DynamicTree().Update(false, static_cast<Jitter2::Real>(0));
    }

    Require(world.DynamicTree().PairCount() == 0, "inactive-inactive pair pruned like CSharp broadphase");

    world.DynamicTree().ActivateProxy(a);
    Require(world.DynamicTree().IsActive(a), "first shape reactivated");
    Require(world.DynamicTree().PairCount() == 0, "reactivated overlap pair waits for tree update");
    world.DynamicTree().Update(false, static_cast<Jitter2::Real>(0));
    Require(world.DynamicTree().PairCount() == 1, "reactivated overlapping pair counted");
}

void DynamicTreeTracksExpandedNodesAndMovedProxies()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();

    SphereShape a(1);
    SphereShape b(1);
    first.AddShape(a);
    second.AddShape(b);
    second.Position(JVector(10, 0, 0));

    Require(world.DynamicTree().Nodes().size() >= 2, "tree exposes nodes");
    Require(world.DynamicTree().Root() != DynamicTree::NullNode, "tree root exists");
    Require(!world.DynamicTree().Nodes()[static_cast<std::size_t>(world.DynamicTree().Root())].IsLeaf(), "tree root is internal for two shapes");
    Require(world.DynamicTree().UpdatedProxyCount() == 0, "updated proxy count starts zero");

    second.Position(JVector(static_cast<Jitter2::Real>(1.5), 0, 0));
    world.DynamicTree().DeactivateProxy(b);
    world.DynamicTree().ActivateProxy(b);
    world.DynamicTree().Update(false, static_cast<Jitter2::Real>(1.0 / 60.0));

    Require(world.DynamicTree().UpdatedProxyCount() > 0, "forced proxy update tracked during update");
    Require(world.DynamicTree().PairCount() == 1, "moved overlapping pair cached");
    const std::span<const double> timings = world.DynamicTree().DebugTimings();
    Require(
        timings.size() == static_cast<std::size_t>(DynamicTree::Timings::Last),
        "tree debug timings exposes one value per timing bucket");
    for (double timing : timings)
    {
        Require(timing >= 0.0, "tree debug timing is non-negative");
    }
    Require(
        std::string(DynamicTree::TimingName(DynamicTree::Timings::ScanOverlaps)) == "ScanOverlaps",
        "tree timing name matches CSharp enum name");
    const auto [slotCount, pairCount] = world.DynamicTree().HashSetInfo();
    Require(slotCount >= pairCount, "hash set info tracks pair capacity and count");
}

void DynamicTreeEnumeratesTreeBoxesByDepth()
{
    DynamicTree empty;
    std::size_t visited = 0;
    empty.EnumerateTreeBoxes([&visited](const TreeBox&, int) { ++visited; });
    Require(visited == 0, "empty tree enumerates no tree boxes");

    DynamicTree tree;
    FixedRayProxy first(JBoundingBox(JVector(-1, -1, -1), JVector(1, 1, 1)));
    FixedRayProxy second(JBoundingBox(JVector(4, -1, -1), JVector(6, 1, 1)));

    tree.AddProxy(first);

    int firstDepth = 0;
    tree.EnumerateTreeBoxes(
        [&visited, &firstDepth](const TreeBox&, int depth)
        {
            ++visited;
            firstDepth = depth;
        });

    Require(visited == 1, "single-node tree enumerates root box");
    Require(firstDepth == 1, "single-node tree starts at depth one");

    tree.AddProxy(second);

    visited = 0;
    std::size_t rootBoxes = 0;
    std::size_t leafBoxes = 0;
    int maxDepth = 0;
    tree.EnumerateTreeBoxes(
        [&visited, &rootBoxes, &leafBoxes, &maxDepth](const TreeBox&, int depth)
        {
            ++visited;
            if (depth == 1)
            {
                ++rootBoxes;
            }
            if (depth == 2)
            {
                ++leafBoxes;
            }
            maxDepth = std::max(maxDepth, depth);
        });

    Require(visited == 3, "two-proxy tree enumerates root and leaves");
    Require(rootBoxes == 1, "two-proxy tree has one root depth box");
    Require(leafBoxes == 2, "two-proxy tree has two leaf depth boxes");
    Require(maxDepth == 2, "two-proxy tree depth matches C# traversal");
}

void CalculateCostTracksEmptyAndNonEmptyTree()
{
    World world;
    RequireClose(static_cast<Jitter2::Real>(world.DynamicTree().CalculateCost()), 0, static_cast<Jitter2::Real>(1e-6), "empty tree cost");

    auto& body = world.CreateRigidBody();
    SphereShape sphere(1);
    body.AddShape(sphere);

    Require(world.DynamicTree().CalculateCost() > 0.0, "non-empty tree cost");
    world.Remove(body);
    RequireClose(static_cast<Jitter2::Real>(world.DynamicTree().CalculateCost()), 0, static_cast<Jitter2::Real>(1e-6), "empty tree cost after remove");
}

void RayCastReturnsClosestShape()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);

    nearBody.AddShape(nearShape);
    farBody.AddShape(farShape);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector normal;
    Jitter2::Real lambda = 0;
    const bool hit = world.DynamicTree().RayCast(JVector::Zero(), JVector::UnitX(), proxy, normal, lambda);

    Require(hit, "ray cast hit");
    Require(proxy == &nearShape, "ray cast closest proxy");
    RequireClose(lambda, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "ray cast closest lambda");
    Require(normal == JVector(-1, 0, 0), "ray cast normal");
}

void RayCastUsesPreAndPostFilters()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);

    nearBody.AddShape(nearShape);
    farBody.AddShape(farShape);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector normal;
    Jitter2::Real lambda = 0;

    bool hit = world.DynamicTree().RayCast(
        JVector::Zero(),
        JVector::UnitX(),
        [&nearShape](const DynamicTree::Proxy& candidate) { return &candidate != &nearShape; },
        {},
        proxy,
        normal,
        lambda);

    Require(hit, "ray cast pre-filter hit");
    Require(proxy == &farShape, "ray cast pre-filter skipped near shape");
    RequireClose(lambda, static_cast<Jitter2::Real>(8), static_cast<Jitter2::Real>(1e-6), "ray cast pre-filter lambda");

    proxy = nullptr;
    hit = world.DynamicTree().RayCast(
        JVector::Zero(),
        JVector::UnitX(),
        {},
        [](const DynamicTree::RayCastResult& result) { return result.Lambda > static_cast<Jitter2::Real>(4.5); },
        proxy,
        normal,
        lambda);

    Require(hit, "ray cast post-filter hit");
    Require(proxy == &farShape, "ray cast post-filter skipped near shape");
}

void RayCastRespectsMaxLambda()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape shape(1);

    body.AddShape(shape);
    body.Position(JVector(5, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector normal;
    Jitter2::Real lambda = 0;

    bool hit = world.DynamicTree().RayCast(
        JVector::Zero(),
        JVector::UnitX(),
        static_cast<Jitter2::Real>(3.9),
        {},
        {},
        proxy,
        normal,
        lambda);

    Require(!hit, "bounded ray cast excludes hit");
    Require(proxy == nullptr, "bounded ray cast proxy null");
    RequireClose(lambda, static_cast<Jitter2::Real>(3.9), static_cast<Jitter2::Real>(1e-6), "bounded ray cast keeps max lambda");

    hit = world.DynamicTree().RayCast(
        JVector::Zero(),
        JVector::UnitX(),
        static_cast<Jitter2::Real>(4.1),
        {},
        {},
        proxy,
        normal,
        lambda);

    Require(hit, "bounded ray cast includes hit");
    Require(proxy == &shape, "bounded ray cast proxy");
    RequireClose(lambda, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "bounded ray cast lambda");
}

void RayCastCanRunNestedQueryFromFilter()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);

    nearBody.AddShape(nearShape);
    farBody.AddShape(farShape);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    bool nested = false;
    DynamicTree::Proxy* proxy = nullptr;
    JVector normal;
    Jitter2::Real lambda = 0;

    const bool hit = world.DynamicTree().RayCast(
        JVector::Zero(),
        JVector::UnitX(),
        {},
        [&world, &nested](const DynamicTree::RayCastResult&)
        {
            if (!nested)
            {
                nested = true;
                std::vector<DynamicTree::Proxy*> hits;
                world.DynamicTree().Query(hits, JBoundingBox(JVector(4, -2, -2), JVector(6, 2, 2)));
                Require(!hits.empty(), "nested query produced hits");
            }
            return true;
        },
        proxy,
        normal,
        lambda);

    Require(hit, "ray cast with nested query hit");
    Require(nested, "nested query ran");
    Require(proxy == &nearShape, "nested query ray hit closest shape");
}

void RayQueryReturnsIntersectedProxies()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();
    auto& offRay = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);
    SphereShape sideShape(1);

    first.AddShape(nearShape);
    second.AddShape(farShape);
    offRay.AddShape(sideShape);
    first.Position(JVector(5, 0, 0));
    second.Position(JVector(9, 0, 0));
    offRay.Position(JVector(5, 5, 0));

    std::vector<DynamicTree::Proxy*> hits;
    world.DynamicTree().Query(hits, JVector::Zero(), JVector::UnitX());

    Require(Contains(hits, nearShape), "ray query includes near shape");
    Require(Contains(hits, farShape), "ray query includes far shape");
    Require(!Contains(hits, sideShape), "ray query excludes side shape");
}

void FindNearestPointReturnsClosestProxy()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    BoxShape nearBox(2);
    BoxShape farBox(2);

    nearBody.AddShape(nearBox);
    farBody.AddShape(farBox);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real distance = 0;

    const bool hit = world.DynamicTree().FindNearestPoint(
        JVector::Zero(), proxy, pointA, pointB, normal, distance);

    Require(hit, "nearest point hit");
    Require(proxy == &nearBox, "nearest point proxy");
    Require(pointA == JVector::Zero(), "nearest point query point");
    RequireClose(pointB.X, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "nearest point closest x");
    RequireClose(distance, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "nearest point distance");
    Require(normal == JVector(1, 0, 0), "nearest point normal");
}

void FindNearestSphereReturnsRadiusAdjustedDistance()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    BoxShape nearBox(2);
    BoxShape farBox(2);

    nearBody.AddShape(nearBox);
    farBody.AddShape(farBox);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real distance = 0;

    const bool hit = world.DynamicTree().FindNearestSphere(
        static_cast<Jitter2::Real>(1), JVector::Zero(), proxy, pointA, pointB, normal, distance);

    Require(hit, "nearest sphere hit");
    Require(proxy == &nearBox, "nearest sphere proxy");
    RequireClose(pointA.X, static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1e-6), "nearest sphere pointA x");
    RequireClose(pointB.X, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-6), "nearest sphere pointB x");
    RequireClose(distance, static_cast<Jitter2::Real>(3), static_cast<Jitter2::Real>(1e-6), "nearest sphere distance");
    RequireClose(normal.X, static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1e-4), "nearest sphere normal x");
    RequireClose(normal.Y, static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(1e-4), "nearest sphere normal y");
    RequireClose(normal.Z, static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(1e-4), "nearest sphere normal z");
}

void FindNearestSphereRespectsMaxDistanceAndOverlap()
{
    World world;
    auto& separatedBody = world.CreateRigidBody();
    auto& overlappingBody = world.CreateRigidBody();
    BoxShape separated(2);
    BoxShape overlapping(2);

    separatedBody.AddShape(separated);
    overlappingBody.AddShape(overlapping);
    separatedBody.Position(JVector(5, 0, 0));
    overlappingBody.Position(JVector::Zero());

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real distance = 0;

    bool hit = world.DynamicTree().FindNearestSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        static_cast<Jitter2::Real>(0),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        distance);

    Require(hit, "nearest sphere zero max distance finds overlap");
    Require(proxy == &overlapping, "nearest sphere overlap proxy");
    RequireClose(distance, static_cast<Jitter2::Real>(0), static_cast<Jitter2::Real>(1e-6), "nearest sphere overlap distance");

    world.Remove(overlappingBody);
    proxy = nullptr;
    hit = world.DynamicTree().FindNearestSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        static_cast<Jitter2::Real>(2.99),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        distance);

    Require(!hit, "nearest sphere max distance excludes separated shape");
    Require(proxy == nullptr, "nearest sphere excluded proxy null");
}

void FindNearestFiltersCanSkipCandidates()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    BoxShape nearBox(2);
    BoxShape farBox(2);

    nearBody.AddShape(nearBox);
    farBody.AddShape(farBox);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(8, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real distance = 0;

    bool hit = world.DynamicTree().FindNearestSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        [&nearBox](const DynamicTree::Proxy& candidate) { return &candidate != &nearBox; },
        {},
        proxy,
        pointA,
        pointB,
        normal,
        distance);

    Require(hit, "nearest pre-filter hit");
    Require(proxy == &farBox, "nearest pre-filter skipped closer proxy");
    RequireClose(distance, static_cast<Jitter2::Real>(6), static_cast<Jitter2::Real>(1e-4), "nearest pre-filter distance");

    proxy = nullptr;
    hit = world.DynamicTree().FindNearestSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        {},
        [](const DynamicTree::FindNearestResult& result)
        {
            return result.Distance > static_cast<Jitter2::Real>(3.5);
        },
        proxy,
        pointA,
        pointB,
        normal,
        distance);

    Require(hit, "nearest post-filter hit");
    Require(proxy == &farBox, "nearest post-filter skipped closer proxy");
}

void SweepCastSphereReturnsClosestShape()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);

    nearBody.AddShape(nearShape);
    farBody.AddShape(farShape);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real lambda = 0;

    const bool hit = world.DynamicTree().SweepCastSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        JVector::UnitX(),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        lambda);

    Require(hit, "sweep sphere hit");
    Require(proxy == &nearShape, "sweep sphere closest proxy");
    RequireClose(lambda, static_cast<Jitter2::Real>(3), static_cast<Jitter2::Real>(1e-4), "sweep sphere lambda");
    RequireClose(pointA.X, static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1e-4), "sweep sphere point a x");
    RequireClose(pointB.X, static_cast<Jitter2::Real>(4), static_cast<Jitter2::Real>(1e-4), "sweep sphere point b x");
    RequireClose(normal.X, static_cast<Jitter2::Real>(1), static_cast<Jitter2::Real>(1e-4), "sweep sphere normal x");
}

void SweepCastUsesFiltersAndMaxLambda()
{
    World world;
    auto& nearBody = world.CreateRigidBody();
    auto& farBody = world.CreateRigidBody();
    SphereShape nearShape(1);
    SphereShape farShape(1);

    nearBody.AddShape(nearShape);
    farBody.AddShape(farShape);
    nearBody.Position(JVector(5, 0, 0));
    farBody.Position(JVector(9, 0, 0));

    DynamicTree::Proxy* proxy = nullptr;
    JVector pointA;
    JVector pointB;
    JVector normal;
    Jitter2::Real lambda = 0;

    bool hit = world.DynamicTree().SweepCastSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        JVector::UnitX(),
        static_cast<Jitter2::Real>(2.9),
        {},
        {},
        proxy,
        pointA,
        pointB,
        normal,
        lambda);

    Require(!hit, "sweep max lambda excludes hit");
    Require(proxy == nullptr, "sweep excluded proxy null");

    hit = world.DynamicTree().SweepCastSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        JVector::UnitX(),
        [&nearShape](const DynamicTree::Proxy& candidate) { return &candidate != &nearShape; },
        {},
        proxy,
        pointA,
        pointB,
        normal,
        lambda);

    Require(hit, "sweep pre-filter hit");
    Require(proxy == &farShape, "sweep pre-filter skipped near");
    RequireClose(lambda, static_cast<Jitter2::Real>(7), static_cast<Jitter2::Real>(1e-4), "sweep pre-filter lambda");

    proxy = nullptr;
    hit = world.DynamicTree().SweepCastSphere(
        static_cast<Jitter2::Real>(1),
        JVector::Zero(),
        JVector::UnitX(),
        {},
        [](const DynamicTree::SweepCastResult& result)
        {
            return result.Lambda > static_cast<Jitter2::Real>(3.5);
        },
        proxy,
        pointA,
        pointB,
        normal,
        lambda);

    Require(hit, "sweep post-filter hit");
    Require(proxy == &farShape, "sweep post-filter skipped near");
}

} // namespace

JITTER_TEST_CASE("DynamicTree registers attached shapes")
{
    AddShapeRegistersProxy();
}

JITTER_TEST_CASE("DynamicTree accepts generic proxies")
{
    DynamicTreeAcceptsGenericProxy();
}

JITTER_TEST_CASE("DynamicTree rejects duplicate and foreign proxies")
{
    DynamicTreeRejectsDuplicateAndForeignProxyRemoval();
}

JITTER_TEST_CASE("DynamicTree nearest and sweep use proxy interfaces")
{
    DynamicTreeNearestAndSweepUseProxyInterfaces();
}

JITTER_TEST_CASE("DynamicTree query updates immediately after movement")
{
    MovingBodyUpdatesQueryImmediately();
}

JITTER_TEST_CASE("DynamicTree shape bounds update after rotation")
{
    RotatingBodyUpdatesBoxBounds();
}

JITTER_TEST_CASE("DynamicTree unregisters removed shapes")
{
    RemoveShapeUnregistersProxy();
}

JITTER_TEST_CASE("DynamicTree unregisters removed body shapes")
{
    RemoveBodyUnregistersShapes();
}

JITTER_TEST_CASE("World clear removes tree proxies")
{
    ClearRemovesAllTreeProxies();
}

JITTER_TEST_CASE("DynamicTree overlap enumeration uses filter")
{
    EnumerateOverlapsUsesFilter();
}

JITTER_TEST_CASE("DynamicTree overlap enumeration supports multithreading")
{
    EnumerateOverlapsSupportsMultithreadedSlotScan();
}

JITTER_TEST_CASE("DynamicTree multithreaded update stores moved overlap pairs")
{
    DynamicTreeMultithreadedUpdateStoresMovedOverlapPairs();
}

JITTER_TEST_CASE("DynamicTree proxy activation controls active partition")
{
    DynamicTreeProxyActivationControlsActivePartition();
}

JITTER_TEST_CASE("DynamicTree tracks expanded nodes and moved proxies")
{
    DynamicTreeTracksExpandedNodesAndMovedProxies();
}

JITTER_TEST_CASE("DynamicTree enumerates tree boxes by depth")
{
    DynamicTreeEnumeratesTreeBoxesByDepth();
}

JITTER_TEST_CASE("DynamicTree cost tracks empty and non-empty")
{
    CalculateCostTracksEmptyAndNonEmptyTree();
}

JITTER_TEST_CASE("DynamicTree ray cast returns closest shape")
{
    RayCastReturnsClosestShape();
}

JITTER_TEST_CASE("DynamicTree ray cast uses filters")
{
    RayCastUsesPreAndPostFilters();
}

JITTER_TEST_CASE("DynamicTree ray cast respects max lambda")
{
    RayCastRespectsMaxLambda();
}

JITTER_TEST_CASE("DynamicTree ray cast permits nested queries")
{
    RayCastCanRunNestedQueryFromFilter();
}

JITTER_TEST_CASE("DynamicTree ray query returns intersected proxies")
{
    RayQueryReturnsIntersectedProxies();
}

JITTER_TEST_CASE("DynamicTree nearest point returns closest proxy")
{
    FindNearestPointReturnsClosestProxy();
}

JITTER_TEST_CASE("DynamicTree nearest sphere adjusts distance by radius")
{
    FindNearestSphereReturnsRadiusAdjustedDistance();
}

JITTER_TEST_CASE("DynamicTree nearest sphere max distance and overlap")
{
    FindNearestSphereRespectsMaxDistanceAndOverlap();
}

JITTER_TEST_CASE("DynamicTree nearest queries use filters")
{
    FindNearestFiltersCanSkipCandidates();
}

JITTER_TEST_CASE("DynamicTree sweep cast returns closest shape")
{
    SweepCastSphereReturnsClosestShape();
}

JITTER_TEST_CASE("DynamicTree sweep cast uses filters and max lambda")
{
    SweepCastUsesFiltersAndMaxLambda();
}
