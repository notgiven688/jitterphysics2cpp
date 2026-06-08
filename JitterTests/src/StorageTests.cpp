#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Jitter2/Jitter2.hpp>

#include "TestSupport.hpp"

using Jitter2::DataStructures::ShardedDictionary;
using Jitter2::DataStructures::SlimBag;
using Jitter2::Dynamics::Constraints::ConstraintData;
using Jitter2::Dynamics::Constraints::SmallConstraintData;
using Jitter2::Dynamics::Constraints::TypedConstraint;
using Jitter2::Unmanaged::JHandle;
using Jitter2::Unmanaged::PartitionedBuffer;
using Jitter2::World;
using JitterTests::Require;

namespace
{

struct StorageData
{
    int _index = 0;
    int Value = 0;
};

static_assert(sizeof(StorageData) >= sizeof(int));

struct TinyConstraintData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;
    JHandle<Jitter2::RigidBodyData> Body1;
    JHandle<Jitter2::RigidBodyData> Body2;
    Jitter2::Real Value = 0;
};

struct LargeConstraintData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;
    JHandle<Jitter2::RigidBodyData> Body1;
    JHandle<Jitter2::RigidBodyData> Body2;
    std::array<std::byte, Jitter2::ConstraintSizeSmall> Padding {};
};

static_assert(sizeof(TinyConstraintData) <= sizeof(SmallConstraintData));
static_assert(sizeof(LargeConstraintData) > sizeof(SmallConstraintData));
static_assert(sizeof(LargeConstraintData) <= sizeof(ConstraintData));

class TinyConstraint final : public TypedConstraint<TinyConstraintData>
{
public:
    void PrepareForIteration(Jitter2::Real) override
    {
        PrepareForIterationTiny(Data(), static_cast<Jitter2::Real>(0));
    }

    void Iterate(Jitter2::Real) override
    {
        IterateTiny(Data(), static_cast<Jitter2::Real>(0));
    }

    [[nodiscard]] Jitter2::Real Value() const
    {
        return Data().Value;
    }

    static void PrepareForIterationTiny(TinyConstraintData& data, Jitter2::Real)
    {
        data.Value = static_cast<Jitter2::Real>(1);
    }

    static void IterateTiny(TinyConstraintData&, Jitter2::Real)
    {
    }

protected:
    void Create() override
    {
        DispatchId(RegisteredDispatchId());
    }

private:
    static std::uint32_t RegisteredDispatchId()
    {
        static const std::uint32_t id =
            RegisterFullConstraint<TinyConstraintData, &PrepareForIterationTiny, &IterateTiny>();
        return id;
    }
};

class LargeConstraint final : public TypedConstraint<LargeConstraintData>
{
public:
    void PrepareForIteration(Jitter2::Real) override
    {
        PrepareForIterationLarge(Data(), static_cast<Jitter2::Real>(0));
    }

    void Iterate(Jitter2::Real) override
    {
        IterateLarge(Data(), static_cast<Jitter2::Real>(0));
    }

    static void PrepareForIterationLarge(LargeConstraintData&, Jitter2::Real)
    {
    }

    static void IterateLarge(LargeConstraintData&, Jitter2::Real)
    {
    }

protected:
    void Create() override
    {
        DispatchId(RegisteredDispatchId());
    }

private:
    static std::uint32_t RegisteredDispatchId()
    {
        static const std::uint32_t id =
            RegisterFullConstraint<LargeConstraintData, &PrepareForIterationLarge, &IterateLarge>();
        return id;
    }
};

void HandlesSurviveResize()
{
    PartitionedBuffer<StorageData> buffer(2);
    JHandle<StorageData> a = buffer.Allocate(true, true);
    JHandle<StorageData> b = buffer.Allocate(false, true);

    a.Data().Value = 10;
    b.Data().Value = 20;

    JHandle<StorageData> c = buffer.Allocate(true, true);
    c.Data().Value = 30;

    Require(buffer.Capacity() >= 4, "buffer resized");
    Require(a.Data().Value == 10, "handle a survived resize");
    Require(b.Data().Value == 20, "handle b survived resize");
    Require(c.Data().Value == 30, "handle c survived resize");
    Require(buffer.Count() == 3, "count after resize");
}

void ConstraintDataSizesMatchPrecisionConstants()
{
    Require(sizeof(SmallConstraintData) == Jitter2::ConstraintSizeSmall, "small constraint data size");
    Require(sizeof(ConstraintData) == Jitter2::ConstraintSizeFull, "full constraint data size");
}

void TypedConstraintsSelectSmallAndFullBuffers()
{
    World world;
    auto& bodyA = world.CreateRigidBody();
    auto& bodyB = world.CreateRigidBody();

    TinyConstraint& tiny = world.CreateConstraint<TinyConstraint>(bodyA, bodyB);
    LargeConstraint& large = world.CreateConstraint<LargeConstraint>(bodyA, bodyB);

    Require(tiny.IsSmallConstraint(), "typed small constraint reports small storage");
    Require(!tiny.SmallHandle().IsZero(), "typed small constraint has small handle");
    Require(tiny.Handle().IsZero(), "typed small constraint does not allocate full handle");
    tiny.PrepareForIteration(static_cast<Jitter2::Real>(0));
    Require(tiny.Value() == static_cast<Jitter2::Real>(1), "typed small constraint accesses payload data");

    Require(!large.IsSmallConstraint(), "typed full constraint reports full storage");
    Require(large.SmallHandle().IsZero(), "typed full constraint does not allocate small handle");
    Require(!large.Handle().IsZero(), "typed full constraint has full handle");
}

void ActiveInactivePartitioning()
{
    PartitionedBuffer<StorageData> buffer(4);
    JHandle<StorageData> a = buffer.Allocate(false, true);
    JHandle<StorageData> b = buffer.Allocate(false, true);
    JHandle<StorageData> c = buffer.Allocate(true, true);

    a.Data().Value = 1;
    b.Data().Value = 2;
    c.Data().Value = 3;

    Require(buffer.ActiveCount() == 1, "initial active count");
    Require(buffer.IsActive(c), "c active");
    Require(!buffer.IsActive(a), "a inactive");

    buffer.MoveToActive(a);
    Require(buffer.ActiveCount() == 2, "active count after activating a");
    Require(buffer.IsActive(a), "a active");

    buffer.MoveToInactive(c);
    Require(buffer.ActiveCount() == 1, "active count after deactivating c");
    Require(!buffer.IsActive(c), "c inactive");
    Require(a.Data().Value == 1, "a data preserved after partition swaps");
    Require(b.Data().Value == 2, "b data preserved after partition swaps");
    Require(c.Data().Value == 3, "c data preserved after partition swaps");
}

void FreeCompactsAndUpdatesHandles()
{
    PartitionedBuffer<StorageData> buffer(4);
    JHandle<StorageData> a = buffer.Allocate(true, true);
    JHandle<StorageData> b = buffer.Allocate(true, true);
    JHandle<StorageData> c = buffer.Allocate(false, true);

    a.Data().Value = 11;
    b.Data().Value = 22;
    c.Data().Value = 33;

    buffer.Free(b);

    Require(buffer.Count() == 2, "count after free");
    Require(buffer.ActiveCount() == 1, "active count after freeing active element");
    Require(a.Data().Value == 11, "a data after free");
    Require(c.Data().Value == 33, "c data after free");
    Require(buffer.GetIndex(a) < buffer.Count(), "a index valid after free");
    Require(buffer.GetIndex(c) < buffer.Count(), "c index valid after free");
}

void SlimBagAddRemoveAndClear()
{
    SlimBag<int> bag(2);
    bag.Add(1);
    bag.Add(2);
    bag.Add(3);

    Require(bag.Count() == 3, "slim bag count after add");
    Require(bag.InternalSize() >= 4, "slim bag resized");

    bag.Remove(2);
    Require(bag.Count() == 2, "slim bag count after remove");
    Require((bag[0] == 1 || bag[1] == 1), "slim bag keeps first remaining item");
    Require((bag[0] == 3 || bag[1] == 3), "slim bag keeps second remaining item");

    bag.Clear();
    Require(bag.Count() == 0, "slim bag clear");
}

void SlimBagConcurrentAddStoresAllItems()
{
    constexpr int ThreadCount = 4;
    constexpr int ItemsPerThread = 250;
    constexpr int Total = ThreadCount * ItemsPerThread;

    SlimBag<int> bag(1);
    std::vector<std::thread> threads;
    threads.reserve(ThreadCount);

    for (int thread = 0; thread < ThreadCount; ++thread)
    {
        threads.emplace_back(
            [&bag, thread]
            {
                for (int i = 0; i < ItemsPerThread; ++i)
                {
                    bag.ConcurrentAdd(thread * ItemsPerThread + i);
                }
            });
    }

    for (std::thread& thread : threads)
    {
        thread.join();
    }

    Require(bag.Count() == Total, "slim bag concurrent count");

    std::vector<bool> seen(static_cast<std::size_t>(Total), false);
    for (int i = 0; i < bag.Count(); ++i)
    {
        const int value = bag[i];
        Require(value >= 0 && value < Total, "slim bag concurrent value range");
        seen[static_cast<std::size_t>(value)] = true;
    }

    for (bool valueSeen : seen)
    {
        Require(valueSeen, "slim bag concurrent value present");
    }
}

void ShardedDictionaryStoresAndTakesMoveOnlyValues()
{
    ShardedDictionary<int, std::unique_ptr<int>> dictionary(8);

    {
        std::lock_guard lock(dictionary.GetLock(10));
        dictionary.Add(10, std::make_unique<int>(42));
    }

    {
        std::lock_guard lock(dictionary.GetLock(10));
        std::unique_ptr<int>* value = dictionary.TryGetValue(10);
        Require(value != nullptr && *value != nullptr, "sharded dictionary finds move-only value");
        Require(**value == 42, "sharded dictionary value");
    }

    std::unique_ptr<int> taken;
    {
        std::lock_guard lock(dictionary.GetLock(10));
        taken = dictionary.Take(10);
    }

    Require(taken != nullptr && *taken == 42, "sharded dictionary takes move-only value");

    {
        std::lock_guard lock(dictionary.GetLock(10));
        Require(dictionary.TryGetValue(10) == nullptr, "sharded dictionary value removed after take");
    }
}

} // namespace

JITTER_TEST_CASE("PartitionedBuffer handles survive resize")
{
    HandlesSurviveResize();
}

JITTER_TEST_CASE("Constraint data sizes match precision constants")
{
    ConstraintDataSizesMatchPrecisionConstants();
}

JITTER_TEST_CASE("Typed constraints select small and full buffers")
{
    TypedConstraintsSelectSmallAndFullBuffers();
}

JITTER_TEST_CASE("PartitionedBuffer active inactive partitioning")
{
    ActiveInactivePartitioning();
}

JITTER_TEST_CASE("PartitionedBuffer free compacts and updates handles")
{
    FreeCompactsAndUpdatesHandles();
}

JITTER_TEST_CASE("SlimBag add remove and clear")
{
    SlimBagAddRemoveAndClear();
}

JITTER_TEST_CASE("SlimBag concurrent add stores all items")
{
    SlimBagConcurrentAddStoresAllItems();
}

JITTER_TEST_CASE("ShardedDictionary stores and takes move-only values")
{
    ShardedDictionaryStoresAndTakesMoveOnlyValues();
}
