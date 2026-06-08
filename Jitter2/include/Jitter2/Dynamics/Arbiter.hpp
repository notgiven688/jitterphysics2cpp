#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Jitter2/Dynamics/ArbiterKey.hpp>
#include <Jitter2/Dynamics/Contact.hpp>

namespace Jitter2
{

// Manages contact information between two different rigid bodies.
// An arbiter is created when two shapes begin overlapping and is removed when they separate
// or when one of the involved bodies is removed from the world. Each arbiter can hold up to
// four cached contact points (see ContactData).
// In most cases arbiters are keyed by shape identifiers. For arbiters created by the engine, the ordering
// is canonical: the shape with the smaller ID is always first.
// The Handle property provides access to the underlying ContactData
// stored in unmanaged memory. This data is only valid while the arbiter exists and must not
// be accessed concurrently with World.Step(Real, bool).
class Arbiter
{
public:
    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }

    [[nodiscard]] Unmanaged::JHandle<ContactData> Handle() const { return handle_; }
    [[nodiscard]] ContactData& Data() { return handle_.Data(); }
    [[nodiscard]] const ContactData& Data() const { return handle_.Data(); }

    [[nodiscard]] const ArbiterKey& Key() const { return handle_.Data().Key; }

private:
    static std::unique_ptr<Arbiter> GetFromPool()
    {
        std::vector<std::unique_ptr<Arbiter>>& pool = Pool();
        if (pool.empty())
        {
            return std::make_unique<Arbiter>();
        }

        std::unique_ptr<Arbiter> arbiter = std::move(pool.back());
        pool.pop_back();
        return arbiter;
    }

    static void ReturnToPool(std::unique_ptr<Arbiter> arbiter)
    {
        if (!arbiter)
        {
            return;
        }

        arbiter->handle_ = Unmanaged::JHandle<ContactData>::Zero();
        arbiter->body1_ = nullptr;
        arbiter->body2_ = nullptr;
        arbiter->lastSeenStep_ = 0;
        Pool().push_back(std::move(arbiter));
    }

    static std::vector<std::unique_ptr<Arbiter>>& Pool()
    {
        thread_local std::vector<std::unique_ptr<Arbiter>> pool;
        return pool;
    }

    void Create(
        Unmanaged::JHandle<ContactData> handle,
        RigidBody& body1,
        RigidBody& body2,
        ArbiterKey key,
        Real speculativeRelaxationFactor)
    {
        handle_ = handle;
        body1_ = &body1;
        body2_ = &body2;
        ContactData& data = handle_.Data();
        data.Init(body1, body2, speculativeRelaxationFactor);
        data.Key = key;
    }

    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    Unmanaged::JHandle<ContactData> handle_;
    std::uint64_t lastSeenStep_ = 0;

    friend class World;
};

} // namespace Jitter2
