#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <Jitter2/IDebugDrawer.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Precision.hpp>
#include <Jitter2/Unmanaged/PartitionedBuffer.hpp>

namespace Jitter2::Dynamics::Constraints
{

namespace Detail
{
inline constexpr std::size_t ConstraintHeaderSize =

    sizeof(int) + sizeof(std::uint32_t) + sizeof(std::uint64_t)
    + 2 * sizeof(Unmanaged::JHandle<RigidBodyData>);
static_assert(ConstraintHeaderSize <= ConstraintSizeSmall);
} // namespace Detail

// Low-level data for constraints that fit within Precision.ConstraintSizeSmall bytes.
// This structure is stored in unmanaged memory and accessed via Constraint.SmallHandle.
// It contains a solver dispatch id, a stable constraint identifier, and handles to the connected bodies.
// The data is valid only while the constraint is registered with the world. Do not cache references
// across simulation steps. Not safe to access concurrently with World.Step(Real, bool).
struct SmallConstraintData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;
    // Handle to the first body's simulation data.
    Unmanaged::JHandle<RigidBodyData> Body1;
    // Handle to the second body's simulation data.
    Unmanaged::JHandle<RigidBodyData> Body2;
    std::array<std::byte, ConstraintSizeSmall - Detail::ConstraintHeaderSize> Reserved {};

    [[nodiscard]] bool IsEnabled() const { return DispatchId != 0; }
    void PrepareForIteration(SmallConstraintData& constraint, Real inverseDt) const;
    void Iterate(SmallConstraintData& constraint, Real inverseDt) const;
};

// Low-level data for constraints, stored in unmanaged memory.
// This structure is stored in unmanaged memory and accessed via Constraint.Handle.
// It contains a solver dispatch id, a stable constraint identifier, and handles to the connected bodies.
// The data is valid only while the constraint is registered with the world. Do not cache references
// across simulation steps. Not safe to access concurrently with World.Step(Real, bool).
struct ConstraintData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;
    // Handle to the first body's simulation data.
    Unmanaged::JHandle<RigidBodyData> Body1;
    // Handle to the second body's simulation data.
    Unmanaged::JHandle<RigidBodyData> Body2;
    std::array<std::byte, ConstraintSizeFull - Detail::ConstraintHeaderSize> Reserved {};

    [[nodiscard]] bool IsEnabled() const { return DispatchId != 0; }
    void PrepareForIteration(ConstraintData& constraint, Real inverseDt) const;
    void Iterate(ConstraintData& constraint, Real inverseDt) const;
};

static_assert(sizeof(SmallConstraintData) == ConstraintSizeSmall);
static_assert(sizeof(ConstraintData) == ConstraintSizeFull);
static_assert(std::is_trivially_copyable_v<SmallConstraintData>);
static_assert(std::is_trivially_copyable_v<ConstraintData>);

class ConstraintDispatchTable
{
public:
    using Function = void (*)(void*, Real);

    struct Entry
    {
        Function Prepare = nullptr;
        Function Iterate = nullptr;
    };

    static std::uint32_t Register(Function prepare, Function iterate)
    {
        if (prepare == nullptr)
        {
            throw std::invalid_argument("prepare must not be null.");
        }

        if (iterate == nullptr)
        {
            throw std::invalid_argument("iterate must not be null.");
        }

        std::lock_guard lock(Sync());
        std::vector<Entry>& entries = Entries();
        if (entries.size() == static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
        {
            throw std::logic_error("Too many registered constraint dispatch entries.");
        }

        const std::uint32_t id = static_cast<std::uint32_t>(entries.size());
        entries.push_back(Entry {prepare, iterate});
        return id;
    }

    [[nodiscard]] static const Entry& Get(std::uint32_t dispatchId)
    {
        return Entries().at(static_cast<std::size_t>(dispatchId));
    }

private:
    static std::vector<Entry>& Entries()
    {
        static std::vector<Entry> entries {Entry {}};
        return entries;
    }

    static std::mutex& Sync()
    {
        static std::mutex sync;
        return sync;
    }
};

inline void SmallConstraintData::PrepareForIteration(SmallConstraintData& constraint, Real inverseDt) const
{
    const ConstraintDispatchTable::Entry& dispatch = ConstraintDispatchTable::Get(DispatchId);
    dispatch.Prepare(&constraint, inverseDt);
}

inline void SmallConstraintData::Iterate(SmallConstraintData& constraint, Real inverseDt) const
{
    const ConstraintDispatchTable::Entry& dispatch = ConstraintDispatchTable::Get(DispatchId);
    dispatch.Iterate(&constraint, inverseDt);
}

inline void ConstraintData::PrepareForIteration(ConstraintData& constraint, Real inverseDt) const
{
    const ConstraintDispatchTable::Entry& dispatch = ConstraintDispatchTable::Get(DispatchId);
    dispatch.Prepare(&constraint, inverseDt);
}

inline void ConstraintData::Iterate(ConstraintData& constraint, Real inverseDt) const
{
    const ConstraintDispatchTable::Entry& dispatch = ConstraintDispatchTable::Get(DispatchId);
    dispatch.Iterate(&constraint, inverseDt);
}

// Generic base class for constraints that store custom data of type T.
// T: The unmanaged data structure containing constraint-specific state. Must fit within
// ConstraintData (i.e., Precision.ConstraintSizeFull bytes).
// Derive from this class to create constraints with custom data layouts. The Data
// property provides typed access to the constraint's unmanaged memory.
class Constraint : public IDebugDrawable
{
public:
    static constexpr Real DefaultAngularSoftness = static_cast<Real>(0.001);
    static constexpr Real DefaultAngularBias = static_cast<Real>(0.2);
    static constexpr Real DefaultAngularLimitSoftness = static_cast<Real>(0.001);
    static constexpr Real DefaultAngularLimitBias = static_cast<Real>(0.1);
    static constexpr Real DefaultLinearSoftness = static_cast<Real>(0.00001);
    static constexpr Real DefaultLinearBias = static_cast<Real>(0.2);
    static constexpr Real DefaultLinearLimitSoftness = static_cast<Real>(0.0001);
    static constexpr Real DefaultLinearLimitBias = static_cast<Real>(0.2);

    virtual ~Constraint() = default;

    Constraint(const Constraint&) = delete;
    Constraint& operator=(const Constraint&) = delete;
    Constraint(Constraint&&) = delete;
    Constraint& operator=(Constraint&&) = delete;

    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }
    [[nodiscard]] std::uint64_t ConstraintId() const { return constraintId_; }
    [[nodiscard]] Unmanaged::JHandle<ConstraintData> Handle() const { return handle_; }
    [[nodiscard]] Unmanaged::JHandle<SmallConstraintData> SmallHandle() const { return smallHandle_; }

    [[nodiscard]] virtual bool IsSmallConstraint() const { return false; }
    [[nodiscard]] bool IsEnabled() const
    {
        if (!smallHandle_.IsZero())
        {
            return smallHandle_.Data().IsEnabled();
        }

        if (!handle_.IsZero())
        {
            return handle_.Data().IsEnabled();
        }

        return enabled_;
    }

    void IsEnabled(bool value)
    {
        if (value && dispatchId_ == 0)
        {
            throw std::logic_error("The constraint has no registered solver dispatch.");
        }

        enabled_ = value;
        const std::uint32_t dispatchId = value ? dispatchId_ : 0u;
        if (!smallHandle_.IsZero())
        {
            smallHandle_.Data().DispatchId = dispatchId;
        }
        else if (!handle_.IsZero())
        {
            handle_.Data().DispatchId = dispatchId;
        }
    }

    virtual void PrepareForIteration(Real inverseDt) = 0;
    virtual void Iterate(Real inverseDt) = 0;

    // Resets the cached warm-start state used by the solver for this constraint.
    // This clears only persistent solver impulses. Constraint configuration remains unchanged.
    // Useful after restoring snapshots or other discontinuous state changes where preserving
    // warm-starting is undesirable.
    virtual void ResetWarmStart() {}

    // Draws a debug visualization of this constraint.
    // drawer: The debug drawer to receive visualization primitives.
    // Thrown if the derived class does not override this method.
    void DebugDraw(IDebugDrawer&) override
    {
        throw std::logic_error("DebugDraw is not implemented for this constraint.");
    }

protected:
    Constraint() = default;

    // Sets the solver dispatch id used by this instance. Override this in derived classes
    // to assign the correct registered solver pair. The id is later written into the
    // unmanaged header when the constraint is enabled.
    virtual void Create() {}

    void DispatchId(std::uint32_t value)
    {
        dispatchId_ = value;
    }

    template<typename TData, void (*Prepare)(TData&, Real), void (*Iterate)(TData&, Real)>
    static std::uint32_t RegisterFullConstraint()
    {
        return ConstraintDispatchTable::Register(
            &Dispatch<TData, Prepare>,
            &Dispatch<TData, Iterate>);
    }

    template<typename TData, void (*Prepare)(TData&, Real), void (*Iterate)(TData&, Real)>
    static std::uint32_t RegisterSmallConstraint()
    {
        return ConstraintDispatchTable::Register(
            &Dispatch<TData, Prepare>,
            &Dispatch<TData, Iterate>);
    }

    void VerifyCreated() const
    {
        if (body1_ == nullptr || body2_ == nullptr)
        {
            throw std::logic_error("The constraint has not been created by the world.");
        }
    }

private:
    void Attach(
        Unmanaged::JHandle<ConstraintData> handle,
        RigidBody& body1,
        RigidBody& body2,
        std::uint64_t constraintId)
    {
        handle_ = handle;
        smallHandle_ = Unmanaged::JHandle<SmallConstraintData>::Zero();
        body1_ = &body1;
        body2_ = &body2;
        constraintId_ = constraintId;
        handle_.Data().Body1 = body1.Handle();
        handle_.Data().Body2 = body2.Handle();
        handle_.Data().ConstraintId = constraintId;
        Create();
        IsEnabled(true);
    }

    void Attach(
        Unmanaged::JHandle<SmallConstraintData> handle,
        RigidBody& body1,
        RigidBody& body2,
        std::uint64_t constraintId)
    {
        handle_ = Unmanaged::JHandle<ConstraintData>::Zero();
        smallHandle_ = handle;
        body1_ = &body1;
        body2_ = &body2;
        constraintId_ = constraintId;
        smallHandle_.Data().Body1 = body1.Handle();
        smallHandle_.Data().Body2 = body2.Handle();
        smallHandle_.Data().ConstraintId = constraintId;
        Create();
        IsEnabled(true);
    }

    void Detach()
    {
        handle_ = Unmanaged::JHandle<ConstraintData>::Zero();
        smallHandle_ = Unmanaged::JHandle<SmallConstraintData>::Zero();
        body1_ = nullptr;
        body2_ = nullptr;
        constraintId_ = 0;
        enabled_ = true;
    }

    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    Unmanaged::JHandle<ConstraintData> handle_;
    Unmanaged::JHandle<SmallConstraintData> smallHandle_;
    std::uint64_t constraintId_ = 0;
    bool enabled_ = true;
    std::uint32_t dispatchId_ = 0;

    template<typename TData, void (*Function)(TData&, Real)>
    static void Dispatch(void* constraint, Real inverseDt)
    {
        Function(*reinterpret_cast<TData*>(constraint), inverseDt);
    }

    friend class ::Jitter2::World;
};

template<typename TData>
class TypedConstraint : public Constraint
{
public:
    static_assert(std::is_trivially_copyable_v<TData>, "Typed constraint data must be trivially copyable.");
    static_assert(sizeof(TData) <= sizeof(ConstraintData), "The size of the constraint data is too large.");

    [[nodiscard]] bool IsSmallConstraint() const override
    {
        return sizeof(TData) <= sizeof(SmallConstraintData);
    }

protected:
    [[nodiscard]] TData& Data()
    {
        VerifyCreated();
        if (!SmallHandle().IsZero())
        {
            return reinterpret_cast<TData&>(SmallHandle().Data());
        }

        return reinterpret_cast<TData&>(Handle().Data());
    }

    [[nodiscard]] const TData& Data() const
    {
        VerifyCreated();
        if (!SmallHandle().IsZero())
        {
            return reinterpret_cast<const TData&>(SmallHandle().Data());
        }

        return reinterpret_cast<const TData&>(Handle().Data());
    }
};

} // namespace Jitter2::Dynamics::Constraints
