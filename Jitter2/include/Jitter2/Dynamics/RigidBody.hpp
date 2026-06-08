#pragma once

#include <algorithm>
#include <any>
#include <array>
#include <cstddef>
#include <cmath>
#include <functional>
#include <limits>
#include <span>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Jitter2/Collision/CollisionIsland.hpp>
#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Collision/Shapes/ShapeHelper.hpp>
#include <Jitter2/DebugCheck.hpp>
#include <Jitter2/IDebugDrawer.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JTriangle.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>
#include <Jitter2/Precision.hpp>
#include <Jitter2/Unmanaged/PartitionedBuffer.hpp>

namespace Jitter2
{

class World;
class Arbiter;

namespace Dynamics::Constraints
{
class Constraint;
}

enum class MotionType
{

// Fully simulated; responds to forces, impulses, contacts and constraints.

    Dynamic = 0,

// User-controlled body that is not affected by forces or collisions, but can affect dynamic bodies.
// Treated as having infinite mass in the solver. May have a non-zero velocity set by user code.
// Takes part in collision island building.

    Kinematic = 1,

// Immovable body (zero velocity) treated as having infinite mass by the solver.
// The position and orientation may be changed directly by user code, which will update the
// broadphase and may affect contacts on the next step.

    Static = 2
};

enum class MassInertiaUpdateMode
{

// Recompute mass and inertia from the currently attached shapes.

    Update = 0,

// Keep the current mass and inertia unchanged.

    Preserve = 1
};

namespace Detail
{
template<bool IsDouble>
struct RigidBodyDataPadding
{
};

template<>
struct RigidBodyDataPadding<true>
{
    std::array<std::byte, 8> Reserved {};
};
} // namespace Detail

struct alignas(64) RigidBodyData
{
    int _index = 0;
    int _lockFlag = 0;
    LinearMath::JVector Position = LinearMath::JVector::Zero();
    LinearMath::JVector Velocity = LinearMath::JVector::Zero();
    LinearMath::JVector AngularVelocity = LinearMath::JVector::Zero();
    LinearMath::JVector DeltaVelocity = LinearMath::JVector::Zero();
    LinearMath::JVector DeltaAngularVelocity = LinearMath::JVector::Zero();
    LinearMath::JQuaternion Orientation = LinearMath::JQuaternion::Identity();
    LinearMath::JMatrix InverseInertiaWorld = LinearMath::JMatrix::Identity();
    Real InverseMass = static_cast<Real>(1);
    int Flags = 0;
    [[no_unique_address]] Detail::RigidBodyDataPadding<IsDoublePrecision> Padding;

    [[nodiscard]] bool IsActive() const
    {
        return (Flags & 4) != 0;
    }

    void IsActive(bool value)
    {
        if (value)
        {
            Flags |= 4;
        }
        else
        {
            Flags &= ~4;
        }
    }

    [[nodiscard]] bool EnableGyroscopicForces() const
    {
        return (Flags & 8) != 0;
    }

    void EnableGyroscopicForces(bool value)
    {
        if (value)
        {
            Flags |= 8;
        }
        else
        {
            Flags &= ~8;
        }
    }

    [[nodiscard]] MotionType MotionTypeValue() const
    {
        return static_cast<MotionType>(Flags & 0b11);
    }

    void MotionTypeValue(MotionType value)
    {
        Flags = (Flags & ~0b11) | static_cast<int>(value);
    }
};

static_assert(sizeof(RigidBodyData) == RigidBodyDataSize);
static_assert(alignof(RigidBodyData) == 64);
static_assert(sizeof(RigidBodyData) % 64 == 0);

class RigidBody : public IDebugDrawable
{
public:
    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;
    RigidBody(RigidBody&&) = delete;
    RigidBody& operator=(RigidBody&&) = delete;

    [[nodiscard]] RigidBodyData& Data() { return handle_.Data(); }
    [[nodiscard]] const RigidBodyData& Data() const { return handle_.Data(); }
    [[nodiscard]] Unmanaged::JHandle<RigidBodyData> Handle() const { return handle_; }

    [[nodiscard]] std::uint64_t RigidBodyId() const { return rigidBodyId_; }
    [[nodiscard]] World& GetWorld() const { return *world_; }

    [[nodiscard]] LinearMath::JVector Position() const { return Data().Position; }
    void Position(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");
        Data().Position = value;
        Move();
    }

    [[nodiscard]] LinearMath::JQuaternion Orientation() const { return Data().Orientation; }
    void Orientation(const LinearMath::JQuaternion& value)
    {
        DebugCheck::IsUnitQuaternion(value, "value");
        Data().Orientation = value;
        Move();
    }

    [[nodiscard]] LinearMath::JVector Velocity() const { return Data().Velocity; }
    void Velocity(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");

        if (MotionTypeValue() == MotionType::Static)
        {
            throw std::logic_error("Can not set velocity for static objects, objects must be kinematic or dynamic. See MotionType.");
        }

        Data().Velocity = value;
        if (!LinearMath::MathHelper::CloseToZero(value))
        {
            RequestActivation(false);
        }
    }

    [[nodiscard]] LinearMath::JVector AngularVelocity() const { return Data().AngularVelocity; }
    void AngularVelocity(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");

        if (MotionTypeValue() == MotionType::Static)
        {
            throw std::logic_error("Can not set angular velocity for static objects, objects must be kinematic or dynamic. See MotionType.");
        }

        Data().AngularVelocity = value;
        if (!LinearMath::MathHelper::CloseToZero(value))
        {
            RequestActivation(false);
        }
    }

    [[nodiscard]] MotionType MotionTypeValue() const { return Data().MotionTypeValue(); }
    void MotionTypeValue(MotionType value);

    [[nodiscard]] bool IsActive() const { return Data().IsActive(); }

    // Instructs the engine to activate or deactivate the body at the beginning of
    // the next time step. The current state does not change immediately.
    // Activation and deactivation are applied through the body's simulation island.
    // Static bodies cannot be activated by this method, although moving a static body can wake connected
    // non-static bodies.
    // active: If true, the body will be activated; if false, deactivated.
    void SetActivationState(bool active)
    {
        if (active)
        {
            RequestActivation(false);
        }
        else
        {
            RequestDeactivation();
        }
    }

    [[nodiscard]] Collision::Island& Island() const { return *InternalIsland; }

    using ArbiterFunction = std::function<void(Arbiter&)>;

    ArbiterFunction BeginCollide;
    ArbiterFunction EndCollide;

    [[nodiscard]] Real DeactivationTime() const { return deactivationTimeThreshold_; }
    void DeactivationTime(Real value)
    {
        deactivationTimeThreshold_ = ArgumentCheck::NonNegative(value, "value");
    }

    [[nodiscard]] std::pair<Real, Real> DeactivationThreshold() const
    {
        return {std::sqrt(inactiveThresholdAngularSq_), std::sqrt(inactiveThresholdLinearSq_)};
    }

    void DeactivationThreshold(Real angular, Real linear)
    {
        ArgumentCheck::NonNegative(linear, "linear");
        ArgumentCheck::NonNegative(angular, "angular");

// Gets or sets the deactivation threshold. If the magnitudes of both the angular and linear velocity
// remain below the specified values for the duration of DeactivationTime, its island can be deactivated.
// The threshold values are given in rad/s and length units/s, respectively.
// Values must be non-negative.
// Default values: angular = 0.1, linear = 0.1.
// Thrown if either the linear or angular threshold is negative.

        inactiveThresholdAngularSq_ = angular * angular;

// Gets or sets the damping factors for linear and angular motion. A damping factor of 0 means the body is not
// damped, while 1 brings the body to a halt immediately. Damping is applied when calling
// World.Step(Real, bool). Jitter multiplies the respective velocity each step by 1 minus the damping
// factor. Note that the values are not scaled by time; a smaller time-step in World.Step(Real, bool)
// results in increased damping.
// The damping factors must be within the range [0, 1].
// Default values: linear = 0.002, angular = 0.005.
// Thrown if either the linear or angular damping value is less than 0 or greater than 1.

        inactiveThresholdLinearSq_ = linear * linear;
    }

    [[nodiscard]] Real Mass() const
    {
        return static_cast<Real>(1) / inverseMass_;
    }

    [[nodiscard]] LinearMath::JMatrix InverseInertia() const
    {
        return inverseInertia_;
    }

    [[nodiscard]] bool IsStatic() const
    {
        return MotionTypeValue() == MotionType::Static;
    }

    void IsStatic(bool value)
    {
        MotionTypeValue(value ? MotionType::Static : MotionType::Dynamic);
    }

    [[nodiscard]] const std::vector<Collision::Shapes::RigidBodyShape*>& Shapes() const
    {
        return shapes_;
    }

    [[nodiscard]] const std::vector<RigidBody*>& Connections() const
    {
        return InternalConnections;
    }

    [[nodiscard]] const std::unordered_set<Arbiter*>& Contacts() const
    {
        return InternalContacts;
    }

    [[nodiscard]] const std::unordered_set<Dynamics::Constraints::Constraint*>& Constraints() const
    {
        return InternalConstraints;
    }

    // Clears the cached contact manifold state for all current contacts of this body.
    // This discards cached contact points and accumulated impulses, forcing contact
    // manifolds to be rebuilt on the next simulation step. This is useful after
    // discontinuous user-driven transform changes such as teleports.
    // Do not call this concurrently with World.Step(Real, bool).
    void ClearContactCache();

private:
    static bool ShouldUpdateMassInertia(MassInertiaUpdateMode mode)
    {
        switch (mode)
        {
        case MassInertiaUpdateMode::Update:
            return true;
        case MassInertiaUpdateMode::Preserve:
            return false;
        }

        throw std::out_of_range("MassInertiaUpdateMode is out of range.");
    }

    void AttachToShape(Collision::Shapes::RigidBodyShape& shape)
    {
        if (shape.IsRegistered())
        {
            throw std::invalid_argument("Shape can not be added. Shape already registered elsewhere.");
        }

        shape.AttachTo(this);
        shape.Position = Data().Position;
        shape.Orientation = Data().Orientation;
        shape.UpdateWorldBoundingBox();
        RegisterShape(shape);
    }

public:
    void AddShape(Collision::Shapes::RigidBodyShape& shape,
        MassInertiaUpdateMode mode = MassInertiaUpdateMode::Update)
    {
        AttachToShape(shape);
        shapes_.push_back(&shape);

        if (ShouldUpdateMassInertia(mode))
        {

// Computes the mass and inertia of this body from all attached shapes, assuming unit density.
// The mass contributions of all shapes are summed. If no shapes are attached, the body
// is assigned a mass of 1 and an identity inertia tensor.
// Thrown if the computed inertia matrix is not invertible. This may occur if a shape has invalid mass properties.

            SetMassInertia();
        }
    }

    void AddShapes(
        std::span<Collision::Shapes::RigidBodyShape* const> shapes,
        MassInertiaUpdateMode mode = MassInertiaUpdateMode::Update)
    {
        for (Collision::Shapes::RigidBodyShape* shape : shapes)
        {
            if (shape == nullptr)
            {
                throw std::invalid_argument("Shape collection contains a null shape.");
            }

            AttachToShape(*shape);
            shapes_.push_back(shape);
        }

        if (ShouldUpdateMassInertia(mode))
        {
            SetMassInertia();
        }
    }

    // Adds a shape to the body.
    // shape: The shape to be added.
    // massInertiaMode: Controls whether the body's mass and inertia are recomputed after the shape is added.
    // Thrown if the shape is already registered elsewhere.
    // Thrown if shape is null.
    // Thrown when massInertiaMode is not a valid MassInertiaUpdateMode value.
    void AddShape(Collision::Shapes::RigidBodyShape& shape, bool setMassInertia)
    {
        AddShape(shape, setMassInertia ? MassInertiaUpdateMode::Update : MassInertiaUpdateMode::Preserve);
    }

    void AddShape(std::span<Collision::Shapes::RigidBodyShape* const> shapes, bool setMassInertia)
    {

// Adds several shapes to the rigid body at once. Mass properties are
// recalculated only once, if requested.
// Shapes are added sequentially. If an exception is thrown, shapes processed before the
// failure remain attached to the body.
// shapes: The shapes to add.
// massInertiaMode: Controls whether the body's mass and inertia are recomputed after the shapes are added.
// Thrown if shapes or any contained shape is null.
// Thrown if any shape is already attached to a body.
// Thrown when massInertiaMode is not a valid MassInertiaUpdateMode value.

        AddShapes(shapes, setMassInertia ? MassInertiaUpdateMode::Update : MassInertiaUpdateMode::Preserve);
    }

    void RemoveShape(Collision::Shapes::RigidBodyShape& shape,
        MassInertiaUpdateMode mode = MassInertiaUpdateMode::Update)
    {
        auto iterator = std::find(shapes_.begin(), shapes_.end(), &shape);
        if (iterator == shapes_.end())
        {
            throw std::invalid_argument("Shape is not part of this body.");
        }

        shapes_.erase(iterator);
        UnregisterShape(shape);
        shape.AttachTo(nullptr);
        shape.Position = LinearMath::JVector::Zero();
        shape.Orientation = LinearMath::JQuaternion::Identity();
        shape.UpdateWorldBoundingBox();

        if (ShouldUpdateMassInertia(mode))
        {
            SetMassInertia();
        }
    }

    void RemoveShapes(
        std::span<Collision::Shapes::RigidBodyShape* const> shapes,
        MassInertiaUpdateMode mode = MassInertiaUpdateMode::Update)
    {
        std::unordered_set<std::uint64_t> shapeIds;

        for (Collision::Shapes::RigidBodyShape* shape : shapes)
        {
            if (shape == nullptr)
            {
                throw std::invalid_argument("Shape collection contains a null shape.");
            }

            if (shape->GetRigidBody() != this)
            {
                throw std::invalid_argument("Shape is not attached to this body.");
            }

            shapeIds.insert(shape->ShapeId());
        }

        for (auto iterator = shapes_.end(); iterator != shapes_.begin();)
        {
            --iterator;
            Collision::Shapes::RigidBodyShape* shape = *iterator;
            if (!shapeIds.contains(shape->ShapeId()))
            {
                continue;
            }

            UnregisterShape(*shape);
            shape->AttachTo(nullptr);
            shape->Position = LinearMath::JVector::Zero();
            shape->Orientation = LinearMath::JQuaternion::Identity();
            shape->UpdateWorldBoundingBox();
            iterator = shapes_.erase(iterator);
        }

        if (ShouldUpdateMassInertia(mode))
        {
            SetMassInertia();
        }
    }

    // Removes a specified shape from the rigid body.
    // shape: The shape to remove from the rigid body.
    // massInertiaMode: Controls whether the body's mass and inertia are recomputed after the shape is removed.
    // Thrown if the specified shape is not part of this rigid body.
    // Thrown if shape is null.
    // Thrown when massInertiaMode is not a valid MassInertiaUpdateMode value.
    void RemoveShape(Collision::Shapes::RigidBodyShape& shape, bool setMassInertia)
    {
        RemoveShape(shape, setMassInertia ? MassInertiaUpdateMode::Update : MassInertiaUpdateMode::Preserve);
    }

    void RemoveShape(std::span<Collision::Shapes::RigidBodyShape* const> shapes, bool setMassInertia)
    {

// Removes several shapes from the body.
// shapes: The shapes to remove from the rigid body.
// massInertiaMode: Controls whether the body's mass and inertia are recomputed after the shapes are removed.
// Thrown if shapes or any contained shape is null.
// Thrown if at least one shape is not part of this rigid body.
// Thrown when massInertiaMode is not a valid MassInertiaUpdateMode value.

        RemoveShapes(shapes, setMassInertia ? MassInertiaUpdateMode::Update : MassInertiaUpdateMode::Preserve);
    }

    void ClearShapes(MassInertiaUpdateMode mode = MassInertiaUpdateMode::Update)
    {

        // Removes several shapes from the body.
        // shapes: The shapes to remove from the rigid body.
        // Thrown if shapes or any contained shape is null.
        // Thrown if at least one shape is not part of this rigid body.
        RemoveShapes(std::span<Collision::Shapes::RigidBodyShape* const>(shapes_.data(), shapes_.size()), mode);
    }

    // Removes all shapes associated with the rigid body.
    // massInertiaMode: Controls whether the body's mass and inertia are recomputed after the shapes are removed.
    void ClearShapes(bool setMassInertia)
    {
        ClearShapes(setMassInertia ? MassInertiaUpdateMode::Update : MassInertiaUpdateMode::Preserve);
    }

    void SetMassInertia()
    {
        if (shapes_.empty())
        {
            inverseMass_ = static_cast<Real>(1);
            inverseInertia_ = LinearMath::JMatrix::Identity();
            UpdateWorldInertia();
            return;
        }

        Real mass = 0;
        LinearMath::JMatrix inertia = LinearMath::JMatrix::Zero();
        for (Collision::Shapes::RigidBodyShape* shape : shapes_)
        {
            LinearMath::JMatrix shapeInertia;
            LinearMath::JVector centerOfMass;
            Real shapeMass = 0;
            shape->CalculateMassInertia(shapeInertia, centerOfMass, shapeMass);
            (void)centerOfMass;
            inertia = inertia + shapeInertia;
            mass += shapeMass;
        }

        if (!LinearMath::JMatrix::Inverse(inertia, inverseInertia_))
        {
            throw std::logic_error(
                "Inertia matrix is not invertible. This might happen if a shape has "
                "invalid mass properties. If you encounter this while calling "
                "AddShape or AddShapes, call the method with massInertiaMode set to Preserve.");
        }

        inverseMass_ = static_cast<Real>(1) / mass;
        UpdateWorldInertia();
    }

    // Computes the inertia from all attached shapes, then uniformly scales it to match the specified mass.
    // This is equivalent to calling SetMassInertia() and then scaling the resulting
    // inertia tensor so the body has the desired total mass. Use this when you want shape-derived
    // inertia proportions but a specific total mass (e.g., for gameplay tuning).
    // mass: The desired total mass of the body. Must be positive.
    // Thrown if the specified mass is zero or negative.
    void SetMassInertia(Real mass)
    {
        ArgumentCheck::Finite(mass, "mass");

        if (mass <= static_cast<Real>(0))
        {
            throw std::invalid_argument("Mass can not be zero or negative.");
        }

        SetMassInertia();
        inverseInertia_ = inverseInertia_ * (static_cast<Real>(1) / (inverseMass_ * mass));
        inverseMass_ = static_cast<Real>(1) / mass;
        UpdateWorldInertia();
    }

    void SetMassInertia(const LinearMath::JMatrix& inertia, Real mass, bool setAsInverse = false)
    {
        ArgumentCheck::Finite(inertia, "inertia");
        ArgumentCheck::Finite(mass, "mass");

        if (setAsInverse)
        {
            if (mass < static_cast<Real>(0))
            {
                throw std::invalid_argument("Inverse mass must be finite and not negative.");
            }

            inverseInertia_ = inertia;
            inverseMass_ = mass;
        }
        else
        {
            if (mass <= static_cast<Real>(0))
            {
                throw std::invalid_argument("Mass can not be zero or negative.");
            }

            if (!LinearMath::JMatrix::Inverse(inertia, inverseInertia_))
            {
                throw std::invalid_argument("Inertia matrix is not invertible.");
            }

            inverseMass_ = static_cast<Real>(1) / mass;
        }

        UpdateWorldInertia();
    }

    [[nodiscard]] LinearMath::JVector Force() const { return force_; }
    void Force(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");
        force_ = value;
    }
    [[nodiscard]] LinearMath::JVector Torque() const { return torque_; }
    void Torque(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");
        torque_ = value;
    }

    void AddForce(const LinearMath::JVector& force, bool wakeup = true)
    {
        DebugCheck::IsFinite(force, "force");

        if (MotionTypeValue() != MotionType::Dynamic || LinearMath::MathHelper::CloseToZero(force))
        {
            return;
        }

        if (wakeup)
        {
            RequestActivation(false);
        }
        else if (!IsActive())
        {
            return;
        }

        Force(force_ + force);
    }

    void AddForce(const LinearMath::JVector& force, const LinearMath::JVector& position, bool wakeup = true)
    {
        DebugCheck::IsFinite(force, "force");
        DebugCheck::IsFinite(position, "position");

        if (MotionTypeValue() != MotionType::Dynamic || LinearMath::MathHelper::CloseToZero(force))
        {
            return;
        }

        if (wakeup)
        {
            RequestActivation(false);
        }
        else if (!IsActive())
        {
            return;
        }

        Force(force_ + force);
        Torque(torque_ + LinearMath::JVector::Cross(position - Data().Position, force));
    }

    void ApplyImpulse(const LinearMath::JVector& impulse, bool wakeup = true)
    {
        DebugCheck::IsFinite(impulse, "impulse");

        if (MotionTypeValue() != MotionType::Dynamic || LinearMath::MathHelper::CloseToZero(impulse))
        {
            return;
        }

        if (!wakeup && !IsActive())
        {
            return;
        }

        RequestActivation(false);
        Data().Velocity += impulse * inverseMass_;
    }

    void ApplyImpulse(const LinearMath::JVector& impulse, const LinearMath::JVector& position, bool wakeup = true)
    {
        DebugCheck::IsFinite(impulse, "impulse");
        DebugCheck::IsFinite(position, "position");

        if (MotionTypeValue() != MotionType::Dynamic || LinearMath::MathHelper::CloseToZero(impulse))
        {
            return;
        }

        if (!wakeup && !IsActive())
        {
            return;
        }

        RequestActivation(false);
        const LinearMath::JVector angularImpulse = LinearMath::JVector::Cross(position - Data().Position, impulse);
        Data().Velocity += impulse * inverseMass_;
        Data().AngularVelocity += Data().InverseInertiaWorld * angularImpulse;
    }

    // Predicts the position of the body after a given time step using linear extrapolation.
    // This does not simulate forces or collisions — it assumes constant velocity.
    // dt: The time step to extrapolate forward.
    // Returns: The predicted position after dt.
    [[nodiscard]] LinearMath::JVector PredictPosition(Real dt) const
    {
        return Data().Position + Data().Velocity * dt;
    }

    // Predicts the orientation of the body after a given time step using angular velocity.
    // This does not simulate forces or collisions — it assumes constant angular velocity.
    // dt: The time step to extrapolate forward.
    // Returns: The predicted orientation after dt.
    [[nodiscard]] LinearMath::JQuaternion PredictOrientation(Real dt) const
    {
        return LinearMath::JQuaternion::Normalize(
            LinearMath::MathHelper::RotationQuaternion(Data().AngularVelocity, dt) * Data().Orientation);
    }

    // Predicts the pose (position and orientation) of the body after a given time step using simple extrapolation.
    // This method is intended for rendering purposes and does not modify the simulation state.
    // dt: The time step to extrapolate forward.
    // position: The predicted position after dt.
    // orientation: The predicted orientation after dt.
    void PredictPose(Real dt, LinearMath::JVector& position, LinearMath::JQuaternion& orientation) const
    {
        position = PredictPosition(dt);
        orientation = PredictOrientation(dt);
    }

    [[nodiscard]] std::pair<Real, Real> Damping() const
    {
        return {
            static_cast<Real>(1) - linearDampingMultiplier_,
            static_cast<Real>(1) - angularDampingMultiplier_,
        };
    }

    void Damping(Real linear, Real angular)
    {
        ArgumentCheck::InRange(linear, static_cast<Real>(0), static_cast<Real>(1), "linear");
        ArgumentCheck::InRange(angular, static_cast<Real>(0), static_cast<Real>(1), "angular");

        linearDampingMultiplier_ = static_cast<Real>(1) - linear;
        angularDampingMultiplier_ = static_cast<Real>(1) - angular;
    }

    [[nodiscard]] bool AffectedByGravity() const { return affectedByGravity_; }
    void AffectedByGravity(bool value) { affectedByGravity_ = value; }
    [[nodiscard]] bool EnableGyroscopicForces() const { return Data().EnableGyroscopicForces(); }
    void EnableGyroscopicForces(bool value) { Data().EnableGyroscopicForces(value); }
    [[nodiscard]] bool EnableSpeculativeContacts() const { return enableSpeculativeContacts_; }
    void EnableSpeculativeContacts(bool value) { enableSpeculativeContacts_ = value; }

    // Generates a rough triangle approximation of the shapes of the body.
    // Since the generation is slow this should only be used for debugging
    // purposes.
    // This method tessellates all attached shapes and is not suitable for real-time use.
    // It uses a shared static list internally and is not thread-safe.
    // drawer: The debug drawer to receive the generated triangles.
    void DebugDraw(IDebugDrawer& drawer) override
    {
        std::vector<LinearMath::JTriangle> debugTriangles;

        for (Collision::Shapes::RigidBodyShape* shape : shapes_)
        {
            Collision::Shapes::ShapeHelper::Tessellate(*shape, debugTriangles);

            for (const LinearMath::JTriangle& triangle : debugTriangles)
            {
                drawer.DrawTriangle(
                    LinearMath::JQuaternion::Transform(triangle.V0, Data().Orientation) + Data().Position,
                    LinearMath::JQuaternion::Transform(triangle.V1, Data().Orientation) + Data().Position,
                    LinearMath::JQuaternion::Transform(triangle.V2, Data().Orientation) + Data().Position);
            }

            debugTriangles.clear();
        }
    }

    [[nodiscard]] Real Friction() const { return friction_; }
    void Friction(Real value)
    {
        friction_ = ArgumentCheck::NonNegative(value, "value");
    }

    [[nodiscard]] Real Restitution() const { return restitution_; }
    void Restitution(Real value)
    {
        restitution_ = ArgumentCheck::InRange(value, static_cast<Real>(0), static_cast<Real>(1), "value");
    }

    std::any Tag;
    int SetIndex = -1;
    Collision::Island* InternalIsland = nullptr;
    std::vector<RigidBody*> InternalConnections;
    std::unordered_set<Arbiter*> InternalContacts;
    std::unordered_set<Dynamics::Constraints::Constraint*> InternalConstraints;
    int InternalIslandMarker = 0;
    Real InternalSleepTime = static_cast<Real>(0);

private:
    RigidBody(Unmanaged::JHandle<RigidBodyData> handle, World& world, std::uint64_t rigidBodyId)
        : handle_(handle),
          world_(&world),
          rigidBodyId_(rigidBodyId)
    {
        Data().Orientation = LinearMath::JQuaternion::Identity();
        Data().InverseMass = inverseMass_;
        Data().InverseInertiaWorld = inverseInertia_;
        Data().MotionTypeValue(MotionType::Dynamic);
    }

    void Invalidate()
    {
        for (Collision::Shapes::RigidBodyShape* shape : shapes_)
        {
            UnregisterShape(*shape);
            shape->AttachTo(nullptr);
            shape->Position = LinearMath::JVector::Zero();
            shape->Orientation = LinearMath::JQuaternion::Identity();
            shape->UpdateWorldBoundingBox();
        }
        shapes_.clear();

        handle_ = Unmanaged::JHandle<RigidBodyData>::Zero();
        world_ = nullptr;
    }

    void UpdateAttachedShapeBounds()
    {
        for (Collision::Shapes::RigidBodyShape* shape : shapes_)
        {
            shape->Position = Data().Position;
            shape->Orientation = Data().Orientation;
            shape->UpdateWorldBoundingBox();
            UpdateShapeProxy(*shape);
        }
    }

    void SynchronizeAttachedShapeTransforms()
    {
        for (Collision::Shapes::RigidBodyShape* shape : shapes_)
        {
            shape->Position = Data().Position;
            shape->Orientation = Data().Orientation;
        }
    }

    void Move()
    {
        UpdateWorldInertia();
        ClearContactCache();
        UpdateAttachedShapeBounds();
        RequestActivation(true);
    }

    void UpdateWorldInertia()
    {
        if (Data().MotionTypeValue() == MotionType::Dynamic)
        {
            const LinearMath::JMatrix bodyOrientation =
                LinearMath::JMatrix::CreateFromQuaternion(Data().Orientation);
            Data().InverseInertiaWorld =
                LinearMath::JMatrix::MultiplyTransposed(bodyOrientation * inverseInertia_, bodyOrientation);
            Data().InverseMass = inverseMass_;
        }
        else
        {
            Data().InverseInertiaWorld = LinearMath::JMatrix::Zero();
            Data().InverseMass = static_cast<Real>(0);
        }
    }

    void ClearQueuedForces()
    {
        force_ = LinearMath::JVector::Zero();
        torque_ = LinearMath::JVector::Zero();
        Data().DeltaVelocity = LinearMath::JVector::Zero();
        Data().DeltaAngularVelocity = LinearMath::JVector::Zero();
    }

    static LinearMath::JVector SolveGyroscopic(
        const LinearMath::JMatrix& inertiaWorld,
        const LinearMath::JVector& omega,
        Real dt)
    {
        const LinearMath::JVector inertiaOmega = inertiaWorld * omega;
        const LinearMath::JVector f = dt * LinearMath::JVector::Cross(omega, inertiaOmega);

        const LinearMath::JMatrix jacobian =
            inertiaWorld + dt
                * (LinearMath::JMatrix::CreateCrossProduct(omega) * inertiaWorld
                    - LinearMath::JMatrix::CreateCrossProduct(inertiaOmega));

        LinearMath::JMatrix invJacobian;
        if (!LinearMath::JMatrix::Inverse(jacobian, invJacobian))
        {
            return omega;
        }

        return omega - invJacobian * f;
    }

    void Update(Real stepDt, Real substepDt, const LinearMath::JVector& gravity)
    {
        if (Data().AngularVelocity.LengthSquared() < inactiveThresholdAngularSq_
            && Data().Velocity.LengthSquared() < inactiveThresholdLinearSq_)
        {
            InternalSleepTime += stepDt;
        }
        else
        {
            InternalSleepTime = static_cast<Real>(0);
        }

        if (InternalIsland != nullptr && InternalSleepTime < deactivationTimeThreshold_)
        {
            InternalIsland->MarkedAsActive = true;
        }

        if (Data().MotionTypeValue() == MotionType::Dynamic)
        {
            Data().AngularVelocity *= angularDampingMultiplier_;
            Data().Velocity *= linearDampingMultiplier_;

            Data().DeltaVelocity = force_ * Data().InverseMass * substepDt;
            Data().DeltaAngularVelocity = Data().InverseInertiaWorld * torque_ * substepDt;
            if (affectedByGravity_)
            {
                Data().DeltaVelocity += gravity * substepDt;
            }

            force_ = LinearMath::JVector::Zero();
            torque_ = LinearMath::JVector::Zero();

            UpdateWorldInertia();
        }

        SynchronizeAttachedShapeTransforms();
    }

    void IntegrateForces()
    {
        if (!IsActive() || MotionTypeValue() != MotionType::Dynamic)
        {
            return;
        }

        Data().Velocity += Data().DeltaVelocity;
        Data().AngularVelocity += Data().DeltaAngularVelocity;
    }

    void IntegrateVelocity(Real substepDt)
    {
        if (!IsActive() || MotionTypeValue() == MotionType::Static)
        {
            return;
        }

        Data().Position += Data().Velocity * substepDt;
        Data().Orientation = LinearMath::JQuaternion::Normalize(
            LinearMath::MathHelper::RotationQuaternion(Data().AngularVelocity, substepDt) * Data().Orientation);

        if (!Data().EnableGyroscopicForces())
        {
            return;
        }

        LinearMath::JMatrix inertiaWorld;
        if (LinearMath::JMatrix::Inverse(Data().InverseInertiaWorld, inertiaWorld))
        {
            Data().AngularVelocity = SolveGyroscopic(inertiaWorld, Data().AngularVelocity, substepDt);
        }
    }

    void RegisterShape(Collision::Shapes::RigidBodyShape& shape);
    void UnregisterShape(Collision::Shapes::RigidBodyShape& shape);
    void UpdateShapeProxy(Collision::Shapes::RigidBodyShape& shape);
    void RequestActivation(bool wakeUpStatic);
    void RequestDeactivation();
    void RaiseBeginCollide(Arbiter& arbiter)
    {
        if (BeginCollide)
        {
            BeginCollide(arbiter);
        }
    }

    void RaiseEndCollide(Arbiter& arbiter)
    {
        if (EndCollide)
        {
            EndCollide(arbiter);
        }
    }

    Unmanaged::JHandle<RigidBodyData> handle_;
    World* world_ = nullptr;
    std::uint64_t rigidBodyId_ = 0;
    Real restitution_ = 0;
    Real friction_ = static_cast<Real>(0.2);
    Real linearDampingMultiplier_ = static_cast<Real>(0.998);
    Real angularDampingMultiplier_ = static_cast<Real>(0.995);
    LinearMath::JVector force_ = LinearMath::JVector::Zero();
    LinearMath::JVector torque_ = LinearMath::JVector::Zero();
    std::vector<Collision::Shapes::RigidBodyShape*> shapes_;
    LinearMath::JMatrix inverseInertia_ = LinearMath::JMatrix::Identity();
    Real inverseMass_ = static_cast<Real>(1);
    bool affectedByGravity_ = true;
    bool enableSpeculativeContacts_ = false;
    Real inactiveThresholdLinearSq_ = static_cast<Real>(0.1);
    Real inactiveThresholdAngularSq_ = static_cast<Real>(0.1);
    Real deactivationTimeThreshold_ = static_cast<Real>(1);

    friend class World;
};

} // namespace Jitter2
