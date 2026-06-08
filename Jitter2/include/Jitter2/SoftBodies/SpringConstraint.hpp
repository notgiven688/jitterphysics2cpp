#pragma once

#include <cstdint>

#include <Jitter2/ArgumentCheck.hpp>
#include <Jitter2/DebugCheck.hpp>
#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::SoftBodies
{

// Low-level data for the spring constraint, stored in unmanaged memory.
struct SpringData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAnchor1 = LinearMath::JVector::Zero();
    LinearMath::JVector LocalAnchor2 = LinearMath::JVector::Zero();

    Real BiasFactor = Dynamics::Constraints::Constraint::DefaultLinearBias;
    Real Softness = Dynamics::Constraints::Constraint::DefaultLinearSoftness;
    Real Distance = static_cast<Real>(0);

    Real EffectiveMass = static_cast<Real>(0);
    Real AccumulatedImpulse = static_cast<Real>(0);
    Real Bias = static_cast<Real>(0);

    LinearMath::JVector Jacobian = LinearMath::JVector::Zero();
};

// Constrains two bodies to maintain a target distance between anchor points,
// applying spring-like forces. Removes one translational degree of freedom along
// the line connecting the anchors.
// This constraint is designed for soft body vertices, which act as mass points.
// Angular velocities of the connected bodies are not taken into account.
// The spring behavior is controlled by Softness and Bias,
// which can be set directly or computed from physical parameters using
// SetSpringParameters.
class SpringConstraint : public Dynamics::Constraints::TypedConstraint<SpringData>
{
public:

    // Initializes the constraint from world-space anchor points.
    // anchor1: Anchor point on the first rigid body, in world space.
    // anchor2: Anchor point on the second rigid body, in world space.
    // Computes local anchor offsets from the current body positions.
    // Default values: Softness = Constraint::DefaultLinearSoftness,
    // Bias = Constraint::DefaultLinearBias.
    // The TargetDistance is set to the initial distance between the anchors.
    void Initialize(const LinearMath::JVector& anchor1, const LinearMath::JVector& anchor2)
    {
        VerifyCreated();
        ArgumentCheck::Finite(anchor1, "anchor1");
        ArgumentCheck::Finite(anchor2, "anchor2");

        SpringData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        data.LocalAnchor1 = anchor1 - body1.Position;
        data.LocalAnchor2 = anchor2 - body2.Position;
        data.Softness = DefaultLinearSoftness;
        data.BiasFactor = DefaultLinearBias;
        data.Distance = (anchor2 - anchor1).Length();
    }

    // Sets the spring parameters using physical properties. This method calculates and sets
    // the Softness and Bias properties. It assumes that the mass
    // of the involved bodies and the timestep size does not change.
    // frequency: The frequency in Hz.
    // damping: The damping ratio (0 = no damping, 1 = critical damping).
    // dt: The timestep of the simulation.
    void SetSpringParameters(Real frequency, Real damping, Real dt)
    {
        ArgumentCheck::Positive(frequency, "frequency");
        ArgumentCheck::NonNegative(damping, "damping");
        ArgumentCheck::Positive(dt, "dt");

        SpringData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        const Real effectiveMass = static_cast<Real>(1) / (body1.InverseMass + body2.InverseMass);
        const Real omega = static_cast<Real>(2) * pi * frequency;
        const Real d = static_cast<Real>(2) * effectiveMass * damping * omega;
        const Real k = effectiveMass * omega * omega;

        data.Softness = static_cast<Real>(1) / (d + dt * k);
        data.BiasFactor = dt * k * data.Softness;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    // Gets the accumulated impulse applied by the spring.
    [[nodiscard]] Real Impulse() const { return Data().AccumulatedImpulse; }

    // Gets or sets the anchor point on the first body in world space.
    [[nodiscard]] LinearMath::JVector Anchor1() const
    {
        VerifyCreated();
        const SpringData& data = Data();
        return data.Body1.Data().Position + data.LocalAnchor1;
    }

    // Gets or sets the anchor point on the first body in world space.
    void Anchor1(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        SpringData& data = Data();
        data.LocalAnchor1 = value - data.Body1.Data().Position;
    }

    // Gets or sets the anchor point on the second body in world space.
    [[nodiscard]] LinearMath::JVector Anchor2() const
    {
        VerifyCreated();
        const SpringData& data = Data();
        return data.Body2.Data().Position + data.LocalAnchor2;
    }

    // Gets or sets the anchor point on the second body in world space.
    void Anchor2(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        SpringData& data = Data();
        data.LocalAnchor2 = value - data.Body2.Data().Position;
    }

    // Gets or sets the target resting distance of the spring.
    // Value: Units: meters. Default is the initial distance between anchors at initialization.
    [[nodiscard]] Real TargetDistance() const { return Data().Distance; }

    // Gets or sets the target resting distance of the spring.
    // Value: Units: meters. Default is the initial distance between anchors at initialization.
    void TargetDistance(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().Distance = value;
    }

    // Gets the current distance between the anchor points.
    [[nodiscard]] Real Distance() const
    {
        VerifyCreated();
        const SpringData& data = Data();

        const LinearMath::JVector p1 = data.Body1.Data().Position + data.LocalAnchor1;
        const LinearMath::JVector p2 = data.Body2.Data().Position + data.LocalAnchor2;

        return (p2 - p1).Length();
    }

    // Gets or sets the softness (compliance) of the spring constraint.
    // Value: Default is 0.001. Higher values allow more positional error but produce a softer spring.
    // Scaled by inverse timestep during solving.
    [[nodiscard]] Real Softness() const { return Data().Softness; }

    // Gets or sets the softness (compliance) of the spring constraint.
    // Value: Default is 0.001. Higher values allow more positional error but produce a softer spring.
    // Scaled by inverse timestep during solving.
    void Softness(Real value)
    {
        Dynamics::Constraints::Detail::CheckNonNegative(value, "softness");
        Data().Softness = value;
    }

    // Gets or sets the bias factor (error correction strength) of the spring constraint.
    // Value: Default is 0.2. Higher values correct distance errors more aggressively.
    [[nodiscard]] Real Bias() const { return Data().BiasFactor; }

    // Gets or sets the bias factor (error correction strength) of the spring constraint.
    // Value: Default is 0.2. Higher values correct distance errors more aggressively.
    void Bias(Real value)
    {
        Dynamics::Constraints::Detail::CheckNonNegative(value, "bias");
        Data().BiasFactor = value;
    }

    // Prepares the spring constraint for iteration.
    // inverseDt: The inverse substep duration (1/dt).
    void PrepareForIteration(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        PrepareForIterationSpringConstraint(Data(), inverseDt);
    }

    // Prepares the spring constraint for iteration.
    // data: The constraint data reference.
    // inverseDt: The inverse substep duration (1/dt).
    static void PrepareForIterationSpringConstraint(SpringData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector p1 = body1.Position + data.LocalAnchor1;
        const LinearMath::JVector p2 = body2.Position + data.LocalAnchor2;
        const LinearMath::JVector dp = p2 - p1;

        const Real error = dp.Length() - data.Distance;

        LinearMath::JVector n = p2 - p1;
        if (n.LengthSquared() > static_cast<Real>(1e-12))
        {
            n.Normalize();
        }

        data.Jacobian = n;
        data.EffectiveMass = body1.InverseMass + body2.InverseMass;
        data.EffectiveMass += data.Softness * inverseDt;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;

        data.Bias = error * data.BiasFactor * inverseDt;

        body1.Velocity -= body1.InverseMass * data.AccumulatedImpulse * data.Jacobian;
        body2.Velocity += body2.InverseMass * data.AccumulatedImpulse * data.Jacobian;
    }

    // Performs one iteration of the spring constraint solver.
    // inverseDt: The inverse substep duration (1/dt).
    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateSpringConstraint(Data(), inverseDt);
    }

    // Performs one iteration of the spring constraint solver.
    // data: The constraint data reference.
    // inverseDt: The inverse substep duration (1/dt).
    static void IterateSpringConstraint(SpringData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const Real jv = LinearMath::JVector::Dot(body2.Velocity - body1.Velocity, data.Jacobian);

        const Real softnessScalar = data.AccumulatedImpulse * data.Softness * inverseDt;

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

        const Real oldAccumulatedImpulse = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        lambda = data.AccumulatedImpulse - oldAccumulatedImpulse;

        body1.Velocity -= body1.InverseMass * lambda * data.Jacobian;
        body2.Velocity += body2.InverseMass * lambda * data.Jacobian;
    }


    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const SpringData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector p1 = body1.Position + data.LocalAnchor1;
        const LinearMath::JVector p2 = body2.Position + data.LocalAnchor2;

        drawer.DrawSegment(body1.Position, p1);
        drawer.DrawSegment(body2.Position, p2);
        drawer.DrawSegment(p1, p2);
        drawer.DrawPoint(p1);
        drawer.DrawPoint(p2);
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
            RegisterFullConstraint<SpringData, &PrepareForIterationSpringConstraint, &IterateSpringConstraint>();
        return id;
    }
};

} // namespace Jitter2::SoftBodies
