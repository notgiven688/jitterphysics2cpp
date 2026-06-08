#pragma once

#include <algorithm>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct LinearMotorData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAxis1 = LinearMath::JVector::UnitX();
    LinearMath::JVector LocalAxis2 = LinearMath::JVector::UnitX();

    Real Velocity = static_cast<Real>(0);
    Real MaxForce = static_cast<Real>(0);
    Real MaxLambda = static_cast<Real>(0);

    Real EffectiveMass = static_cast<Real>(0);

    Real AccumulatedImpulse = static_cast<Real>(0);
};

// A motor constraint that drives relative translational velocity along an axis fixed
// in the reference frame of each body.
class LinearMotor : public TypedConstraint<LinearMotorData>
{
public:

    // Initializes the motor with axes for each body.
    // axis1: Motor axis on the first body in world space.
    // axis2: Motor axis on the second body in world space.
    // Stores the axes in local frames. Both axes are normalized internally.
    // Default values: TargetVelocity = 0, MaximumForce = 0.
    // Thrown when axis1 or axis2 is zero or contains a non-finite value.
    void Initialize(LinearMath::JVector axis1, LinearMath::JVector axis2)
    {
        VerifyCreated();
        Detail::CheckNonZero(axis1, "axis1");
        Detail::CheckNonZero(axis2, "axis2");

        LinearMotorData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        axis1.Normalize();
        axis2.Normalize();
        data.LocalAxis1 = LinearMath::JQuaternion::ConjugatedTransform(axis1, body1.Orientation);
        data.LocalAxis2 = LinearMath::JQuaternion::ConjugatedTransform(axis2, body2.Orientation);

        data.MaxForce = static_cast<Real>(0);
        data.Velocity = static_cast<Real>(0);
    }

    void Initialize(const LinearMath::JVector& axis)
    {
        Initialize(axis, axis);
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] LinearMath::JVector LocalAxis1() const { return Data().LocalAxis1; }
    void LocalAxis1(const LinearMath::JVector& value)
    {
        ArgumentCheck::UnitVector(value, "value");
        Data().LocalAxis1 = value;
    }

    [[nodiscard]] LinearMath::JVector LocalAxis2() const { return Data().LocalAxis2; }
    void LocalAxis2(const LinearMath::JVector& value)
    {
        ArgumentCheck::UnitVector(value, "value");
        Data().LocalAxis2 = value;
    }

    [[nodiscard]] Real TargetVelocity() const { return Data().Velocity; }
    void TargetVelocity(Real value)
    {
        DebugCheck::IsFinite(value, "value");
        Data().Velocity = value;
    }

    [[nodiscard]] Real MaximumForce() const { return Data().MaxForce; }
    void MaximumForce(Real value)
    {
        ArgumentCheck::NonNegative(value, "value");
        Data().MaxForce = value;
    }

    [[nodiscard]] Real Impulse() const { return Data().AccumulatedImpulse; }

    void PrepareForIteration(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        PrepareForIterationLinearMotor(Data(), inverseDt);
    }

    static void PrepareForIterationLinearMotor(LinearMotorData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector j1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector j2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        data.EffectiveMass = body1.InverseMass + body2.InverseMass;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;
        data.MaxLambda = (static_cast<Real>(1) / inverseDt) * data.MaxForce;

        body1.Velocity -= j1 * data.AccumulatedImpulse * body1.InverseMass;
        body2.Velocity += j2 * data.AccumulatedImpulse * body2.InverseMass;
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateLinearMotor(Data(), inverseDt);
    }

    static void IterateLinearMotor(LinearMotorData& data, Real)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector j1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector j2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        const Real jv =
            -LinearMath::JVector::Dot(j1, body1.Velocity)
            + LinearMath::JVector::Dot(j2, body2.Velocity);
        Real lambda = -(jv - data.Velocity) * data.EffectiveMass;

    // Gets the accumulated impulse applied by this motor during the last step.

        const Real oldAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        data.AccumulatedImpulse = std::clamp(data.AccumulatedImpulse, -data.MaxLambda, data.MaxLambda);
        lambda = data.AccumulatedImpulse - oldAccumulated;

        body1.Velocity -= j1 * lambda * body1.InverseMass;
        body2.Velocity += j2 * lambda * body2.InverseMass;
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const LinearMotorData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();
        const LinearMath::JVector axis1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector axis2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        constexpr Real axisLength = static_cast<Real>(0.5);
        drawer.DrawSegment(body1.Position, body1.Position + axis1 * axisLength);
        drawer.DrawSegment(body2.Position, body2.Position + axis2 * axisLength);
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
            RegisterFullConstraint<LinearMotorData, &PrepareForIterationLinearMotor, &IterateLinearMotor>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
