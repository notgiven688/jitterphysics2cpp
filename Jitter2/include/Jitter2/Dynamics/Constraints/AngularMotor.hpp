#pragma once

#include <algorithm>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct AngularMotorData
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

// Represents a motor that drives relative angular velocity between two axes fixed
// in the reference frames of their respective bodies.
class AngularMotor : public TypedConstraint<AngularMotorData>
{
public:

    // Initializes the motor with separate axes for each body.
    // axis1: The motor axis on the first body in world space.
    // axis2: The motor axis on the second body in world space.
    // Stores the axes in local frames. Both axes are normalized internally.
    // Default values: TargetVelocity = 0, MaximumForce = 0.
    // Thrown when axis1 or axis2 is zero or contains a non-finite value.
    void Initialize(LinearMath::JVector axis1, LinearMath::JVector axis2)
    {
        VerifyCreated();
        Detail::CheckNonZero(axis1, "axis1");
        Detail::CheckNonZero(axis2, "axis2");

        AngularMotorData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        axis1.Normalize();
        axis2.Normalize();
        data.LocalAxis1 = LinearMath::JQuaternion::ConjugatedTransform(axis1, body1.Orientation);
        data.LocalAxis2 = LinearMath::JQuaternion::ConjugatedTransform(axis2, body2.Orientation);

        data.MaxForce = static_cast<Real>(0);
        data.Velocity = static_cast<Real>(0);
    }

    // Initializes the motor with the same axis for both bodies.
    // axis: The motor axis in world space, used for both bodies.
    // Thrown when axis is zero or contains a non-finite value.
    void Initialize(const LinearMath::JVector& axis)
    {
        Initialize(axis, axis);
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] Real TargetVelocity() const { return Data().Velocity; }
    void TargetVelocity(Real value)
    {
        DebugCheck::IsFinite(value, "value");
        Data().Velocity = value;
    }

    [[nodiscard]] LinearMath::JVector LocalAxis1() const { return Data().LocalAxis1; }
    [[nodiscard]] LinearMath::JVector LocalAxis2() const { return Data().LocalAxis2; }

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

        PrepareForIterationAngularMotor(Data(), inverseDt);
    }

    static void PrepareForIterationAngularMotor(AngularMotorData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector j1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector j2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        data.EffectiveMass =
            LinearMath::JVector::Dot(body1.InverseInertiaWorld * j1, j1)
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * j2, j2);
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;
        data.MaxLambda = (static_cast<Real>(1) / inverseDt) * data.MaxForce;

        body1.AngularVelocity -= body1.InverseInertiaWorld * (j1 * data.AccumulatedImpulse);
        body2.AngularVelocity += body2.InverseInertiaWorld * (j2 * data.AccumulatedImpulse);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateAngularMotor(Data(), inverseDt);
    }

    static void IterateAngularMotor(AngularMotorData& data, Real)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector j1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector j2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        const Real jv =
            -LinearMath::JVector::Dot(j1, body1.AngularVelocity)
            + LinearMath::JVector::Dot(j2, body2.AngularVelocity);
        Real lambda = -(jv - data.Velocity) * data.EffectiveMass;

        const Real oldAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        data.AccumulatedImpulse = std::clamp(data.AccumulatedImpulse, -data.MaxLambda, data.MaxLambda);
        lambda = data.AccumulatedImpulse - oldAccumulated;

        body1.AngularVelocity -= body1.InverseInertiaWorld * (j1 * lambda);
        body2.AngularVelocity += body2.InverseInertiaWorld * (j2 * lambda);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const AngularMotorData& data = Data();
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
            RegisterFullConstraint<AngularMotorData, &PrepareForIterationAngularMotor, &IterateAngularMotor>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
