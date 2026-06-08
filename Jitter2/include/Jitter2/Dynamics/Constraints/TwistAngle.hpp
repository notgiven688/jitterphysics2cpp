#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct TwistLimitData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector B = LinearMath::JVector::UnitX();

    LinearMath::JQuaternion Q0 = LinearMath::JQuaternion::Identity();

    Real Angle1 = static_cast<Real>(0);
    Real Angle2 = static_cast<Real>(0);
    unsigned short Clamp = 0;

    Real BiasFactor = Constraint::DefaultAngularBias;
    Real Softness = Constraint::DefaultAngularSoftness;

    Real EffectiveMass = static_cast<Real>(0);
    Real AccumulatedImpulse = static_cast<Real>(0);
    Real Bias = static_cast<Real>(0);

    LinearMath::JVector Jacobian = LinearMath::JVector::Zero();
};

// Constrains the relative twist of two bodies. This constraint removes one angular
// degree of freedom when the limit is enforced.
class TwistAngle : public TypedConstraint<TwistLimitData>
{
public:

    // Initializes the constraint with a fixed twist angle (no rotation allowed).
    // axis1: The twist axis for body 1 in world space.
    // axis2: The twist axis for body 2 in world space.
    // Thrown when axis1 or axis2 is zero or contains a non-finite value.
    void Initialize(const LinearMath::JVector& axis1, const LinearMath::JVector& axis2)
    {
        Initialize(axis1, axis2, AngularLimit::Fixed());
    }

    // Initializes the constraint from world-space axes and angular limits.
    // axis1: The twist axis for body 1 in world space.
    // axis2: The twist axis for body 2 in world space.
    // limit: The allowed relative twist angle range.
    // Stores each axis in the local frame of its body and records the initial relative orientation.
    // Default values: Softness = Constraint.DefaultAngularSoftness, Bias = Constraint.DefaultAngularBias.
    // Thrown when axis1 or axis2 is zero or contains a non-finite value,
    // or when either value in limit is not finite.
    void Initialize(LinearMath::JVector axis1, LinearMath::JVector axis2, const AngularLimit& limit)
    {
        VerifyCreated();
        Detail::CheckNonZero(axis1, "axis1");
        Detail::CheckNonZero(axis2, "axis2");
        Detail::CheckFinite(limit.From, "limit.From");
        Detail::CheckFinite(limit.To, "limit.To");

        TwistLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        data.Softness = DefaultAngularSoftness;
        data.BiasFactor = DefaultAngularBias;

        axis1.Normalize();
        axis2.Normalize();

        data.Angle1 = StableMath::Sin(limit.From * static_cast<Real>(0.5));
        data.Angle2 = StableMath::Sin(limit.To * static_cast<Real>(0.5));

        const LinearMath::JVector u1 =
            LinearMath::JQuaternion::ConjugatedTransform(axis1, body1.Orientation);
        data.B = LinearMath::JQuaternion::ConjugatedTransform(axis2, body2.Orientation);

        const LinearMath::JQuaternion qRel = body2.Orientation.Conjugate() * body1.Orientation;
        const LinearMath::JVector u1InB2 = LinearMath::JQuaternion::Transform(u1, qRel);
        const LinearMath::JQuaternion qCorrection =
            LinearMath::JQuaternion::CreateFromToRotation(u1InB2, data.B);
        data.Q0 = qCorrection * qRel;
    }

    void Limit(const AngularLimit& value)
    {
        Detail::CheckFinite(value.From, "limit.From");
        Detail::CheckFinite(value.To, "limit.To");

        TwistLimitData& data = Data();
        data.Angle1 = StableMath::Sin(value.From * static_cast<Real>(0.5));
        data.Angle2 = StableMath::Sin(value.To * static_cast<Real>(0.5));
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] Real Angle() const
    {
        VerifyCreated();
        const TwistLimitData& data = Data();
        LinearMath::JQuaternion quat0 =
            data.Q0 * data.Body1.Data().Orientation.Conjugate() * data.Body2.Data().Orientation;

        if (quat0.W < static_cast<Real>(0))
        {
            quat0 = quat0 * static_cast<Real>(-1);
        }

        const Real error = LinearMath::JVector::Dot(data.B, LinearMath::JVector(quat0.X, quat0.Y, quat0.Z));
        return static_cast<Real>(2) * StableMath::Asin(error);
    }

    [[nodiscard]] Real Softness() const { return Data().Softness; }
    void Softness(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().Softness = value;
    }

    [[nodiscard]] Real Bias() const { return Data().BiasFactor; }
    void Bias(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().BiasFactor = value;
    }

    [[nodiscard]] Real Impulse() const { return Data().AccumulatedImpulse; }

    void PrepareForIteration(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        PrepareForIterationTwistAngle(Data(), inverseDt);
    }

    static void PrepareForIterationTwistAngle(TwistLimitData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JQuaternion q1 = body1.Orientation;
        const LinearMath::JQuaternion q2 = body2.Orientation;

        const LinearMath::JMatrix m =
            Detail::ProjectMultiplyLeftRight(data.Q0 * q1.Conjugate(), q2) * static_cast<Real>(-0.5);

        const LinearMath::JQuaternion q = data.Q0 * q1.Conjugate() * q2;

        data.Jacobian = LinearMath::JMatrix::TransposedTransform(data.B, m);

        data.EffectiveMass =
            LinearMath::JVector::Dot(
                (body1.InverseInertiaWorld + body2.InverseInertiaWorld) * data.Jacobian,
                data.Jacobian);

        data.EffectiveMass += data.Softness * inverseDt;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;

        Real error = LinearMath::JVector::Dot(data.B, LinearMath::JVector(q.X, q.Y, q.Z));

        if (q.W < static_cast<Real>(0))
        {
            error *= static_cast<Real>(-1);
            data.Jacobian *= static_cast<Real>(-1);
        }

        data.Clamp = 0;

        if (error >= data.Angle2)
        {
            data.Clamp = 1;
            error -= data.Angle2;
        }
        else if (error < data.Angle1)
        {
            data.Clamp = 2;
            error -= data.Angle1;
        }
        else
        {
            data.AccumulatedImpulse = static_cast<Real>(0);
            return;
        }

        data.Bias = error * data.BiasFactor * inverseDt;

        body1.AngularVelocity += body1.InverseInertiaWorld * (data.AccumulatedImpulse * data.Jacobian);
        body2.AngularVelocity -= body2.InverseInertiaWorld * (data.AccumulatedImpulse * data.Jacobian);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateTwistAngle(Data(), inverseDt);
    }

    static void IterateTwistAngle(TwistLimitData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (data.Clamp == 0)
        {
            return;
        }

        const Real jv = LinearMath::JVector::Dot(
            body1.AngularVelocity - body2.AngularVelocity,
            data.Jacobian);

        const Real softnessScalar = data.AccumulatedImpulse * (data.Softness * inverseDt);

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

    // Gets the accumulated impulse applied by this constraint during the last step.

        const Real originalAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = std::min(data.AccumulatedImpulse, static_cast<Real>(0));
        }
        else
        {
            data.AccumulatedImpulse = std::max(data.AccumulatedImpulse, static_cast<Real>(0));
        }

        lambda = data.AccumulatedImpulse - originalAccumulated;

        body1.AngularVelocity += body1.InverseInertiaWorld * (lambda * data.Jacobian);
        body2.AngularVelocity -= body2.InverseInertiaWorld * (lambda * data.Jacobian);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const TwistLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();
        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.B, body2.Orientation);

        constexpr Real axisLength = static_cast<Real>(0.5);
        drawer.DrawSegment(body1.Position, body1.Position + axis * axisLength);
        drawer.DrawSegment(body2.Position, body2.Position + axis * axisLength);
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
            RegisterFullConstraint<TwistLimitData, &PrepareForIterationTwistAngle, &IterateTwistAngle>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
