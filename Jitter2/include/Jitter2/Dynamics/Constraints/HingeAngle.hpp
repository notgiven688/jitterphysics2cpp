#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct HingeAngleData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    Real MinAngle = AngularLimit::Fixed().From;
    Real MaxAngle = AngularLimit::Fixed().To;

    Real BiasFactor = Constraint::DefaultAngularBias;
    Real LimitBias = Constraint::DefaultAngularLimitBias;

    Real LimitSoftness = Constraint::DefaultAngularLimitSoftness;
    Real Softness = Constraint::DefaultAngularSoftness;

    LinearMath::JVector Axis = LinearMath::JVector::UnitX();
    LinearMath::JQuaternion Q0 = LinearMath::JQuaternion::Identity();

    LinearMath::JVector AccumulatedImpulse = LinearMath::JVector::Zero();
    LinearMath::JVector Bias = LinearMath::JVector::Zero();

    LinearMath::JMatrix EffectiveMass = LinearMath::JMatrix::Zero();
    LinearMath::JMatrix Jacobian = LinearMath::JMatrix::Zero();

    unsigned short Clamp = 0;
};

// Constrains two bodies to rotate relative to each other around a single axis,
// removing two angular degrees of freedom. Optionally enforces angular limits.
class HingeAngle : public TypedConstraint<HingeAngleData>
{
public:

    // Initializes the constraint with a rotation axis and angular limits.
    // axis: The hinge axis in world space around which rotation is allowed.
    // limit: The angular limits defining the allowed rotation range.
    // Stores the axis in the local frame of body 2 and records the initial relative orientation.
    // Default values: Softness = Constraint.DefaultAngularSoftness, LimitSoftness = Constraint.DefaultAngularLimitSoftness,
    // Bias = Constraint.DefaultAngularBias, LimitBias = Constraint.DefaultAngularLimitBias.
    // Thrown when axis is zero or contains a non-finite value, or when
    // either value in limit is not finite.
    void Initialize(LinearMath::JVector axis, const AngularLimit& limit)
    {
        VerifyCreated();
        Detail::CheckNonZero(axis, "axis");
        Detail::CheckFinite(limit.From, "limit.From");
        Detail::CheckFinite(limit.To, "limit.To");

        HingeAngleData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        data.Softness = DefaultAngularSoftness;
        data.LimitSoftness = DefaultAngularLimitSoftness;
        data.BiasFactor = DefaultAngularBias;
        data.LimitBias = DefaultAngularLimitBias;

        data.MinAngle = StableMath::Sin(limit.From * static_cast<Real>(0.5));
        data.MaxAngle = StableMath::Sin(limit.To * static_cast<Real>(0.5));

        axis.Normalize();
        data.Axis = LinearMath::JQuaternion::ConjugatedTransform(axis, body2.Orientation);
        data.Q0 = body2.Orientation.Conjugate() * body1.Orientation;
    }

    void Limit(const AngularLimit& value)
    {
        Detail::CheckFinite(value.From, "limit.From");
        Detail::CheckFinite(value.To, "limit.To");

        HingeAngleData& data = Data();
        data.MinAngle = StableMath::Sin(value.From * static_cast<Real>(0.5));
        data.MaxAngle = StableMath::Sin(value.To * static_cast<Real>(0.5));
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = LinearMath::JVector::Zero();
    }

    [[nodiscard]] Real Angle() const
    {
        VerifyCreated();
        const HingeAngleData& data = Data();
        LinearMath::JQuaternion quat0 =
            data.Q0 * data.Body1.Data().Orientation.Conjugate() * data.Body2.Data().Orientation;

        if (quat0.W < static_cast<Real>(0))
        {
            quat0 = quat0 * static_cast<Real>(-1);
        }

        const Real error = LinearMath::JVector::Dot(data.Axis, LinearMath::JVector(quat0.X, quat0.Y, quat0.Z));
        return static_cast<Real>(2) * StableMath::Asin(error);
    }

    [[nodiscard]] Real Softness() const { return Data().Softness; }
    void Softness(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().Softness = value;
    }

    [[nodiscard]] Real LimitSoftness() const { return Data().LimitSoftness; }
    void LimitSoftness(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().LimitSoftness = value;
    }

    [[nodiscard]] Real Bias() const { return Data().BiasFactor; }
    void Bias(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().BiasFactor = value;
    }

    [[nodiscard]] Real LimitBias() const { return Data().LimitBias; }
    void LimitBias(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().LimitBias = value;
    }

    [[nodiscard]] LinearMath::JVector Impulse() const { return Data().AccumulatedImpulse; }

    void PrepareForIteration(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        PrepareForIterationHingeAngle(Data(), inverseDt);
    }

    static void PrepareForIterationHingeAngle(HingeAngleData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JQuaternion q1 = body1.Orientation;
        const LinearMath::JQuaternion q2 = body2.Orientation;

        const LinearMath::JVector p0 = LinearMath::MathHelper::CreateOrthonormal(data.Axis);
        const LinearMath::JVector p1 = LinearMath::JVector::Cross(data.Axis, p0);

        const LinearMath::JQuaternion quat0 = data.Q0 * q1.Conjugate() * q2;
        const LinearMath::JVector quatVector(quat0.X, quat0.Y, quat0.Z);

        LinearMath::JVector error;
        error.X = LinearMath::JVector::Dot(p0, quatVector);
        error.Y = LinearMath::JVector::Dot(p1, quatVector);
        error.Z = LinearMath::JVector::Dot(data.Axis, quatVector);

        data.Clamp = 0;

        LinearMath::JMatrix m0 =
            Detail::ProjectMultiplyLeftRight(data.Q0 * q1.Conjugate(), q2) * static_cast<Real>(-0.5);

        if (quat0.W < static_cast<Real>(0))
        {
            error *= static_cast<Real>(-1);
            m0 = m0 * static_cast<Real>(-1);
        }

        data.Jacobian = LinearMath::JMatrix::FromColumns(
            LinearMath::JMatrix::TransposedTransform(p0, m0),
            LinearMath::JMatrix::TransposedTransform(p1, m0),
            LinearMath::JMatrix::TransposedTransform(data.Axis, m0));

        data.EffectiveMass = LinearMath::JMatrix::TransposedMultiply(
            data.Jacobian,
            (body1.InverseInertiaWorld + body2.InverseInertiaWorld) * data.Jacobian);

        data.EffectiveMass.M11 += data.Softness * inverseDt;
        data.EffectiveMass.M22 += data.Softness * inverseDt;
        data.EffectiveMass.M33 += data.LimitSoftness * inverseDt;

        const Real maxA = data.MaxAngle;
        const Real minA = data.MinAngle;

        if (error.Z > maxA)
        {
            data.Clamp = 1;
            error.Z -= maxA;
        }
        else if (error.Z < minA)
        {
            data.Clamp = 2;
            error.Z -= minA;
        }
        else
        {
            data.AccumulatedImpulse.Z = static_cast<Real>(0);
            data.EffectiveMass.M33 = static_cast<Real>(1);
            data.EffectiveMass.M31 = data.EffectiveMass.M13 = static_cast<Real>(0);
            data.EffectiveMass.M32 = data.EffectiveMass.M23 = static_cast<Real>(0);
            data.Jacobian.M13 = data.Jacobian.M23 = data.Jacobian.M33 = static_cast<Real>(0);
        }

        LinearMath::JMatrix inverseEffectiveMass;
        LinearMath::JMatrix::Inverse(data.EffectiveMass, inverseEffectiveMass);
        data.EffectiveMass = inverseEffectiveMass;

        data.Bias = error * inverseDt;
        data.Bias.X *= data.BiasFactor;
        data.Bias.Y *= data.BiasFactor;
        data.Bias.Z *= data.LimitBias;

    // Gets the accumulated impulse applied by this constraint during the last step.

        const LinearMath::JVector angularImpulse = data.Jacobian * data.AccumulatedImpulse;
        body1.AngularVelocity += body1.InverseInertiaWorld * angularImpulse;
        body2.AngularVelocity -= body2.InverseInertiaWorld * angularImpulse;
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateHingeAngle(Data(), inverseDt);
    }

    static void IterateHingeAngle(HingeAngleData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector jv =
            LinearMath::JMatrix::TransposedTransform(
                body1.AngularVelocity - body2.AngularVelocity,
                data.Jacobian);

        LinearMath::JVector softness = data.AccumulatedImpulse * inverseDt;
        softness.X *= data.Softness;
        softness.Y *= data.Softness;
        softness.Z *= data.LimitSoftness;

        LinearMath::JVector lambda = -(data.EffectiveMass * (jv + data.Bias + softness));
        LinearMath::JVector originalAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse.Z = std::min(static_cast<Real>(0), data.AccumulatedImpulse.Z);
        }
        else if (data.Clamp == 2)
        {
            data.AccumulatedImpulse.Z = std::max(static_cast<Real>(0), data.AccumulatedImpulse.Z);
        }
        else
        {
            originalAccumulated.Z = static_cast<Real>(0);
            data.AccumulatedImpulse.Z = static_cast<Real>(0);
        }

        lambda = data.AccumulatedImpulse - originalAccumulated;
        const LinearMath::JVector angularImpulse = data.Jacobian * lambda;
        body1.AngularVelocity += body1.InverseInertiaWorld * angularImpulse;
        body2.AngularVelocity -= body2.InverseInertiaWorld * angularImpulse;
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const HingeAngleData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();
        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.Axis, body2.Orientation);

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
            RegisterFullConstraint<HingeAngleData, &PrepareForIterationHingeAngle, &IterateHingeAngle>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
