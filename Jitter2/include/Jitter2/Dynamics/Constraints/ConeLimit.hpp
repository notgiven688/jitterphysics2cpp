#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/StableMath.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/Logger.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct ConeLimitData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAxis1 = LinearMath::JVector::UnitX();
    LinearMath::JVector LocalAxis2 = LinearMath::JVector::UnitX();

    Real BiasFactor = Constraint::DefaultAngularBias;
    Real Softness = Constraint::DefaultAngularSoftness;

    Real EffectiveMass = static_cast<Real>(0);
    Real AccumulatedImpulse = static_cast<Real>(0);
    Real Bias = static_cast<Real>(0);

    Real LimitLow = static_cast<Real>(1);
    Real LimitHigh = static_cast<Real>(1);

    short Clamp = 0;

    std::array<LinearMath::JVector, 2> J0 {};
};

// Limits the relative tilt between two bodies, removing one angular degree of freedom when active.
class ConeLimit : public TypedConstraint<ConeLimitData>
{
public:

    // Initializes the cone limit using two world-space axes and an angular range.
    // axisBody1: The reference axis for body 1 in world space.
    // axisBody2: The reference axis for body 2 in world space.
    // limit: The minimum and maximum allowed tilt angles.
    // Each axis is stored as a local axis on the corresponding body. The constraint measures
    // the angle between these axes and restricts it to the given range.
    // Default values: Softness = Constraint.DefaultAngularSoftness, Bias = Constraint.DefaultAngularBias.
    // Thrown when axisBody1 or axisBody2 is zero or contains a non-finite value.
    // Thrown when limit is outside the range [0, pi], or when its upper value is smaller
    // than its lower value.
    void Initialize(LinearMath::JVector axisBody1, LinearMath::JVector axisBody2, const AngularLimit& limit)
    {
        VerifyCreated();
        Detail::CheckNonZero(axisBody1, "axisBody1");
        Detail::CheckNonZero(axisBody2, "axisBody2");
        CheckConeLimit(limit);

        ConeLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        axisBody1.Normalize();
        axisBody2.Normalize();

        data.LocalAxis1 = LinearMath::JQuaternion::ConjugatedTransform(axisBody1, body1.Orientation);
        data.LocalAxis2 = LinearMath::JQuaternion::ConjugatedTransform(axisBody2, body2.Orientation);

        data.Softness = DefaultAngularSoftness;
        data.BiasFactor = DefaultAngularBias;

        data.LimitLow = StableMath::Cos(limit.From);
        data.LimitHigh = StableMath::Cos(limit.To);
    }

    // Initializes the cone limit using a world-space axis and an angular range.
    // axis: The reference axis in world space for the initial pose.
    // limit: The minimum and maximum allowed tilt angles.
    // Stores the axis as a local axis on each body. The constraint measures the angle between
    // these axes and restricts it to the given range.
    // Default values: Softness = Constraint.DefaultAngularSoftness, Bias = Constraint.DefaultAngularBias.
    // Thrown when axis is zero or contains a non-finite value.
    // Thrown when limit is outside the range [0, pi], or when its upper value is smaller
    // than its lower value.
    void Initialize(const LinearMath::JVector& axis, const AngularLimit& limit)
    {
        if (limit.From > static_cast<Real>(0))
        {
            Logger::Warning(
                "ConeLimit.Initialize(): The lower limit is larger 0 but this overload initializes both body axes "
                "from the same world-space axis (rest angle = 0). Use the two-axis overload if you need a non-zero "
                "minimum angle.");
        }

        Initialize(axis, axis, limit);
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] Real Angle() const
    {
        VerifyCreated();
        const ConeLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector a1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector a2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);
        return StableMath::Acos(LinearMath::JVector::Dot(a1, a2));
    }

    [[nodiscard]] LinearMath::JVector AxisBody1() const
    {
        VerifyCreated();
        const ConeLimitData& data = Data();
        return LinearMath::JQuaternion::Transform(data.LocalAxis1, data.Body1.Data().Orientation);
    }

    void AxisBody1(LinearMath::JVector value)
    {
        VerifyCreated();
        ArgumentCheck::NonZero(value, "value");

        ConeLimitData& data = Data();
        value.Normalize();
        data.LocalAxis1 = LinearMath::JQuaternion::ConjugatedTransform(value, data.Body1.Data().Orientation);
    }

    [[nodiscard]] LinearMath::JVector AxisBody2() const
    {
        VerifyCreated();
        const ConeLimitData& data = Data();
        return LinearMath::JQuaternion::Transform(data.LocalAxis2, data.Body2.Data().Orientation);
    }

    void AxisBody2(LinearMath::JVector value)
    {
        VerifyCreated();
        ArgumentCheck::NonZero(value, "value");

        ConeLimitData& data = Data();
        value.Normalize();
        data.LocalAxis2 = LinearMath::JQuaternion::ConjugatedTransform(value, data.Body2.Data().Orientation);
    }

    [[nodiscard]] AngularLimit Limit() const
    {
        const ConeLimitData& data = Data();
        return AngularLimit(StableMath::Acos(data.LimitLow), StableMath::Acos(data.LimitHigh));
    }

    void Limit(const AngularLimit& value)
    {
        CheckConeLimit(value);
        ConeLimitData& data = Data();
        data.LimitLow = StableMath::Cos(value.From);
        data.LimitHigh = StableMath::Cos(value.To);
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

        PrepareForIterationConeLimit(Data(), inverseDt);
    }

    static void PrepareForIterationConeLimit(ConeLimitData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector a1 =
            LinearMath::JQuaternion::Transform(data.LocalAxis1, body1.Orientation);
        const LinearMath::JVector a2 =
            LinearMath::JQuaternion::Transform(data.LocalAxis2, body2.Orientation);

        std::array<LinearMath::JVector, 2>& jacobian = data.J0;
        jacobian[0] = LinearMath::JVector::Cross(a2, a1);
        jacobian[1] = LinearMath::JVector::Cross(a1, a2);

        data.Clamp = 0;

        Real error = LinearMath::JVector::Dot(a1, a2);

        if (error < data.LimitHigh)
        {
            data.Clamp = 1;
            error -= data.LimitHigh;
        }
        else if (error > data.LimitLow)
        {
            data.Clamp = 2;
            error -= data.LimitLow;
        }
        else
        {
            data.AccumulatedImpulse = static_cast<Real>(0);
            return;
        }

        data.EffectiveMass =
            LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[0], jacobian[0])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[1], jacobian[1]);

        data.EffectiveMass += data.Softness * inverseDt;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;

        data.Bias = -error * data.BiasFactor * inverseDt;

        body1.AngularVelocity += body1.InverseInertiaWorld * (data.AccumulatedImpulse * jacobian[0]);
        body2.AngularVelocity += body2.InverseInertiaWorld * (data.AccumulatedImpulse * jacobian[1]);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateConeLimit(Data(), inverseDt);
    }

    static void IterateConeLimit(ConeLimitData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (data.Clamp == 0)
        {
            return;
        }

        std::array<LinearMath::JVector, 2>& jacobian = data.J0;

        const Real jv =
            LinearMath::JVector::Dot(body1.AngularVelocity, jacobian[0])
            + LinearMath::JVector::Dot(body2.AngularVelocity, jacobian[1]);

        const Real softnessScalar = data.AccumulatedImpulse * data.Softness * inverseDt;

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

    // Gets the accumulated impulse applied by this constraint during the last step.

        const Real oldAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = std::min(data.AccumulatedImpulse, static_cast<Real>(0));
        }
        else
        {
            data.AccumulatedImpulse = std::max(data.AccumulatedImpulse, static_cast<Real>(0));
        }

        lambda = data.AccumulatedImpulse - oldAccumulated;

        body1.AngularVelocity += body1.InverseInertiaWorld * (lambda * jacobian[0]);
        body2.AngularVelocity += body2.InverseInertiaWorld * (lambda * jacobian[1]);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const ConeLimitData& data = Data();
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
            RegisterFullConstraint<ConeLimitData, &PrepareForIterationConeLimit, &IterateConeLimit>();
        return id;
    }

    static void CheckConeLimit(const AngularLimit& limit)
    {
        constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
        Detail::CheckFinite(limit.From, "limit.From");
        Detail::CheckFinite(limit.To, "limit.To");
        if (limit.From < static_cast<Real>(0) || limit.To < limit.From || limit.To > pi)
        {
            throw std::out_of_range("Cone limits must satisfy 0 <= From <= To <= pi.");
        }
    }
};

} // namespace Jitter2::Dynamics::Constraints
