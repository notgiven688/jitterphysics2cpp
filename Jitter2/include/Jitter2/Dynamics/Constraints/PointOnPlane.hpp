#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct SliderData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAxis = LinearMath::JVector::UnitX();

    LinearMath::JVector LocalAnchor1 = LinearMath::JVector::Zero();
    LinearMath::JVector LocalAnchor2 = LinearMath::JVector::Zero();

    Real BiasFactor = Constraint::DefaultLinearBias;
    Real Softness = Constraint::DefaultLinearSoftness;

    Real EffectiveMass = static_cast<Real>(0);
    Real AccumulatedImpulse = static_cast<Real>(0);
    Real Bias = static_cast<Real>(0);

    Real Min = LinearLimit::Fixed().From;
    Real Max = LinearLimit::Fixed().To;

    unsigned short Clamp = 0;

    std::array<LinearMath::JVector, 4> J0 {};
};

// Constrains a fixed point in the reference frame of one body to a plane that is fixed in
// the reference frame of another body. This constraint removes one degree of translational
// freedom if the limit is enforced.
class PointOnPlane : public TypedConstraint<SliderData>
{
public:
    void Initialize(
        const LinearMath::JVector& axis,
        const LinearMath::JVector& anchor1,
        const LinearMath::JVector& anchor2)
    {
        Initialize(axis, anchor1, anchor2, LinearLimit::Fixed());
    }

    void Initialize(
        LinearMath::JVector axis,
        const LinearMath::JVector& anchor1,
        const LinearMath::JVector& anchor2,
        const LinearLimit& limit)
    {
        VerifyCreated();
        ArgumentCheck::NonZero(axis, "axis");
        ArgumentCheck::Finite(anchor1, "anchor1");
        ArgumentCheck::Finite(anchor2, "anchor2");
        ArgumentCheck::NotNaN(limit.From, "limit.From");
        ArgumentCheck::NotNaN(limit.To, "limit.To");

        SliderData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        axis.Normalize();
        data.LocalAnchor1 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor1 - body1.Position,
            body1.Orientation);
        data.LocalAnchor2 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor2 - body2.Position,
            body2.Orientation);
        data.LocalAxis = LinearMath::JQuaternion::ConjugatedTransform(axis, body1.Orientation);

        data.BiasFactor = DefaultLinearBias;
        data.Softness = DefaultLinearSoftness;
        data.Min = limit.From;
        data.Max = limit.To;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] Real Distance() const
    {
        VerifyCreated();
        const SliderData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);
        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        return LinearMath::JVector::Dot(body2.Position + r2 - body1.Position - r1, axis);
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

        PrepareForIterationPointOnPlane(Data(), inverseDt);
    }

    static void PrepareForIterationPointOnPlane(SliderData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);
        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;
        const LinearMath::JVector u = p2 - p1;

        data.Clamp = 0;

        std::array<LinearMath::JVector, 4>& jacobian = data.J0;
        jacobian[0] = -axis;
        jacobian[1] = -LinearMath::JVector::Cross(r1 + u, axis);
        jacobian[2] = axis;
        jacobian[3] = LinearMath::JVector::Cross(r2, axis);

        Real error = LinearMath::JVector::Dot(u, axis);
        data.EffectiveMass = static_cast<Real>(1);

        if (error > data.Max)
        {
            error -= data.Max;
            data.Clamp = 1;
        }
        else if (error < data.Min)
        {
            error -= data.Min;
            data.Clamp = 2;
        }
        else
        {
            data.AccumulatedImpulse = static_cast<Real>(0);
            return;
        }

        data.EffectiveMass =
            body1.InverseMass + body2.InverseMass
            + LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[1], jacobian[1])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[3], jacobian[3]);

        data.EffectiveMass += data.Softness * inverseDt;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;

        data.Bias = error * data.BiasFactor * inverseDt;

    // Gets the accumulated impulse applied by this constraint during the last step.

        const Real accumulated = data.AccumulatedImpulse;
        body1.Velocity += jacobian[0] * (body1.InverseMass * accumulated);
        body1.AngularVelocity += body1.InverseInertiaWorld * (jacobian[1] * accumulated);

        body2.Velocity += jacobian[2] * (body2.InverseMass * accumulated);
        body2.AngularVelocity += body2.InverseInertiaWorld * (jacobian[3] * accumulated);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IteratePointOnPlane(Data(), inverseDt);
    }

    static void IteratePointOnPlane(SliderData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        if (data.Clamp == 0)
        {
            return;
        }

        std::array<LinearMath::JVector, 4>& jacobian = data.J0;

        const Real jv =
            LinearMath::JVector::Dot(jacobian[0], body1.Velocity)
            + LinearMath::JVector::Dot(jacobian[1], body1.AngularVelocity)
            + LinearMath::JVector::Dot(jacobian[2], body2.Velocity)
            + LinearMath::JVector::Dot(jacobian[3], body2.AngularVelocity);

        const Real softness = data.AccumulatedImpulse * data.Softness * inverseDt;
        Real lambda = -(jv + data.Bias + softness) * data.EffectiveMass;

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

        body1.Velocity += jacobian[0] * (body1.InverseMass * lambda);
        body1.AngularVelocity += body1.InverseInertiaWorld * (jacobian[1] * lambda);

        body2.Velocity += jacobian[2] * (body2.InverseMass * lambda);
        body2.AngularVelocity += body2.InverseInertiaWorld * (jacobian[3] * lambda);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const SliderData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);
        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;

        constexpr Real normalLength = static_cast<Real>(0.5);
        drawer.DrawSegment(p1, p1 + axis * normalLength);
        drawer.DrawSegment(body2.Position, p2);
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
            RegisterFullConstraint<SliderData, &PrepareForIterationPointOnPlane, &IteratePointOnPlane>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
