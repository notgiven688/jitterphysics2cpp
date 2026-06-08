#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct PointOnLineData
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
    Real LimitBias = Constraint::DefaultLinearLimitBias;
    Real Softness = Constraint::DefaultLinearSoftness;
    Real LimitSoftness = Constraint::DefaultLinearLimitSoftness;

    LinearMath::JMatrix EffectiveMass = LinearMath::JMatrix::Zero();
    LinearMath::JVector AccumulatedImpulse = LinearMath::JVector::Zero();
    LinearMath::JVector Bias = LinearMath::JVector::Zero();

    Real Min = LinearLimit::Fixed().From;
    Real Max = LinearLimit::Fixed().To;

    unsigned short Clamp = 0;
};

// Constrains a fixed point in the reference frame of one body to a line that is fixed in
// the reference frame of another body. This constraint removes two degrees of translational
// freedom; three if the limit is enforced.
class PointOnLine : public TypedConstraint<PointOnLineData>
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

        PointOnLineData& data = Data();
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
        data.LimitSoftness = DefaultLinearLimitSoftness;
        data.LimitBias = DefaultLinearLimitBias;
        data.Min = limit.From;
        data.Max = limit.To;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = LinearMath::JVector::Zero();
    }

    [[nodiscard]] Real Distance() const
    {
        VerifyCreated();
        const PointOnLineData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;
        const LinearMath::JVector u = p2 - p1;

        const LinearMath::JVector axis =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);

        return LinearMath::JVector::Dot(u, axis);
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

    [[nodiscard]] Real LimitSoftness() const { return Data().LimitSoftness; }
    void LimitSoftness(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().LimitSoftness = value;
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

        PrepareForIterationPointOnLine(Data(), inverseDt);
    }

    static void PrepareForIterationPointOnLine(PointOnLineData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector aw =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);

        LinearMath::JVector n1 = LinearMath::MathHelper::CreateOrthonormal(data.LocalAxis);
        n1 = LinearMath::JQuaternion::Transform(n1, body1.Orientation);

        const LinearMath::JVector n2 = LinearMath::JVector::Cross(aw, n1);

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;

        data.Clamp = 0;

        const LinearMath::JVector u = p2 - p1;

        std::array<LinearMath::JVector, 12> jacobian {};

        jacobian[0] = -n1;
        jacobian[1] = -LinearMath::JVector::Cross(r1 + u, n1);
        jacobian[2] = n1;
        jacobian[3] = LinearMath::JVector::Cross(r2, n1);

        jacobian[4] = -n2;
        jacobian[5] = -LinearMath::JVector::Cross(r1 + u, n2);
        jacobian[6] = n2;
        jacobian[7] = LinearMath::JVector::Cross(r2, n2);

        jacobian[8] = -aw;
        jacobian[9] = -LinearMath::JVector::Cross(r1 + u, aw);
        jacobian[10] = aw;
        jacobian[11] = LinearMath::JVector::Cross(r2, aw);

        LinearMath::JVector error(
            LinearMath::JVector::Dot(u, n1),
            LinearMath::JVector::Dot(u, n2),
            LinearMath::JVector::Dot(u, aw));

        data.EffectiveMass = LinearMath::JMatrix::Identity();

        data.EffectiveMass.M11 =
            body1.InverseMass + body2.InverseMass
            + LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[1], jacobian[1])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[3], jacobian[3]);

        data.EffectiveMass.M22 =
            body1.InverseMass + body2.InverseMass
            + LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[5], jacobian[5])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[7], jacobian[7]);

        data.EffectiveMass.M12 =
            LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[1], jacobian[5])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[3], jacobian[7]);

        data.EffectiveMass.M21 =
            LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[5], jacobian[1])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[7], jacobian[3]);

        if (error.Z > data.Max)
        {
            error.Z -= data.Max;
            data.Clamp = 1;
        }
        else if (error.Z < data.Min)
        {
            error.Z -= data.Min;
            data.Clamp = 2;
        }
        else
        {
            data.AccumulatedImpulse.Z = static_cast<Real>(0);
        }

        if (data.Clamp != 0)
        {
            data.EffectiveMass.M33 =
                body1.InverseMass + body2.InverseMass
                + LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[9], jacobian[9])
                + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[11], jacobian[11]);

            data.EffectiveMass.M13 =
                LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[1], jacobian[9])
                + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[3], jacobian[11]);

            data.EffectiveMass.M31 =
                LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[9], jacobian[1])
                + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[11], jacobian[3]);

            data.EffectiveMass.M23 =
                LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[5], jacobian[9])
                + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[7], jacobian[11]);

            data.EffectiveMass.M32 =
                LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[9], jacobian[5])
                + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[11], jacobian[7]);
        }

        data.EffectiveMass.M11 += data.Softness * inverseDt;
        data.EffectiveMass.M22 += data.Softness * inverseDt;
        data.EffectiveMass.M33 += data.LimitSoftness * inverseDt;

        LinearMath::JMatrix inverseEffectiveMass;
        LinearMath::JMatrix::Inverse(data.EffectiveMass, inverseEffectiveMass);
        data.EffectiveMass = inverseEffectiveMass;

        data.Bias = error * inverseDt;
        data.Bias.X *= data.BiasFactor;
        data.Bias.Y *= data.BiasFactor;
        data.Bias.Z *= data.LimitBias;

    // Gets the accumulated impulse applied by this constraint during the last step.

        const LinearMath::JVector accumulated = data.AccumulatedImpulse;
        body1.Velocity += body1.InverseMass
            * (jacobian[0] * accumulated.X + jacobian[4] * accumulated.Y + jacobian[8] * accumulated.Z);
        body1.AngularVelocity += body1.InverseInertiaWorld
            * (jacobian[1] * accumulated.X + jacobian[5] * accumulated.Y + jacobian[9] * accumulated.Z);

        body2.Velocity += body2.InverseMass
            * (jacobian[2] * accumulated.X + jacobian[6] * accumulated.Y + jacobian[10] * accumulated.Z);
        body2.AngularVelocity += body2.InverseInertiaWorld
            * (jacobian[3] * accumulated.X + jacobian[7] * accumulated.Y + jacobian[11] * accumulated.Z);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IteratePointOnLine(Data(), inverseDt);
    }

    static void IteratePointOnLine(PointOnLineData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector aw =
            LinearMath::JQuaternion::Transform(data.LocalAxis, body1.Orientation);

        LinearMath::JVector n1 = LinearMath::MathHelper::CreateOrthonormal(data.LocalAxis);
        n1 = LinearMath::JQuaternion::Transform(n1, body1.Orientation);

        const LinearMath::JVector n2 = LinearMath::JVector::Cross(aw, n1);

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;

        const LinearMath::JVector u = p2 - p1;

        std::array<LinearMath::JVector, 12> jacobian {};

        jacobian[0] = -n1;
        jacobian[1] = -LinearMath::JVector::Cross(r1 + u, n1);
        jacobian[2] = n1;
        jacobian[3] = LinearMath::JVector::Cross(r2, n1);

        jacobian[4] = -n2;
        jacobian[5] = -LinearMath::JVector::Cross(r1 + u, n2);
        jacobian[6] = n2;
        jacobian[7] = LinearMath::JVector::Cross(r2, n2);

        jacobian[8] = -aw;
        jacobian[9] = -LinearMath::JVector::Cross(r1 + u, aw);
        jacobian[10] = aw;
        jacobian[11] = LinearMath::JVector::Cross(r2, aw);

        LinearMath::JVector jv;
        jv.X =
            LinearMath::JVector::Dot(jacobian[0], body1.Velocity)
            + LinearMath::JVector::Dot(jacobian[1], body1.AngularVelocity)
            + LinearMath::JVector::Dot(jacobian[2], body2.Velocity)
            + LinearMath::JVector::Dot(jacobian[3], body2.AngularVelocity);
        jv.Y =
            LinearMath::JVector::Dot(jacobian[4], body1.Velocity)
            + LinearMath::JVector::Dot(jacobian[5], body1.AngularVelocity)
            + LinearMath::JVector::Dot(jacobian[6], body2.Velocity)
            + LinearMath::JVector::Dot(jacobian[7], body2.AngularVelocity);
        jv.Z =
            LinearMath::JVector::Dot(jacobian[8], body1.Velocity)
            + LinearMath::JVector::Dot(jacobian[9], body1.AngularVelocity)
            + LinearMath::JVector::Dot(jacobian[10], body2.Velocity)
            + LinearMath::JVector::Dot(jacobian[11], body2.AngularVelocity);

        LinearMath::JVector softnessVector = data.AccumulatedImpulse * inverseDt;
        softnessVector.X *= data.Softness;
        softnessVector.Y *= data.Softness;
        softnessVector.Z *= data.LimitSoftness;

        LinearMath::JVector lambda = -(data.EffectiveMass * (jv + data.Bias + softnessVector));
        LinearMath::JVector originalAccumulated = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;

        if ((data.Clamp & 1U) != 0)
        {
            data.AccumulatedImpulse.Z = std::min(data.AccumulatedImpulse.Z, static_cast<Real>(0));
        }
        else if ((data.Clamp & 2U) != 0)
        {
            data.AccumulatedImpulse.Z = std::max(data.AccumulatedImpulse.Z, static_cast<Real>(0));
        }
        else
        {
            data.AccumulatedImpulse.Z = static_cast<Real>(0);
            originalAccumulated.Z = static_cast<Real>(0);
        }

        lambda = data.AccumulatedImpulse - originalAccumulated;

        body1.Velocity += body1.InverseMass
            * (jacobian[0] * lambda.X + jacobian[4] * lambda.Y + jacobian[8] * lambda.Z);
        body1.AngularVelocity += body1.InverseInertiaWorld
            * (jacobian[1] * lambda.X + jacobian[5] * lambda.Y + jacobian[9] * lambda.Z);

        body2.Velocity += body2.InverseMass
            * (jacobian[2] * lambda.X + jacobian[6] * lambda.Y + jacobian[10] * lambda.Z);
        body2.AngularVelocity += body2.InverseInertiaWorld
            * (jacobian[3] * lambda.X + jacobian[7] * lambda.Y + jacobian[11] * lambda.Z);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const PointOnLineData& data = Data();
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

        constexpr Real lineLength = static_cast<Real>(1);
        drawer.DrawSegment(p1 - axis * lineLength, p1 + axis * lineLength);
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
            RegisterFullConstraint<PointOnLineData, &PrepareForIterationPointOnLine, &IteratePointOnLine>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
