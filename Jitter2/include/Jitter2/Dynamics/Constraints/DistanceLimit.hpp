#pragma once

#include <array>
#include <algorithm>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct DistanceLimitData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAnchor1 = LinearMath::JVector::Zero();
    LinearMath::JVector LocalAnchor2 = LinearMath::JVector::Zero();

    Real BiasFactor = Constraint::DefaultLinearBias;
    Real Softness = Constraint::DefaultLinearSoftness;
    Real Distance = static_cast<Real>(0);

    Real LimitMin = LinearLimit::Fixed().From;
    Real LimitMax = LinearLimit::Fixed().To;

    Real EffectiveMass = static_cast<Real>(0);
    Real AccumulatedImpulse = static_cast<Real>(0);
    Real Bias = static_cast<Real>(0);

    std::array<LinearMath::JVector, 4> J0 {};

    short Clamp = 0;
};

// Constrains the distance between a fixed point in the reference frame of one body and a fixed
// point in the reference frame of another body. This constraint removes one translational degree
// of freedom. For a distance of zero, use the BallSocket constraint.
class DistanceLimit : public TypedConstraint<DistanceLimitData>
{
public:

    // Initializes the constraint with a fixed distance between anchor points.
    // anchor1: Anchor point on the first body in world space.
    // anchor2: Anchor point on the second body in world space.
    // Thrown when anchor1 or anchor2 contains a non-finite value.
    void Initialize(const LinearMath::JVector& anchor1, const LinearMath::JVector& anchor2)
    {
        Initialize(anchor1, anchor2, LinearLimit::Fixed());
    }

    void Initialize(
        const LinearMath::JVector& anchor1,
        const LinearMath::JVector& anchor2,
        const LinearLimit& limit)
    {
        VerifyCreated();
        ArgumentCheck::Finite(anchor1, "anchor1");
        ArgumentCheck::Finite(anchor2, "anchor2");
        ArgumentCheck::NotNaN(limit.From, "limit.From");
        ArgumentCheck::NotNaN(limit.To, "limit.To");

        DistanceLimitData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        data.LocalAnchor1 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor1 - body1.Position,
            body1.Orientation);
        data.LocalAnchor2 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor2 - body2.Position,
            body2.Orientation);

        data.Softness = DefaultLinearSoftness;
        data.BiasFactor = DefaultLinearBias;
        data.Distance = (anchor2 - anchor1).Length();
        data.LimitMin = limit.From;
        data.LimitMax = limit.To;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = static_cast<Real>(0);
    }

    [[nodiscard]] LinearMath::JVector Anchor1() const
    {
        VerifyCreated();
        const DistanceLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        return LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation) + body1.Position;
    }

    void Anchor1(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        DistanceLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        data.LocalAnchor1 = LinearMath::JQuaternion::ConjugatedTransform(
            value - body1.Position,
            body1.Orientation);
    }

    [[nodiscard]] LinearMath::JVector Anchor2() const
    {
        VerifyCreated();
        const DistanceLimitData& data = Data();
        const RigidBodyData& body2 = data.Body2.Data();
        return LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation) + body2.Position;
    }

    void Anchor2(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        DistanceLimitData& data = Data();
        const RigidBodyData& body2 = data.Body2.Data();
        data.LocalAnchor2 = LinearMath::JQuaternion::ConjugatedTransform(
            value - body2.Position,
            body2.Orientation);
    }

    [[nodiscard]] Real TargetDistance() const { return Data().Distance; }
    void TargetDistance(Real value)
    {
        DebugCheck::IsNonNegative(value, "value");
        Data().Distance = value;
    }

    [[nodiscard]] Real Distance() const
    {
        VerifyCreated();
        const DistanceLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        return (body2.Position + r2 - body1.Position - r1).Length();
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

        PrepareForIterationFixedAngle(Data(), inverseDt);
    }

    static void PrepareForIterationFixedAngle(DistanceLimitData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;
        const LinearMath::JVector dp = p2 - p1;

        Real error = dp.Length() - data.Distance;
        data.Clamp = 0;

        if (error >= data.LimitMax)
        {
            data.Clamp = 1;
            error -= data.LimitMax;
        }
        else if (error < data.LimitMin)
        {
            data.Clamp = 2;
            error -= data.LimitMin;
        }
        else
        {
            data.AccumulatedImpulse = static_cast<Real>(0);
            return;
        }

        LinearMath::JVector n = dp;
        if (n.LengthSquared() > static_cast<Real>(1e-12))
        {
            n.Normalize();
        }

        std::array<LinearMath::JVector, 4>& jacobian = data.J0;
        jacobian[0] = -n;
        jacobian[1] = -LinearMath::JVector::Cross(r1, n);
        jacobian[2] = n;
        jacobian[3] = LinearMath::JVector::Cross(r2, n);

        data.EffectiveMass =
            body1.InverseMass
            + body2.InverseMass
            + LinearMath::JVector::Dot(body1.InverseInertiaWorld * jacobian[1], jacobian[1])
            + LinearMath::JVector::Dot(body2.InverseInertiaWorld * jacobian[3], jacobian[3]);

        data.EffectiveMass += data.Softness * inverseDt;
        data.EffectiveMass = static_cast<Real>(1) / data.EffectiveMass;
        data.Bias = error * data.BiasFactor * inverseDt;

        body1.Velocity += jacobian[0] * (body1.InverseMass * data.AccumulatedImpulse);
        body1.AngularVelocity += body1.InverseInertiaWorld * (jacobian[1] * data.AccumulatedImpulse);

        body2.Velocity += jacobian[2] * (body2.InverseMass * data.AccumulatedImpulse);
        body2.AngularVelocity += body2.InverseInertiaWorld * (jacobian[3] * data.AccumulatedImpulse);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateFixedAngle(Data(), inverseDt);
    }

    static void IterateFixedAngle(DistanceLimitData& data, Real inverseDt)
    {
        if (data.Clamp == 0)
        {
            return;
        }

        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();
        std::array<LinearMath::JVector, 4>& jacobian = data.J0;

        const Real jv =
            LinearMath::JVector::Dot(body1.Velocity, jacobian[0])
            + LinearMath::JVector::Dot(body1.AngularVelocity, jacobian[1])
            + LinearMath::JVector::Dot(body2.Velocity, jacobian[2])
            + LinearMath::JVector::Dot(body2.AngularVelocity, jacobian[3]);

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

        body1.Velocity += jacobian[0] * (body1.InverseMass * lambda);
        body1.AngularVelocity += body1.InverseInertiaWorld * (jacobian[1] * lambda);

        body2.Velocity += jacobian[2] * (body2.InverseMass * lambda);
        body2.AngularVelocity += body2.InverseInertiaWorld * (jacobian[3] * lambda);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const DistanceLimitData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector r1 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        const LinearMath::JVector r2 =
            LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + r1;
        const LinearMath::JVector p2 = body2.Position + r2;

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
            RegisterFullConstraint<DistanceLimitData, &PrepareForIterationFixedAngle, &IterateFixedAngle>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
