#pragma once

#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct BallSocketData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    LinearMath::JVector LocalAnchor1 = LinearMath::JVector::Zero();
    LinearMath::JVector LocalAnchor2 = LinearMath::JVector::Zero();

    LinearMath::JVector U = LinearMath::JVector::Zero();
    LinearMath::JVector R1 = LinearMath::JVector::Zero();
    LinearMath::JVector R2 = LinearMath::JVector::Zero();

    Real BiasFactor = Constraint::DefaultLinearBias;
    Real Softness = Constraint::DefaultLinearSoftness;

    LinearMath::JMatrix EffectiveMass = LinearMath::JMatrix::Zero();
    LinearMath::JVector AccumulatedImpulse = LinearMath::JVector::Zero();
    LinearMath::JVector Bias = LinearMath::JVector::Zero();
};

// Implements a ball-and-socket joint that anchors a point on each body together,
// removing three translational degrees of freedom.
class BallSocket : public TypedConstraint<BallSocketData>
{
public:

    // Initializes the constraint from a world-space anchor point.
    // anchor: The anchor point in world space, shared by both bodies.
    // Computes local anchor points for each body from their current poses.
    // Default values: Bias = Constraint.DefaultLinearBias, Softness = Constraint.DefaultLinearSoftness.
    // Thrown when anchor contains a non-finite value.
    void Initialize(const LinearMath::JVector& anchor)
    {
        VerifyCreated();
        Detail::CheckFinite(anchor, "anchor");

        BallSocketData& data = Data();
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        data.LocalAnchor1 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor - body1.Position,
            body1.Orientation);
        data.LocalAnchor2 = LinearMath::JQuaternion::ConjugatedTransform(
            anchor - body2.Position,
            body2.Orientation);

        data.BiasFactor = DefaultLinearBias;
        data.Softness = DefaultLinearSoftness;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = LinearMath::JVector::Zero();
    }

    [[nodiscard]] LinearMath::JVector Anchor1() const
    {
        VerifyCreated();
        const BallSocketData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        return LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation) + body1.Position;
    }

    void Anchor1(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        BallSocketData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        data.LocalAnchor1 = LinearMath::JQuaternion::ConjugatedTransform(
            value - body1.Position,
            body1.Orientation);
    }

    [[nodiscard]] LinearMath::JVector Anchor2() const
    {
        VerifyCreated();
        const BallSocketData& data = Data();
        const RigidBodyData& body2 = data.Body2.Data();
        return LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation) + body2.Position;
    }

    void Anchor2(const LinearMath::JVector& value)
    {
        VerifyCreated();
        DebugCheck::IsFinite(value, "value");

        BallSocketData& data = Data();
        const RigidBodyData& body2 = data.Body2.Data();
        data.LocalAnchor2 = LinearMath::JQuaternion::ConjugatedTransform(
            value - body2.Position,
            body2.Orientation);
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

    [[nodiscard]] LinearMath::JVector Impulse() const { return Data().AccumulatedImpulse; }

    void PrepareForIteration(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        PrepareForIterationBallSocket(Data(), inverseDt);
    }

    static void PrepareForIterationBallSocket(BallSocketData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        data.R1 = LinearMath::JQuaternion::Transform(data.LocalAnchor1, body1.Orientation);
        data.R2 = LinearMath::JQuaternion::Transform(data.LocalAnchor2, body2.Orientation);

        const LinearMath::JVector p1 = body1.Position + data.R1;
        const LinearMath::JVector p2 = body2.Position + data.R2;

        const LinearMath::JMatrix cr1 = LinearMath::JMatrix::CreateCrossProduct(data.R1);
        const LinearMath::JMatrix cr2 = LinearMath::JMatrix::CreateCrossProduct(data.R2);

        data.EffectiveMass =
            body1.InverseMass * LinearMath::JMatrix::Identity()
            + cr1 * LinearMath::JMatrix::MultiplyTransposed(body1.InverseInertiaWorld, cr1)
            + body2.InverseMass * LinearMath::JMatrix::Identity()
            + cr2 * LinearMath::JMatrix::MultiplyTransposed(body2.InverseInertiaWorld, cr2);

        const Real softness = data.Softness * inverseDt;
        data.EffectiveMass.M11 += softness;
        data.EffectiveMass.M22 += softness;
        data.EffectiveMass.M33 += softness;

        LinearMath::JMatrix inverseEffectiveMass;
        LinearMath::JMatrix::Inverse(data.EffectiveMass, inverseEffectiveMass);
        data.EffectiveMass = inverseEffectiveMass;

        data.Bias = (p2 - p1) * data.BiasFactor * inverseDt;

    // Gets the accumulated impulse applied by this constraint during the last step.

        const LinearMath::JVector accumulated = data.AccumulatedImpulse;
        body1.Velocity -= accumulated * body1.InverseMass;
        body1.AngularVelocity -= body1.InverseInertiaWorld * (cr1 * accumulated);

        body2.Velocity += accumulated * body2.InverseMass;
        body2.AngularVelocity += body2.InverseInertiaWorld * (cr2 * accumulated);
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateBallSocket(Data(), inverseDt);
    }

    static void IterateBallSocket(BallSocketData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JMatrix cr1 = LinearMath::JMatrix::CreateCrossProduct(data.R1);
        const LinearMath::JMatrix cr2 = LinearMath::JMatrix::CreateCrossProduct(data.R2);

        const LinearMath::JVector softnessVector = data.AccumulatedImpulse * data.Softness * inverseDt;
        const LinearMath::JVector jv =
            -body1.Velocity + cr1 * body1.AngularVelocity
            + body2.Velocity - cr2 * body2.AngularVelocity;

        const LinearMath::JVector lambda = -(data.EffectiveMass * (jv + data.Bias + softnessVector));
        data.AccumulatedImpulse += lambda;

        body1.Velocity -= lambda * body1.InverseMass;
        body1.AngularVelocity -= body1.InverseInertiaWorld * (cr1 * lambda);

        body2.Velocity += lambda * body2.InverseMass;
        body2.AngularVelocity += body2.InverseInertiaWorld * (cr2 * lambda);
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const BallSocketData& data = Data();
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
            RegisterFullConstraint<BallSocketData, &PrepareForIterationBallSocket, &IterateBallSocket>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
