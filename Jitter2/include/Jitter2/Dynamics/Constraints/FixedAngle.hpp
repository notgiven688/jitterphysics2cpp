#pragma once

#include <cstdint>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

struct FixedAngleData
{
    int _internal = 0;
    std::uint32_t DispatchId = 0;
    std::uint64_t ConstraintId = 0;

    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;

    Real MinAngle = static_cast<Real>(0);
    Real MaxAngle = static_cast<Real>(0);

    Real BiasFactor = Constraint::DefaultAngularBias;
    Real Softness = Constraint::DefaultAngularSoftness;

    LinearMath::JQuaternion Q0 = LinearMath::JQuaternion::Identity();

    LinearMath::JVector AccumulatedImpulse = LinearMath::JVector::Zero();
    LinearMath::JVector Bias = LinearMath::JVector::Zero();

    LinearMath::JMatrix EffectiveMass = LinearMath::JMatrix::Zero();
    LinearMath::JMatrix Jacobian = LinearMath::JMatrix::Zero();

    unsigned short Clamp = 0;
};

// Constrains the relative orientation between two bodies, eliminating three degrees of rotational freedom.
class FixedAngle : public TypedConstraint<FixedAngleData>
{
public:

    // Initializes the constraint using the current relative orientation of the bodies.
    // Records the current relative orientation as the target.
    // Default values: Softness = Constraint.DefaultAngularSoftness, Bias = Constraint.DefaultAngularBias.
    void Initialize()
    {
        VerifyCreated();

        FixedAngleData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        data.Softness = DefaultAngularSoftness;
        data.BiasFactor = DefaultAngularBias;
        data.Q0 = body2.Orientation.Conjugate() * body1.Orientation;
    }


    void ResetWarmStart() override
    {
        Data().AccumulatedImpulse = LinearMath::JVector::Zero();
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

        PrepareForIterationFixedAngle(Data(), inverseDt);
    }

    static void PrepareForIterationFixedAngle(FixedAngleData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JQuaternion q1 = body1.Orientation;
        const LinearMath::JQuaternion q2 = body2.Orientation;
        const LinearMath::JQuaternion quat0 = data.Q0 * q1.Conjugate() * q2;

        LinearMath::JVector error(quat0.X, quat0.Y, quat0.Z);
        data.Clamp = 1024;
        data.Jacobian = Detail::ProjectMultiplyLeftRight(data.Q0 * q1.Conjugate(), q2);

        if (quat0.W < static_cast<Real>(0))
        {
            error *= static_cast<Real>(-1);
            data.Jacobian = data.Jacobian * static_cast<Real>(-1);
        }

        data.EffectiveMass = data.Jacobian
            * LinearMath::JMatrix::MultiplyTransposed(
                body1.InverseInertiaWorld + body2.InverseInertiaWorld,
                data.Jacobian);

        data.EffectiveMass.M11 += data.Softness * inverseDt;
        data.EffectiveMass.M22 += data.Softness * inverseDt;
        data.EffectiveMass.M33 += data.Softness * inverseDt;

        LinearMath::JMatrix inverseEffectiveMass;
        LinearMath::JMatrix::Inverse(data.EffectiveMass, inverseEffectiveMass);
        data.EffectiveMass = inverseEffectiveMass;

        data.Bias = -error * data.BiasFactor * inverseDt;

        const LinearMath::JVector angularImpulse =
            LinearMath::JMatrix::TransposedTransform(data.AccumulatedImpulse, data.Jacobian);
        body1.AngularVelocity += body1.InverseInertiaWorld * angularImpulse;
        body2.AngularVelocity -= body2.InverseInertiaWorld * angularImpulse;
    }

    void Iterate(Real inverseDt) override
    {
        if (!IsEnabled())
        {
            return;
        }

        IterateFixedAngle(Data(), inverseDt);
    }

    static void IterateFixedAngle(FixedAngleData& data, Real inverseDt)
    {
        RigidBodyData& body1 = data.Body1.Data();
        RigidBodyData& body2 = data.Body2.Data();

        const LinearMath::JVector jv = data.Jacobian * (body1.AngularVelocity - body2.AngularVelocity);
        const LinearMath::JVector softnessVector = data.AccumulatedImpulse * (data.Softness * inverseDt);
        const LinearMath::JVector lambda = -(data.EffectiveMass * (jv + data.Bias + softnessVector));

        data.AccumulatedImpulse += lambda;

        const LinearMath::JVector angularImpulse =
            LinearMath::JMatrix::TransposedTransform(lambda, data.Jacobian);
        body1.AngularVelocity += body1.InverseInertiaWorld * angularImpulse;
        body2.AngularVelocity -= body2.InverseInertiaWorld * angularImpulse;
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        VerifyCreated();

        const FixedAngleData& data = Data();
        const RigidBodyData& body1 = data.Body1.Data();
        const RigidBodyData& body2 = data.Body2.Data();

        constexpr Real axisLength = static_cast<Real>(0.5);
        const LinearMath::JVector x1 = body1.Orientation.GetBasisX() * axisLength;
        const LinearMath::JVector y1 = body1.Orientation.GetBasisY() * axisLength;
        const LinearMath::JVector z1 = body1.Orientation.GetBasisZ() * axisLength;
        const LinearMath::JVector x2 = body2.Orientation.GetBasisX() * axisLength;
        const LinearMath::JVector y2 = body2.Orientation.GetBasisY() * axisLength;
        const LinearMath::JVector z2 = body2.Orientation.GetBasisZ() * axisLength;

        drawer.DrawSegment(body1.Position, body1.Position + x1);
        drawer.DrawSegment(body1.Position, body1.Position + y1);
        drawer.DrawSegment(body1.Position, body1.Position + z1);
        drawer.DrawSegment(body2.Position, body2.Position + x2);
        drawer.DrawSegment(body2.Position, body2.Position + y2);
        drawer.DrawSegment(body2.Position, body2.Position + z2);
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
            RegisterFullConstraint<FixedAngleData, &PrepareForIterationFixedAngle, &IterateFixedAngle>();
        return id;
    }
};

} // namespace Jitter2::Dynamics::Constraints
