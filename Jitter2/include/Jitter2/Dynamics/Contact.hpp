#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>

#if !defined(JITTER_DOUBLE_PRECISION) || !JITTER_DOUBLE_PRECISION
#if defined(__SSE__) || defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#define JITTER2_CONTACT_USE_SSE 1
#endif
#endif

#include <Jitter2/Dynamics/ArbiterKey.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/LinearMath/JMatrix.hpp>
#include <Jitter2/LinearMath/JQuaternion.hpp>
#include <Jitter2/LinearMath/JVector.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2
{

namespace Detail
{
class VectorReal
{
public:
#if defined(JITTER2_CONTACT_USE_SSE)
    static constexpr bool IsHardwareAccelerated = true;

    VectorReal() : value_(_mm_setzero_ps()) {}
    explicit VectorReal(__m128 value) : value_(value) {}

    static VectorReal Create(Real value)
    {
        return VectorReal(_mm_set1_ps(value));
    }

    static VectorReal Create(Real x, Real y, Real z, Real w)
    {
        return VectorReal(_mm_set_ps(w, z, y, x));
    }

    [[nodiscard]] Real GetElement(int index) const
    {
        alignas(16) Real values[4];
        _mm_store_ps(values, value_);
        return values[index];
    }

    [[nodiscard]] Real Sum3() const
    {
        alignas(16) Real values[4];
        _mm_store_ps(values, value_);
        return values[0] + values[1] + values[2];
    }

    friend VectorReal operator+(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_add_ps(left.value_, right.value_));
    }

    friend VectorReal operator-(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_sub_ps(left.value_, right.value_));
    }

    friend VectorReal operator*(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_mul_ps(left.value_, right.value_));
    }

    friend VectorReal operator/(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_div_ps(left.value_, right.value_));
    }

    [[nodiscard]] static VectorReal Min(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_min_ps(left.value_, right.value_));
    }

    [[nodiscard]] static VectorReal Max(VectorReal left, VectorReal right)
    {
        return VectorReal(_mm_max_ps(left.value_, right.value_));
    }

private:
    __m128 value_;
#else
    static constexpr bool IsHardwareAccelerated = false;

    constexpr VectorReal() : value_ {} {}
    explicit constexpr VectorReal(std::array<Real, 4> value) : value_(value) {}

    static constexpr VectorReal Create(Real value)
    {
        return VectorReal({value, value, value, value});
    }

    static constexpr VectorReal Create(Real x, Real y, Real z, Real w)
    {
        return VectorReal({x, y, z, w});
    }

    [[nodiscard]] constexpr Real GetElement(int index) const
    {
        return value_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] constexpr Real Sum3() const
    {
        return value_[0] + value_[1] + value_[2];
    }

    friend constexpr VectorReal operator+(VectorReal left, VectorReal right)
    {
        return VectorReal({
            left.value_[0] + right.value_[0],
            left.value_[1] + right.value_[1],
            left.value_[2] + right.value_[2],
            left.value_[3] + right.value_[3]});
    }

    friend constexpr VectorReal operator-(VectorReal left, VectorReal right)
    {
        return VectorReal({
            left.value_[0] - right.value_[0],
            left.value_[1] - right.value_[1],
            left.value_[2] - right.value_[2],
            left.value_[3] - right.value_[3]});
    }

    friend constexpr VectorReal operator*(VectorReal left, VectorReal right)
    {
        return VectorReal({
            left.value_[0] * right.value_[0],
            left.value_[1] * right.value_[1],
            left.value_[2] * right.value_[2],
            left.value_[3] * right.value_[3]});
    }

    friend constexpr VectorReal operator/(VectorReal left, VectorReal right)
    {
        return VectorReal({
            left.value_[0] / right.value_[0],
            left.value_[1] / right.value_[1],
            left.value_[2] / right.value_[2],
            left.value_[3] / right.value_[3]});
    }

    [[nodiscard]] static constexpr VectorReal Min(VectorReal left, VectorReal right)
    {
        return VectorReal({
            std::min(left.value_[0], right.value_[0]),
            std::min(left.value_[1], right.value_[1]),
            std::min(left.value_[2], right.value_[2]),
            std::min(left.value_[3], right.value_[3])});
    }

    [[nodiscard]] static constexpr VectorReal Max(VectorReal left, VectorReal right)
    {
        return VectorReal({
            std::max(left.value_[0], right.value_[0]),
            std::max(left.value_[1], right.value_[1]),
            std::max(left.value_[2], right.value_[2]),
            std::max(left.value_[3], right.value_[3])});
    }

private:
    std::array<Real, 4> value_;
#endif
};
} // namespace Detail

struct ContactData
{
    enum class SolveMode : unsigned int
    {
        // No velocity updates.
        None = 0,
        // Update linear velocity of body 1.
        LinearBody1 = 1u << 0,
        // Update angular velocity of body 1.
        AngularBody1 = 1u << 1,
        // Update linear velocity of body 2.
        LinearBody2 = 1u << 2,
        // Update angular velocity of body 2.
        AngularBody2 = 1u << 3,
        // Update both linear and angular velocity of body 1.
        FullBody1 = (1u << 0) | (1u << 1),
        // Update both linear and angular velocity of body 2.
        FullBody2 = (1u << 2) | (1u << 3),
        // Update linear velocities of both bodies.
        Linear = (1u << 0) | (1u << 2),
        // Update angular velocities of both bodies.
        Angular = (1u << 1) | (1u << 3),
        // Update all velocity components of both bodies.
        Full = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3),
    };

    static constexpr unsigned int MaskContact0 = 0b0001;
    static constexpr unsigned int MaskContact1 = 0b0010;
    static constexpr unsigned int MaskContact2 = 0b0100;
    static constexpr unsigned int MaskContact3 = 0b1000;
    // Bit mask for contact slot 3.
    static constexpr unsigned int MaskContactAll = MaskContact0 | MaskContact1 | MaskContact2 | MaskContact3;

    struct Contact
    {
        static constexpr Real MaximumBias = static_cast<Real>(100);
        static constexpr Real BiasFactor = static_cast<Real>(0.2);
        static constexpr Real AllowedPenetration = static_cast<Real>(0.01);
        static constexpr Real BreakThreshold = static_cast<Real>(0.02);

        enum class Flags : unsigned int
        {
            None = 0,
            // Indicates this contact was created in the current step.
            NewContact = 1u << 1,
        };

        Flags Flag = Flags::None;
        Real Bias = static_cast<Real>(0);
        Real PenaltyBias = static_cast<Real>(0);

        Detail::VectorReal NormalTangentX = Detail::VectorReal::Create(static_cast<Real>(0));
        Detail::VectorReal NormalTangentY = Detail::VectorReal::Create(static_cast<Real>(0));
        Detail::VectorReal NormalTangentZ = Detail::VectorReal::Create(static_cast<Real>(0));
        Detail::VectorReal MassNormalTangent = Detail::VectorReal::Create(static_cast<Real>(0));
        Detail::VectorReal Accumulated = Detail::VectorReal::Create(static_cast<Real>(0));

        LinearMath::JVector Normal = LinearMath::JVector::Zero();
        LinearMath::JVector Tangent1 = LinearMath::JVector::Zero();
        LinearMath::JVector Tangent2 = LinearMath::JVector::Zero();

        Real MassNormal = static_cast<Real>(0);
        Real MassTangent1 = static_cast<Real>(0);
        Real MassTangent2 = static_cast<Real>(0);

        Real AccumulatedNormalImpulse = static_cast<Real>(0);
        Real AccumulatedTangentImpulse1 = static_cast<Real>(0);
        Real AccumulatedTangentImpulse2 = static_cast<Real>(0);

        LinearMath::JVector Position1 = LinearMath::JVector::Zero();
        LinearMath::JVector Position2 = LinearMath::JVector::Zero();
        LinearMath::JVector RelativePosition1 = LinearMath::JVector::Zero();
        LinearMath::JVector RelativePosition2 = LinearMath::JVector::Zero();

        void Initialize(
            RigidBodyData& body1,
            RigidBodyData& body2,
            const LinearMath::JVector& point1,
            const LinearMath::JVector& point2,
            const LinearMath::JVector& normal,
            bool newContact,
            Real restitution)
        {
            RelativePosition1 = point1 - body1.Position;
            RelativePosition2 = point2 - body2.Position;
            Position1 = LinearMath::JQuaternion::ConjugatedTransform(RelativePosition1, body1.Orientation);
            Position2 = LinearMath::JQuaternion::ConjugatedTransform(RelativePosition2, body2.Orientation);

            if (!newContact)
            {
                return;
            }

            Flag = Flags::NewContact;
            AccumulatedNormalImpulse = static_cast<Real>(0);
            AccumulatedTangentImpulse1 = static_cast<Real>(0);
            AccumulatedTangentImpulse2 = static_cast<Real>(0);
            Accumulated = Detail::VectorReal::Create(static_cast<Real>(0));

            LinearMath::JVector deltaVelocity =
                body2.Velocity + LinearMath::JVector::Cross(body2.AngularVelocity, RelativePosition2);
            deltaVelocity -= body1.Velocity + LinearMath::JVector::Cross(body1.AngularVelocity, RelativePosition1);

            const Real relNormalVelocity = LinearMath::JVector::Dot(deltaVelocity, normal);

            Bias = static_cast<Real>(0);
            if (relNormalVelocity < static_cast<Real>(-1))
            {
                Bias = -restitution * relNormalVelocity;
            }

            LinearMath::JVector tangent1 = deltaVelocity - normal * relNormalVelocity;
            Real lengthSq = tangent1.LengthSquared();

            if (lengthSq > static_cast<Real>(1e-12))
            {
                tangent1 *= static_cast<Real>(1) / std::sqrt(lengthSq);
            }
            else
            {
                tangent1 = LinearMath::MathHelper::CreateOrthonormal(normal);
            }

            const LinearMath::JVector tangent2 = LinearMath::JVector::Cross(tangent1, normal);

            Normal = normal;
            Tangent1 = tangent1;
            Tangent2 = tangent2;
            NormalTangentX = Detail::VectorReal::Create(normal.X, tangent1.X, tangent2.X, static_cast<Real>(0));
            NormalTangentY = Detail::VectorReal::Create(normal.Y, tangent1.Y, tangent2.Y, static_cast<Real>(0));
            NormalTangentZ = Detail::VectorReal::Create(normal.Z, tangent1.Z, tangent2.Z, static_cast<Real>(0));
        }

        [[nodiscard]] bool UpdatePosition(const ContactData& contactData) const
        {
            RigidBodyData& body1 = contactData.Body1.Data();
            RigidBodyData& body2 = contactData.Body2.Data();

            const LinearMath::JVector relativePos1 =
                LinearMath::JQuaternion::Transform(Position1, body1.Orientation);
            const LinearMath::JVector point1 = relativePos1 + body1.Position;

            const LinearMath::JVector relativePos2 =
                LinearMath::JQuaternion::Transform(Position2, body2.Orientation);
            const LinearMath::JVector point2 = relativePos2 + body2.Position;

            LinearMath::JVector dist = point1 - point2;
            const Real penetration = LinearMath::JVector::Dot(dist, Normal);

            if (penetration < -BreakThreshold * static_cast<Real>(0.1))
            {
                return false;
            }

            dist -= penetration * Normal;
            return dist.LengthSquared() <= BreakThreshold * BreakThreshold;
        }

        void PrepareForIteration(ContactData& contactData, Real inverseDt)
        {
            RigidBodyData& body1 = contactData.Body1.Data();
            RigidBodyData& body2 = contactData.Body2.Data();

            RelativePosition1 = LinearMath::JQuaternion::Transform(Position1, body1.Orientation);
            RelativePosition2 = LinearMath::JQuaternion::Transform(Position2, body2.Orientation);

            const Real inverseMass = body1.InverseMass + body2.InverseMass;

            if ((static_cast<unsigned int>(Flag) & static_cast<unsigned int>(Flags::NewContact)) == 0)
            {
                Bias = static_cast<Real>(0);
            }

            const LinearMath::JVector point1 = RelativePosition1 + body1.Position;
            const LinearMath::JVector point2 = RelativePosition2 + body2.Position;
            const LinearMath::JVector dist = point1 - point2;
            const Real penetration = LinearMath::JVector::Dot(dist, Normal);

            if (penetration < -BreakThreshold)
            {
                Bias = penetration * inverseDt * contactData.SpeculativeRelaxationFactor;
            }

            Flag = static_cast<Flags>(
                static_cast<unsigned int>(Flag) & ~static_cast<unsigned int>(Flags::NewContact));

            Real kTangent1 = inverseMass;
            Real kTangent2 = inverseMass;
            Real kNormal = inverseMass;
            LinearMath::JVector angularNormalContribution = LinearMath::JVector::Zero();
            LinearMath::JVector angularTangent1Contribution = LinearMath::JVector::Zero();
            LinearMath::JVector angularTangent2Contribution = LinearMath::JVector::Zero();

            LinearMath::JVector impulse = AccumulatedNormalImpulse * Normal;
            impulse += AccumulatedTangentImpulse1 * Tangent1;
            impulse += AccumulatedTangentImpulse2 * Tangent2;

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody1))
            {
                body1.Velocity -= impulse * body1.InverseMass;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody1))
            {
                const LinearMath::JVector angularNormal =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Normal),
                        body1.InverseInertiaWorld);
                const LinearMath::JVector angularTangent1 =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Tangent1),
                        body1.InverseInertiaWorld);
                const LinearMath::JVector angularTangent2 =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Tangent2),
                        body1.InverseInertiaWorld);

                angularNormalContribution += LinearMath::JVector::Cross(angularNormal, RelativePosition1);
                angularTangent1Contribution += LinearMath::JVector::Cross(angularTangent1, RelativePosition1);
                angularTangent2Contribution += LinearMath::JVector::Cross(angularTangent2, RelativePosition1);

                LinearMath::JVector angularImpulse = AccumulatedNormalImpulse * angularNormal;
                angularImpulse += AccumulatedTangentImpulse1 * angularTangent1;
                angularImpulse += AccumulatedTangentImpulse2 * angularTangent2;
                body1.AngularVelocity -= angularImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody2))
            {
                body2.Velocity += impulse * body2.InverseMass;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody2))
            {
                const LinearMath::JVector angularNormal =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Normal),
                        body2.InverseInertiaWorld);
                const LinearMath::JVector angularTangent1 =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Tangent1),
                        body2.InverseInertiaWorld);
                const LinearMath::JVector angularTangent2 =
                    ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Tangent2),
                        body2.InverseInertiaWorld);

                angularNormalContribution += LinearMath::JVector::Cross(angularNormal, RelativePosition2);
                angularTangent1Contribution += LinearMath::JVector::Cross(angularTangent1, RelativePosition2);
                angularTangent2Contribution += LinearMath::JVector::Cross(angularTangent2, RelativePosition2);

                LinearMath::JVector angularImpulse = AccumulatedNormalImpulse * angularNormal;
                angularImpulse += AccumulatedTangentImpulse1 * angularTangent1;
                angularImpulse += AccumulatedTangentImpulse2 * angularTangent2;
                body2.AngularVelocity += angularImpulse;
            }

            kNormal += LinearMath::JVector::Dot(Normal, angularNormalContribution);
            kTangent1 += LinearMath::JVector::Dot(Tangent1, angularTangent1Contribution);
            kTangent2 += LinearMath::JVector::Dot(Tangent2, angularTangent2Contribution);

            MassNormal = static_cast<Real>(1) / kNormal;
            MassTangent1 = static_cast<Real>(1) / kTangent1;
            MassTangent2 = static_cast<Real>(1) / kTangent2;

            PenaltyBias = BiasFactor * inverseDt * std::max(static_cast<Real>(0), penetration - AllowedPenetration);
            PenaltyBias = std::min(PenaltyBias, MaximumBias);
        }

        void Iterate(ContactData& contactData, bool applyBias)
        {
            RigidBodyData& body1 = contactData.Body1.Data();
            RigidBodyData& body2 = contactData.Body2.Data();

            LinearMath::JVector deltaVelocity =
                body2.Velocity + LinearMath::JVector::Cross(body2.AngularVelocity, RelativePosition2);
            deltaVelocity -= body1.Velocity + LinearMath::JVector::Cross(body1.AngularVelocity, RelativePosition1);

            const Real bias = applyBias ? std::max(PenaltyBias, Bias) : Bias;

            const Real normalVelocity = LinearMath::JVector::Dot(Normal, deltaVelocity);
            const Real tangentVelocity1 = LinearMath::JVector::Dot(Tangent1, deltaVelocity);
            const Real tangentVelocity2 = LinearMath::JVector::Dot(Tangent2, deltaVelocity);

            Real normalImpulse = (-normalVelocity + bias) * MassNormal;

            const Real oldNormalImpulse = AccumulatedNormalImpulse;
            AccumulatedNormalImpulse = std::max(oldNormalImpulse + normalImpulse, static_cast<Real>(0));
            normalImpulse = AccumulatedNormalImpulse - oldNormalImpulse;

            const Real maxTangentImpulse = contactData.Friction * oldNormalImpulse;
            Real tangentImpulse1 = MassTangent1 * -tangentVelocity1;
            Real tangentImpulse2 = MassTangent2 * -tangentVelocity2;

            const Real oldTangentImpulse1 = AccumulatedTangentImpulse1;
            AccumulatedTangentImpulse1 = std::clamp(
                oldTangentImpulse1 + tangentImpulse1,
                -maxTangentImpulse,
                maxTangentImpulse);
            tangentImpulse1 = AccumulatedTangentImpulse1 - oldTangentImpulse1;

            const Real oldTangentImpulse2 = AccumulatedTangentImpulse2;
            AccumulatedTangentImpulse2 = std::clamp(
                oldTangentImpulse2 + tangentImpulse2,
                -maxTangentImpulse,
                maxTangentImpulse);
            tangentImpulse2 = AccumulatedTangentImpulse2 - oldTangentImpulse2;

            LinearMath::JVector impulse = normalImpulse * Normal;
            impulse += tangentImpulse1 * Tangent1;
            impulse += tangentImpulse2 * Tangent2;

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody1))
            {
                body1.Velocity -= body1.InverseMass * impulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody1))
            {
                LinearMath::JVector angularImpulse = normalImpulse
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Normal),
                        body1.InverseInertiaWorld);
                angularImpulse += tangentImpulse1
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Tangent1),
                        body1.InverseInertiaWorld);
                angularImpulse += tangentImpulse2
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition1, Tangent2),
                        body1.InverseInertiaWorld);

                body1.AngularVelocity -= angularImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody2))
            {
                body2.Velocity += body2.InverseMass * impulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody2))
            {
                LinearMath::JVector angularImpulse = normalImpulse
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Normal),
                        body2.InverseInertiaWorld);
                angularImpulse += tangentImpulse1
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Tangent1),
                        body2.InverseInertiaWorld);
                angularImpulse += tangentImpulse2
                    * ContactData::TransformSymmetricInertia(
                        LinearMath::JVector::Cross(RelativePosition2, Tangent2),
                        body2.InverseInertiaWorld);

                body2.AngularVelocity += angularImpulse;
            }
        }

        static Real GetSum3(Detail::VectorReal vector)
        {
            return vector.Sum3();
        }

        static LinearMath::JVector SumBasis(
            Detail::VectorReal impulse,
            Detail::VectorReal basisX,
            Detail::VectorReal basisY,
            Detail::VectorReal basisZ)
        {
            return LinearMath::JVector(
                GetSum3(impulse * basisX),
                GetSum3(impulse * basisY),
                GetSum3(impulse * basisZ));
        }

        void SyncScalarMasses()
        {
            MassNormal = MassNormalTangent.GetElement(0);
            MassTangent1 = MassNormalTangent.GetElement(1);
            MassTangent2 = MassNormalTangent.GetElement(2);
        }

        void SyncScalarAccumulated()
        {
            AccumulatedNormalImpulse = Accumulated.GetElement(0);
            AccumulatedTangentImpulse1 = Accumulated.GetElement(1);
            AccumulatedTangentImpulse2 = Accumulated.GetElement(2);
        }

        void PrepareForIterationAccelerated(ContactData& contactData, Real inverseDt)
        {
            RigidBodyData& body1 = contactData.Body1.Data();
            RigidBodyData& body2 = contactData.Body2.Data();

            RelativePosition1 = LinearMath::JQuaternion::Transform(Position1, body1.Orientation);
            RelativePosition2 = LinearMath::JQuaternion::Transform(Position2, body2.Orientation);

            Detail::VectorReal kNormalTangent = Detail::VectorReal::Create(body1.InverseMass + body2.InverseMass);

            if ((static_cast<unsigned int>(Flag) & static_cast<unsigned int>(Flags::NewContact)) == 0)
            {
                Bias = static_cast<Real>(0);
            }

            const LinearMath::JVector point1 = RelativePosition1 + body1.Position;
            const LinearMath::JVector point2 = RelativePosition2 + body2.Position;
            const LinearMath::JVector dist = point1 - point2;

            const LinearMath::JVector normal(
                NormalTangentX.GetElement(0),
                NormalTangentY.GetElement(0),
                NormalTangentZ.GetElement(0));
            const Real penetration = LinearMath::JVector::Dot(dist, normal);

            if (penetration < -BreakThreshold)
            {
                Bias = penetration * inverseDt * contactData.SpeculativeRelaxationFactor;
            }

            Flag = static_cast<Flags>(
                static_cast<unsigned int>(Flag) & ~static_cast<unsigned int>(Flags::NewContact));

            Detail::VectorReal ktnx = Detail::VectorReal::Create(static_cast<Real>(0));
            Detail::VectorReal ktny = Detail::VectorReal::Create(static_cast<Real>(0));
            Detail::VectorReal ktnz = Detail::VectorReal::Create(static_cast<Real>(0));

            const LinearMath::JVector linearImpulse =
                SumBasis(Accumulated, NormalTangentX, NormalTangentY, NormalTangentZ);

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody1))
            {
                body1.Velocity -= body1.InverseMass * linearImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody1))
            {
                const Detail::VectorReal rp1X = Detail::VectorReal::Create(RelativePosition1.X);
                const Detail::VectorReal rp1Y = Detail::VectorReal::Create(RelativePosition1.Y);
                const Detail::VectorReal rp1Z = Detail::VectorReal::Create(RelativePosition1.Z);

                const Detail::VectorReal rrx = rp1Y * NormalTangentZ - rp1Z * NormalTangentY;
                const Detail::VectorReal rry = rp1Z * NormalTangentX - rp1X * NormalTangentZ;
                const Detail::VectorReal rrz = rp1X * NormalTangentY - rp1Y * NormalTangentX;

                const Detail::VectorReal ixx = Detail::VectorReal::Create(body1.InverseInertiaWorld.M11);
                const Detail::VectorReal ixy = Detail::VectorReal::Create(body1.InverseInertiaWorld.M21);
                const Detail::VectorReal ixz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M31);
                const Detail::VectorReal iyy = Detail::VectorReal::Create(body1.InverseInertiaWorld.M22);
                const Detail::VectorReal iyz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M23);
                const Detail::VectorReal izz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M33);

                const Detail::VectorReal e1 = ixx * rrx + ixy * rry + ixz * rrz;
                const Detail::VectorReal e2 = ixy * rrx + iyy * rry + iyz * rrz;
                const Detail::VectorReal e3 = ixz * rrx + iyz * rry + izz * rrz;

                const LinearMath::JVector angularImpulse1 =
                    SumBasis(Accumulated, e1, e2, e3);

                body1.AngularVelocity -= angularImpulse1;

                ktnx = e2 * rp1Z - e3 * rp1Y;
                ktny = e3 * rp1X - e1 * rp1Z;
                ktnz = e1 * rp1Y - e2 * rp1X;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody2))
            {
                body2.Velocity += body2.InverseMass * linearImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody2))
            {
                const Detail::VectorReal rp2X = Detail::VectorReal::Create(RelativePosition2.X);
                const Detail::VectorReal rp2Y = Detail::VectorReal::Create(RelativePosition2.Y);
                const Detail::VectorReal rp2Z = Detail::VectorReal::Create(RelativePosition2.Z);

                const Detail::VectorReal rrx = rp2Y * NormalTangentZ - rp2Z * NormalTangentY;
                const Detail::VectorReal rry = rp2Z * NormalTangentX - rp2X * NormalTangentZ;
                const Detail::VectorReal rrz = rp2X * NormalTangentY - rp2Y * NormalTangentX;

                const Detail::VectorReal ixx = Detail::VectorReal::Create(body2.InverseInertiaWorld.M11);
                const Detail::VectorReal ixy = Detail::VectorReal::Create(body2.InverseInertiaWorld.M21);
                const Detail::VectorReal ixz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M31);
                const Detail::VectorReal iyy = Detail::VectorReal::Create(body2.InverseInertiaWorld.M22);
                const Detail::VectorReal iyz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M23);
                const Detail::VectorReal izz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M33);

                const Detail::VectorReal f1 = ixx * rrx + ixy * rry + ixz * rrz;
                const Detail::VectorReal f2 = ixy * rrx + iyy * rry + iyz * rrz;
                const Detail::VectorReal f3 = ixz * rrx + iyz * rry + izz * rrz;

                const LinearMath::JVector angularImpulse2 =
                    SumBasis(Accumulated, f1, f2, f3);

                body2.AngularVelocity += angularImpulse2;

                ktnx = ktnx + f2 * rp2Z - f3 * rp2Y;
                ktny = ktny + f3 * rp2X - f1 * rp2Z;
                ktnz = ktnz + f1 * rp2Y - f2 * rp2X;
            }

            const Detail::VectorReal kres =
                NormalTangentX * ktnx + NormalTangentY * ktny + NormalTangentZ * ktnz;
            kNormalTangent = kNormalTangent + kres;

            MassNormalTangent = Detail::VectorReal::Create(static_cast<Real>(1)) / kNormalTangent;
            SyncScalarMasses();

            PenaltyBias = BiasFactor * inverseDt * std::max(static_cast<Real>(0), penetration - AllowedPenetration);
            PenaltyBias = std::min(PenaltyBias, MaximumBias);
        }

        void IterateAccelerated(ContactData& contactData, bool applyBias)
        {
            RigidBodyData& body1 = contactData.Body1.Data();
            RigidBodyData& body2 = contactData.Body2.Data();

            LinearMath::JVector deltaVelocity =
                body2.Velocity + LinearMath::JVector::Cross(body2.AngularVelocity, RelativePosition2);
            deltaVelocity -= body1.Velocity + LinearMath::JVector::Cross(body1.AngularVelocity, RelativePosition1);

            const Real bias = applyBias ? std::max(PenaltyBias, Bias) : Bias;

            const Detail::VectorReal vdots =
                NormalTangentX * Detail::VectorReal::Create(deltaVelocity.X)
                + NormalTangentY * Detail::VectorReal::Create(deltaVelocity.Y)
                + NormalTangentZ * Detail::VectorReal::Create(deltaVelocity.Z);

            Detail::VectorReal impulse =
                MassNormalTangent
                * (Detail::VectorReal::Create(bias, static_cast<Real>(0), static_cast<Real>(0), static_cast<Real>(0)) - vdots);
            const Detail::VectorReal oldImpulse = Accumulated;

            const Real maxTangentImpulse = contactData.Friction * Accumulated.GetElement(0);
            Accumulated = oldImpulse + impulse;

            const Detail::VectorReal minImpulse = Detail::VectorReal::Create(
                static_cast<Real>(0),
                -maxTangentImpulse,
                -maxTangentImpulse,
                static_cast<Real>(0));
            const Detail::VectorReal maxImpulse = Detail::VectorReal::Create(
                std::numeric_limits<Real>::max(),
                maxTangentImpulse,
                maxTangentImpulse,
                static_cast<Real>(0));

            Accumulated = Detail::VectorReal::Min(Detail::VectorReal::Max(Accumulated, minImpulse), maxImpulse);
            impulse = Accumulated - oldImpulse;
            SyncScalarAccumulated();

            const LinearMath::JVector linearImpulse =
                SumBasis(impulse, NormalTangentX, NormalTangentY, NormalTangentZ);

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody1))
            {
                body1.Velocity -= body1.InverseMass * linearImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody1))
            {
                const Detail::VectorReal rp1X = Detail::VectorReal::Create(RelativePosition1.X);
                const Detail::VectorReal rp1Y = Detail::VectorReal::Create(RelativePosition1.Y);
                const Detail::VectorReal rp1Z = Detail::VectorReal::Create(RelativePosition1.Z);

                const Detail::VectorReal rrx = rp1Y * NormalTangentZ - rp1Z * NormalTangentY;
                const Detail::VectorReal rry = rp1Z * NormalTangentX - rp1X * NormalTangentZ;
                const Detail::VectorReal rrz = rp1X * NormalTangentY - rp1Y * NormalTangentX;

                const Detail::VectorReal ixx = Detail::VectorReal::Create(body1.InverseInertiaWorld.M11);
                const Detail::VectorReal ixy = Detail::VectorReal::Create(body1.InverseInertiaWorld.M21);
                const Detail::VectorReal ixz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M31);
                const Detail::VectorReal iyy = Detail::VectorReal::Create(body1.InverseInertiaWorld.M22);
                const Detail::VectorReal iyz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M23);
                const Detail::VectorReal izz = Detail::VectorReal::Create(body1.InverseInertiaWorld.M33);

                const Detail::VectorReal e1 = ixx * rrx + ixy * rry + ixz * rrz;
                const Detail::VectorReal e2 = ixy * rrx + iyy * rry + iyz * rrz;
                const Detail::VectorReal e3 = ixz * rrx + iyz * rry + izz * rrz;

                const LinearMath::JVector angularImpulse1 = SumBasis(impulse, e1, e2, e3);
                body1.AngularVelocity -= angularImpulse1;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::LinearBody2))
            {
                body2.Velocity += body2.InverseMass * linearImpulse;
            }

            if (ContactData::Has(contactData.Mode, SolveMode::AngularBody2))
            {
                const Detail::VectorReal rp2X = Detail::VectorReal::Create(RelativePosition2.X);
                const Detail::VectorReal rp2Y = Detail::VectorReal::Create(RelativePosition2.Y);
                const Detail::VectorReal rp2Z = Detail::VectorReal::Create(RelativePosition2.Z);

                const Detail::VectorReal rrx = rp2Y * NormalTangentZ - rp2Z * NormalTangentY;
                const Detail::VectorReal rry = rp2Z * NormalTangentX - rp2X * NormalTangentZ;
                const Detail::VectorReal rrz = rp2X * NormalTangentY - rp2Y * NormalTangentX;

                const Detail::VectorReal ixx = Detail::VectorReal::Create(body2.InverseInertiaWorld.M11);
                const Detail::VectorReal ixy = Detail::VectorReal::Create(body2.InverseInertiaWorld.M21);
                const Detail::VectorReal ixz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M31);
                const Detail::VectorReal iyy = Detail::VectorReal::Create(body2.InverseInertiaWorld.M22);
                const Detail::VectorReal iyz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M23);
                const Detail::VectorReal izz = Detail::VectorReal::Create(body2.InverseInertiaWorld.M33);

                const Detail::VectorReal f1 = ixx * rrx + ixy * rry + ixz * rrz;
                const Detail::VectorReal f2 = ixy * rrx + iyy * rry + iyz * rrz;
                const Detail::VectorReal f3 = ixz * rrx + iyz * rry + izz * rrz;

                const LinearMath::JVector angularImpulse2 = SumBasis(impulse, f1, f2, f3);
                body2.AngularVelocity += angularImpulse2;
            }
        }
    };

    int _internal = 0;
    unsigned int UsageMask = 0;
    Unmanaged::JHandle<RigidBodyData> Body1;
    Unmanaged::JHandle<RigidBodyData> Body2;
    ArbiterKey Key;
    Real Restitution = static_cast<Real>(0);
    Real Friction = static_cast<Real>(0);
    Real SpeculativeRelaxationFactor = static_cast<Real>(0.9);
    SolveMode Mode = SolveMode::None;
    std::array<Contact, 4> Contacts {};

    void Init(RigidBody& body1, RigidBody& body2, Real speculativeRelaxationFactor = static_cast<Real>(0.9))
    {
        Body1 = body1.Handle();
        Body2 = body2.Handle();
        Friction = std::max(body1.Friction(), body2.Friction());
        Restitution = std::max(body1.Restitution(), body2.Restitution());
        SpeculativeRelaxationFactor = speculativeRelaxationFactor;
        Mode = SolveMode::None;
        UsageMask = 0;
    }

    void ResetMode(SolveMode removeFlags = SolveMode::None)
    {
        unsigned int mode = Body1.Data().MotionTypeValue() == MotionType::Dynamic
            ? static_cast<unsigned int>(SolveMode::FullBody1)
            : 0;
        mode |= Body2.Data().MotionTypeValue() == MotionType::Dynamic
            ? static_cast<unsigned int>(SolveMode::FullBody2)
            : 0;
        mode &= ~static_cast<unsigned int>(removeFlags);
        Mode = static_cast<SolveMode>(mode);
    }

    // Prepares all active contacts for solver iterations by computing effective masses and warm-starting impulses.
    // idt: Inverse of the timestep (1/dt).
    void PrepareForIteration(Real inverseDt)
    {
        for (int i = 0; i < 4; ++i)
        {
            if ((UsageMask & (1u << i)) != 0)
            {
                if constexpr (Detail::VectorReal::IsHardwareAccelerated)
                {
                    Contacts[static_cast<std::size_t>(i)].PrepareForIterationAccelerated(*this, inverseDt);
                }
                else
                {
                    Contacts[static_cast<std::size_t>(i)].PrepareForIteration(*this, inverseDt);
                }
            }
        }
    }

    // Performs one solver iteration over all active contacts, applying corrective impulses.
    // applyBias: If true, applies position-correction bias.
    void Iterate(bool applyBias)
    {
        for (int i = 0; i < 4; ++i)
        {
            if ((UsageMask & (1u << i)) != 0)
            {
                if constexpr (Detail::VectorReal::IsHardwareAccelerated)
                {
                    Contacts[static_cast<std::size_t>(i)].IterateAccelerated(*this, applyBias);
                }
                else
                {
                    Contacts[static_cast<std::size_t>(i)].Iterate(*this, applyBias);
                }
            }
        }
    }

    // Updates contact positions after integration and removes contacts that have separated beyond the break threshold.
    void UpdatePosition()
    {
        // Bit mask indicating all four contact slots are in use.
        UsageMask &= MaskContactAll;
        UsageMask |= UsageMask << 4;

        for (int i = 0; i < 4; ++i)
        {
            const unsigned int mask = 1u << i;
            if ((UsageMask & mask) != 0
                && !Contacts[static_cast<std::size_t>(i)].UpdatePosition(*this))
            {
                UsageMask &= ~mask;
            }
        }
    }

    void AddContact(
        const LinearMath::JVector& point1,
        const LinearMath::JVector& point2,
        const LinearMath::JVector& normal)
    {
        if ((UsageMask & MaskContactAll) == MaskContactAll)
        {
            SortCachedPoints(point1, point2, normal);
            return;
        }

        Contact* closest = nullptr;
        Real distanceSq = std::numeric_limits<Real>::max();

        RigidBodyData& body1 = Body1.Data();
        RigidBodyData& body2 = Body2.Data();

        const LinearMath::JVector relP1 = point1 - body1.Position;

        for (int i = 0; i < 4; ++i)
        {
            if ((UsageMask & (1u << i)) == 0)
            {
                continue;
            }

            const Real distSq = (Contacts[static_cast<std::size_t>(i)].RelativePosition1 - relP1).LengthSquared();
            if (distSq < distanceSq)
            {
                distanceSq = distSq;
                closest = &Contacts[static_cast<std::size_t>(i)];
            }
        }

        if (closest != nullptr && distanceSq < Contact::BreakThreshold * Contact::BreakThreshold)
        {
            closest->Initialize(body1, body2, point1, point2, normal, false, Restitution);
            return;
        }

        for (int i = 0; i < 4; ++i)
        {
            const unsigned int mask = 1u << i;
            if ((UsageMask & mask) == 0)
            {
                Contacts[static_cast<std::size_t>(i)].Initialize(body1, body2, point1, point2, normal, true, Restitution);
                UsageMask |= mask;
                return;
            }
        }
    }

    static bool Has(SolveMode mode, SolveMode flag)
    {
        return (static_cast<unsigned int>(mode) & static_cast<unsigned int>(flag)) != 0;
    }

private:
    static LinearMath::JVector TransformSymmetricInertia(
        const LinearMath::JVector& vector,
        const LinearMath::JMatrix& matrix)
    {
        return LinearMath::JVector(
            vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31,
            vector.X * matrix.M21 + vector.Y * matrix.M22 + vector.Z * matrix.M23,
            vector.X * matrix.M31 + vector.Y * matrix.M23 + vector.Z * matrix.M33);
    }

    static Real CalcArea4Points(
        const LinearMath::JVector& p0,
        const LinearMath::JVector& p1,
        const LinearMath::JVector& p2,
        const LinearMath::JVector& p3)
    {
        const LinearMath::JVector a0 = p0 - p1;
        const LinearMath::JVector a1 = p0 - p2;
        const LinearMath::JVector a2 = p0 - p3;
        const LinearMath::JVector b0 = p2 - p3;
        const LinearMath::JVector b1 = p1 - p3;
        const LinearMath::JVector b2 = p1 - p2;

        const LinearMath::JVector tmp0 = LinearMath::JVector::Cross(a0, b0);
        const LinearMath::JVector tmp1 = LinearMath::JVector::Cross(a1, b1);
        const LinearMath::JVector tmp2 = LinearMath::JVector::Cross(a2, b2);

        return std::max({tmp0.LengthSquared(), tmp1.LengthSquared(), tmp2.LengthSquared()});
    }

    static unsigned int SelectCachedPointReplacementMask(
        Real area0,
        Real area1,
        Real area2,
        Real area3)
    {
        const Real epsilon = static_cast<Real>(-0.0001);

        Real biggestArea = static_cast<Real>(0);
        unsigned int mask = 0;

        if (area0 > biggestArea + epsilon)
        {
            biggestArea = area0;
            // Bit mask for contact slot 0.
            mask = MaskContact0;
        }

        if (area1 > biggestArea + epsilon)
        {
            biggestArea = area1;
            // Bit mask for contact slot 1.
            mask = MaskContact1;
        }

        if (area2 > biggestArea + epsilon)
        {
            biggestArea = area2;
            // Bit mask for contact slot 2.
            mask = MaskContact2;
        }

        if (area3 > biggestArea + epsilon)
        {
            mask = MaskContact3;
        }

        return mask;
    }

    static unsigned int SelectCachedPointReplacementMask(
        const LinearMath::JVector& newPoint,
        const LinearMath::JVector& contact0,
        const LinearMath::JVector& contact1,
        const LinearMath::JVector& contact2,
        const LinearMath::JVector& contact3)
    {
        const Real area0 = CalcArea4Points(newPoint, contact1, contact2, contact3);
        const Real area1 = CalcArea4Points(newPoint, contact0, contact2, contact3);
        const Real area2 = CalcArea4Points(newPoint, contact0, contact1, contact3);
        const Real area3 = CalcArea4Points(newPoint, contact0, contact1, contact2);

        return SelectCachedPointReplacementMask(area0, area1, area2, area3);
    }

    void SortCachedPoints(
        const LinearMath::JVector& point1,
        const LinearMath::JVector& point2,
        const LinearMath::JVector& normal)
    {
        RigidBodyData& body1 = Body1.Data();
        RigidBodyData& body2 = Body2.Data();

        const LinearMath::JVector relativePoint1 = point1 - body1.Position;
        const unsigned int mask = SelectCachedPointReplacementMask(
            relativePoint1,
            Contacts[0].RelativePosition1,
            Contacts[1].RelativePosition1,
            Contacts[2].RelativePosition1,
            Contacts[3].RelativePosition1);

        int index = 0;
        if (mask == MaskContact1)
        {
            index = 1;
        }
        else if (mask == MaskContact2)
        {
            index = 2;
        }
        else if (mask == MaskContact3)
        {
            index = 3;
        }

        Contacts[static_cast<std::size_t>(index)].Initialize(body1, body2, point1, point2, normal, false, Restitution);
        UsageMask |= mask;
    }
};

inline ContactData::SolveMode operator|(ContactData::SolveMode left, ContactData::SolveMode right)
{
    return static_cast<ContactData::SolveMode>(
        static_cast<unsigned int>(left) | static_cast<unsigned int>(right));
}

inline ContactData::SolveMode operator&(ContactData::SolveMode left, ContactData::SolveMode right)
{
    return static_cast<ContactData::SolveMode>(
        static_cast<unsigned int>(left) & static_cast<unsigned int>(right));
}

inline ContactData::SolveMode operator~(ContactData::SolveMode value)
{
    return static_cast<ContactData::SolveMode>(~static_cast<unsigned int>(value));
}

} // namespace Jitter2
