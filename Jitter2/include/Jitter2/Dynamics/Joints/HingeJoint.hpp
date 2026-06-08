#pragma once

#include <Jitter2/Dynamics/Constraints/AngularMotor.hpp>
#include <Jitter2/Dynamics/Constraints/BallSocket.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/HingeAngle.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/Dynamics/Joints/Joint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Constructs a hinge joint utilizing a HingeAngle, a BallSocket, and an optional AngularMotor.
class HingeJoint : public Joint
{
public:

    // Initializes a new hinge joint.
    // Thrown when hingeCenter contains a non-finite value, when
    // hingeAxis is zero or contains a non-finite value, when either body does not belong
    // to world, or when both body references are the same.

    HingeJoint(
        World& world,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& hingeCenter,
        LinearMath::JVector hingeAxis,
        bool hasMotor = false)
        : HingeJoint(world, body1, body2, hingeCenter, hingeAxis, AngularLimit::Full(), hasMotor)
    {
    }


    HingeJoint(
        World& world,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& hingeCenter,
        LinearMath::JVector hingeAxis,
        const AngularLimit& angle,
        bool hasMotor = false)
        : body1_(&body1),
          body2_(&body2)
    {
        Detail::CheckFinite(hingeCenter, "hingeCenter");
        Detail::CheckNonZero(hingeAxis, "hingeAxis");
        hingeAxis.Normalize();

        hingeAngle_ = &world.CreateConstraint<HingeAngle>(body1, body2);
        hingeAngle_->Initialize(hingeAxis, angle);
        Register(*hingeAngle_);

        ballSocket_ = &world.CreateConstraint<BallSocket>(body1, body2);
        ballSocket_->Initialize(hingeCenter);
        Register(*ballSocket_);

        if (hasMotor)
        {
            motor_ = &world.CreateConstraint<AngularMotor>(body1, body2);
            motor_->Initialize(hingeAxis);
            Register(*motor_);
        }
    }

    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }
    [[nodiscard]] HingeAngle& HingeAngleConstraint() const { return *hingeAngle_; }
    [[nodiscard]] BallSocket& BallSocketConstraint() const { return *ballSocket_; }
    [[nodiscard]] AngularMotor* Motor() const { return motor_; }

private:
    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    HingeAngle* hingeAngle_ = nullptr;
    BallSocket* ballSocket_ = nullptr;
    AngularMotor* motor_ = nullptr;
};

} // namespace Jitter2::Dynamics::Constraints
