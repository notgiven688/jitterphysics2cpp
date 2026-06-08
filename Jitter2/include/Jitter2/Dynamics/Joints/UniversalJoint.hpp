#pragma once

#include <Jitter2/Dynamics/Constraints/AngularMotor.hpp>
#include <Jitter2/Dynamics/Constraints/BallSocket.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/TwistAngle.hpp>
#include <Jitter2/Dynamics/Joints/Joint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Creates a universal joint utilizing a TwistAngle, BallSocket, and an optional AngularMotor
// constraint.
class UniversalJoint : public Joint
{
public:
    UniversalJoint(
        World& world,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& center,
        LinearMath::JVector rotateAxis1,
        LinearMath::JVector rotateAxis2,
        bool hasMotor = false)
        : body1_(&body1),
          body2_(&body2)
    {
        Detail::CheckFinite(center, "center");
        Detail::CheckNonZero(rotateAxis1, "rotateAxis1");
        Detail::CheckNonZero(rotateAxis2, "rotateAxis2");
        rotateAxis1.Normalize();
        rotateAxis2.Normalize();

        twistAngle_ = &world.CreateConstraint<TwistAngle>(body1, body2);
        twistAngle_->Initialize(rotateAxis1, rotateAxis2);
        Register(*twistAngle_);

        ballSocket_ = &world.CreateConstraint<BallSocket>(body1, body2);
        ballSocket_->Initialize(center);
        Register(*ballSocket_);

        if (hasMotor)
        {
            motor_ = &world.CreateConstraint<AngularMotor>(body1, body2);
            motor_->Initialize(rotateAxis1, rotateAxis2);
            Register(*motor_);
        }
    }

    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }
    [[nodiscard]] TwistAngle& TwistAngleConstraint() const { return *twistAngle_; }
    [[nodiscard]] BallSocket& BallSocketConstraint() const { return *ballSocket_; }
    [[nodiscard]] AngularMotor* Motor() const { return motor_; }

private:
    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    TwistAngle* twistAngle_ = nullptr;
    BallSocket* ballSocket_ = nullptr;
    AngularMotor* motor_ = nullptr;
};

} // namespace Jitter2::Dynamics::Constraints
