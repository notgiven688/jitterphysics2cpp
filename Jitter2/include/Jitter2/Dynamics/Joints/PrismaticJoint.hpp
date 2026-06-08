#pragma once

#include <cmath>
#include <stdexcept>

#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/FixedAngle.hpp>
#include <Jitter2/Dynamics/Constraints/HingeAngle.hpp>
#include <Jitter2/Dynamics/Constraints/Limit.hpp>
#include <Jitter2/Dynamics/Constraints/LinearMotor.hpp>
#include <Jitter2/Dynamics/Constraints/PointOnLine.hpp>
#include <Jitter2/Dynamics/Joints/Joint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Constructs a prismatic joint utilizing a PointOnLine constraint in conjunction with
// FixedAngle, HingeAngle, and LinearMotor constraints.
class PrismaticJoint : public Joint
{
public:

    PrismaticJoint(
        World& world,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& center,
        LinearMath::JVector axis,
        bool pinned = true,
        bool hasMotor = false)
        : PrismaticJoint(world, body1, body2, center, axis, LinearLimit::Full(), pinned, hasMotor)
    {
    }

    // Initializes a new prismatic joint.
    // Thrown when center contains a non-finite value, when axis is zero
    // or contains a non-finite value, when either limit value is NaN, when either body does not belong to
    // world, or when both body references are the same.

    PrismaticJoint(
        World& world,
        RigidBody& body1,
        RigidBody& body2,
        const LinearMath::JVector& center,
        LinearMath::JVector axis,
        const LinearLimit& limit,
        bool pinned = true,
        bool hasMotor = false)
        : body1_(&body1),
          body2_(&body2)
    {
        Detail::CheckFinite(center, "center");
        Detail::CheckNonZero(axis, "axis");
        if (std::isnan(limit.From) || std::isnan(limit.To))
        {
            throw std::invalid_argument("limit must not contain NaN.");
        }

        axis.Normalize();

        slider_ = &world.CreateConstraint<PointOnLine>(body1, body2);
        slider_->Initialize(axis, center, center, limit);
        Register(*slider_);

        if (pinned)
        {
            fixedAngle_ = &world.CreateConstraint<FixedAngle>(body1, body2);
            fixedAngle_->Initialize();
            Register(*fixedAngle_);
        }
        else
        {
            hingeAngle_ = &world.CreateConstraint<HingeAngle>(body1, body2);
            hingeAngle_->Initialize(axis, AngularLimit::Full());
            Register(*hingeAngle_);
        }

        if (hasMotor)
        {
            motor_ = &world.CreateConstraint<LinearMotor>(body1, body2);
            motor_->Initialize(axis, axis);
            Register(*motor_);
        }
    }

    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }
    [[nodiscard]] PointOnLine& Slider() const { return *slider_; }
    [[nodiscard]] FixedAngle* FixedAngleConstraint() const { return fixedAngle_; }
    [[nodiscard]] HingeAngle* HingeAngleConstraint() const { return hingeAngle_; }
    [[nodiscard]] LinearMotor* Motor() const { return motor_; }

private:
    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    PointOnLine* slider_ = nullptr;
    FixedAngle* fixedAngle_ = nullptr;
    HingeAngle* hingeAngle_ = nullptr;
    LinearMotor* motor_ = nullptr;
};

} // namespace Jitter2::Dynamics::Constraints
