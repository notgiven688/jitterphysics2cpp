#pragma once

#include <Jitter2/Dynamics/Constraints/BallSocket.hpp>
#include <Jitter2/Dynamics/Constraints/ConstraintUtility.hpp>
#include <Jitter2/Dynamics/Constraints/FixedAngle.hpp>
#include <Jitter2/Dynamics/Joints/Joint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Creates a rigid weld joint between two bodies using a Constraints.FixedAngle
// constraint for orientation locking and a BallSocket constraint
// for positional locking. This effectively removes all relative motion between
// the connected bodies.
class WeldJoint : public Joint
{
public:

    // Initializes a new weld joint.
    // Thrown when center contains a non-finite value, when either body does not belong
    // to world, or when both body references are the same.

    WeldJoint(World& world, RigidBody& body1, RigidBody& body2, const LinearMath::JVector& center)
        : body1_(&body1),
          body2_(&body2)
    {
        Detail::CheckFinite(center, "center");

        fixedAngle_ = &world.CreateConstraint<FixedAngle>(body1, body2);
        fixedAngle_->Initialize();
        Register(*fixedAngle_);

        ballSocket_ = &world.CreateConstraint<BallSocket>(body1, body2);
        ballSocket_->Initialize(center);
        Register(*ballSocket_);
    }

    [[nodiscard]] RigidBody& Body1() const { return *body1_; }
    [[nodiscard]] RigidBody& Body2() const { return *body2_; }
    [[nodiscard]] FixedAngle& FixedAngleConstraint() const { return *fixedAngle_; }
    [[nodiscard]] BallSocket& BallSocketConstraint() const { return *ballSocket_; }

private:
    RigidBody* body1_ = nullptr;
    RigidBody* body2_ = nullptr;
    FixedAngle* fixedAngle_ = nullptr;
    BallSocket* ballSocket_ = nullptr;
};

} // namespace Jitter2::Dynamics::Constraints
