#pragma once

#include <algorithm>
#include <vector>

#include <Jitter2/IDebugDrawer.hpp>
#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/World.hpp>

namespace Jitter2::Dynamics::Constraints
{

// Base class for joints, which are composite constraints built from multiple Constraint instances.
class Joint : public IDebugDrawable
{
public:
    virtual ~Joint() = default;

    Joint(const Joint&) = delete;
    Joint& operator=(const Joint&) = delete;
    Joint(Joint&&) = delete;
    Joint& operator=(Joint&&) = delete;

    [[nodiscard]] const std::vector<Constraint*>& Constraints() const { return constraints_; }

    // Enables all constraints that this joint is composed of.
    void Enable()
    {
        for (Constraint* constraint : constraints_)
        {
            if (constraint->Handle().IsZero())
            {
                continue;
            }
            constraint->IsEnabled(true);
        }
    }

    // Disables all constraints that this joint is composed of temporarily.
    // For a complete removal use Joint.Remove().
    void Disable()
    {
        for (Constraint* constraint : constraints_)
        {
            if (constraint->Handle().IsZero())
            {
                continue;
            }
            constraint->IsEnabled(false);
        }
    }

    // Removes all constraints that this joint is composed of from the physics world.
    void Remove()
    {
        for (Constraint* constraint : constraints_)
        {
            if (constraint->Handle().IsZero())
            {
                continue;
            }
            constraint->Body1().GetWorld().Remove(*constraint);
        }

        constraints_.clear();
    }

    void DebugDraw(IDebugDrawer& drawer) override
    {
        for (Constraint* constraint : constraints_)
        {
            constraint->DebugDraw(drawer);
        }
    }

protected:
    Joint() = default;

    void Register(Constraint& constraint)
    {
        constraints_.push_back(&constraint);
    }

    void Deregister(Constraint& constraint)
    {
        auto iterator = std::find(constraints_.begin(), constraints_.end(), &constraint);
        if (iterator != constraints_.end())
        {
            constraints_.erase(iterator);
        }
    }

private:
    std::vector<Constraint*> constraints_;
};

} // namespace Jitter2::Dynamics::Constraints
