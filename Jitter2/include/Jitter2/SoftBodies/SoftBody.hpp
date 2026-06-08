#pragma once

#include <vector>

#include <Jitter2/Dynamics/Constraints/Constraint.hpp>
#include <Jitter2/Dynamics/RigidBody.hpp>
#include <Jitter2/Dynamics/World.hpp>
#include <Jitter2/SoftBodies/SoftBodyShape.hpp>

namespace Jitter2::SoftBodies
{

// Represents a soft body in the physics simulation. A soft body is composed of vertices (rigid bodies),
// springs (constraints), and shapes.
class SoftBody
{
public:

    // Initializes a new instance of the SoftBody class.
    // world: The world in which the soft body will be created.
    explicit SoftBody(World& world)
        : world_(&world)
    {
        postStepToken_ = world_->PostStep.Add([this](Real dt)
        {

            // Called after each world step to update the activation state of the soft body's shapes.
            // dt: The time step.
            WorldOnPostStep(dt);
        });
    }

    virtual ~SoftBody()
    {
        UnregisterPostStep();
    }

    // Gets the world in which the soft body exists.
    [[nodiscard]] World& GetWorld() const { return *world_; }

    // Gets the list of vertices (rigid bodies) that make up the soft body.
    [[nodiscard]] std::vector<RigidBody*>& Vertices() { return vertices_; }

    // Gets the list of vertices (rigid bodies) that make up the soft body.
    [[nodiscard]] const std::vector<RigidBody*>& Vertices() const { return vertices_; }

    // Gets the list of springs (constraints) that connect the vertices of the soft body.
    [[nodiscard]] std::vector<Dynamics::Constraints::Constraint*>& Springs() { return springs_; }

    // Gets the list of springs (constraints) that connect the vertices of the soft body.
    [[nodiscard]] const std::vector<Dynamics::Constraints::Constraint*>& Springs() const { return springs_; }

    // Gets the list of shapes that define the geometry of the soft body.
    [[nodiscard]] std::vector<SoftBodyShape*>& Shapes() { return shapes_; }

    // Gets the list of shapes that define the geometry of the soft body.
    [[nodiscard]] const std::vector<SoftBodyShape*>& Shapes() const { return shapes_; }

    // Gets a value indicating whether the soft body is active. A soft body is considered active
    // if its first vertex is active.
    [[nodiscard]] bool IsActive() const
    {
        return !vertices_.empty() && vertices_.front()->IsActive();
    }

    void AddVertex(RigidBody& vertex)
    {
        vertices_.push_back(&vertex);
    }

    // Adds a shape to the soft body and registers it with the world's dynamic tree.
    // shape: The shape to add.
    void AddShape(SoftBodyShape& shape)
    {
        shapes_.push_back(&shape);
        world_->DynamicTree().AddProxy(shape);
    }

    // Adds a spring (constraint) to the soft body.
    // constraint: The constraint to add.
    void AddSpring(Dynamics::Constraints::Constraint& constraint)
    {
        springs_.push_back(&constraint);
    }

    // Destroys the soft body, removing all its components from the simulation world.
    virtual void Destroy()
    {
        UnregisterPostStep();

        for (SoftBodyShape* shape : shapes_)
        {
            world_->DynamicTree().RemoveProxy(*shape);
        }
        shapes_.clear();

        for (Dynamics::Constraints::Constraint* spring : springs_)
        {
            world_->Remove(*spring);
        }
        springs_.clear();

        for (RigidBody* vertex : vertices_)
        {
            world_->Remove(*vertex);
        }
        vertices_.clear();
    }

protected:

    // Called after each world step to update the activation state of the soft body's shapes.
    // dt: The time step.
    virtual void WorldOnPostStep(Real)
    {
        if (IsActive() == active_)
        {
            return;
        }

        active_ = IsActive();

        for (SoftBodyShape* shape : shapes_)
        {
            if (active_)
            {
                world_->DynamicTree().ActivateProxy(*shape);
            }
            else
            {
                world_->DynamicTree().DeactivateProxy(*shape);
            }
        }
    }

private:
    void UnregisterPostStep()
    {
        if (world_ != nullptr && postStepToken_ != 0)
        {
            world_->PostStep.Remove(postStepToken_);
            postStepToken_ = 0;
        }
    }

    World* world_ = nullptr;
    std::vector<RigidBody*> vertices_;
    std::vector<Dynamics::Constraints::Constraint*> springs_;
    std::vector<SoftBodyShape*> shapes_;
    World::WorldStepFunction::Token postStepToken_ = 0;
    bool active_ = true;
};

} // namespace Jitter2::SoftBodies
