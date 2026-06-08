#pragma once

#include <array>
#include <limits>

#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/SoftBodies/SoftBody.hpp>
#include <Jitter2/SoftBodies/SoftBodyShape.hpp>

namespace Jitter2::SoftBodies
{

// Represents a tetrahedral shape in a soft body simulation.
class SoftBodyTetrahedron : public SoftBodyShape
{
public:

    // Initializes a new instance of the SoftBodyTetrahedron class.
    // body: The soft body this shape belongs to.
    // vertex1: The first vertex.
    // vertex2: The second vertex.
    // vertex3: The third vertex.
    // vertex4: The fourth vertex.
    SoftBodyTetrahedron(
        SoftBody& body,
        RigidBody& vertex1,
        RigidBody& vertex2,
        RigidBody& vertex3,
        RigidBody& vertex4)
        : SoftBodyShape(body),
          vertices_ {&vertex1, &vertex2, &vertex3, &vertex4}
    {
        UpdateWorldBoundingBox();
    }

    // Gets the four vertices (rigid bodies) of the tetrahedron.
    [[nodiscard]] const std::array<RigidBody*, 4>& Vertices() const { return vertices_; }


    LinearMath::JVector Velocity() const override
    {
        LinearMath::JVector velocity = LinearMath::JVector::Zero();
        for (const RigidBody* vertex : vertices_)
        {
            velocity += vertex->Velocity();
        }

        return velocity * static_cast<Real>(0.25);
    }


    RigidBody& GetClosest(const LinearMath::JVector& position) const override
    {
        Real distance = std::numeric_limits<Real>::max();
        int closest = 0;

        for (int index = 0; index < 4; ++index)
        {
            const Real length = (position - vertices_[static_cast<std::size_t>(index)]->Position()).LengthSquared();
            if (length < distance)
            {
                distance = length;
                closest = index;
            }
        }

        return *vertices_[static_cast<std::size_t>(closest)];
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        Real maxDot = std::numeric_limits<Real>::lowest();
        int furthest = 0;

        for (int index = 0; index < 4; ++index)
        {
            const Real dot = LinearMath::JVector::Dot(
                direction,
                vertices_[static_cast<std::size_t>(index)]->Position());
            if (dot > maxDot)
            {
                maxDot = dot;
                furthest = index;
            }
        }

        result = vertices_[static_cast<std::size_t>(furthest)]->Position();
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        point = (vertices_[0]->Position() + vertices_[1]->Position()
            + vertices_[2]->Position() + vertices_[3]->Position()) * static_cast<Real>(0.25);
    }


    void UpdateWorldBoundingBox(Real dt = static_cast<Real>(0)) override
    {
        (void)dt;
        constexpr Real extraMargin = static_cast<Real>(0.01);
        LinearMath::JBoundingBox box = LinearMath::JBoundingBox::SmallBox();

        for (const RigidBody* vertex : vertices_)
        {
            LinearMath::JBoundingBox::AddPointInPlace(box, vertex->Position());
        }

        box.Min -= LinearMath::JVector::One() * extraMargin;
        box.Max += LinearMath::JVector::One() * extraMargin;
        worldBoundingBox_ = box;
    }

private:
    std::array<RigidBody*, 4> vertices_ {};
};

} // namespace Jitter2::SoftBodies
