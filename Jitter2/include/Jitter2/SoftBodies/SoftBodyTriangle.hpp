#pragma once

#include <algorithm>

#include <Jitter2/LinearMath/JBoundingBox.hpp>
#include <Jitter2/SoftBodies/SoftBody.hpp>
#include <Jitter2/SoftBodies/SoftBodyShape.hpp>

namespace Jitter2::SoftBodies
{

// Represents a triangular shape in a soft body simulation.
class SoftBodyTriangle : public SoftBodyShape
{
public:

    // Initializes a new instance of the SoftBodyTriangle class.
    // body: The soft body this shape belongs to.
    // vertex1: The first vertex.
    // vertex2: The second vertex.
    // vertex3: The third vertex.
    SoftBodyTriangle(SoftBody& body, RigidBody& vertex1, RigidBody& vertex2, RigidBody& vertex3)
        : SoftBodyShape(body),
          vertex1_(&vertex1),
          vertex2_(&vertex2),
          vertex3_(&vertex3)
    {
        UpdateWorldBoundingBox();
    }

    // Gets the first vertex (rigid body) of the triangle.
    [[nodiscard]] RigidBody& Vertex1() const { return *vertex1_; }

    // Gets the second vertex (rigid body) of the triangle.
    [[nodiscard]] RigidBody& Vertex2() const { return *vertex2_; }

    // Gets the third vertex (rigid body) of the triangle.
    [[nodiscard]] RigidBody& Vertex3() const { return *vertex3_; }

    // Gets or sets the thickness of the triangle.
    [[nodiscard]] Real Thickness() const { return halfThickness_ * static_cast<Real>(2); }

    // Gets or sets the thickness of the triangle.
    void Thickness(Real value) { halfThickness_ = value * static_cast<Real>(0.5); }


    LinearMath::JVector Velocity() const override
    {
        return (vertex1_->Data().Velocity + vertex2_->Data().Velocity + vertex3_->Data().Velocity)
            * (static_cast<Real>(1) / static_cast<Real>(3));
    }


    RigidBody& GetClosest(const LinearMath::JVector& position) const override
    {
        const Real len1 = (position - vertex1_->Position()).LengthSquared();
        const Real len2 = (position - vertex2_->Position()).LengthSquared();
        const Real len3 = (position - vertex3_->Position()).LengthSquared();

        return (len1 < len2 && len1 < len3) ? *vertex1_
            : (len2 < len3) ? *vertex2_
            : *vertex3_;
    }


    void UpdateWorldBoundingBox(Real dt = static_cast<Real>(0)) override
    {
        (void)dt;
        const Real extraMargin = std::max(halfThickness_, static_cast<Real>(0.01));
        LinearMath::JBoundingBox box = LinearMath::JBoundingBox::SmallBox();

        LinearMath::JBoundingBox::AddPointInPlace(box, vertex1_->Position());
        LinearMath::JBoundingBox::AddPointInPlace(box, vertex2_->Position());
        LinearMath::JBoundingBox::AddPointInPlace(box, vertex3_->Position());

        const LinearMath::JVector extra(extraMargin);
        box.Min -= extra;
        box.Max += extra;
        worldBoundingBox_ = box;
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        const LinearMath::JVector a = vertex1_->Position();
        const LinearMath::JVector b = vertex2_->Position();
        const LinearMath::JVector c = vertex3_->Position();

        Real maxDot = LinearMath::JVector::Dot(a, direction);
        result = a;

        Real dot = LinearMath::JVector::Dot(b, direction);
        if (dot > maxDot)
        {
            maxDot = dot;
            result = b;
        }

        dot = LinearMath::JVector::Dot(c, direction);
        if (dot > maxDot)
        {
            result = c;
        }

        result += LinearMath::JVector::Normalize(direction) * halfThickness_;
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        point = (vertex1_->Position() + vertex2_->Position() + vertex3_->Position())
            * (static_cast<Real>(1) / static_cast<Real>(3));
    }

private:
    RigidBody* vertex1_ = nullptr;
    RigidBody* vertex2_ = nullptr;
    RigidBody* vertex3_ = nullptr;
    Real halfThickness_ = static_cast<Real>(0.05);
};

} // namespace Jitter2::SoftBodies
