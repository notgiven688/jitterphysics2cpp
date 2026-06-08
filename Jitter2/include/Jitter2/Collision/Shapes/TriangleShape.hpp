#pragma once

#include <stdexcept>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/Collision/Shapes/TriangleMesh.hpp>
#include <Jitter2/LinearMath/JTriangle.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a triangle shape defined by a reference to a TriangleMesh and an index.
class TriangleShape final : public RigidBodyShape
{
public:

    // Initializes a new instance of the TriangleShape class.
    // mesh: The triangle mesh to which this triangle belongs.
    // index: The index representing the position of the triangle within the mesh.
    // Thrown when mesh is null.
    // Thrown when index is negative or greater than or equal to the number of triangles in the mesh.

    TriangleShape(TriangleMesh& mesh, int index)
        : mesh_(&mesh),
          index_(index)
    {
        if (index < 0 || static_cast<std::size_t>(index) >= mesh_->Indices().size())
        {
            throw std::out_of_range("Triangle index is out of range.");
        }

        UpdateWorldBoundingBox();
    }

    [[nodiscard]] TriangleMesh& Mesh() const { return *mesh_; }
    [[nodiscard]] int Index() const { return index_; }

    void GetWorldVertices(
        LinearMath::JVector& a,
        LinearMath::JVector& b,
        LinearMath::JVector& c) const
    {
        GetLocalVertices(a, b, c);
        if (GetRigidBody() == nullptr)
        {
            return;
        }

        a = LinearMath::JQuaternion::Transform(a, Orientation) + Position;
        b = LinearMath::JQuaternion::Transform(b, Orientation) + Position;
        c = LinearMath::JQuaternion::Transform(c, Orientation) + Position;
    }

    void CalculateBoundingBox(
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const override
    {
        constexpr Real extraMargin = static_cast<Real>(0.01);

        LinearMath::JVector a;
        LinearMath::JVector b;
        LinearMath::JVector c;
        GetLocalVertices(a, b, c);
        a = LinearMath::JQuaternion::Transform(a, orientation);
        b = LinearMath::JQuaternion::Transform(b, orientation);
        c = LinearMath::JQuaternion::Transform(c, orientation);

        box = LinearMath::JBoundingBox::SmallBox();
        LinearMath::JBoundingBox::AddPointInPlace(box, a);
        LinearMath::JBoundingBox::AddPointInPlace(box, b);
        LinearMath::JBoundingBox::AddPointInPlace(box, c);

        const LinearMath::JVector extra(extraMargin);
        box.Min += position - extra;
        box.Max += position + extra;
    }

    bool LocalRayCast(
        const LinearMath::JVector& origin,
        const LinearMath::JVector& direction,
        LinearMath::JVector& normal,
        Real& lambda) const override
    {
        LinearMath::JVector a;
        LinearMath::JVector b;
        LinearMath::JVector c;
        GetLocalVertices(a, b, c);
        return LinearMath::JTriangle(a, b, c).RayIntersect(
            origin,
            direction,
            LinearMath::JTriangle::CullMode::BackFacing,
            normal,
            lambda);
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        LinearMath::JVector a;
        LinearMath::JVector b;
        LinearMath::JVector c;
        GetLocalVertices(a, b, c);

        result = a;
        Real best = LinearMath::JVector::Dot(a, direction);
        Real dot = LinearMath::JVector::Dot(b, direction);
        if (dot > best)
        {
            best = dot;
            result = b;
        }

        dot = LinearMath::JVector::Dot(c, direction);
        if (dot > best)
        {
            result = c;
        }
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        LinearMath::JVector a;
        LinearMath::JVector b;
        LinearMath::JVector c;
        GetLocalVertices(a, b, c);
        point = (a + b + c) * static_cast<Real>(1.0 / 3.0);
    }

    void CalculateMassInertia(
        LinearMath::JMatrix&,
        LinearMath::JVector&,
        Real&) const override
    {
        throw std::logic_error(
            "TriangleShape has no mass properties. If you encounter this while calling RigidBody.AddShape, "
            "call AddShape with setMassInertia set to false.");
    }

private:
    void GetLocalVertices(
        LinearMath::JVector& a,
        LinearMath::JVector& b,
        LinearMath::JVector& c) const
    {
        const TriangleMesh::Triangle& triangle = mesh_->Indices().at(static_cast<std::size_t>(index_));
        a = mesh_->Vertices().at(static_cast<std::size_t>(triangle.IndexA));
        b = mesh_->Vertices().at(static_cast<std::size_t>(triangle.IndexB));
        c = mesh_->Vertices().at(static_cast<std::size_t>(triangle.IndexC));
    }

    TriangleMesh* mesh_;
    int index_;
};

} // namespace Jitter2::Collision::Shapes
