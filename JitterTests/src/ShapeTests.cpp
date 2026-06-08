#include <cmath>
#include <vector>

#include <Jitter2/Jitter2.hpp>
#include "TestSupport.hpp"

using Jitter2::Real;
using Jitter2::MassInertiaUpdateMode;
using Jitter2::World;
using Jitter2::Collision::Shapes::BoxShape;
using Jitter2::Collision::Shapes::CapsuleShape;
using Jitter2::Collision::Shapes::ConeShape;
using Jitter2::Collision::Shapes::ConvexHullShape;
using Jitter2::Collision::Shapes::CylinderShape;
using Jitter2::Collision::Shapes::PointCloudShape;
using Jitter2::Collision::Shapes::SphereShape;
using Jitter2::Collision::Shapes::TriangleMesh;
using Jitter2::Collision::Shapes::TriangleShape;
using Jitter2::Collision::Shapes::TransformedShape;
using Jitter2::Collision::Shapes::VertexSupportMap;
using Jitter2::LinearMath::JBoundingBox;
using Jitter2::LinearMath::JMatrix;
using Jitter2::LinearMath::JQuaternion;
using Jitter2::LinearMath::JTriangle;
using Jitter2::LinearMath::JVector;
using JitterTests::Require;
using JitterTests::RequireClose;

namespace
{

void BoundingBoxBasics()
{
    JBoundingBox box(JVector(-1, -2, -3), JVector(1, 2, 3));

    Require(box.Contains(JVector::Zero()), "box contains origin");
    Require(!box.Contains(JVector(2, 0, 0)), "box rejects outside point");
    RequireClose(box.GetVolume(), static_cast<Real>(48), static_cast<Real>(1e-6), "box volume");
    RequireClose(box.GetSurfaceArea(), static_cast<Real>(88), static_cast<Real>(1e-6), "box surface area");

    const JBoundingBox inner(JVector(-0.5f, -1, -1), JVector(0.5f, 1, 1));
    const JBoundingBox far(JVector(10, 10, 10), JVector(11, 11, 11));
    Require(box.Contains(inner) == JBoundingBox::ContainmentType::Contains, "box contains inner");
    Require(JBoundingBox::Disjoint(box, far), "box disjoint");

    Real enter = 0;
    Require(box.RayIntersect(JVector(-10, 0, 0), JVector(1, 0, 0), enter), "ray intersects box");
    RequireClose(enter, static_cast<Real>(9), static_cast<Real>(1e-6), "ray enter distance");
}

void BoxShapeBasics()
{
    BoxShape box(2, 4, 6);
    Require(box.Size() == JVector(2, 4, 6), "box size");
    Require(box.WorldBoundingBox() == JBoundingBox(JVector(-1, -2, -3), JVector(1, 2, 3)), "box bounds");

    JVector support;
    box.SupportMap(JVector(-1, 2, -3), support);
    Require(support == JVector(-1, 2, -3), "box support map");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    box.CalculateMassInertia(inertia, com, mass);
    RequireClose(mass, static_cast<Real>(48), static_cast<Real>(1e-6), "box mass");
    Require(com == JVector::Zero(), "box com");
}

void SphereShapeBasics()
{
    SphereShape sphere(2);
    Require(sphere.WorldBoundingBox() == JBoundingBox(JVector(-2), JVector(2)), "sphere bounds");

    JVector support;
    sphere.SupportMap(JVector(0, 3, 0), support);
    Require(support == JVector(0, 2, 0), "sphere support map");

    JVector normal;
    Real lambda = 0;
    Require(sphere.LocalRayCast(JVector(-4, 0, 0), JVector(1, 0, 0), normal, lambda), "sphere ray cast");
    RequireClose(lambda, static_cast<Real>(2), static_cast<Real>(1e-6), "sphere ray lambda");
    Require(normal == JVector(-1, 0, 0), "sphere ray normal");
}

void ShapeWorldRayCastUsesBodyTransform()
{
    World world;
    auto& body = world.CreateRigidBody();
    SphereShape sphere(1);
    body.AddShape(sphere);
    body.Position(JVector(5, 0, 0));

    JVector normal;
    Real lambda = 0;
    Require(sphere.RayCast(JVector::Zero(), JVector::UnitX(), normal, lambda), "world sphere ray cast");
    RequireClose(lambda, static_cast<Real>(4), static_cast<Real>(1e-6), "world sphere ray lambda");
    Require(normal == JVector(-1, 0, 0), "world sphere ray normal");
}

void BoxWorldRayCastUsesBodyRotation()
{
    World world;
    auto& body = world.CreateRigidBody();
    BoxShape box(4, 2, 2);
    body.AddShape(box);
    body.Orientation(JQuaternion::CreateRotationZ(static_cast<Real>(3.14159265358979323846 / 2.0)));

    JVector normal;
    Real lambda = 0;
    Require(box.RayCast(JVector(0, -5, 0), JVector::UnitY(), normal, lambda), "world box ray cast");
    RequireClose(lambda, static_cast<Real>(3), static_cast<Real>(1e-5), "world box ray lambda");
    RequireClose(normal.X, static_cast<Real>(0), static_cast<Real>(1e-5), "world box ray normal x");
    RequireClose(normal.Y, static_cast<Real>(-1), static_cast<Real>(1e-5), "world box ray normal y");
}

void CapsuleShapeBasics()
{
    CapsuleShape capsule(1, 4);
    Require(capsule.WorldBoundingBox() == JBoundingBox(JVector(-1, -3, -1), JVector(1, 3, 1)), "capsule bounds");

    JVector support;
    capsule.SupportMap(JVector(0, 1, 0), support);
    Require(support == JVector(0, 3, 0), "capsule support map");

    capsule.Orientation = JQuaternion::CreateRotationZ(static_cast<Real>(3.14159265358979323846 / 2.0));
    capsule.UpdateWorldBoundingBox();
    RequireClose(capsule.WorldBoundingBox().Max.X, static_cast<Real>(3), static_cast<Real>(1e-5), "rotated capsule bounds x");
    RequireClose(capsule.WorldBoundingBox().Max.Y, static_cast<Real>(1), static_cast<Real>(1e-5), "rotated capsule bounds y");
}

void CylinderShapeBasics()
{
    CylinderShape cylinder(4, 2);
    Require(cylinder.WorldBoundingBox() == JBoundingBox(JVector(-2, -2, -2), JVector(2, 2, 2)), "cylinder bounds");

    JVector support;
    cylinder.SupportMap(JVector(3, 0, 4), support);
    RequireClose(support.X, static_cast<Real>(1.2), static_cast<Real>(1e-6), "cylinder support x");
    RequireClose(support.Y, static_cast<Real>(0), static_cast<Real>(1e-6), "cylinder support y");
    RequireClose(support.Z, static_cast<Real>(1.6), static_cast<Real>(1e-6), "cylinder support z");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    cylinder.CalculateMassInertia(inertia, com, mass);
    constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
    RequireClose(mass, pi * static_cast<Real>(16), static_cast<Real>(1e-5), "cylinder mass");
    Require(com == JVector::Zero(), "cylinder com");
}

void ConeShapeBasics()
{
    ConeShape cone(2, 4);
    Require(cone.WorldBoundingBox() == JBoundingBox(JVector(-2, -1, -2), JVector(2, 3, 2)), "cone bounds");

    JVector support;
    cone.SupportMap(JVector(0, 1, 0), support);
    Require(support == JVector(0, 3, 0), "cone tip support");

    cone.SupportMap(JVector(1, 0, 0), support);
    Require(support == JVector(2, -1, 0), "cone base support");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    cone.CalculateMassInertia(inertia, com, mass);
    constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
    RequireClose(mass, static_cast<Real>(16.0 / 3.0) * pi, static_cast<Real>(1e-5), "cone mass");
    Require(com == JVector::Zero(), "cone com");
}

void TriangleMeshAndShapeBasics()
{
    const std::vector<JTriangle> soup {
        JTriangle(JVector(0, 0, 0), JVector(1, 0, 0), JVector(0, 1, 0)),
        JTriangle(JVector(1, 0, 0), JVector(1, 1, 0), JVector(0, 1, 0)),
    };
    TriangleMesh mesh(soup);

    Require(mesh.Vertices().size() == 4, "triangle mesh deduplicates vertices");
    Require(mesh.Indices().size() == 2, "triangle mesh triangle count");
    Require(mesh.Indices()[0].NeighborA == 1, "triangle mesh neighbor across BC");

    TriangleShape shape(mesh, 0);
    Require(shape.WorldBoundingBox() == JBoundingBox(
        JVector(static_cast<Real>(-0.01), static_cast<Real>(-0.01), static_cast<Real>(-0.01)),
        JVector(static_cast<Real>(1.01), static_cast<Real>(1.01), static_cast<Real>(0.01))),
        "triangle shape bounds");

    JVector support;
    shape.SupportMap(JVector(0, 1, 0), support);
    Require(support == JVector(0, 1, 0), "triangle support");

    JVector normal;
    Real lambda = 0;
    Require(shape.LocalRayCast(
        JVector(static_cast<Real>(0.25), static_cast<Real>(0.25), 1),
        JVector(0, 0, -1),
        normal,
        lambda),
        "triangle shape local ray");
    RequireClose(lambda, static_cast<Real>(1), static_cast<Real>(1e-6), "triangle shape ray lambda");

    bool massThrew = false;
    try
    {
        JMatrix inertia;
        JVector com;
        Real mass = 0;
        shape.CalculateMassInertia(inertia, com, mass);
    }
    catch (const std::logic_error&)
    {
        massThrew = true;
    }
    Require(massThrew, "triangle shape mass throws");
}

void TriangleShapeAttachesWithPreservedMass()
{
    World world;
    auto& body = world.CreateRigidBody();
    const Real originalMass = body.Mass();

    std::vector<JVector> vertices {
        JVector(0, 0, 0),
        JVector(1, 0, 0),
        JVector(0, 1, 0),
    };
    std::vector<int> indices {0, 1, 2};
    TriangleMesh mesh(vertices, indices);
    TriangleShape shape(mesh, 0);

    body.AddShape(shape, MassInertiaUpdateMode::Preserve);
    body.Position(JVector(5, 0, 0));

    RequireClose(body.Mass(), originalMass, static_cast<Real>(1e-6), "triangle preserve mass");
    Require(shape.GetRigidBody() == &body, "triangle attached body");
    RequireClose(shape.WorldBoundingBox().Min.X, static_cast<Real>(4.99), static_cast<Real>(1e-6),
        "triangle attached bounds x");
}

void PointCloudShapeBasics()
{
    const std::vector<JVector> vertices {
        JVector(-1, -1, -1),
        JVector(1, -1, -1),
        JVector(-1, 1, -1),
        JVector(1, 1, -1),
        JVector(-1, -1, 1),
        JVector(1, -1, 1),
        JVector(-1, 1, 1),
        JVector(1, 1, 1),
    };

    VertexSupportMap supportMap(vertices);
    JVector support;
    supportMap.SupportMap(JVector(1, 0, 0), support);
    Require(support == JVector(1, 1, 1), "vertex support last maximum wins");

    PointCloudShape shape(vertices);
    Require(shape.WorldBoundingBox() == JBoundingBox(JVector(-1), JVector(1)), "point cloud bounds");
    shape.Shift(JVector(2, 0, 0));
    shape.SupportMap(JVector(-1, 0, 0), support);
    Require(support == JVector(1, 1, 1), "shifted point cloud support");
    Require(shape.WorldBoundingBox() == JBoundingBox(JVector(1, -1, -1), JVector(3, 1, 1)),
        "shifted point cloud bounds");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    shape.CalculateMassInertia(inertia, com, mass);
    RequireClose(mass, static_cast<Real>(8), static_cast<Real>(1e-6), "point cloud box mass approximation");
    RequireClose(com.X, static_cast<Real>(2), static_cast<Real>(1e-5), "point cloud center x");
    RequireClose(com.Y, static_cast<Real>(0), static_cast<Real>(1e-5), "point cloud center y");
    RequireClose(com.Z, static_cast<Real>(0), static_cast<Real>(1e-5), "point cloud center z");

    PointCloudShape clone = shape.Clone();
    Require(clone.ShapeId() != shape.ShapeId(), "point cloud clone gets new shape id");
    clone.SupportMap(JVector(-1, 0, 0), support);
    Require(support == JVector(1, 1, 1), "point cloud clone preserves shifted support");
    Require(clone.WorldBoundingBox() == JBoundingBox(JVector(1, -1, -1), JVector(3, 1, 1)),
        "point cloud clone preserves bounds");
}

void TransformedShapeBasics()
{
    SphereShape sphere(1);
    TransformedShape transformed(sphere, JVector(2, 0, 0));

    JVector support;
    transformed.SupportMap(JVector::UnitX(), support);
    Require(support == JVector(3, 0, 0), "transformed support");
    Require(transformed.WorldBoundingBox() == JBoundingBox(JVector(1, -1, -1), JVector(3, 1, 1)),
        "transformed bounds");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    transformed.CalculateMassInertia(inertia, com, mass);
    constexpr Real pi = static_cast<Real>(3.1415926535897932384626433832795L);
    RequireClose(mass, static_cast<Real>(4.0 / 3.0) * pi, static_cast<Real>(1e-5),
        "transformed mass");
    Require(com == JVector(2, 0, 0), "transformed center");
}

void ConvexHullShapeBasics()
{
    const std::vector<JTriangle> tetrahedron {
        JTriangle(JVector(0, 0, 0), JVector(0, 1, 0), JVector(1, 0, 0)),
        JTriangle(JVector(0, 0, 0), JVector(1, 0, 0), JVector(0, 0, 1)),
        JTriangle(JVector(0, 0, 0), JVector(0, 0, 1), JVector(0, 1, 0)),
        JTriangle(JVector(1, 0, 0), JVector(0, 1, 0), JVector(0, 0, 1)),
    };

    ConvexHullShape hull(tetrahedron);
    Require(hull.WorldBoundingBox() == JBoundingBox(JVector(0), JVector(1)), "convex hull bounds");

    JVector support;
    hull.SupportMap(JVector(0, 0, 1), support);
    Require(support == JVector(0, 0, 1), "convex hull support");

    JMatrix inertia;
    JVector com;
    Real mass = 0;
    hull.CalculateMassInertia(inertia, com, mass);
    RequireClose(mass, static_cast<Real>(1.0 / 6.0), static_cast<Real>(1e-6), "convex hull mass");
    RequireClose(com.X, static_cast<Real>(0.25), static_cast<Real>(1e-6), "convex hull com x");
    RequireClose(com.Y, static_cast<Real>(0.25), static_cast<Real>(1e-6), "convex hull com y");
    RequireClose(com.Z, static_cast<Real>(0.25), static_cast<Real>(1e-6), "convex hull com z");

    ConvexHullShape clone = hull.Clone();
    Require(clone.ShapeId() != hull.ShapeId(), "convex hull clone gets new shape id");
    clone.Shift(JVector(1, 0, 0));
    clone.SupportMap(JVector(0, 0, 1), support);
    Require(support == JVector(1, 0, 1), "convex hull clone has independent shift");
}

void RigidBodyShapeAttachment()
{
    World world;
    auto& body = world.CreateRigidBody();
    body.Position(JVector(10, 0, 0));

    SphereShape sphere(1);
    body.AddShape(sphere);

    Require(body.Shapes().size() == 1, "shape appears in body shape list");
    Require(body.Shapes()[0] == &sphere, "shape pointer stored");
    Require(sphere.GetRigidBody() == &body, "shape body pointer set");
    Require(sphere.WorldBoundingBox() == JBoundingBox(JVector(9, -1, -1), JVector(11, 1, 1)), "attached sphere bounds use body position");

    body.Position(JVector(0, 5, 0));
    Require(sphere.WorldBoundingBox() == JBoundingBox(JVector(-1, 4, -1), JVector(1, 6, 1)), "shape bounds update with body position");
}

void ShapeMassAndPreserveMode()
{
    World world;
    auto& body = world.CreateRigidBody();

    SphereShape first(1);
    SphereShape second(1);

    body.AddShape(first);
    const Real oneShapeMass = body.Mass();
    body.AddShape(second, MassInertiaUpdateMode::Preserve);

    RequireClose(body.Mass(), oneShapeMass, static_cast<Real>(1e-6), "preserve mode keeps mass");

    body.SetMassInertia();
    Require(body.Mass() > oneShapeMass, "manual mass update sees second shape");
}

void ShapeRemoveAndClear()
{
    World world;
    auto& body = world.CreateRigidBody();

    SphereShape sphere(1);
    BoxShape box(1);

    body.AddShape(sphere);
    body.AddShape(box);
    Require(body.Shapes().size() == 2, "two shapes added");

    body.RemoveShape(sphere);
    Require(body.Shapes().size() == 1, "remove shape updates list");
    Require(sphere.GetRigidBody() == nullptr, "removed shape detached");
    Require(sphere.WorldBoundingBox() == JBoundingBox(JVector(-1), JVector(1)), "removed sphere bounds reset to local origin");

    body.ClearShapes();
    Require(body.Shapes().empty(), "clear shapes empties list");
    Require(box.GetRigidBody() == nullptr, "cleared shape detached");
    RequireClose(body.Mass(), static_cast<Real>(1), static_cast<Real>(1e-6), "empty body mass reset");
}

void AddingAttachedShapeThrows()
{
    World world;
    auto& first = world.CreateRigidBody();
    auto& second = world.CreateRigidBody();
    SphereShape sphere(1);

    first.AddShape(sphere);

    bool threw = false;
    try
    {
        second.AddShape(sphere);
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }

    Require(threw, "adding attached shape throws");
}

void ShapeHelperTessellatesSupportMaps()
{
    const auto sphere = Jitter2::Collision::SupportPrimitives::CreateSphere(static_cast<Real>(1));
    const std::vector<JTriangle> low = Jitter2::Collision::Shapes::ShapeHelper::Tessellate(sphere, 1);
    const std::vector<JTriangle> medium = Jitter2::Collision::Shapes::ShapeHelper::Tessellate(sphere, 2);
    const std::vector<JTriangle> high = Jitter2::Collision::Shapes::ShapeHelper::Tessellate(sphere, 4);

    Require(low.size() == 20, "tessellate subdivision 1 count");
    Require(medium.size() == 80, "tessellate subdivision 2 count");
    Require(high.size() == 1280, "tessellate subdivision 4 count");

    for (const JTriangle& triangle : high)
    {
        Require(triangle.GetNormal().LengthSquared() > static_cast<Real>(1e-16), "tessellate skips degenerate triangles");
    }
}

} // namespace

JITTER_TEST_CASE("JBoundingBox basics")
{
    BoundingBoxBasics();
}

JITTER_TEST_CASE("Primitive shape basics")
{
    BoxShapeBasics();
    SphereShapeBasics();
    CapsuleShapeBasics();
    CylinderShapeBasics();
    ConeShapeBasics();
    TriangleMeshAndShapeBasics();
    PointCloudShapeBasics();
    TransformedShapeBasics();
    ConvexHullShapeBasics();
}

JITTER_TEST_CASE("Shape world ray casts use body transform")
{
    ShapeWorldRayCastUsesBodyTransform();
    BoxWorldRayCastUsesBodyRotation();
}

JITTER_TEST_CASE("RigidBody shape attachment")
{
    RigidBodyShapeAttachment();
    TriangleShapeAttachesWithPreservedMass();
}

JITTER_TEST_CASE("RigidBody shape mass preserve mode")
{
    ShapeMassAndPreserveMode();
}

JITTER_TEST_CASE("RigidBody shape remove and clear")
{
    ShapeRemoveAndClear();
}

JITTER_TEST_CASE("Adding attached shape throws")
{
    AddingAttachedShapeThrows();
}

JITTER_TEST_CASE("ShapeHelper tessellates support maps")
{
    ShapeHelperTessellatesSupportMaps();
}
