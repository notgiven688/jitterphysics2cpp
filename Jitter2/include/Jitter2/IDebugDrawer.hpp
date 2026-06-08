#pragma once

#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2
{

// Defines an interface for drawing debug visualization elements.
class IDebugDrawer
{
public:
    virtual ~IDebugDrawer() = default;

    // Draws a line segment between two points.
    // pA: The start point of the segment.
    // pB: The end point of the segment.
    virtual void DrawSegment(const LinearMath::JVector& pA, const LinearMath::JVector& pB) = 0;

    // Draws a triangle defined by three vertices.
    // pA: The first vertex of the triangle.
    // pB: The second vertex of the triangle.
    // pC: The third vertex of the triangle.
    virtual void DrawTriangle(
        const LinearMath::JVector& pA,
        const LinearMath::JVector& pB,
        const LinearMath::JVector& pC) = 0;

    // Draws a point at the specified position.
    // p: The position of the point.
    virtual void DrawPoint(const LinearMath::JVector& p) = 0;
};

// Defines an interface for objects that can be debug-drawn.
class IDebugDrawable
{
public:
    virtual ~IDebugDrawable() = default;

    // Passes an IDebugDrawer to draw basic debug information for the object.
    // drawer: The debug drawer used for rendering debug information.
    virtual void DebugDraw(IDebugDrawer& drawer) = 0;
};

} // namespace Jitter2
