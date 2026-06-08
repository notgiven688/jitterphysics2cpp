#pragma once

namespace Jitter2::Collision::Shapes
{

// Defines a method to create a new instance of a shape for use with another rigid body.
// T: The concrete shape type implementing this interface.
template <typename T>
class ICloneableShape
{
public:
    virtual ~ICloneableShape() = default;

    // Creates a copy of the current shape instance that shares underlying geometry data.
    // Returns: A new shape instance of type T that shares immutable data
    // with the original but has its own instance state.
    [[nodiscard]] virtual T Clone() const = 0;
};

} // namespace Jitter2::Collision::Shapes
