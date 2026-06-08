#pragma once

#include <unordered_set>

namespace Jitter2
{

class RigidBody;

namespace Collision
{

class Island
{
public:
    [[nodiscard]] const std::unordered_set<RigidBody*>& Bodies() const
    {
        return InternalBodies;
    }

    // Clears all the bodies from the lists within this island.
    void ClearLists()
    {
        InternalBodies.clear();
    }

    std::unordered_set<RigidBody*> InternalBodies;
    bool MarkedAsActive = false;
    bool NeedsUpdate = false;
    int SetIndex = -1;
};

} // namespace Collision
} // namespace Jitter2
