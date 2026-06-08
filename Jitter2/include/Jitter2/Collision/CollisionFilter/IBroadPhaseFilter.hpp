#pragma once

#include <Jitter2/Collision/DynamicTree/IDynamicTreeProxy.hpp>

namespace Jitter2::Collision
{

// Provides a hook into the broadphase collision detection pipeline.
// Implement this interface to intercept shape pairs before narrowphase detection.
// This can be used to filter out specific pairs, implement custom collision layers,
// or handle collisions for custom proxy types. See World::BroadPhaseFilter.
class IBroadPhaseFilter
{
public:
    virtual ~IBroadPhaseFilter() = default;

    // Called for each pair of proxies whose bounding boxes overlap.
    // proxyA: The first proxy.
    // proxyB: The second proxy.
    // Returns: true to continue with narrowphase detection; false to skip this pair.
    virtual bool Filter(IDynamicTreeProxy& proxyA, IDynamicTreeProxy& proxyB) = 0;
};

} // namespace Jitter2::Collision
