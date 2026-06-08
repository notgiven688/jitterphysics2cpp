#pragma once

#include <algorithm>
#include <memory>
#include <queue>
#include <vector>

#include <Jitter2/Collision/CollisionIsland.hpp>
#include <Jitter2/DataStructures/PartitionedSet.hpp>
#include <Jitter2/Dynamics/Arbiter.hpp>
#include <Jitter2/Dynamics/Constraints/Constraint.hpp>

namespace Jitter2::Collision
{

// Helper class to update islands. Methods must not be called concurrently for the same world.
// Scratch data and pooled islands are stored per thread. Separate worlds may use this helper
// concurrently on different external threads as long as each individual world is not used
// concurrently.
class IslandHelper
{
public:
    using IslandSet = DataStructures::PartitionedSet<Island>;

    static Island& GetFromPool()
    {
        if (!Pool().empty())
        {
            Island* island = Pool().back();
            Pool().pop_back();
            island->MarkedAsActive = true;
            island->NeedsUpdate = false;
            return *island;
        }

        Storage().push_back(std::make_unique<Island>());
        Island& island = *Storage().back();
        island.MarkedAsActive = true;
        island.NeedsUpdate = false;
        return island;
    }

    static void ReturnToPool(Island& island)
    {
        Pool().push_back(&island);
    }

    static void ArbiterCreated(IslandSet& islands, Arbiter& arbiter)
    {
        RigidBody& b1 = arbiter.Body1();
        RigidBody& b2 = arbiter.Body2();

        b1.InternalContacts.insert(&arbiter);
        b2.InternalContacts.insert(&arbiter);

        AddConnection(islands, b1, b2);
    }

    static void ArbiterRemoved(IslandSet& islands, Arbiter& arbiter)
    {
        arbiter.Body1().InternalContacts.erase(&arbiter);
        arbiter.Body2().InternalContacts.erase(&arbiter);

        RemoveConnection(islands, arbiter.Body1(), arbiter.Body2());
    }

    static void ConstraintCreated(IslandSet& islands, Dynamics::Constraints::Constraint& constraint)
    {
        constraint.Body1().InternalConstraints.insert(&constraint);
        constraint.Body2().InternalConstraints.insert(&constraint);

        AddConnection(islands, constraint.Body1(), constraint.Body2());
    }

    static void ConstraintRemoved(IslandSet& islands, Dynamics::Constraints::Constraint& constraint)
    {
        constraint.Body1().InternalConstraints.erase(&constraint);
        constraint.Body2().InternalConstraints.erase(&constraint);

        RemoveConnection(islands, constraint.Body1(), constraint.Body2());
    }

    static void BodyAdded(IslandSet& islands, RigidBody& body)
    {
        body.InternalIsland = &GetFromPool();
        islands.Add(*body.InternalIsland, true);
        body.InternalIsland->InternalBodies.insert(&body);
    }

    static void BodyRemoved(IslandSet& islands, RigidBody& body)
    {
        body.InternalIsland->ClearLists();
        ReturnToPool(*body.InternalIsland);
        islands.Remove(*body.InternalIsland);
    }

    static void AddConnection(IslandSet& islands, RigidBody& body1, RigidBody& body2)
    {
        const bool needsUpdate = !islands.IsActive(*body1.InternalIsland)
            || !islands.IsActive(*body2.InternalIsland);
        const bool bothNotStatic = body1.Data().MotionTypeValue() != MotionType::Static
            && body2.Data().MotionTypeValue() != MotionType::Static;

        if (bothNotStatic)
        {
            MergeIslands(islands, body1, body2);
            body1.InternalConnections.push_back(&body2);
            body2.InternalConnections.push_back(&body1);
        }

        if (needsUpdate)
        {
            if (body1.Data().MotionTypeValue() != MotionType::Static)
            {
                body1.InternalIsland->NeedsUpdate = true;
            }

            if (body2.Data().MotionTypeValue() != MotionType::Static)
            {
                body2.InternalIsland->NeedsUpdate = true;
            }
        }
    }

    static void RemoveConnection(IslandSet& islands, RigidBody& body1, RigidBody& body2)
    {
        RemoveRef(body1.InternalConnections, body2);
        RemoveRef(body2.InternalConnections, body1);

        if (body1.InternalIsland == body2.InternalIsland)
        {
            SplitIslands(islands, body1, body2);
        }
    }

private:
    static std::vector<std::unique_ptr<Island>>& Storage()
    {
        static std::vector<std::unique_ptr<Island>> storage;
        return storage;
    }

    static std::vector<Island*>& Pool()
    {
        thread_local std::vector<Island*> pool;
        return pool;
    }

    static std::queue<RigidBody*>& LeftSearchQueue()
    {
        thread_local std::queue<RigidBody*> queue;
        return queue;
    }

    static std::queue<RigidBody*>& RightSearchQueue()
    {
        thread_local std::queue<RigidBody*> queue;
        return queue;
    }

    static std::vector<RigidBody*>& VisitedBodiesLeft()
    {
        thread_local std::vector<RigidBody*> bodies;
        return bodies;
    }

    static std::vector<RigidBody*>& VisitedBodiesRight()
    {
        thread_local std::vector<RigidBody*> bodies;
        return bodies;
    }

    static void RemoveRef(std::vector<RigidBody*>& list, RigidBody& body)
    {
        const auto iterator = std::find(list.begin(), list.end(), &body);
        if (iterator == list.end())
        {
            return;
        }

        *iterator = list.back();
        list.pop_back();
    }

    static void SplitIslands(IslandSet& islands, RigidBody& body1, RigidBody& body2)
    {
        std::queue<RigidBody*>& leftSearchQueue = LeftSearchQueue();
        std::queue<RigidBody*>& rightSearchQueue = RightSearchQueue();
        std::vector<RigidBody*>& visitedBodiesLeft = VisitedBodiesLeft();
        std::vector<RigidBody*>& visitedBodiesRight = VisitedBodiesRight();

        const bool sourceIslandActive = islands.IsActive(*body1.InternalIsland);
        const bool sourceNeedsUpdate = body1.InternalIsland->NeedsUpdate;
        const bool sourceMarkedAsActive = body1.InternalIsland->MarkedAsActive;

        leftSearchQueue.push(&body1);
        rightSearchQueue.push(&body2);

        visitedBodiesLeft.push_back(&body1);
        visitedBodiesRight.push_back(&body2);

        body1.InternalIslandMarker = 1;
        body2.InternalIslandMarker = 2;

        const auto cleanup = [&]()
        {
            for (RigidBody* body : visitedBodiesLeft)
            {
                body->InternalIslandMarker = 0;
            }

            for (RigidBody* body : visitedBodiesRight)
            {
                body->InternalIslandMarker = 0;
            }

            leftSearchQueue = {};
            rightSearchQueue = {};
            visitedBodiesLeft.clear();
            visitedBodiesRight.clear();
        };

        while (!leftSearchQueue.empty() && !rightSearchQueue.empty())
        {
            RigidBody* currentNode = leftSearchQueue.front();
            leftSearchQueue.pop();
            if (currentNode->Data().MotionTypeValue() != MotionType::Static)
            {
                for (RigidBody* connectedNode : currentNode->InternalConnections)
                {
                    if (connectedNode->InternalIslandMarker == 0)
                    {
                        leftSearchQueue.push(connectedNode);
                        visitedBodiesLeft.push_back(connectedNode);
                        connectedNode->InternalIslandMarker = 1;
                    }
                    else if (connectedNode->InternalIslandMarker == 2)
                    {
                        cleanup();
                        return;
                    }
                }
            }

            currentNode = rightSearchQueue.front();
            rightSearchQueue.pop();
            if (currentNode->Data().MotionTypeValue() != MotionType::Static)
            {
                for (RigidBody* connectedNode : currentNode->InternalConnections)
                {
                    if (connectedNode->InternalIslandMarker == 0)
                    {
                        rightSearchQueue.push(connectedNode);
                        visitedBodiesRight.push_back(connectedNode);
                        connectedNode->InternalIslandMarker = 2;
                    }
                    else if (connectedNode->InternalIslandMarker == 1)
                    {
                        cleanup();
                        return;
                    }
                }
            }
        }

        Island& island = GetFromPool();
        island.NeedsUpdate = sourceNeedsUpdate;
        island.MarkedAsActive = sourceMarkedAsActive;
        islands.Add(island, sourceIslandActive);

        if (leftSearchQueue.empty())
        {
            for (RigidBody* body : visitedBodiesLeft)
            {
                body2.InternalIsland->InternalBodies.erase(body);
                island.InternalBodies.insert(body);
                body->InternalIsland = &island;
            }
        }
        else if (rightSearchQueue.empty())
        {
            for (RigidBody* body : visitedBodiesRight)
            {
                body1.InternalIsland->InternalBodies.erase(body);
                island.InternalBodies.insert(body);
                body->InternalIsland = &island;
            }
        }

        cleanup();
    }

    static void MergeIslands(IslandSet& islands, RigidBody& body1, RigidBody& body2)
    {
        if (body1.InternalIsland == body2.InternalIsland)
        {
            return;
        }

        const bool needsUpdate = body1.InternalIsland->NeedsUpdate || body2.InternalIsland->NeedsUpdate;
        const bool markedAsActive = body1.InternalIsland->MarkedAsActive || body2.InternalIsland->MarkedAsActive;

        RigidBody* smallIslandOwner = nullptr;
        RigidBody* largeIslandOwner = nullptr;

        if (body1.InternalIsland->InternalBodies.size() > body2.InternalIsland->InternalBodies.size())
        {
            smallIslandOwner = &body2;
            largeIslandOwner = &body1;
        }
        else
        {
            smallIslandOwner = &body1;
            largeIslandOwner = &body2;
        }

        Island& giveBackIsland = *smallIslandOwner->InternalIsland;

        ReturnToPool(giveBackIsland);
        islands.Remove(giveBackIsland);

        for (RigidBody* body : giveBackIsland.InternalBodies)
        {
            body->InternalIsland = largeIslandOwner->InternalIsland;
            largeIslandOwner->InternalIsland->InternalBodies.insert(body);
        }

        largeIslandOwner->InternalIsland->NeedsUpdate =
            largeIslandOwner->InternalIsland->NeedsUpdate || needsUpdate;
        largeIslandOwner->InternalIsland->MarkedAsActive =
            largeIslandOwner->InternalIsland->MarkedAsActive || markedAsActive;

        giveBackIsland.ClearLists();
    }
};

} // namespace Jitter2::Collision
