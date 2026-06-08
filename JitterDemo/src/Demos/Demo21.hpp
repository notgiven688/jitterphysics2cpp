// Member functions for DemoScene; included inside class DemoScene.

    void BuildVoxelWorldScene()
    {
        VoxelProxy = std::make_unique<VoxelWorld>();
        World.DynamicTree().AddProxy(*VoxelProxy, false);

        VoxelBroadPhaseFilter = std::make_unique<VoxelCollisionFilter>(World, *VoxelProxy);
        World.BroadPhaseFilter = VoxelBroadPhaseFilter.get();

        VoxelRenderer = std::make_unique<InstancedDrawable>(CreateCubeMesh());
    }

    void DrawVoxelWorld(Vec3 cameraPosition, Vec3 cameraDirection)
    {
        if (VoxelRenderer == nullptr)
        {
            return;
        }

        constexpr int chunkSize = 16;
        constexpr int renderRadius = 8;

        const int camChunkX = static_cast<int>(std::floor(cameraPosition.X / static_cast<float>(chunkSize)));
        const int camChunkZ = static_cast<int>(std::floor(cameraPosition.Z / static_cast<float>(chunkSize)));

        std::vector<ChunkKey> chunksNeeded;
        for (int x = -renderRadius; x <= renderRadius; ++x)
        {
            for (int z = -renderRadius; z <= renderRadius; ++z)
            {
                if (x * x + z * z > renderRadius * renderRadius)
                {
                    continue;
                }

                ChunkKey key {camChunkX + x, camChunkZ + z};
                if (VoxelChunkCache.find(key) == VoxelChunkCache.end())
                {
                    chunksNeeded.push_back(key);
                }
            }
        }

        if (!chunksNeeded.empty())
        {
            const int chunkCount = static_cast<int>(chunksNeeded.size());
            const int numTasks = std::min(
                chunkCount,
                Jitter2::Parallelization::ThreadPool::Instance().ThreadCount());

            const auto buildChunks = [this, &chunksNeeded](Jitter2::Parallelization::Batch batch)
            {
                for (int i = batch.Start; i < batch.End; ++i)
                {
                    const ChunkKey key = chunksNeeded[static_cast<std::size_t>(i)];
                    std::vector<VoxelRenderData> voxels = BuildVoxelChunk(key.X, key.Z);

                    std::lock_guard lock(VoxelChunkCacheMutex);
                    VoxelChunkCache.try_emplace(key, std::move(voxels));
                }
            };

            if (numTasks <= 1)
            {
                buildChunks(Jitter2::Parallelization::Batch {0, chunkCount});
            }
            else
            {
                Jitter2::Parallelization::ForBatch(0, chunkCount, numTasks, buildChunks);
            }
        }

        for (int x = -renderRadius; x <= renderRadius; ++x)
        {
            for (int z = -renderRadius; z <= renderRadius; ++z)
            {
                if (x * x + z * z > renderRadius * renderRadius)
                {
                    continue;
                }

                ChunkKey key {camChunkX + x, camChunkZ + z};

                const Vec3 chunkCenter {
                    static_cast<float>(key.X * chunkSize) + static_cast<float>(chunkSize) * 0.5f,
                    cameraPosition.Y,
                    static_cast<float>(key.Z * chunkSize) + static_cast<float>(chunkSize) * 0.5f,
                };

                if (!IsVoxelChunkInView(chunkCenter, cameraPosition, cameraDirection))
                {
                    continue;
                }

                const auto iterator = VoxelChunkCache.find(key);
                if (iterator == VoxelChunkCache.end())
                {
                    continue;
                }

                for (const VoxelRenderData& v : iterator->second)
                {
                    const JVector position(
                        static_cast<Jitter2::Real>(v.X) + static_cast<Jitter2::Real>(0.5),
                        static_cast<Jitter2::Real>(v.Y) + static_cast<Jitter2::Real>(0.5),
                        static_cast<Jitter2::Real>(v.Z) + static_cast<Jitter2::Real>(0.5));
                    VoxelRenderer->Push(
                        Translation(position),
                        VoxelPalette()[static_cast<std::size_t>(v.Type)]);
                }
            }
        }

        ++VoxelFrameCount;
        if (VoxelFrameCount > 60)
        {
            VoxelFrameCount = 0;
            CleanupVoxelCache(camChunkX, camChunkZ);
        }
    }

    static bool IsVoxelChunkInView(Vec3 chunkCenter, Vec3 cameraPosition, Vec3 cameraDirection)
    {
        const Vec3 toChunk = chunkCenter - cameraPosition;
        if (Dot(toChunk, toChunk) < 32.0f * 32.0f)
        {
            return true;
        }

        return Dot(cameraDirection, Normalize(toChunk)) > 0.5f;
    }

    std::vector<VoxelRenderData> BuildVoxelChunk(int cx, int cz)
    {
        std::vector<VoxelRenderData> list;
        {
            std::lock_guard lock(VoxelListPoolMutex);
            if (!VoxelListPool.empty())
            {
                list = std::move(VoxelListPool.back());
                VoxelListPool.pop_back();
            }
        }

        if (list.capacity() < 512)
        {
            list.reserve(512);
        }
        list.clear();

        constexpr int chunkSize = 16;
        const int startX = cx * chunkSize;
        const int startZ = cz * chunkSize;
        const int endX = startX + chunkSize;
        const int endZ = startZ + chunkSize;

        for (int x = startX; x < endX; ++x)
        {
            for (int z = startZ; z < endZ; ++z)
            {
                const float h = VoxelWorld::GetHeight(x, z);

                const int maxY = static_cast<int>(h) + 2;
                const int minY = VoxelWorld::MinHeight - 2;

                for (int y = minY; y <= maxY; ++y)
                {
                    if (!VoxelWorld::IsSolid(x, y, z, h))
                    {
                        continue;
                    }

                    const bool topAir = !VoxelWorld::IsSolid(x, y + 1, z, h);
                    const bool bottomAir = !VoxelWorld::IsSolid(x, y - 1, z, h);
                    bool exposed = topAir || bottomAir;

                    if (!exposed)
                    {
                        if (!VoxelWorld::IsSolid(x + 1, y, z)
                            || !VoxelWorld::IsSolid(x - 1, y, z)
                            || !VoxelWorld::IsSolid(x, y, z + 1)
                            || !VoxelWorld::IsSolid(x, y, z - 1))
                        {
                            exposed = true;
                        }
                    }

                    if (exposed)
                    {
                        VoxelType type = VoxelType::Rock;

                        if (topAir)
                        {
                            if (y > 10)
                            {
                                type = VoxelType::Snow;
                            }
                            else if (y < -5)
                            {
                                type = VoxelType::Rock;
                            }
                            else
                            {
                                type = VoxelType::Grass;
                            }
                        }

                        list.push_back(VoxelRenderData {x, y, z, type});
                    }
                }
            }
        }

        return list;
    }

    void CleanupVoxelCache(int camX, int camZ)
    {
        constexpr int renderRadius = 8;
        const int cleanupRadius = renderRadius + 4;
        const int distSq = cleanupRadius * cleanupRadius;
        std::vector<ChunkKey> toRemove;
        std::vector<std::vector<VoxelRenderData>> removedLists;

        {
            std::lock_guard cacheLock(VoxelChunkCacheMutex);

            for (const auto& entry : VoxelChunkCache)
            {
                const int dx = entry.first.X - camX;
                const int dz = entry.first.Z - camZ;
                if (dx * dx + dz * dz > distSq)
                {
                    toRemove.push_back(entry.first);
                }
            }

            for (const ChunkKey& key : toRemove)
            {
                auto iterator = VoxelChunkCache.find(key);
                if (iterator == VoxelChunkCache.end())
                {
                    continue;
                }

                removedLists.push_back(std::move(iterator->second));
                VoxelChunkCache.erase(iterator);
            }
        }

        std::lock_guard poolLock(VoxelListPoolMutex);
        for (std::vector<VoxelRenderData>& list : removedLists)
        {
            VoxelListPool.push_back(std::move(list));
        }
    }

    static const std::array<Vec3, 3>& VoxelPalette()
    {
        static constexpr std::array<Vec3, 3> palette {{
            Vec3 {0.1f, 0.5f, 0.1f},
            Vec3 {0.3f, 0.3f, 0.3f},
            Vec3 {0.9f, 0.9f, 0.9f},
        }};
        return palette;
    }
