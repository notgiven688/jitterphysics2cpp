// Member functions for DemoScene; included inside class DemoScene.

    static std::vector<TriangleVertexIndex> GetHeightmapIndices(int width, int height)
    {
        std::vector<TriangleVertexIndex> indices;
        indices.reserve(static_cast<std::size_t>((width - 1) * (height - 1) * 2));

        for (int j = 0; j < height - 1; ++j)
        {
            for (int i = 0; i < width - 1; ++i)
            {
                const std::uint32_t a = static_cast<std::uint32_t>(j * width + i);
                const std::uint32_t b = static_cast<std::uint32_t>(j * width + (i + 1));
                const std::uint32_t c = static_cast<std::uint32_t>((j + 1) * width + i);
                const std::uint32_t d = static_cast<std::uint32_t>((j + 1) * width + (i + 1));

                indices.push_back(TriangleVertexIndex {b, a, c});
                indices.push_back(TriangleVertexIndex {d, b, c});
            }
        }

        return indices;
    }

    static void FillHeightmapVertices(std::vector<Vertex>& vertices, int width, int height)
    {
        for (int j = 0; j < height; ++j)
        {
            for (int i = 0; i < width; ++i)
            {
                const int index = j * width + i;
                vertices[static_cast<std::size_t>(index)].Position = Vec3 {
                    static_cast<float>(i),
                    Heightmap::GetHeight(i, j),
                    static_cast<float>(j),
                };
                vertices[static_cast<std::size_t>(index)].Texture = Vec2 {
                    static_cast<float>(i) * 0.5f,
                    static_cast<float>(j) * 0.5f,
                };
            }
        }
    }

    void BuildHeightmapScene()
    {
        HeightmapProxy = std::make_unique<HeightmapTester>(Heightmap::GetBoundingBox());

        HeightmapBroadPhaseFilter = std::make_unique<HeightmapDetection>(World, *HeightmapProxy);
        World.BroadPhaseFilter = HeightmapBroadPhaseFilter.get();
        World.DynamicTree().AddProxy(*HeightmapProxy, false);

        HeightmapRenderer = std::make_unique<MutableMeshDrawable>();
        HeightmapRenderer->SetTriangles(GetHeightmapIndices(Heightmap::Width, Heightmap::Height));
        HeightmapRenderer->DrawableMaterial = ClothMaterial::Create();
        FillHeightmapVertices(HeightmapRenderer->Mesh.Vertices, Heightmap::Width, Heightmap::Height);
        HeightmapRenderer->RefreshGeometry();
    }
