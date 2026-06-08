std::vector<unsigned char> ReadAllBytes(const std::string& filename)
{
    const std::array<std::string, 4> candidates {{
        filename,
        std::string("JitterDemo/") + filename,
        std::string("../JitterDemo/") + filename,
        std::string("../../JitterDemo/") + filename,
    }};

    for (const std::string& candidate : candidates)
    {
        std::ifstream stream(candidate, std::ios::binary);
        if (!stream)
        {
            continue;
        }

        return std::vector<unsigned char>(
            std::istreambuf_iterator<char>(stream),
            std::istreambuf_iterator<char>());
    }

    throw std::runtime_error("Unable to read file: " + filename);
}

std::uint16_t ReadLe16(const std::vector<unsigned char>& data, std::size_t offset)
{
    if (offset + 2 > data.size())
    {
        throw std::runtime_error("Unexpected end of little-endian uint16 data.");
    }

    return static_cast<std::uint16_t>(data[offset])
        | (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
}

std::uint32_t ReadLe32(const std::vector<unsigned char>& data, std::size_t offset)
{
    if (offset + 4 > data.size())
    {
        throw std::runtime_error("Unexpected end of little-endian uint32 data.");
    }

    return static_cast<std::uint32_t>(data[offset])
        | (static_cast<std::uint32_t>(data[offset + 1]) << 8U)
        | (static_cast<std::uint32_t>(data[offset + 2]) << 16U)
        | (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
}

std::string ReadZipSingleEntryText(const std::string& filename)
{
    const std::vector<unsigned char> data = ReadAllBytes(filename);
    if (data.size() < 30 || ReadLe32(data, 0) != 0x04034b50U)
    {
        throw std::runtime_error("Invalid ZIP local header: " + filename);
    }

    const std::uint16_t flags = ReadLe16(data, 6);
    const std::uint16_t compression = ReadLe16(data, 8);
    const std::uint32_t compressedSize = ReadLe32(data, 18);
    const std::uint32_t uncompressedSize = ReadLe32(data, 22);
    const std::uint16_t fileNameLength = ReadLe16(data, 26);
    const std::uint16_t extraLength = ReadLe16(data, 28);

    if ((flags & 0x0008U) != 0)
    {
        throw std::runtime_error("ZIP data descriptors are not supported: " + filename);
    }

    const std::size_t payloadOffset = 30U + fileNameLength + extraLength;
    if (payloadOffset + compressedSize > data.size())
    {
        throw std::runtime_error("Invalid ZIP compressed payload range: " + filename);
    }

    if (compression == 0)
    {
        return std::string(
            reinterpret_cast<const char*>(data.data() + payloadOffset),
            reinterpret_cast<const char*>(data.data() + payloadOffset + compressedSize));
    }

    if (compression != 8)
    {
        throw std::runtime_error("Only stored and deflated ZIP entries are supported: " + filename);
    }

    std::string result;
    result.resize(uncompressedSize);

    z_stream stream {};
    stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(data.data() + payloadOffset));
    stream.avail_in = compressedSize;
    stream.next_out = reinterpret_cast<Bytef*>(result.data());
    stream.avail_out = uncompressedSize;

    if (inflateInit2(&stream, -MAX_WBITS) != Z_OK)
    {
        throw std::runtime_error("Unable to initialize ZIP inflater: " + filename);
    }

    const int inflateResult = inflate(&stream, Z_FINISH);
    inflateEnd(&stream);

    if (inflateResult != Z_STREAM_END)
    {
        throw std::runtime_error("Unable to inflate ZIP entry: " + filename);
    }

    return result;
}

std::string ReadTextAsset(const std::string& filename)
{
    if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".zip")
    {
        return ReadZipSingleEntryText(std::string("assets/") + filename);
    }

    std::vector<unsigned char> bytes = ReadAllBytes(std::string("assets/") + filename);
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::vector<unsigned char> LoadTgaBgra(const std::string& filename, int& width, int& height)
{
    const std::vector<unsigned char> data = ReadAllBytes(filename);
    if (data.size() < 18)
    {
        throw std::runtime_error("Invalid TGA header: " + filename);
    }

    const int dataOffset = 18;
    const int imageType = data[2];
    width = (static_cast<int>(data[13]) << 8) | static_cast<int>(data[12]);
    height = (static_cast<int>(data[15]) << 8) | static_cast<int>(data[14]);
    const int bitsPerPixel = data[16];
    const int descriptor = data[17];

    if (!((imageType == 2 || imageType == 10) && (bitsPerPixel == 24 || bitsPerPixel == 32)))
    {
        throw std::runtime_error("Only 24-bit and 32-bit true color TGA images are supported: " + filename);
    }

    const int bytesPerPixel = bitsPerPixel / 8;
    const bool horizontalLower = (descriptor & 0x20) == 0;
    const bool verticalLower = (descriptor & 0x10) != 0;
    const int pixelCount = width * height;
    std::vector<unsigned char> decoded(static_cast<std::size_t>(4 * pixelCount));

    const auto flipImage =
        [width, height, horizontalLower, verticalLower](int index)
        {
            int row = index / width;
            int column = index - row * width;
            if (horizontalLower)
            {
                row = height - 1 - row;
            }
            if (verticalLower)
            {
                column = width - 1 - column;
            }
            return row * width + column;
        };

    int pixelIndex = 0;
    std::size_t position = dataOffset;
    while (pixelIndex < pixelCount)
    {
        int skip = 0;
        int count = pixelCount;

        if (imageType == 10)
        {
            if (position >= data.size())
            {
                throw std::runtime_error("Unexpected end of RLE TGA data: " + filename);
            }
            skip = (data[position] & 0x80U) != 0 ? 1 : 0;
            count = (data[position] & 0x7FU) + 1;
            position += 1;
        }

        for (int i = 0; i < count; ++i)
        {
            if (position + static_cast<std::size_t>(bytesPerPixel) > data.size())
            {
                throw std::runtime_error("Unexpected end of TGA pixel data: " + filename);
            }

            const int index = 4 * flipImage(pixelIndex++);
            decoded[static_cast<std::size_t>(index + 0)] = data[position + 0];
            decoded[static_cast<std::size_t>(index + 1)] = data[position + 1];
            decoded[static_cast<std::size_t>(index + 2)] = data[position + 2];
            decoded[static_cast<std::size_t>(index + 3)] =
                bytesPerPixel == 4 ? data[position + 3] : static_cast<unsigned char>(0);
            position += static_cast<std::size_t>(bytesPerPixel * (1 - skip));
        }

        position += static_cast<std::size_t>(bytesPerPixel * skip);
    }

    return decoded;
}

struct CpuMesh
{
    std::vector<Vertex> Vertices;
    std::vector<TriangleVertexIndex> Indices;
    std::vector<MeshGroup> Groups;

    void TransformScale(float scale)
    {
        for (Vertex& vertex : Vertices)
        {
            vertex.Position = vertex.Position * scale;
        }
    }
};

std::vector<std::string> SplitWhitespace(const std::string& line)
{
    std::istringstream stream(line);
    std::vector<std::string> parts;
    std::string part;
    while (stream >> part)
    {
        parts.push_back(part);
    }
    return parts;
}

std::vector<std::string> SplitSlash(const std::string& token)
{
    std::vector<std::string> parts;
    std::string current;
    for (char ch : token)
    {
        if (ch == '/')
        {
            parts.push_back(current);
            current.clear();
        }
        else
        {
            current.push_back(ch);
        }
    }
    parts.push_back(current);
    return parts;
}

CpuMesh LoadObj(const std::string& filename, float scale = 1.0f, bool reverseWinding = false)
{
    const std::string contents = ReadTextAsset(filename);
    std::istringstream lines(contents);
    std::string line;

    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;

    std::vector<std::string> trimmedLines;
    while (std::getline(lines, line))
    {
        const std::vector<std::string> parts = SplitWhitespace(line);
        if (parts.empty())
        {
            continue;
        }

        trimmedLines.push_back(line);
        if (parts[0] == "v" && parts.size() >= 4)
        {
            positions.push_back(Vec3 {
                std::stof(parts[1]),
                std::stof(parts[2]),
                std::stof(parts[3]),
            });
        }
        else if (parts[0] == "vn" && parts.size() >= 4)
        {
            normals.push_back(Vec3 {
                std::stof(parts[1]),
                std::stof(parts[2]),
                std::stof(parts[3]),
            });
        }
        else if (parts[0] == "vt" && parts.size() >= 3)
        {
            uvs.push_back(Vec2 {
                std::stof(parts[1]),
                std::stof(parts[2]),
            });
        }
    }

    const bool hasUvs = !uvs.empty();
    const bool hasNormals = !normals.empty();

    std::unordered_map<std::string, int> vertexCache;
    CpuMesh mesh;
    MeshGroup current {};
    bool groupOpen = false;

    const auto parseIndex =
        [](const std::string& value)
        {
            return std::stoi(value) - 1;
        };

    const auto addVertex =
        [&mesh, &positions, &normals, &uvs, hasNormals, hasUvs, &parseIndex](const std::string& token)
        {
            const std::vector<std::string> parts = SplitSlash(token);
            const Vec3 position = positions[static_cast<std::size_t>(parseIndex(parts[0]))];
            const Vec3 normal =
                hasNormals && parts.size() > 2 && !parts[2].empty()
                    ? normals[static_cast<std::size_t>(parseIndex(parts[2]))]
                    : Vec3 {1.0f, 0.0f, 0.0f};
            const Vec2 uv =
                hasUvs && parts.size() > 1 && !parts[1].empty()
                    ? uvs[static_cast<std::size_t>(parseIndex(parts[1]))]
                    : Vec2 {};
            mesh.Vertices.emplace_back(position, normal, uv);
            return static_cast<int>(mesh.Vertices.size() - 1);
        };

    const auto getOrAdd =
        [&vertexCache, &addVertex](const std::string& token)
        {
            const auto iterator = vertexCache.find(token);
            if (iterator != vertexCache.end())
            {
                return iterator->second;
            }

            const int index = addVertex(token);
            vertexCache.emplace(token, index);
            return index;
        };

    for (const std::string& rawLine : trimmedLines)
    {
        const std::vector<std::string> parts = SplitWhitespace(rawLine);
        if (parts.empty())
        {
            continue;
        }

        if (parts[0] == "o")
        {
            if (groupOpen)
            {
                current.ToExclusive = static_cast<int>(mesh.Indices.size());
                mesh.Groups.push_back(current);
            }
            groupOpen = true;
            current = MeshGroup {};
            current.FromInclusive = static_cast<int>(mesh.Indices.size());
            current.Name = parts.size() > 1 ? parts[1] : std::string();
        }
        else if (parts[0] == "f" && parts.size() >= 4)
        {
            const int i0 = getOrAdd(parts[1]);
            const int i1 = getOrAdd(parts[2]);
            const int i2 = getOrAdd(parts[3]);
            if (reverseWinding)
            {
                mesh.Indices.push_back(TriangleVertexIndex {
                    static_cast<std::uint32_t>(i1),
                    static_cast<std::uint32_t>(i0),
                    static_cast<std::uint32_t>(i2)});
            }
            else
            {
                mesh.Indices.push_back(TriangleVertexIndex {
                    static_cast<std::uint32_t>(i0),
                    static_cast<std::uint32_t>(i1),
                    static_cast<std::uint32_t>(i2)});
            }
        }
    }

    if (groupOpen)
    {
        current.ToExclusive = static_cast<int>(mesh.Indices.size());
        mesh.Groups.push_back(current);
    }

    if (scale != 1.0f)
    {
        mesh.TransformScale(scale);
    }

    return mesh;
}
