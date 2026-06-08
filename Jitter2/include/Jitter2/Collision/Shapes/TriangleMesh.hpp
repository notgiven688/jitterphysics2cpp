#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Jitter2/LinearMath/JTriangle.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a triangle mesh defined by a collection of vertices and triangle indices.
class TriangleMesh
{
public:
    class DegenerateTriangleException final : public std::runtime_error
    {
    public:
        explicit DegenerateTriangleException(const LinearMath::JTriangle& triangle)
            : std::runtime_error(CreateMessage(triangle)),
              Triangle(triangle)
        {
        }

        LinearMath::JTriangle Triangle;

    private:
        static std::string CreateMessage(const LinearMath::JTriangle& triangle)
        {
            std::ostringstream message;
            message << "Degenerate triangle found: " << triangle << ".";
            return message.str();
        }
    };

    struct Triangle
    {
        int IndexA = 0;
        int IndexB = 0;
        int IndexC = 0;
        int NeighborA = -1;
        int NeighborB = -1;
        int NeighborC = -1;
        LinearMath::JVector Normal = LinearMath::JVector::Zero();
    };

    // Creates a mesh from a "soup" of triangles. Vertices are automatically identified and deduplicated.
    // soup: The triangles used to build the mesh.
    // ignoreDegenerated: If true, degenerate triangles are skipped.
    // Thrown when a degenerate triangle is found and ignoreDegenerated is false.

    TriangleMesh(const std::vector<LinearMath::JTriangle>& soup, bool ignoreDegenerate = false)
    {
        BuildFromSoup(soup, ignoreDegenerate);
    }

    TriangleMesh(

    // Gets the vertices of the mesh.

        const std::vector<LinearMath::JVector>& vertices,

    // Gets the triangle indices of the mesh.

        const std::vector<int>& indices,
        bool ignoreDegenerate = false)
    {
        BuildFromIndexed(vertices, indices, ignoreDegenerate);
    }

    TriangleMesh(
        const std::vector<LinearMath::JVector>& vertices,
        const std::vector<std::uint16_t>& indices,
        bool ignoreDegenerate = false)
    {
        BuildFromIndexed(vertices, ConvertIndices(indices), ignoreDegenerate);
    }

    TriangleMesh(
        const std::vector<LinearMath::JVector>& vertices,
        const std::vector<std::uint32_t>& indices,
        bool ignoreDegenerate = false)
    {
        BuildFromIndexed(vertices, ConvertIndices(indices), ignoreDegenerate);
    }

    [[nodiscard]] const std::vector<LinearMath::JVector>& Vertices() const { return vertices_; }
    [[nodiscard]] const std::vector<Triangle>& Indices() const { return indices_; }

private:
    struct Edge
    {
        int IndexA = 0;
        int IndexB = 0;

        bool operator==(const Edge& other) const
        {
            return IndexA == other.IndexA && IndexB == other.IndexB;
        }
    };

    struct EdgeHash
    {
        std::size_t operator()(const Edge& edge) const
        {
            const std::size_t h1 = std::hash<int> {}(edge.IndexA);
            const std::size_t h2 = std::hash<int> {}(edge.IndexB);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
        }
    };

    int FindOrAddVertex(const LinearMath::JVector& vertex)
    {
        for (std::size_t index = 0; index < vertices_.size(); ++index)
        {
            if (vertices_[index] == vertex)
            {
                return static_cast<int>(index);
            }
        }

        vertices_.push_back(vertex);
        return static_cast<int>(vertices_.size() - 1);
    }

    template<typename TIndex>
    static std::vector<int> ConvertIndices(const std::vector<TIndex>& indices)
    {
        std::vector<int> result;
        result.reserve(indices.size());
        for (TIndex index : indices)
        {
            result.push_back(static_cast<int>(index));
        }
        return result;
    }

    void AddTriangle(int indexA, int indexB, int indexC, bool ignoreDegenerate)
    {
        const LinearMath::JVector& a = vertices_.at(static_cast<std::size_t>(indexA));
        const LinearMath::JVector& b = vertices_.at(static_cast<std::size_t>(indexB));
        const LinearMath::JVector& c = vertices_.at(static_cast<std::size_t>(indexC));
        LinearMath::JVector normal = LinearMath::JVector::Cross(b - a, c - a);

        if (LinearMath::MathHelper::CloseToZero(normal, static_cast<Real>(1e-12)))
        {
            if (ignoreDegenerate)
            {
                return;
            }
            throw DegenerateTriangleException(LinearMath::JTriangle(a, b, c));
        }

        normal.Normalize();
        Triangle triangle;
        triangle.IndexA = indexA;
        triangle.IndexB = indexB;
        triangle.IndexC = indexC;
        triangle.Normal = normal;
        indices_.push_back(triangle);
    }

    void BuildFromSoup(const std::vector<LinearMath::JTriangle>& soup, bool ignoreDegenerate)
    {
        vertices_.clear();
        indices_.clear();

        for (const LinearMath::JTriangle& triangle : soup)
        {
            const int a = FindOrAddVertex(triangle.V0);
            const int b = FindOrAddVertex(triangle.V1);
            const int c = FindOrAddVertex(triangle.V2);
            AddTriangle(a, b, c, ignoreDegenerate);
        }

        AssignNeighbors();
    }

    void BuildFromIndexed(
        const std::vector<LinearMath::JVector>& vertices,
        const std::vector<int>& indices,
        bool ignoreDegenerate)
    {
        if (indices.size() % 3 != 0)
        {
            throw std::invalid_argument("Indices must be a multiple of 3.");
        }

        vertices_.clear();
        indices_.clear();
        std::vector<int> remap(vertices.size());
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            remap[index] = FindOrAddVertex(vertices[index]);
        }

        for (std::size_t index = 0; index < indices.size(); index += 3)
        {
            const int inputA = indices[index];
            const int inputB = indices[index + 1];
            const int inputC = indices[index + 2];
            if (inputA < 0 || inputB < 0 || inputC < 0
                || static_cast<std::size_t>(inputA) >= vertices.size()
                || static_cast<std::size_t>(inputB) >= vertices.size()
                || static_cast<std::size_t>(inputC) >= vertices.size())
            {
                std::ostringstream message;
                message << "Indices " << inputA << "," << inputB << "," << inputC << " out of bounds.";
                throw std::out_of_range(message.str());
            }

            AddTriangle(
                remap[static_cast<std::size_t>(inputA)],
                remap[static_cast<std::size_t>(inputB)],
                remap[static_cast<std::size_t>(inputC)],
                ignoreDegenerate);
        }

        AssignNeighbors();
    }

    void AssignNeighbors()
    {
        std::unordered_map<Edge, int, EdgeHash> edgeToTriangle;

        for (std::size_t i = 0; i < indices_.size(); ++i)
        {
            const Triangle& triangle = indices_[i];
            const int index = static_cast<int>(i);
            edgeToTriangle.try_emplace(Edge {triangle.IndexA, triangle.IndexB}, index);
            edgeToTriangle.try_emplace(Edge {triangle.IndexB, triangle.IndexC}, index);
            edgeToTriangle.try_emplace(Edge {triangle.IndexC, triangle.IndexA}, index);
        }

        for (Triangle& triangle : indices_)
        {
            triangle.NeighborA = FindNeighbor(edgeToTriangle, Edge {triangle.IndexC, triangle.IndexB});
            triangle.NeighborB = FindNeighbor(edgeToTriangle, Edge {triangle.IndexA, triangle.IndexC});
            triangle.NeighborC = FindNeighbor(edgeToTriangle, Edge {triangle.IndexB, triangle.IndexA});
        }
    }

    static int FindNeighbor(const std::unordered_map<Edge, int, EdgeHash>& edgeToTriangle, const Edge& edge)
    {
        const auto iterator = edgeToTriangle.find(edge);
        return iterator != edgeToTriangle.end() ? iterator->second : -1;
    }

    std::vector<LinearMath::JVector> vertices_;
    std::vector<Triangle> indices_;
};

} // namespace Jitter2::Collision::Shapes
