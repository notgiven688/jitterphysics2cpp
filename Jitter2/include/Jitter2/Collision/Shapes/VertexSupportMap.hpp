#pragma once

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Jitter2/LinearMath/JVector.hpp>

namespace Jitter2::Collision::Shapes
{

// Implements a SIMD accelerated support map for a set of vertices.
class VertexSupportMap
{
public:

    // Initializes a new support map from a set of vertices.
    // vertices: The vertices defining the convex hull.
    // Thrown when vertices is empty.
    explicit VertexSupportMap(const std::vector<LinearMath::JVector>& vertices)
    {
        if (vertices.empty())
        {
            throw std::invalid_argument("Vertex set must contain at least one vertex.");
        }

        auto data = std::make_shared<Data>();
        data->Vertices = vertices;
        data->XValues.resize(vertices.size());
        data->YValues.resize(vertices.size());
        data->ZValues.resize(vertices.size());

        data->Center = LinearMath::JVector::Zero();
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const LinearMath::JVector& vertex = vertices[index];
            data->XValues[index] = vertex.X;
            data->YValues[index] = vertex.Y;
            data->ZValues[index] = vertex.Z;

            data->Center.X += vertex.X;
            data->Center.Y += vertex.Y;
            data->Center.Z += vertex.Z;
        }
        data->Center *= static_cast<Real>(1.0) / static_cast<Real>(vertices.size());

        data_ = std::move(data);
    }

    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const
    {
        Real maxDotProduct = std::numeric_limits<Real>::lowest();
        const std::size_t length = data_->XValues.size();
        std::size_t resultIndex = 0;

        for (std::size_t index = 0; index < length; ++index)
        {
            const Real dx = data_->XValues[index] * direction.X;
            const Real dy = data_->YValues[index] * direction.Y;
            const Real dz = data_->ZValues[index] * direction.Z;

            const Real dotProduct = dx + (dy + dz);
            if (dotProduct < maxDotProduct)
            {
                continue;
            }

            maxDotProduct = dotProduct;
            resultIndex = index;
        }

        result = LinearMath::JVector(
            data_->XValues[resultIndex],
            data_->YValues[resultIndex],
            data_->ZValues[resultIndex]);
    }

    void GetCenter(LinearMath::JVector& point) const
    {
        point = data_->Center;
    }

    [[nodiscard]] const std::vector<LinearMath::JVector>& Vertices() const { return data_->Vertices; }

private:
    struct Data
    {
        std::vector<Real> XValues;
        std::vector<Real> YValues;
        std::vector<Real> ZValues;
        std::vector<LinearMath::JVector> Vertices;
        LinearMath::JVector Center = LinearMath::JVector::Zero();
    };

    std::shared_ptr<Data> data_;
};

} // namespace Jitter2::Collision::Shapes
