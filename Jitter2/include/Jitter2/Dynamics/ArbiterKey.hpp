#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Jitter2
{

struct ArbiterKey
{
    std::uint64_t Key1 = 0;
    std::uint64_t Key2 = 0;

    bool operator==(const ArbiterKey& other) const
    {
        return Key1 == other.Key1 && Key2 == other.Key2;
    }

    bool operator!=(const ArbiterKey& other) const
    {
        return !(*this == other);
    }
};

struct ArbiterKeyHash
{
    std::size_t operator()(const ArbiterKey& key) const
    {
        const std::size_t h1 = std::hash<std::uint64_t> {}(key.Key1);
        const std::size_t h2 = std::hash<std::uint64_t> {}(key.Key2);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
    }
};

} // namespace Jitter2
