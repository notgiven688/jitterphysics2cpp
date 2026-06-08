#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Jitter2::DataStructures
{

template<typename TKey, typename TValue, typename THash = std::hash<TKey>, typename TEqual = std::equal_to<TKey>>
class ShardedDictionary
{
public:
    explicit ShardedDictionary(int threads = 1)
    {
        const int count = ShardSuggestion(threads);
        locks_.reserve(static_cast<std::size_t>(count));
        dictionaries_.reserve(static_cast<std::size_t>(count));

        for (int i = 0; i < count; ++i)
        {
            locks_.push_back(std::make_unique<std::mutex>());
            dictionaries_.emplace_back();
        }
    }

    // Gets the lock object for the shard containing the specified key.
    // key: The key to locate the shard for.
    // Returns: The lock object for the shard.
    [[nodiscard]] std::mutex& GetLock(const TKey& key)
    {
        return *locks_[static_cast<std::size_t>(GetShardIndex(key))];
    }

    [[nodiscard]] TValue* TryGetValue(const TKey& key)
    {
        auto& dictionary = dictionaries_[static_cast<std::size_t>(GetShardIndex(key))];
        auto iterator = dictionary.find(key);
        if (iterator == dictionary.end())
        {
            return nullptr;
        }

        return &iterator->second;
    }

    [[nodiscard]] const TValue* TryGetValue(const TKey& key) const
    {
        const auto& dictionary = dictionaries_[static_cast<std::size_t>(GetShardIndex(key))];
        auto iterator = dictionary.find(key);
        if (iterator == dictionary.end())
        {
            return nullptr;
        }

        return &iterator->second;
    }

    [[nodiscard]] TValue& operator[](const TKey& key)
    {
        return dictionaries_[static_cast<std::size_t>(GetShardIndex(key))].at(key);
    }

    // Adds a key-value pair to the dictionary.
    // key: The key to add.
    // value: The value to associate with the key.
    void Add(const TKey& key, TValue value)
    {
        auto& dictionary = dictionaries_[static_cast<std::size_t>(GetShardIndex(key))];
        auto [_, inserted] = dictionary.emplace(key, std::move(value));
        if (!inserted)
        {
            throw std::invalid_argument("An item with the same key has already been added.");
        }
    }

    [[nodiscard]] TValue Take(const TKey& key)
    {
        auto& dictionary = dictionaries_[static_cast<std::size_t>(GetShardIndex(key))];
        auto iterator = dictionary.find(key);
        if (iterator == dictionary.end())
        {
            return TValue {};
        }

        TValue value = std::move(iterator->second);
        dictionary.erase(iterator);
        return value;
    }

    // Removes the entry with the specified key from the dictionary.
    // key: The key to remove.
    void Remove(const TKey& key)
    {
        dictionaries_[static_cast<std::size_t>(GetShardIndex(key))].erase(key);
    }

    void Clear()
    {
        for (auto& dictionary : dictionaries_)
        {
            dictionary.clear();
        }
    }

    template<typename TAction>
    void ForEach(TAction&& action)
    {
        for (auto& dictionary : dictionaries_)
        {
            for (auto& [key, value] : dictionary)
            {
                action(key, value);
            }
        }
    }

private:
    [[nodiscard]] static int ShardSuggestion(int threads)
    {
        static constexpr std::array<int, 11> primes {3, 5, 7, 11, 17, 23, 29, 37, 47, 59, 71};

        for (int prime : primes)
        {
            if (prime >= threads)
            {
                return prime;
            }
        }

        return primes.back();
    }

    [[nodiscard]] int GetShardIndex(const TKey& key) const
    {
        return static_cast<int>(hasher_(key) % locks_.size());
    }

    THash hasher_;
    std::vector<std::unique_ptr<std::mutex>> locks_;

// A thread-safe dictionary that partitions entries across multiple shards to reduce lock contention.
// Each shard has its own lock, allowing concurrent access to different shards.
// TKey: The type of keys in the dictionary.
// TValue: The type of values in the dictionary.
// This implementation avoids the GC overhead of System.Collections.Concurrent.ConcurrentDictionary{TKey,TValue}.
// Threading: Individual operations are not thread-safe. Callers must acquire the shard lock
// via GetLock before calling Add, Remove, or the indexer.


    std::vector<std::unordered_map<TKey, TValue, THash, TEqual>> dictionaries_;
};

} // namespace Jitter2::DataStructures
