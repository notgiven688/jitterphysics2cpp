#pragma once

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <vector>

#include <Jitter2/Parallelization/ReaderWriterLock.hpp>

namespace Jitter2::DataStructures
{

template<typename T>
class SlimBag
{
public:
    explicit SlimBag(int initialSize = 4)
        : array_(static_cast<std::size_t>(std::max(1, initialSize)))
    {
    }

    [[nodiscard]] int InternalSize() const
    {
        return static_cast<int>(array_.size());
    }

    [[nodiscard]] int Count() const
    {
        return counter_.load(std::memory_order_acquire);
    }

    void Count(int value)
    {
        assert(value <= Count());
        counter_.store(value, std::memory_order_release);
    }

    // Adds an element to the SlimBag{T}.
    // item: The element to add.
    void Add(const T& item)
    {
        const int count = counter_.load(std::memory_order_relaxed);
        if (count == static_cast<int>(array_.size()))
        {
            array_.resize(array_.size() * 2);
        }

        array_[static_cast<std::size_t>(count)] = item;
        counter_.store(count + 1, std::memory_order_release);
    }

    // Adds an element to the SlimBag{T} in a thread-safe manner.
    // item: The element to add.
    void ConcurrentAdd(const T& item)
    {
        const int localCount = counter_.fetch_add(1, std::memory_order_acq_rel);

    again:
        rwLock_.EnterReadLock();

        if (localCount < static_cast<int>(array_.size()))
        {
            array_[static_cast<std::size_t>(localCount)] = item;
            rwLock_.ExitReadLock();
        }
        else
        {
            rwLock_.ExitReadLock();

            rwLock_.EnterWriteLock();
            if (localCount >= static_cast<int>(array_.size()))
            {
                array_.resize(array_.size() * 2);
            }
            rwLock_.ExitWriteLock();

            goto again;
        }
    }

    // Removes the first occurrence of a specific element from the SlimBag{T}.
    // item: The element to remove.
    void Remove(const T& item)
    {
        const int count = Count();
        int index = -1;
        for (int i = 0; i < count; ++i)
        {
            if (array_[static_cast<std::size_t>(i)] == item)
            {
                index = i;
                break;
            }
        }

        if (index != -1)
        {

            // Removes the element at the specified index from the SlimBag{T}.
            // index: The zero-based index of the element to remove.
            RemoveAt(index);
        }
    }

    void RemoveAt(int index)
    {
        const int count = counter_.fetch_sub(1, std::memory_order_acq_rel) - 1;
        array_[static_cast<std::size_t>(index)] = array_[static_cast<std::size_t>(count)];
    }

    [[nodiscard]] T& operator[](int index)
    {
        return array_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] const T& operator[](int index) const
    {
        return array_[static_cast<std::size_t>(index)];
    }

    // Removes all elements from the SlimBag{T}.
    void Clear()
    {
        counter_.store(0, std::memory_order_release);
    }

    // Nulls out one stale array slot per call to allow garbage collection of removed elements.
    // Tracks the high-water mark of Count. When elements are removed and
    // Count drops below that mark, each call clears one slot from the end
    // of the previously used range. Call this method repeatedly (e.g., once per step) to
    // amortize cleanup cost.
    void TrackAndNullOutOne()
    {
        const int count = Count();
        nullOut_ = std::max(nullOut_, count);
        if (nullOut_ <= count)
        {
            return;
        }

        array_[static_cast<std::size_t>(--nullOut_)] = T {};
    }

private:
    std::vector<T> array_;
    std::atomic<int> counter_ {0};
    int nullOut_ = 0;
    Parallelization::ReaderWriterLock rwLock_;
};

} // namespace Jitter2::DataStructures
