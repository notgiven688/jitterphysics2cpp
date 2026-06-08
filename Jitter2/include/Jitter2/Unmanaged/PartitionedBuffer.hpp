#pragma once

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace Jitter2::Unmanaged
{

template<typename T>
class PartitionedBuffer;

template<typename T>
class JHandle
{
public:
    constexpr JHandle() = default;

    [[nodiscard]] static constexpr JHandle Zero()
    {
        return JHandle();
    }

    [[nodiscard]] bool IsZero() const
    {
        return slot_ == nullptr;
    }

    T& Data() const
    {
        return **slot_;
    }

    bool operator==(const JHandle& other) const
    {
        return slot_ == other.slot_;
    }

    bool operator!=(const JHandle& other) const
    {
        return !(*this == other);
    }

private:
    explicit JHandle(T** slot) : slot_(slot) {}

    T** slot_ = nullptr;

    friend class PartitionedBuffer<T>;
};

template<typename T>
class PartitionedBuffer
{
public:
    class MaximumSizeException : public std::runtime_error
    {
    public:
        explicit MaximumSizeException(const char* message) : std::runtime_error(message) {}
    };

    explicit PartitionedBuffer(std::size_t initialSize = 1024, bool aligned64 = false)
        : aligned64_(aligned64 && alignof(T) >= 64 && sizeof(T) % 64 == 0)
    {
        static_assert(std::is_trivially_copyable_v<T>, "PartitionedBuffer<T> requires trivially copyable data.");
        if (sizeof(T) < sizeof(int))
        {
            throw std::invalid_argument("PartitionedBuffer<T> data must reserve at least four bytes for an internal id.");
        }

        if (initialSize == 0)
        {
            initialSize = 1;
        }

        memory_.resize(initialSize);
        EnsureHandleCapacity(initialSize);
        for (std::size_t i = 0; i < initialSize; ++i)
        {
            InternalId(memory_[i]) = static_cast<int>(i);
        }
    }

    PartitionedBuffer(const PartitionedBuffer&) = delete;
    PartitionedBuffer& operator=(const PartitionedBuffer&) = delete;
    PartitionedBuffer(PartitionedBuffer&&) = delete;
    PartitionedBuffer& operator=(PartitionedBuffer&&) = delete;

    [[nodiscard]] std::size_t Count() const { return count_; }
    [[nodiscard]] std::size_t ActiveCount() const { return activeCount_; }
    [[nodiscard]] std::size_t Capacity() const { return memory_.size(); }
    [[nodiscard]] bool Aligned64() const { return aligned64_; }

    [[nodiscard]] std::size_t TotalBytesAllocated() const
    {
        return memory_.size() * sizeof(T) + slots_.size() * sizeof(T*);
    }

    std::span<T> Active()
    {
        return std::span<T>(memory_.data(), activeCount_);
    }

    std::span<const T> Active() const
    {
        return std::span<const T>(memory_.data(), activeCount_);
    }

    std::span<T> Inactive()
    {
        return std::span<T>(memory_.data() + activeCount_, count_ - activeCount_);
    }

    std::span<const T> Inactive() const
    {
        return std::span<const T>(memory_.data() + activeCount_, count_ - activeCount_);
    }

    std::span<T> Elements()
    {
        return std::span<T>(memory_.data(), count_);
    }

    std::span<const T> Elements() const
    {
        return std::span<const T>(memory_.data(), count_);
    }

    JHandle<T> Allocate(bool active = false, bool clear = false)
    {
        if (count_ == memory_.size())
        {
            Resize(memory_.size() * 2);
        }

        const int handleId = InternalId(memory_[count_]);
        T** slot = GetHandleSlot(handleId);
        *slot = &memory_[count_];

        // Reinterprets a handle as a handle to a different type. Both types must have compatible layouts.
        // TConvert: The target unmanaged type to reinterpret as.
        // handle: The handle to reinterpret.
        // Returns: A handle reinterpreted as the target type.
        // Safety: The caller must ensure that T and TConvert
        // have compatible memory layouts. No runtime validation is performed.
        JHandle<T> handle(slot);
        if (clear)
        {
            ClearPayload(handle.Data());
        }

        ++count_;
        if (active)
        {

            // Moves an object from inactive to active.
            // handle: The handle of the element to move.
            MoveToActive(handle);
        }

        return handle;
    }

    // Removes the associated native structure from the buffer and invalidates the handle.
    // handle: The handle to free.
    // Safety: After calling this method, the handle becomes invalid.
    // Do not use the handle or any cached references to its data.
    void Free(JHandle<T> handle)
    {
        if (handle.IsZero())
        {
            return;
        }

        // Moves an object from active to inactive.
        // handle: The handle of the element to move.
        MoveToInactive(handle);
        --count_;
        SwapByPointer(*handle.slot_, &memory_[count_]);
    }

    // Checks if the element is stored as an active element. O(1).
    // handle: The handle to check.
    // Returns: true if the element is active; otherwise, false.
    bool IsActive(JHandle<T> handle) const
    {
        if (handle.IsZero())
        {
            return false;
        }

        // Gets the index of the element referred to by the handle.
        // handle: The handle to get the index for.
        // Returns: The index of the element in the buffer.
        return GetIndex(handle) < activeCount_;
    }

    void MoveToActive(JHandle<T> handle)
    {
        const std::size_t index = GetIndex(handle);
        if (index < activeCount_)
        {
            return;
        }

        // Swap two entries based on their index. Adjusts handles accordingly.
        // i: The index of the first element.
        // j: The index of the second element.
        Swap(index, activeCount_);
        ++activeCount_;
    }

    void MoveToInactive(JHandle<T> handle)
    {
        const std::size_t index = GetIndex(handle);
        if (index >= activeCount_)
        {
            return;
        }

        --activeCount_;
        Swap(index, activeCount_);
    }

    void Swap(std::size_t first, std::size_t second)
    {
        if (first == second)
        {
            return;
        }

        std::swap(memory_[first], memory_[second]);
        PublishSlot(memory_[first], first);
        PublishSlot(memory_[second], second);
    }

    [[nodiscard]] std::size_t GetIndex(JHandle<T> handle) const
    {
        return static_cast<std::size_t>(*handle.slot_ - memory_.data());
    }

    // Returns the handle of the object. O(1) operation.
    // t: A reference to the element in the buffer.
    // Returns: The handle for the element.
    JHandle<T> GetHandle(T& data)
    {
        return JHandle<T>(GetHandleSlot(InternalId(data)));
    }

private:
    static int& InternalId(T& value)
    {
        return *reinterpret_cast<int*>(&value);
    }

    static const int& InternalId(const T& value)
    {
        return *reinterpret_cast<const int*>(&value);
    }

    static void ClearPayload(T& value)
    {
        if constexpr (sizeof(T) > sizeof(int))
        {
            auto* bytes = reinterpret_cast<std::byte*>(&value);
            std::memset(bytes + sizeof(int), 0, sizeof(T) - sizeof(int));
        }
    }

    void EnsureHandleCapacity(std::size_t requiredCount)
    {
        while (slots_.size() < requiredCount)
        {
            slots_.push_back(std::make_unique<T*>(nullptr));
        }
    }

    T** GetHandleSlot(int id)
    {
        return slots_.at(static_cast<std::size_t>(id)).get();
    }

    void PublishSlot(T& value, std::size_t index)
    {
        *GetHandleSlot(InternalId(value)) = &memory_[index];
    }

    void Resize(std::size_t newSize)
    {
        if (newSize <= memory_.size())
        {
            return;
        }

        if (newSize > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        {
            throw MaximumSizeException("PartitionedBuffer internal id limit reached.");
        }

        const std::size_t oldSize = memory_.size();
        memory_.resize(newSize);
        EnsureHandleCapacity(newSize);

        for (std::size_t i = oldSize; i < newSize; ++i)
        {
            InternalId(memory_[i]) = static_cast<int>(i);
        }

        for (std::size_t i = 0; i < count_; ++i)
        {
            PublishSlot(memory_[i], i);
        }
    }

    void SwapByPointer(T* first, T* second)
    {
        const std::size_t firstIndex = static_cast<std::size_t>(first - memory_.data());
        const std::size_t secondIndex = static_cast<std::size_t>(second - memory_.data());
        Swap(firstIndex, secondIndex);
    }

    std::vector<T> memory_;
    std::vector<std::unique_ptr<T*>> slots_;
    std::size_t count_ = 0;
    std::size_t activeCount_ = 0;
    bool aligned64_ = false;
};

} // namespace Jitter2::Unmanaged
