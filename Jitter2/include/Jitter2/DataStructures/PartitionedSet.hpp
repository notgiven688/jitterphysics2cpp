#pragma once

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace Jitter2::DataStructures
{

template<typename T>
class PartitionedSet
{
public:
    [[nodiscard]] std::size_t ActiveCount() const { return activeCount_; }
    [[nodiscard]] std::size_t Count() const { return elements_.size(); }

    [[nodiscard]] T& operator[](std::size_t index) const { return *elements_[index]; }

    [[nodiscard]] std::span<T* const> Elements() const
    {
        return std::span<T* const>(elements_.data(), elements_.size());
    }

    [[nodiscard]] std::span<T* const> Active() const
    {
        return std::span<T* const>(elements_.data(), activeCount_);
    }

    [[nodiscard]] std::span<T* const> Inactive() const
    {
        return std::span<T* const>(elements_.data() + activeCount_, elements_.size() - activeCount_);
    }

    // Removes all elements from the set.
    void Clear()
    {
        for (T* element : elements_)
        {
            element->SetIndex = -1;
        }

        elements_.clear();
        activeCount_ = 0;
    }

    void Add(T& element, bool active = false)
    {
        element.SetIndex = static_cast<int>(elements_.size());
        elements_.push_back(&element);

        if (active)
        {

            // Moves an element to the active partition.
            // element: The element to move.
            // Returns: true if the element was moved; false if it was already active.
            MoveToActive(element);
        }
    }

    // Determines whether the set contains the specified element.
    [[nodiscard]] bool Contains(const T& element) const
    {
        if (element.SetIndex < 0)
        {
            return false;
        }

        const auto index = static_cast<std::size_t>(element.SetIndex);
        return index < elements_.size() && elements_[index] == &element;
    }

    // Determines whether the specified element is in the active partition.
    [[nodiscard]] bool IsActive(const T& element) const
    {

        // Determines whether the set contains the specified element.
        // element: The element to locate.
        // Returns: true if the element is found; otherwise, false.
        return Contains(element) && static_cast<std::size_t>(element.SetIndex) < activeCount_;
    }

    bool MoveToActive(T& element)
    {
        if (!Contains(element) || static_cast<std::size_t>(element.SetIndex) < activeCount_)
        {
            return false;
        }

        Swap(activeCount_, static_cast<std::size_t>(element.SetIndex));
        ++activeCount_;
        return true;
    }

    // Moves an element to the inactive partition.
    // element: The element to move.
    // Returns: true if the element was moved; false if it was already inactive.
    bool MoveToInactive(T& element)
    {
        if (!Contains(element) || static_cast<std::size_t>(element.SetIndex) >= activeCount_)
        {
            return false;
        }

        --activeCount_;
        Swap(activeCount_, static_cast<std::size_t>(element.SetIndex));
        return true;
    }

    // Removes the specified element from the set.
    // element: The element to remove.
    void Remove(T& element)
    {
        if (!Contains(element))
        {
            return;
        }

        MoveToInactive(element);

        const auto index = static_cast<std::size_t>(element.SetIndex);
        const std::size_t last = elements_.size() - 1;
        if (index != last)
        {
            elements_[index] = elements_[last];
            elements_[index]->SetIndex = static_cast<int>(index);
        }

        elements_.pop_back();
        element.SetIndex = -1;
    }

private:
    void Swap(std::size_t first, std::size_t second)
    {
        if (first == second)
        {
            return;
        }

        std::swap(elements_[first], elements_[second]);
        elements_[first]->SetIndex = static_cast<int>(first);
        elements_[second]->SetIndex = static_cast<int>(second);
    }

    std::vector<T*> elements_;
    std::size_t activeCount_ = 0;
};

} // namespace Jitter2::DataStructures
