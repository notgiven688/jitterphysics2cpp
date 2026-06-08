#pragma once

#include <algorithm>
#include <functional>
#include <sstream>
#include <string>

namespace Jitter2::Parallelization
{

struct Batch
{
    int Start = 0;
    int End = 0;

    // Returns a string representation of the batch.
    [[nodiscard]] std::string ToString() const
    {
        std::ostringstream builder;
        builder << "Batch(Start: " << Start << ", End: " << End << ")";
        return builder.str();
    }
};

// Computes the start and end indices for a specific part of an evenly divided range.
// numElements: The total number of elements to divide.
// numDivisions: The number of divisions (parts).
// part: The zero-based index of the part.
// start: The inclusive start index for the specified part.
// end: The exclusive end index for the specified part.
// Distributes remainder elements across the first parts. For example, with 14 elements
// and 4 divisions: part 0 gets [0,4), part 1 gets [4,8), part 2 gets [8,11), part 3 gets [11,14).
void GetBounds(int numElements, int numDivisions, int part, int& start, int& end);
void ForBatch(
    int lower,
    int upper,
    int numTasks,
    const std::function<void(Batch)>& action,
    bool execute = true);

} // namespace Jitter2::Parallelization
