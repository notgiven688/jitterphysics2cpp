#pragma once

#include <cstddef>
#include <limits>

namespace Jitter2
{

// Provides constants and utilities related to floating-point precision configuration.
// The library can be compiled with single-precision (float) or double-precision (double)
// by defining the JITTER_DOUBLE_PRECISION symbol.

#if defined(JITTER_DOUBLE_PRECISION) && JITTER_DOUBLE_PRECISION
using Real = double;

// Gets a value indicating whether the engine is configured to use double-precision floating-point numbers.
inline constexpr bool IsDoublePrecision = true;

// The size in bytes of a full constraint data structure.
inline constexpr std::size_t ConstraintSizeFull = 512;

// The size in bytes of a small constraint data structure.
inline constexpr std::size_t ConstraintSizeSmall = 256;

// The size in bytes of the Dynamics::RigidBodyData structure.
inline constexpr std::size_t RigidBodyDataSize = 256;
#else
using Real = float;

// Gets a value indicating whether the engine is configured to use double-precision floating-point numbers.
inline constexpr bool IsDoublePrecision = false;

// The size in bytes of a full constraint data structure.
inline constexpr std::size_t ConstraintSizeFull = 256;

// The size in bytes of a small constraint data structure.
inline constexpr std::size_t ConstraintSizeSmall = 128;

// The size in bytes of the Dynamics::RigidBodyData structure.
inline constexpr std::size_t RigidBodyDataSize = 128;
#endif

inline constexpr Real RealZero = static_cast<Real>(0);
inline constexpr Real RealOne = static_cast<Real>(1);

} // namespace Jitter2
