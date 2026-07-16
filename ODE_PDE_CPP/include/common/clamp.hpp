#pragma once

// This header provides a small helper function to clamp values to a min/max range.
// Keeping this separate improves readability in the simulation loops.

namespace common {

template <typename T>
T clamp(const T& x, const T& lo, const T& hi)
{
    // If x is smaller than the low bound, return lo.
    if (x < lo) return lo;

    // If x is larger than the high bound, return hi.
    if (x > hi) return hi;

    // Otherwise, x is already within bounds.
    return x;
}

} // namespace common