#ifndef SEARCH_EXAMPLE_MATH_UTILS__HPP__
#define SEARCH_EXAMPLE_MATH_UTILS__HPP__

#include "bits/stdint-uintn.h"

namespace search_example{
namespace utils{

template <typename T>
inline uint64_t pointIntToKey(const T& row, const T& col){
    return ((static_cast<uint64_t>(row) << 32) + static_cast<uint64_t>(col));
}

} // end namespace utils
} // end namespace search_example

#endif