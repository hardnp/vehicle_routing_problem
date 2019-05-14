#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
    namespace detail {
        Solution parallel_insertion(const Problem& prob, const size_t count);
    }  // namespace detail
}  // namespace vrp
