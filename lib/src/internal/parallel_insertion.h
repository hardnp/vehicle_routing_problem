#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>
#include <vector>

namespace vrp {
    namespace detail {
        std::vector<Solution> parallel_insertion(const Problem& prob,
                                                    size_t count);
    }  // namespace detail
}  // namespace vrp