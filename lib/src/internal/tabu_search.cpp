#include "tabu_search.h"

namespace vrp {
namespace detail {
namespace {
static constexpr const int TABU_SEARCH_ITERATIONS = 100;
}

Solution tabu_search(const Problem& prob, const Solution& initial_sln) {
    Solution best_sln = initial_sln;
    // for (int i = 0; i < TABU_SEARCH_ITERATIONS; ++i) {
    // }
    return best_sln;
}
}  // namespace detail
}  // namespace vrp
