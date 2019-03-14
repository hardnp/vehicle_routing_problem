#include "tabu_search.h"
#include "objective.h"
#include "src/internal/tabu/local_search.h"

#include <cassert>

namespace vrp {
namespace detail {
namespace {
// constants
static constexpr const uint32_t TABU_SEARCH_ITERS = 100;
static constexpr const uint32_t MAX_ITERS =
    std::numeric_limits<uint32_t>::max();
static constexpr const uint32_t ROUTE_SAVING_ITERS = 5;
static constexpr const uint32_t INTRA_RELOCATION_ITERS = 15;
}  // namespace

Solution tabu_search(const Problem& prob, const Solution& initial_sln) {
    // capture-by-ref is guaranteed to work because Problem class doesn't change
    // at this point throughout the whole application run
    thread_local const auto sln_comp = [&prob](const auto& a, const auto& b) {
        return objective(prob, a) < objective(prob, b);
    };

    tabu::LocalSearchMethods ls(prob);

    Solution best_sln = initial_sln;
    // init temporary information:
    best_sln.update_customer_owners(prob);
    best_sln.update_used_vehicles();
    assert(!best_sln.customer_owners.empty());

    std::vector<Solution> slns = {best_sln, best_sln, best_sln, best_sln};
    assert(slns.size() == ls.size());

    // i - iterations counter, can be reset if improvement found
    // ci - constant iterations counter, always counts forward
    for (uint32_t i = 0, ci = 0; i < TABU_SEARCH_ITERS && ci < MAX_ITERS;
         ++i, ++ci) {
        for (size_t m = 0; m < ls.size(); ++m) {
            // re-write current solution
            ls[m](slns[m]);
        }
        auto min_sln_it =
            std::min_element(slns.cbegin(), slns.cend(), sln_comp);

        auto curr_sln = *min_sln_it;

        // found new best: reset best solution, reset iter counter to 0
        if (objective(prob, curr_sln) < objective(prob, best_sln)) {
            best_sln = std::move(curr_sln);
            i = 0;
        }

#if 0  // TODO: implement
        if (ci % ROUTE_SAVING_ITERS == 0) {}
        if (ci % INTRA_RELOCATION_ITERS == 0) {}
#endif
    }

    return best_sln;
}
}  // namespace detail
}  // namespace vrp
