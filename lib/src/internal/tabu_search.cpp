#include "tabu_search.h"
#include "constraints.h"
#include "objective.h"

#include "src/internal/tabu/local_search.h"
#include "src/internal/tabu/tabu_lists.h"

#include "threading.hpp"

#include <cassert>
#include <stdexcept>
#include <unordered_map>

namespace vrp {
namespace detail {
namespace {
// constants
static constexpr const uint32_t TABU_SEARCH_ITERS = 100;
static constexpr const uint32_t MAX_ITERS =
    std::numeric_limits<uint32_t>::max();
static constexpr const uint32_t ROUTE_SAVING_ITERS = 5;
static constexpr const uint32_t ROUTE_SAVING_THRESHOLD = 7;
static constexpr const uint32_t INTRA_RELOCATION_ITERS = 15;

void update_tabu_lists(tabu::TabuLists& lists, const tabu::TabuLists& new_lists,
                       size_t i) {
    switch (i) {
    case 0:
        lists.relocate = std::move(new_lists.relocate);
        break;
    case 1:
        lists.relocate_split = std::move(new_lists.relocate_split);
        break;
    case 2:
        lists.exchange = std::move(new_lists.exchange);
        break;
    case 3:
        lists.two_opt = std::move(new_lists.two_opt);
        break;
    default:
        throw std::out_of_range("tabu list index out of range");
    }
}
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
    tabu::TabuLists lists{};
    assert(slns.size() == ls.size());

    // i - iterations counter, can be reset if improvement found
    // ci - constant iterations counter, always counts forward
    for (uint32_t i = 0, ci = 0; i < TABU_SEARCH_ITERS && ci < MAX_ITERS;
         ++i, ++ci) {
        auto updated_lists = lists;
        threading::parallel_for(ls.size(), [&](size_t m) {
            // for (size_t m = 0; m < ls.size(); ++m) {
            // re-write current solution
            ls[m](slns[m], updated_lists);
        });

        auto min_sln_it =
            std::min_element(slns.cbegin(), slns.cend(), sln_comp);

        --lists;

        update_tabu_lists(lists, updated_lists,
                          std::distance(slns.cbegin(), min_sln_it));

        auto curr_sln = *min_sln_it;

        auto curr_sln_copy = curr_sln;

        // found new best: reset best solution, reset iter counter to 0
        if (objective(prob, curr_sln) < objective(prob, best_sln)) {
            best_sln = std::move(curr_sln);
            i = 0;
        }

        const bool perform_route_saving = ci % ROUTE_SAVING_ITERS == 0;
        const bool perform_intra_relocation = i > INTRA_RELOCATION_ITERS;

        if (perform_route_saving) {
            ls.route_save(curr_sln_copy, ROUTE_SAVING_THRESHOLD);
        }
        if (perform_intra_relocation) {
            ls.intra_relocate(curr_sln_copy);
        }

        // update all solutions to current best
        for (auto& sln : slns) {
            sln = curr_sln_copy;
        }

        // no additional improvements were performed
        if (!perform_route_saving && !perform_intra_relocation) {
            continue;
        }

        if (objective(prob, curr_sln_copy) < objective(prob, best_sln)) {
            best_sln = std::move(curr_sln_copy);
            i = 0;
        }
    }

    return best_sln;
}
}  // namespace detail
}  // namespace vrp
