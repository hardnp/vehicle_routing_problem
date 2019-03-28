#include "tabu_search.h"
#include "constraints.h"
#include "logging.h"
#include "objective.h"
#include "threading.h"

#include "src/internal/tabu/local_search.h"
#include "src/internal/tabu/tabu_lists.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace vrp {
namespace detail {
namespace {
// constants
constexpr const uint32_t TABU_SEARCH_ITERS = 100;
constexpr const uint32_t MAX_ITERS = std::numeric_limits<uint32_t>::max();
constexpr const uint32_t ROUTE_SAVING_ITERS = 5;
constexpr const uint32_t ROUTE_SAVING_THRESHOLD = 7;
constexpr const uint32_t INTRA_RELOCATION_ITERS = 15;
constexpr const double TIME_WINDOWS_PENALTY_BASE = 1.2;

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
    case 4:
        lists.cross = std::move(new_lists.cross);
        break;
    default:
        throw std::out_of_range("tabu list index out of range");
    }
}

inline void do_local_search(const tabu::LocalSearchMethods& ls,
                            std::vector<Solution>& slns,
                            tabu::TabuLists& lists) {
    assert(slns.size() == ls.size());
    threading::parallel_for(ls.size(),
                            [&](size_t m) { ls[m](slns[m], lists); });
}

inline std::vector<Solution> repeat(const Solution& sln, size_t times) {
    std::vector<Solution> slns(times);
    for (auto& s : slns) {
        s = sln;
    }
    return slns;
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

    std::vector<Solution> slns = repeat(best_sln, ls.size());
    tabu::TabuLists lists{};

    int tw_violation_count = 1;

    // i - iterations counter, can be reset if improvement found
    // ci - constant iterations counter, always counts forward
    for (uint32_t i = 0, ci = 0; i < TABU_SEARCH_ITERS && ci < MAX_ITERS;
         ++i, ++ci) {

        ls.penalize_tw(std::pow(TIME_WINDOWS_PENALTY_BASE, tw_violation_count));

        auto updated_lists = lists;
        do_local_search(ls, slns, updated_lists);

        auto min_sln_it =
            std::min_element(slns.cbegin(), slns.cend(), sln_comp);

        --lists;

        update_tabu_lists(lists, updated_lists,
                          std::distance(slns.cbegin(), min_sln_it));

        auto curr_sln = *min_sln_it;

        // penalize for time windows violation
        if (!constraints::satisfies_time_windows(prob, curr_sln)) {
            ++tw_violation_count;
        } else {
            tw_violation_count = 1;
        }

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

        if (perform_route_saving || perform_intra_relocation) {
            slns = repeat(curr_sln_copy, ls.size());
        }
    }

    // post-optimization phase. drastically penalize for TW violation
    ls.penalize_tw(std::pow(objective(prob, best_sln), 2));

    slns = repeat(best_sln, ls.size());
    // no tabu is required now
    lists = tabu::TabuLists();
    do_local_search(ls, slns, lists);

    best_sln = *std::min_element(slns.cbegin(), slns.cend(), sln_comp);

    // TODO: add US heuristic as well
    ls.intra_relocate(best_sln);

    return best_sln;
}
}  // namespace detail
}  // namespace vrp
