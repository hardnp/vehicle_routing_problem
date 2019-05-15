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

#define DYNAMIC_VIOLATIONS 0  // TODO: fix this

// iterations multiplier
constexpr const double MULTIPLIER = 1.0;

// constants
constexpr const uint32_t TABU_SEARCH_ITERS = 100 * MULTIPLIER;
constexpr const uint32_t MAX_ITERS = 10 * TABU_SEARCH_ITERS;
constexpr const uint32_t ROUTE_SAVING_ITERS = 5 * MULTIPLIER;
constexpr const uint32_t MERGE_SPLITS_ITERS = 10 * MULTIPLIER;
constexpr const uint32_t INTRA_RELOCATION_ITERS = 15 * MULTIPLIER;
constexpr const uint32_t BAD_MOVES_ITERS = 10 * MULTIPLIER;
constexpr const double TIME_WINDOWS_PENALTY_BASE = 1.2;

constexpr const uint32_t CONSTRAINTS_FIX_ITERS = 0.1 * TABU_SEARCH_ITERS;

constexpr const uint32_t MAX_VIOLATION_ITERS = 3;

void update_tabu_lists(tabu::TabuLists& lists, const tabu::TabuLists& new_lists,
                       size_t i) {
    lists.pr_common = std::move(new_lists.pr_common);
    switch (i) {
    case 0:
        lists.relocate = new_lists.relocate;
        lists.pr_relocate = new_lists.pr_relocate;
        lists.relocate_new_route = std::move(new_lists.relocate);
        lists.pr_relocate_new_route = std::move(new_lists.pr_relocate);
        break;
    case 1:
        lists.exchange = std::move(new_lists.exchange);
        lists.pr_exchange = std::move(new_lists.pr_exchange);
        break;
    case 2:
        lists.two_opt = std::move(new_lists.two_opt);
        lists.pr_two_opt = std::move(new_lists.pr_two_opt);
        break;
    case 3:
        lists.cross = std::move(new_lists.cross);
        lists.pr_cross = std::move(new_lists.pr_cross);
        break;
    case 4:
        lists.relocate_new_route = new_lists.relocate_new_route;
        lists.pr_relocate_new_route = new_lists.pr_relocate_new_route;
        lists.relocate = std::move(new_lists.relocate_new_route);
        lists.pr_relocate = std::move(new_lists.pr_relocate_new_route);
        break;
    case 5:
        lists.relocate_split = std::move(new_lists.relocate_split);
        lists.pr_relocate_split = std::move(new_lists.pr_relocate_split);
        break;
    default:
        throw std::out_of_range("tabu list index out of range");
    }
}

uint32_t threshold(const Problem& prob) {
    // choose between minimum of 3 and 5% of customers
    return std::max(1u, static_cast<uint32_t>(prob.n_customers() * 0.05)) + 2u;
}

inline void do_local_search(tabu::LocalSearchMethods& ls,
                            std::vector<Solution>& slns, tabu::TabuLists& lists,
                            std::vector<bool>& was_improved) {
    assert(slns.size() == ls.size());
    for (size_t m = 0, size = ls.size(); m != size; ++m) {
        was_improved[m] = ls[m](slns[m], lists);
    }
    ls.step();
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
    thread_local const auto less = [&prob](const auto& a, const auto& b) {
        return objective(prob, a) < objective(prob, b);
    };

    // find min element in improved solutions
    thread_local const auto min_element = [](const std::vector<Solution>& slns,
                                             const std::vector<bool>& impr) {
        if (slns.empty()) {
            return slns.cend();
        }
        auto min = slns.cbegin();
        for (auto first = slns.cbegin() + 1; first != slns.cend(); ++first) {
            if (!impr[std::distance(slns.cbegin(), first)]) {
                continue;
            }
            if (less(*first, *min)) {
                min = first;
            }
        }
        return min;
    };

    const auto route_saving_threshold = threshold(prob);

    tabu::LocalSearchMethods ls(prob);
    ls.violate_tw(false);

    Solution best_sln = initial_sln;

    // init temporary information:
    best_sln.update_customer_owners(prob);
    best_sln.update_used_vehicles();
    assert(!best_sln.customer_owners.empty());

    // keep track of best feasible solution as well
    Solution best_feasible_sln = best_sln;

    const auto objective_baseline = std::pow(objective(prob, best_sln), 1.2);

    std::vector<Solution> slns = repeat(best_sln, ls.size());
    std::vector<bool> was_improved(ls.size(), false);
    tabu::TabuLists lists{};

    int tw_violation_count = 1;
    auto constraints_count = CONSTRAINTS_FIX_ITERS;

#if DYNAMIC_VIOLATIONS
    bool curr_sln_feasible = false;
    bool can_violate_tw = false;
    uint32_t violated_tw_count = 0;
#endif

    // i - iterations counter, can be reset if improvement found
    // ci - constant iterations counter, always counts forward
    for (uint32_t i = 0, ci = 0; i < TABU_SEARCH_ITERS && ci < MAX_ITERS;
         ++i, ++ci) {

        if (ci == 0 || constraints_count < CONSTRAINTS_FIX_ITERS * 0.9) {
            ls.penalize_tw(
                std::pow(TIME_WINDOWS_PENALTY_BASE, tw_violation_count));
        }

        --constraints_count;
        if (!constraints_count) {
            ls.penalize_tw(objective_baseline);
            constraints_count = CONSTRAINTS_FIX_ITERS;
        }

#if DYNAMIC_VIOLATIONS
        // change violation status each K iterations
        if (can_violate_tw) {
            ++violated_tw_count;
        }
        if (curr_sln_feasible) {
            can_violate_tw = true;
            ls.violate_tw(can_violate_tw);
        } else if (violated_tw_count > MAX_VIOLATION_ITERS) {
            violated_tw_count = 0;
            can_violate_tw = false;
            ls.violate_tw(can_violate_tw);
        }
#endif

        auto updated_lists = lists;
        do_local_search(ls, slns, updated_lists, was_improved);

        auto min_sln_it = min_element(slns, was_improved);

        // TODO: check if this is required. doesn't seem like it's working
        std::vector<Solution> feasible_slns;
        feasible_slns.reserve(slns.size());
        for (const auto& sln : slns) {
            if (!constraints::satisfies_all(prob, sln)) {
                continue;
            }
            feasible_slns.emplace_back(sln);
        }
        auto min_feasible_sln_it = min_element(feasible_slns, was_improved);

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

        // found new best: reset best solution, reset iter counter to 0
        if (objective(prob, curr_sln) < objective(prob, best_sln)) {
            best_sln = curr_sln;
            i = 0;
        }

        // found new feasible best: reset best feasible, reset iter counter to 0
        if (min_feasible_sln_it != feasible_slns.cend() &&
            (objective(prob, *min_feasible_sln_it) <
                 objective(prob, best_feasible_sln) ||
             !constraints::satisfies_all(prob, best_feasible_sln))) {
            best_feasible_sln = *min_feasible_sln_it;
            i = 0;
        }

        const bool perform_route_saving = ci % ROUTE_SAVING_ITERS == 0;
        const bool perform_intra_relocation = i > INTRA_RELOCATION_ITERS;
        const bool perform_merge_splits = ci % MERGE_SPLITS_ITERS == 0;

        if (perform_route_saving) {
            ls.route_save(curr_sln, route_saving_threshold);
        }
        if (perform_intra_relocation) {
            ls.intra_relocate(curr_sln);
        }
        if (perform_merge_splits) {
            ls.merge_splits(curr_sln);
        }

#if DYNAMIC_VIOLATIONS
        curr_sln_feasible = constraints::satisfies_all(prob, curr_sln);
#endif

        slns = repeat(curr_sln, ls.size());

        if (std::all_of(was_improved.cbegin(), was_improved.cend(),
                        [](bool v) { return !v; }) &&
            i > INTRA_RELOCATION_ITERS * 2 && !ls.can_do_bad_move()) {
            ls.allow_bad_moves_for(BAD_MOVES_ITERS);
        }
    }

    thread_local const auto do_post_optimization = [&](Solution& best_sln) {
        // post-optimization phase. drastically penalize for TW violation
        ls.penalize_tw(objective_baseline);
        ls.allow_bad_moves_for(0);

        auto curr_sln = best_sln;

        for (size_t i = 0; i < 2; ++i) {
            lists = tabu::TabuLists();
            slns = repeat(curr_sln, ls.size());
            // no tabu is required now
            do_local_search(ls, slns, lists, was_improved);

            curr_sln = *std::min_element(slns.cbegin(), slns.cend(), less);

            // TODO: add US heuristic as well
            ls.intra_relocate(curr_sln);

            if ((constraints::satisfies_all(prob, curr_sln) ||
                 !constraints::satisfies_all(prob, best_sln)) &&
                objective(prob, curr_sln) < objective(prob, best_sln)) {
                best_sln = curr_sln;
            }
        }
    };

    do_post_optimization(best_sln);
    do_post_optimization(best_feasible_sln);

    if (constraints::satisfies_all(prob, best_sln) ||
        !constraints::satisfies_all(prob, best_feasible_sln)) {
        return std::min(best_sln, best_feasible_sln, less);
    }
    return best_feasible_sln;
}
}  // namespace detail
}  // namespace vrp
