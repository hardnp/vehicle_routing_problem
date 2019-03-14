#include "local_search.h"
#include "objective.h"

#include <algorithm>
#include <list>
#include <stdexcept>

namespace vrp {
namespace tabu {
namespace {
std::list<Solution::CustomerIndex>
remove_depots(const std::list<Solution::CustomerIndex>& route) {
    auto copy = route;
    copy.pop_front();
    copy.pop_back();
    return copy;
}

std::list<Solution::CustomerIndex>
add_depots(const std::list<Solution::CustomerIndex>& route) {
    static constexpr const Solution::CustomerIndex depot = 0;
    auto copy = route;
    copy.emplace_front(depot);
    copy.emplace_back(depot);
    return copy;
}

void two_opt_swap(std::list<Solution::CustomerIndex>& route, size_t i,
                  size_t k) {
    if (i >= route.size() || k >= route.size()) {
        throw std::out_of_range("index >= size");
    }
    std::reverse(std::next(route.begin(), i), std::next(route.begin(), k + 1));
}
}  // namespace

LocalSearchMethods::LocalSearchMethods(const Problem& prob) noexcept
    : m_prob(prob) {
    m_methods[0] =
        std::bind(&LocalSearchMethods::relocate, this, std::placeholders::_1);
    m_methods[1] = std::bind(&LocalSearchMethods::relocate_split, this,
                             std::placeholders::_1);
    m_methods[2] =
        std::bind(&LocalSearchMethods::exchange, this, std::placeholders::_1);
    m_methods[3] =
        std::bind(&LocalSearchMethods::two_opt, this, std::placeholders::_1);
}

LocalSearchMethods::methods_t::iterator LocalSearchMethods::begin() {
    return m_methods.begin();
}

LocalSearchMethods::methods_t::iterator LocalSearchMethods::end() {
    return m_methods.end();
}

LocalSearchMethods::methods_t::const_iterator
LocalSearchMethods::cbegin() const {
    return m_methods.cbegin();
}

LocalSearchMethods::methods_t::const_iterator LocalSearchMethods::cend() const {
    return m_methods.cend();
}

size_t LocalSearchMethods::size() const { return m_methods.size(); }

const LocalSearchMethods::methods_t::value_type& LocalSearchMethods::
operator[](size_t i) const {
    if (i >= size()) {
        throw std::out_of_range("index >= size");
    }
    return m_methods[i];
}

void LocalSearchMethods::relocate(Solution& sln) { return; }

void LocalSearchMethods::relocate_split(Solution& sln) { return; }

void LocalSearchMethods::exchange(Solution& sln) { return; }

void LocalSearchMethods::two_opt(Solution& sln) {
    auto curr_best_value = objective(m_prob, sln);

    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        // depots are out of 2-opt scope, so we remove them
        auto route = std::move(remove_depots(sln.routes[ri].second));

        bool can_improve =
            route.size() > 2;  // there must be improvement options
        while (can_improve) {
            bool found_new_best = false;
            for (size_t i = 0; i < route.size() - 1; ++i) {
                if (found_new_best)  // fast loop break
                    break;
                for (size_t k = i + 1; k < route.size(); ++k) {
                    auto old_route =
                        std::move(sln.routes[ri].second);  // save old sln

                    // perform 2-opt on solution
                    auto route_copy = route;
                    two_opt_swap(route_copy, i, k);
                    sln.routes[ri].second = std::move(add_depots(route_copy));

                    // decide whether move is good
                    auto value = objective(m_prob, sln);
                    if (value < curr_best_value) {  // move is good
                        curr_best_value = value;
                        route = std::move(route_copy);
                        found_new_best = true;
                        break;
                    } else {  // move is bad
                        // roll back changes to solution
                        sln.routes[ri].second = std::move(old_route);
                    }
                }
            }

            // if new best found: continue, else: stop
            can_improve = found_new_best;
        }
    }
}
}  // namespace tabu
}  // namespace vrp
