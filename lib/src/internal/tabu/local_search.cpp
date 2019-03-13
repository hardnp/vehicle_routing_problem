#include "local_search.h"
#include "objective.h"

#include <algorithm>
#include <list>
#include <stdexcept>

namespace vrp {
namespace tabu {
namespace {
std::list<Solution::CustomerIndex>&
remove_depots(std::list<Solution::CustomerIndex>& route) {
    route.pop_front();
    route.pop_back();
    return route;
}

std::list<Solution::CustomerIndex>&
add_depots(std::list<Solution::CustomerIndex>& route) {
    static constexpr const Solution::CustomerIndex depot = 0;
    route.emplace_front(depot);
    route.emplace_back(depot);
    return route;
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

Solution LocalSearchMethods::relocate(const Solution& sln) { return sln; }

Solution LocalSearchMethods::relocate_split(const Solution& sln) { return sln; }

Solution LocalSearchMethods::exchange(const Solution& sln) { return sln; }

Solution LocalSearchMethods::two_opt(const Solution& sln) {
    Solution new_sln = sln;
    for (size_t ri = 0; ri < new_sln.routes.size(); ++ri) {
        auto route = new_sln.routes[ri].second;
        remove_depots(route);
        bool can_improve = true;
        while (can_improve) {
            auto curr_best_value = objective(m_prob, new_sln);
            bool found_new_best = false;
            // TODO: start from i = 0 or i = 1??
            for (size_t i = 0; i < route.size() - 1; ++i) {
                if (found_new_best)  // fast loop break
                    break;
                for (size_t k = i + 1; k < route.size(); ++k) {
                    auto s = new_sln;
                    s.routes[ri].second = route;
                    two_opt_swap(s.routes[ri].second, i, k);
                    add_depots(s.routes[ri].second);
                    auto value = objective(m_prob, s);
                    if (value < curr_best_value) {
                        route = s.routes[ri].second;
                        remove_depots(route);
                        found_new_best = true;
                        new_sln = s;
                        break;
                    }
                }
            }
            // if new best found: continue, else: stop
            can_improve = found_new_best;
        }
        add_depots(route);
    }
    return new_sln;
}
}  // namespace tabu
}  // namespace vrp
