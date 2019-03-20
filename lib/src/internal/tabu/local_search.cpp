#include "local_search.h"
#include "objective.h"

#include <algorithm>
#include <cassert>
#include <list>
#include <stack>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

namespace vrp {
namespace tabu {
namespace {

// deliver to customer, false otherwise
inline bool site_dependent(const Problem& prob, size_t vehicle,
                           size_t customer) {
    if (customer >= prob.allowed_vehicles_size()) {
        throw std::out_of_range("customer >= allowed vehicles size");
    }
    const auto& allowed = prob.allowed_vehicles(customer);
    if (vehicle >= allowed.size()) {
        throw std::out_of_range("vehicle >= allowed vehicles(customer) size");
    }
    return allowed[vehicle];
}

inline Solution::CustomerIndex at(const Solution::RouteType& route, size_t i) {
    if (i >= route.size()) {
        throw std::out_of_range("i >= route size");
    }
    return *std::next(route.cbegin(), i);
}

inline Solution::RouteType::iterator atit(Solution::RouteType& route,
                                          size_t i) {
    if (i >= route.size()) {
        throw std::out_of_range("i >= route size");
    }
    return std::next(route.begin(), i);
}

inline Solution::RouteType::const_iterator
atit(const Solution::RouteType& route, size_t i) {
    if (i >= route.size()) {
        throw std::out_of_range("i >= route size");
    }
    return std::next(route.cbegin(), i);
}

inline bool is_loop(const Solution::RouteType& route) {
    if (route.size() > 2) {
        return false;
    }
    if (route.size() <= 1) {  // is indeed loop or just empty
        return true;
    }
    return *route.cbegin() == *std::next(route.cbegin(), 1);
}

void delete_loops_after_relocate(Solution& sln, TabuLists& lists) {
    std::stack<decltype(sln.routes)::const_iterator> loop_indices;
    for (size_t ri = 0, size = sln.routes.size(); ri < size; ++ri) {
        if (is_loop(sln.routes[ri].second)) {
            loop_indices.push(sln.routes.cbegin() + ri);
        }
    }

    auto& relocate_list = lists.relocate.all();
    while (!loop_indices.empty()) {
        auto loop = loop_indices.top();

        size_t ri = std::distance(sln.routes.cbegin(), loop);
        std::decay_t<decltype(relocate_list)> fixed_relocate_list = {};
        while (!relocate_list.empty()) {
            auto entry = *relocate_list.begin();
            relocate_list.erase(relocate_list.begin());
            if (entry.value.second == ri) {
                continue;
            }
            for (size_t i = ri + 1, size = sln.routes.size(); i < size; ++i) {
                if (entry.value.second == i) {
                    entry.value.second--;
                    break;
                }
            }
            fixed_relocate_list.emplace(entry);
        }
        relocate_list = std::move(fixed_relocate_list);

        sln.routes.erase(loop);
        loop_indices.pop();
    }
}  // namespace

/// calculate route distance. this is an oversimplified "objective function
/// part"
inline double distance_on_route(const Problem& prob,
                                const Solution::RouteType& route, size_t first,
                                size_t last) {
    if (first > last || first > route.size()) {
        throw std::out_of_range("invalid indices");
    }
    double distance = 0.0;
    for (; first + 1 != last; ++first) {
        distance += prob.costs[at(route, first)][at(route, first + 1)];
    }
    return distance;
}

template<typename ListIt>
inline double distance_on_route(const Problem& prob, ListIt first,
                                ListIt last) {
    if (first == last) {
        throw std::runtime_error("empty range provided");
    }
    double distance = 0.0;
    auto next_first = std::next(first);
    for (; next_first != last; ++first, ++next_first) {
        distance += prob.costs[*first][*next_first];
    }
    return distance;
}

// calculate distance for a pair of "independent" iterators: dist(i-1, i+1) +
// dist(k-1, k+1), where: (i-1)->(i)->(i+1) && (k-1)->(k)->(k+1)
template<typename ListIt>
inline double paired_distance_on_route(const Problem& prob, ListIt i,
                                       ListIt k) {
    return distance_on_route(prob, std::prev(i, 1), std::next(i, 2)) +
           distance_on_route(prob, std::prev(k, 1), std::next(k, 2));
}

inline Solution::RouteType remove_depots(const Solution::RouteType& route) {
    auto copy = route;
    copy.pop_front();
    copy.pop_back();
    return copy;
}

inline Solution::RouteType add_depots(const Solution::RouteType& route) {
    static constexpr const Solution::CustomerIndex depot = 0;
    auto copy = route;
    copy.emplace_front(depot);
    copy.emplace_back(depot);
    return copy;
}

inline void two_opt_swap(Solution::RouteType& route, size_t i, size_t k) {
    if (i >= route.size() || k >= route.size()) {
        throw std::out_of_range("index >= size");
    }
    const auto begin = std::next(route.begin(), i),
               end = std::next(route.begin(), k + 1);
    std::reverse(begin, end);
}

template<typename ListIt>
inline void two_opt_swap(Solution::RouteType& route, ListIt first,
                         ListIt last) {
    std::reverse(first, last);
}
}  // namespace

LocalSearchMethods::LocalSearchMethods(const Problem& prob) noexcept
    : m_prob(prob) {
    m_methods[0] = std::bind(&LocalSearchMethods::relocate, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[1] = std::bind(&LocalSearchMethods::relocate_split, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[2] = std::bind(&LocalSearchMethods::exchange, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[3] = std::bind(&LocalSearchMethods::two_opt, this,
                             std::placeholders::_1, std::placeholders::_2);
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

// TODO: check constraints & tabu lists
void LocalSearchMethods::relocate(Solution& sln, TabuLists& lists) {
    const auto vehicles_size = m_prob.n_vehicles();
    std::unordered_set<Solution::VehicleIndex> unused_vehicles;
    assert(vehicles_size >= sln.used_vehicles.size());
    unused_vehicles.reserve(vehicles_size - sln.used_vehicles.size());
    for (size_t v = 0; v < vehicles_size; ++v) {
        if (sln.used_vehicles.find(v) == sln.used_vehicles.cend()) {
            unused_vehicles.emplace(v);
        }
    }

    const auto size = m_prob.n_customers();
    for (size_t customer = 1; customer < size; ++customer) {
        // TODO: handle case when route not found - debug only?
        size_t r_in = 0, c_index = 0;
        std::tie(r_in, c_index) = sln.customer_owners[customer];
        auto& route_in = sln.routes[r_in].second;
        if (is_loop(route_in)) {
            continue;
        }
        for (size_t neighbour = 1; neighbour < size; ++neighbour) {
            // TODO: handle case when route not found - debug only?
            size_t r_out = 0, n_index = 0;
            std::tie(r_out, n_index) = sln.customer_owners[neighbour];
            // do not relocate inside the same route
            if (r_in == r_out) {
                continue;
            }

            // TODO: debug this is correct
            // there's a tabu list entry for (customer, route id)
            if (lists.relocate.has(customer, r_out)) {
                continue;
            }

            auto& route_out = sln.routes[r_out].second;
            if (is_loop(route_out)) {
                continue;
            }
            if (!site_dependent(m_prob, sln.routes[r_out].first, customer)) {
                // cannot insert customer in not allowed route
                continue;
            }

            // customer value represents the length of route i -> j, where:
            // ... -> (i -> customer -> j) -> ...
            const auto customer_value =
                distance_on_route(m_prob, route_in, c_index - 1, c_index + 2);
            const auto customer_neighbour_distance =
                m_prob.costs[customer][neighbour];
            const auto customer_before_neighbour_value =
                customer_neighbour_distance +
                m_prob.costs[customer][at(route_out, n_index - 1)];
            const auto customer_after_neighbour_value =
                customer_neighbour_distance +
                m_prob.costs[customer][at(route_out, n_index + 1)];

            // if customer is closer to it's neighbours in __current__ route, do
            // not relocate to neighbours in __new__ route
            if (customer_value < customer_before_neighbour_value &&
                customer_value < customer_after_neighbour_value) {
                continue;
            }

            // we assume there's a better place for our customer at this point

            auto it_in_before = atit(route_in, c_index - 1),
                 it_in_after = atit(route_in, c_index + 1),
                 it_out_before = atit(route_out, n_index - 1),
                 it_out_after = atit(route_out, n_index + 1);

            const auto cost_before =
                distance_on_route(m_prob, it_in_before,
                                  std::next(it_in_after, 1)) +
                distance_on_route(m_prob, it_out_before,
                                  std::next(it_out_after, 1));

            Solution::RouteType::iterator inserted, erased;
            if (customer_before_neighbour_value <
                customer_after_neighbour_value) {
                inserted =
                    route_out.insert(std::next(it_out_before, 1), customer);
            } else {
                inserted = route_out.insert(it_out_after, customer);
            }
            erased = route_in.erase(std::next(it_in_before, 1));

            const auto cost_after =
                distance_on_route(m_prob, it_in_before,
                                  std::next(it_in_after, 1)) +
                distance_on_route(m_prob, it_out_before,
                                  std::next(it_out_after, 1));

            // decide whether move is good
            if (cost_after < cost_before) {
                // move is good
                sln.update_customer_owners(m_prob, r_in);
                sln.update_customer_owners(m_prob, r_out);
                lists.relocate.emplace(customer, r_in);
                break;
            } else {
                // move is bad - roll back the changes
                route_in.insert(erased, customer);
                route_out.erase(inserted);
            }
        }

        // check if creating a new route is better than any relocation
        {
            // indices might have changed during "for(neighbours)" loop
            std::tie(r_in, c_index) = sln.customer_owners[customer];

            if (sln.routes.size() >= vehicles_size) {
                continue;
            }

            // find suitable vehicle
            bool found_suitable_vehicle = false;
            size_t used_vehicle = std::numeric_limits<size_t>::max();
            for (auto v : unused_vehicles) {
                const bool enough_capacity = m_prob.vehicles[v].capacity >=
                                             m_prob.customers[customer].demand;
                if (enough_capacity && site_dependent(m_prob, v, customer)) {
                    sln.routes.emplace_back(v, add_depots({customer}));
                    used_vehicle = v;
                    unused_vehicles.erase(v);  // TODO: is it safe though?
                    found_suitable_vehicle = true;
                    break;
                }
            }
            if (!found_suitable_vehicle) {
                continue;
            }
            assert(used_vehicle != std::numeric_limits<size_t>::max());

            // we assume there's a suitable vehicle at this point

            auto& route_in = sln.routes[r_in].second;

            auto it_in_before = atit(route_in, c_index - 1),
                 it_in_after = atit(route_in, c_index + 1),
                 it_out = atit(sln.routes.back().second, 1);

            const auto cost_before = distance_on_route(
                m_prob, it_in_before, std::next(it_in_after, 1));

            auto erased = route_in.erase(std::next(it_in_before, 1));

            const auto cost_after =
                distance_on_route(m_prob, it_in_before,
                                  std::next(it_in_after, 1)) +
                distance_on_route(m_prob, std::prev(it_out, 1),
                                  std::next(it_out, 2));

            // decide whether move is good
            if (cost_after < cost_before) {
                // move is good
                sln.update_customer_owners(m_prob, r_in);
                sln.update_customer_owners(m_prob, sln.routes.size() - 1);
                sln.used_vehicles.emplace(used_vehicle);
                lists.relocate.emplace(customer, r_in);
            } else {
                // move is bad - roll back the changes
                route_in.insert(erased, customer);
                sln.routes.pop_back();
                unused_vehicles.emplace(used_vehicle);
            }
        }
    }
    delete_loops_after_relocate(sln, lists);
    sln.update_customer_owners(m_prob);
}  // namespace tabu

void LocalSearchMethods::relocate_split(Solution& sln, TabuLists& lists) {
    return;
}

void LocalSearchMethods::exchange(Solution& sln, TabuLists& lists) {
    const auto size = m_prob.n_customers();
    for (size_t customer = 1; customer < size; ++customer) {
        // TODO: handle case when route not found - debug only?
        size_t r1 = 0, c_index = 0;
        std::tie(r1, c_index) = sln.customer_owners[customer];
        auto& route1 = sln.routes[r1].second;
        if (is_loop(route1)) {
            continue;
        }
        for (size_t neighbour = 1; neighbour < size; ++neighbour) {
            // TODO: handle case when route not found - debug only?
            size_t r2 = 0, n_index = 0;
            std::tie(r2, n_index) = sln.customer_owners[neighbour];
            // do not relocate inside the same route
            if (r1 == r2) {
                continue;
            }

            // TODO: debug this is correct
            // there's a tabu list entry for (customer, neighbour)
            if (lists.exchange.has(customer, r2) ||
                lists.exchange.has(neighbour, r1)) {
                continue;
            }

            auto& route2 = sln.routes[r2].second;
            if (is_loop(route2)) {
                continue;
            }

            // check if both customers can be exchanged
            if (!site_dependent(m_prob, sln.routes[r2].first, customer) ||
                !site_dependent(m_prob, sln.routes[r1].first, neighbour)) {
                // cannot exchange customers within forbidden route
                continue;
            }

            // we assume we can exchange two iterators at this point

            // perform exchange (just swap customer indices)
            auto it1 = atit(route1, c_index), it2 = atit(route2, n_index);
            const auto cost_before = paired_distance_on_route(m_prob, it1, it2);
            std::swap(*it1, *it2);
            const auto cost_after = paired_distance_on_route(m_prob, it1, it2);

            // decide whether move is good
            if (cost_after < cost_before) {
                // move is good
                sln.update_customer_owners(m_prob, r1);
                sln.update_customer_owners(m_prob, r2);
                lists.exchange.emplace(customer, r1);
                lists.exchange.emplace(neighbour, r2);
                break;
            } else {
                // move is bad - roll back the changes
                std::swap(*it1, *it2);
            }
        }
    }
}

// TODO: check constraints & tabu lists
void LocalSearchMethods::two_opt(Solution& sln, TabuLists& lists) {
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        auto& route = sln.routes[ri].second;

        // we can only improve routes that have 3+ nodes
        bool can_improve = route.size() > 2;
        while (can_improve) {
            bool found_new_best = false;
            // skip depots && beware of k = i + 1
            for (auto i = std::next(route.begin(), 1);
                 i != std::prev(route.end(), 2); ++i) {
                if (found_new_best)  // fast loop break
                    break;
                // skip depots && start from i + 1
                for (auto k = std::next(i, 1); k != std::prev(route.end(), 1);
                     ++k) {
                    // there's a tabu list entry for (i, k)
                    if (lists.two_opt.has(*i, *k)) {
                        continue;
                    }

                    // cost before: (i-1)->i->(i+1) + (k-1)->k->(k+1)
                    const auto cost_before =
                        paired_distance_on_route(m_prob, i, k);
                    // perform 2-opt
                    std::reverse(i, std::next(k, 1));
                    // cost after: (i-1)->k->(i+1) + (k-1)->i->(k+1)
                    const auto cost_after =
                        paired_distance_on_route(m_prob, i, k);

                    // decide whether move is good
                    if (cost_after < cost_before) {
                        // move is good
                        found_new_best = true;
                        // forbid previously existing edges
                        lists.two_opt.emplace(*i, *std::next(i, 1));
                        lists.two_opt.emplace(*k, *std::next(k, 1));
                        break;
                    } else {
                        // move is bad - roll back the changes
                        std::reverse(i, std::next(k, 1));
                    }
                }
            }

            // if new best found: continue, else: stop
            can_improve = found_new_best;
        }
    }
    sln.update_customer_owners(m_prob);
}

void LocalSearchMethods::route_save(Solution& sln, size_t threshold) {}

void LocalSearchMethods::intra_relocate(Solution& sln) {
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        auto vehicle = sln.routes[ri].first;
        auto& route = sln.routes[ri].second;
        auto curr_best_value = objective(m_prob, vehicle, route);
        for (auto pos = std::next(route.begin(), 1);
             pos != std::prev(route.end(), 1); ++pos) {
            for (auto new_pos = std::next(route.begin(), 1);
                 new_pos != std::prev(route.end(), 1); ++new_pos) {
                if (pos == new_pos) {
                    continue;
                }

                // move customer to new position
                std::swap(*pos, *new_pos);

                // decide whether move is good
                const auto value = objective(m_prob, vehicle, route);
                if (value < curr_best_value) {
                    // move is good
                    curr_best_value = value;
                } else {
                    // move is bad - roll back the changes
                    std::swap(*pos, *new_pos);
                }
            }
        }
    }
    sln.update_customer_owners(m_prob);
}
}  // namespace tabu
}  // namespace vrp
