#include "local_search.h"
#include "constraints.h"
#include "objective.h"

#include "logging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <list>
#include <numeric>
#include <stack>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

// FIXME: this file is getting ridiculously huge. need to fix that

namespace vrp {
namespace tabu {
namespace {
#define USE_PRESERVE_ENTRIES 1

#define SORT_HEURISTIC_OPERANDS 0

// true if vehicle can deliver to customer, false otherwise
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
    if (i > route.size()) {
        throw std::out_of_range("i >= route size");
    }
    return std::next(route.cbegin(), i);
}

template<typename ListIt>
inline double violated_time(const Problem& prob, const SplitInfo& info,
                            double penalty, ListIt first, ListIt last) {
    return penalty *
           (constraints::total_violated_time(prob, info, first, last));
}

/// calculate route distance. this is an oversimplified "objective function
/// part"
template<typename ListIt>
inline double distance_on_route(const Problem& prob, const SplitInfo& info,
                                double penalty, ListIt first, ListIt last) {
    if (first == last) {
        throw std::runtime_error("empty range provided");
    }

    double distance = violated_time(prob, info, penalty, first, last);

    auto next_first = std::next(first);
    for (; next_first != last; ++first, ++next_first) {
        distance += prob.costs[*first][*next_first];
    }

    return distance;
}

inline double distance_on_route(const Problem& prob, const SplitInfo& info,
                                double penalty,
                                const Solution::RouteType& route,
                                size_t first_i, size_t last_i) {
    // FIXME: always use iterator version, remove this one
    if (first_i > last_i || first_i > route.size()) {
        throw std::out_of_range("invalid indices");
    }
    return distance_on_route(prob, info, penalty, atit(route, first_i),
                             atit(route, last_i));
}

// calculate distance for a pair of "independent" iterators: dist(i-1, i+1) +
// dist(k-1, k+1), where: (i-1)->(i)->(i+1) && (k-1)->(k)->(k+1)
template<typename ListIt>
inline double paired_distance_on_route(const Problem& prob,
                                       const SplitInfo& info1,
                                       const SplitInfo& info2, double penalty,
                                       ListIt i, ListIt k) {
    return distance_on_route(prob, info1, penalty, std::prev(i),
                             std::next(i, 2)) +
           distance_on_route(prob, info2, penalty, std::prev(k),
                             std::next(k, 2));
}

/// calculate total demand for given range
template<typename ListIt>
inline TransportationQuantity total_demand(const Problem& prob,
                                           const SplitInfo& info, ListIt first,
                                           ListIt last) {
    if (first == last) {
        throw std::runtime_error("empty range provided");
    }

    TransportationQuantity demand;

    const auto& customers = prob.customers;
    for (; first != last; ++first) {
        const auto& c = customers[*first];
        assert(static_cast<size_t>(c.id) == *first);
        demand += (c.demand * info.at(c.id));
    }

    return demand;
}

inline bool is_loop(const Solution::RouteType& route) {
    if (route.size() > 2) {
        return false;
    }
    if (route.size() <= 1) {  // is indeed loop or just empty
        return true;
    }
    return *route.cbegin() == *std::next(route.cbegin());
}

void delete_loops_after_relocate(Solution& sln) {
    std::stack<decltype(sln.routes)::const_iterator> loop_indices;
    for (size_t ri = 0, size = sln.routes.size(); ri < size; ++ri) {
        if (is_loop(sln.routes[ri].second)) {
            loop_indices.push(sln.routes.cbegin() + ri);
        }
    }

    while (!loop_indices.empty()) {
        sln.routes.erase(loop_indices.top());
        loop_indices.pop();
    }
}

void delete_loops_after_relocate(Solution& sln, TabuLists& lists) {
    std::stack<decltype(sln.routes)::const_iterator> loop_indices;
    for (size_t ri = 0, size = sln.routes.size(); ri < size; ++ri) {
        if (is_loop(sln.routes[ri].second)) {
            loop_indices.push(sln.routes.cbegin() + ri);
        }
    }

    auto delete_loops_in_tabu_list = [&loop_indices](const Solution& sln,
                                                     auto& relocate_list) {
        auto loop_indices_copy = loop_indices;
        while (!loop_indices_copy.empty()) {
            auto loop = loop_indices_copy.top();

            size_t ri = std::distance(sln.routes.cbegin(), loop);
            std::decay_t<decltype(relocate_list)> fixed_relocate_list = {};
            while (!relocate_list.empty()) {
                auto entry = *relocate_list.begin();
                relocate_list.erase(relocate_list.begin());
                if (entry.value.second == ri) {
                    continue;
                }
                for (size_t i = ri + 1, size = sln.routes.size(); i < size;
                     ++i) {
                    if (entry.value.second == i) {
                        entry.value.second--;
                        break;
                    }
                }
                fixed_relocate_list.emplace(entry);
            }
            relocate_list = std::move(fixed_relocate_list);
            loop_indices_copy.pop();
        }
    };

    delete_loops_in_tabu_list(sln, lists.relocate.all());
    delete_loops_in_tabu_list(sln, lists.relocate_new_route.all());

    while (!loop_indices.empty()) {
        sln.routes.erase(loop_indices.top());
        loop_indices.pop();
    }
}

template<typename ListIt>
inline void cross_routes(Solution::RouteType& lhs, ListIt lhs_first,
                         Solution::RouteType& rhs, ListIt rhs_first) {
    auto tmp = rhs;
    auto tmp_first =
        std::next(tmp.begin(), std::distance(rhs.begin(), rhs_first));

    rhs.erase(rhs_first, rhs.end());

    rhs.splice(rhs.end(), lhs, lhs_first, lhs.end());
    lhs.splice(lhs.end(), tmp, tmp_first, tmp.end());
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

inline bool is_split(const SplitInfo& info, size_t c) {
    return static_cast<double>(info.at(c)) < 1.0;
}
}  // namespace

LocalSearchMethods::LocalSearchMethods(const Problem& prob) noexcept
    : m_prob(prob), m_enable_splits(prob.enable_splits()) {
    m_methods[0] = std::bind(&LocalSearchMethods::relocate, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[1] = std::bind(&LocalSearchMethods::exchange, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[2] = std::bind(&LocalSearchMethods::two_opt, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[3] = std::bind(&LocalSearchMethods::cross, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[4] = std::bind(&LocalSearchMethods::relocate_new_route, this,
                             std::placeholders::_1, std::placeholders::_2);
    m_methods[5] = std::bind(&LocalSearchMethods::relocate_split, this,
                             std::placeholders::_1, std::placeholders::_2);

    if (!prob.enable_splits()) {
        for (size_t c = 0, size = prob.n_customers(); c < size; ++c) {
            m_default_split_info.split_info[c] = 1.0;
        }
    }
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

inline void validate_indices(
    size_t r_id, size_t c_id,
    const std::vector<std::pair<Solution::VehicleIndex, Solution::RouteType>>&
        routes) {
    assert(r_id < routes.size());
    assert(c_id < routes[r_id].second.size());
}

bool LocalSearchMethods::relocate(Solution& sln, TabuLists& lists) {
    static double best_ever_value = std::numeric_limits<double>::max();

    bool improved = false;

    // reminder: skip depot!
    const auto& customers = m_prob.customers;
    std::vector<size_t> ascending_sort_customers(customers.size() - 1);
    std::iota(ascending_sort_customers.begin(), ascending_sort_customers.end(),
              1);
    std::vector<size_t> descending_sort_customers(
        ascending_sort_customers.begin(), ascending_sort_customers.end());

#if SORT_HEURISTIC_OPERANDS
    std::sort(ascending_sort_customers.begin(), ascending_sort_customers.end(),
              [&customers](size_t a, size_t b) {
                  return customers[a].demand < customers[b].demand;
              });

    descending_sort_customers = std::move(std::vector<size_t>(
        ascending_sort_customers.rbegin(), ascending_sort_customers.rend()));
#endif

    // sorting customers: try to relocate small customers close to big ones.
    // small ones are usually "outliers", big ones are "cluster centers"
    for (size_t customer : ascending_sort_customers) {
        bool skip_to_next_customer = false;
        auto cfirst = sln.customer_owners[customer].cbegin(),
             clast = sln.customer_owners[customer].cend();
        for (; !skip_to_next_customer && cfirst != clast; ++cfirst) {
            size_t r_in = 0, c_index = 0;
            std::tie(r_in, c_index) = *cfirst;
            validate_indices(r_in, c_index, sln.routes);
            auto& route_in = sln.routes[r_in].second;
            if (is_loop(route_in)) {
                continue;
            }
            SplitInfo& split_in = sln.route_splits[r_in];
            for (size_t neighbour : descending_sort_customers) {
                if (skip_to_next_customer) {
                    break;
                }
                if (customer == neighbour) {
                    continue;
                }

                auto nfirst = sln.customer_owners[neighbour].cbegin(),
                     nlast = sln.customer_owners[neighbour].cend();
                for (; !skip_to_next_customer && nfirst != nlast; ++nfirst) {
                    size_t r_out = 0, n_index = 0;
                    std::tie(r_out, n_index) = *nfirst;
                    validate_indices(r_out, n_index, sln.routes);
                    // do not relocate inside the same route
                    if (r_in == r_out) {
                        continue;
                    }

                    auto& route_out = sln.routes[r_out].second;
                    if (is_loop(route_out)) {
                        continue;
                    }
                    if (!site_dependent(m_prob, sln.routes[r_out].first,
                                        customer)) {
                        // cannot insert customer in not allowed route
                        continue;
                    }

                    SplitInfo& split_out = sln.route_splits[r_out];
                    // FIXME: allow such moves?
                    if (m_enable_splits && split_out.has(customer)) {
                        continue;
                    }

                    // customer value represents the length of route i -> j,
                    // where:
                    // ... -> (i -> customer -> j) -> ...
                    const auto customer_value =
                        distance_on_route(m_prob, split_in, 0, route_in,
                                          c_index - 1, c_index + 2);
                    const auto customer_neighbour_distance =
                        m_prob.costs[customer][neighbour];
                    const auto customer_before_neighbour_value =
                        customer_neighbour_distance +
                        m_prob.costs[customer][at(route_out, n_index - 1)];
                    const auto customer_after_neighbour_value =
                        customer_neighbour_distance +
                        m_prob.costs[customer][at(route_out, n_index + 1)];

                    // if customer is closer to it's neighbours in __current__
                    // route, do not relocate to neighbours in __new__ route
                    if (customer_value < customer_before_neighbour_value &&
                        customer_value < customer_after_neighbour_value) {
                        continue;
                    }

                    // we assume there's a better place for our customer at this
                    // point

                    auto it_in_before = atit(route_in, c_index - 1),
                         it_in_after = atit(route_in, c_index + 1),
                         it_out_before = atit(route_out, n_index - 1),
                         it_out_after = atit(route_out, n_index + 1);

                    const auto cost_before =
                        distance_on_route(m_prob, split_in, m_tw_penalty,
                                          it_in_before,
                                          std::next(it_in_after)) +
                        distance_on_route(m_prob, split_out, m_tw_penalty,
                                          it_out_before,
                                          std::next(it_out_after));

                    Solution::RouteType::iterator inserted, erased;
                    if (customer_before_neighbour_value <
                        customer_after_neighbour_value) {
                        inserted = route_out.insert(std::next(it_out_before),
                                                    customer);
                    } else {
                        inserted = route_out.insert(it_out_after, customer);
                    }

                    transfer_split_entry(m_enable_splits, split_in, split_out,
                                         customer);

                    erased = route_in.erase(std::next(it_in_before));

                    const auto cost_after =
                        distance_on_route(m_prob, split_in, m_tw_penalty,
                                          it_in_before,
                                          std::next(it_in_after)) +
                        distance_on_route(m_prob, split_out, m_tw_penalty,
                                          it_out_before,
                                          std::next(it_out_after));

                    const auto out_demand_after =
                        total_demand(m_prob, split_out, route_out.cbegin(),
                                     route_out.cend());

                    // aspiration criteria
                    bool impossible_move =
                        lists.relocate.has(customer, r_out) &&
                        lists.pr_relocate.has(customer) &&
                        cost_after >= best_ever_value;

                    const auto out_capacity =
                        m_prob.vehicles[sln.routes[r_out].first].capacity;
                    impossible_move |= (out_demand_after > out_capacity);

                    impossible_move |=
                        (!m_can_violate_tw &&
                         (constraints::total_violated_time(
                              m_prob, split_out, route_out.cbegin(),
                              route_out.cend()) != 0));

                    // decide whether move is good
                    if (!impossible_move && cost_after < cost_before) {
                        // move is good
                        sln.customer_owners[customer].erase(r_in);
                        sln.update_customer_owners(m_prob, r_in, c_index);
                        sln.update_customer_owners(m_prob, r_out, n_index - 1);
                        lists.relocate.emplace(customer, r_in);
#if USE_PRESERVE_ENTRIES
                        lists.pr_relocate.emplace(customer);
#endif
                        best_ever_value = std::min(best_ever_value, cost_after);
                        improved = true;
                        skip_to_next_customer = true;
                    } else {
                        // move is bad - roll back the changes
                        route_in.insert(erased, customer);
                        route_out.erase(inserted);
                        transfer_split_entry(m_enable_splits, split_out,
                                             split_in, customer);
                    }
                }
            }
        }
    }
    delete_loops_after_relocate(sln, lists);
    sln.update_customer_owners(m_prob);
    return improved;
}

bool LocalSearchMethods::relocate_new_route(Solution& sln, TabuLists& lists) {
    static double best_ever_value = std::numeric_limits<double>::max();

    const auto vehicles_size = m_prob.n_vehicles();
    if (sln.routes.size() >= vehicles_size) {
        return false;
    }

    std::unordered_set<Solution::VehicleIndex> unused_vehicles;
    assert(vehicles_size >= sln.used_vehicles.size());
    unused_vehicles.reserve(vehicles_size - sln.used_vehicles.size());
    for (size_t v = 0; v < vehicles_size; ++v) {
        if (sln.used_vehicles.find(v) == sln.used_vehicles.cend()) {
            unused_vehicles.emplace(v);
        }
    }
    if (unused_vehicles.empty()) {
        return false;
    }

    bool improved = false;

    // reminder: skip depot!
    const auto& customers = m_prob.customers;
    std::vector<size_t> descending_sort_customers(customers.size() - 1);
    std::iota(descending_sort_customers.begin(),
              descending_sort_customers.end(), 1);

#if SORT_HEURISTIC_OPERANDS
    std::sort(descending_sort_customers.begin(),
              descending_sort_customers.end(),
              [&customers](size_t a, size_t b) {
                  return customers[a].demand > customers[b].demand;
              });
#endif

    // sorting customers: try to relocate big customers to new routes first
    for (size_t customer : descending_sort_customers) {
        bool skip_to_next_customer = false;
        auto cfirst = sln.customer_owners[customer].cbegin(),
             clast = sln.customer_owners[customer].cend();
        for (; !skip_to_next_customer && cfirst != clast; ++cfirst) {
            size_t r_in = 0, c_index = 0;
            std::tie(r_in, c_index) = *cfirst;
            validate_indices(r_in, c_index, sln.routes);
            if (is_loop(sln.routes[r_in].second)) {
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

            sln.route_splits.emplace_back(m_default_split_info);
            SplitInfo& split_in = sln.route_splits[r_in];
            SplitInfo& split_out = sln.route_splits.back();

            auto it_in_before = atit(route_in, c_index - 1),
                 it_in_after = atit(route_in, c_index + 1),
                 it_out = atit(sln.routes.back().second, 1);

            const auto cost_before =
                distance_on_route(m_prob, split_in, m_tw_penalty, it_in_before,
                                  std::next(it_in_after));

            transfer_split_entry(m_enable_splits, split_in, split_out,
                                 customer);

            auto erased = route_in.erase(std::next(it_in_before));

            const auto cost_after =
                distance_on_route(m_prob, split_in, m_tw_penalty, it_in_before,
                                  std::next(it_in_after)) +
                distance_on_route(m_prob, split_out, m_tw_penalty,
                                  std::prev(it_out), std::next(it_out, 2));

            const bool impossible_move =
                lists.pr_relocate_new_route.has(customer) &&
                cost_after >= best_ever_value;

            // decide whether move is good
            if (!impossible_move && cost_after < cost_before) {
                // move is good
                sln.customer_owners[customer].erase(r_in);
                sln.update_customer_owners(m_prob, r_in, c_index);
                sln.update_customer_owners(m_prob, sln.routes.size() - 1);
                sln.used_vehicles.emplace(used_vehicle);
                lists.relocate_new_route.emplace(customer, r_in);
#if USE_PRESERVE_ENTRIES
                lists.pr_relocate_new_route.emplace(customer);
#endif
                best_ever_value = cost_after;
                improved = true;
                skip_to_next_customer = true;
            } else {
                // move is bad - roll back the changes
                route_in.insert(erased, customer);
                sln.routes.pop_back();
                unused_vehicles.emplace(used_vehicle);
                transfer_split_entry(m_enable_splits, split_out, split_in,
                                     customer);
                sln.route_splits.pop_back();
            }
        }
    }
    delete_loops_after_relocate(sln, lists);
    sln.update_customer_owners(m_prob);
    return improved;
}

bool LocalSearchMethods::relocate_split(Solution& sln, TabuLists& lists) {
    return false;
}

bool LocalSearchMethods::exchange(Solution& sln, TabuLists& lists) {
    static double best_ever_value = std::numeric_limits<double>::max();

    bool improved = false;

    // reminder: skip depot!
    const auto& customers = m_prob.customers;
    std::vector<size_t> ascending_sort_customers(customers.size() - 1);
    std::iota(ascending_sort_customers.begin(), ascending_sort_customers.end(),
              1);

#if SORT_HEURISTIC_OPERANDS
    std::sort(ascending_sort_customers.begin(), ascending_sort_customers.end(),
              [&customers](size_t a, size_t b) {
                  return customers[a].demand < customers[b].demand;
              });
#endif

    // sorting customers: try to exchange equal size customers first
    for (size_t customer : ascending_sort_customers) {
        bool skip_to_next_customer = false;
        auto cfirst = sln.customer_owners[customer].cbegin(),
             clast = sln.customer_owners[customer].cend();
        for (; !skip_to_next_customer && cfirst != clast; ++cfirst) {
            size_t r1 = 0, c_index = 0;
            std::tie(r1, c_index) = *cfirst;
            validate_indices(r1, c_index, sln.routes);
            auto& route1 = sln.routes[r1].second;
            if (is_loop(route1)) {
                continue;
            }
            SplitInfo& split1 = sln.route_splits[r1];
            for (size_t neighbour : ascending_sort_customers) {
                if (skip_to_next_customer) {
                    break;
                }
                if (customer == neighbour) {
                    continue;
                }

                auto nfirst = sln.customer_owners[neighbour].cbegin(),
                     nlast = sln.customer_owners[neighbour].cend();
                for (; !skip_to_next_customer && nfirst != nlast; ++nfirst) {
                    size_t r2 = 0, n_index = 0;
                    std::tie(r2, n_index) = *nfirst;
                    validate_indices(r2, n_index, sln.routes);
                    // do not relocate inside the same route
                    if (r1 == r2) {
                        continue;
                    }

                    auto& route2 = sln.routes[r2].second;
                    if (is_loop(route2)) {
                        continue;
                    }

                    // check if both customers can be exchanged
                    if (!site_dependent(m_prob, sln.routes[r2].first,
                                        customer) ||
                        !site_dependent(m_prob, sln.routes[r1].first,
                                        neighbour)) {
                        // cannot exchange customers within forbidden route
                        continue;
                    }

                    SplitInfo& split2 = sln.route_splits[r2];
                    // FIXME: allow such moves?
                    if (m_enable_splits &&
                        (split2.has(customer) || split1.has(neighbour))) {
                        continue;
                    }

                    // we assume we can exchange two iterators at this point

                    // perform exchange (just swap customer indices)
                    auto it1 = atit(route1, c_index),
                         it2 = atit(route2, n_index);
                    const auto cost_before = paired_distance_on_route(
                        m_prob, split1, split2, m_tw_penalty, it1, it2);

                    auto demand1_before = total_demand(
                             m_prob, split1, route1.cbegin(), route1.cend()),
                         demand2_before = total_demand(
                             m_prob, split2, route2.cbegin(), route2.cend());
                    auto demand_it1 = m_prob.customers[*it1].demand,
                         demand_it2 = m_prob.customers[*it2].demand;

                    std::swap(*it1, *it2);

                    transfer_split_entry(m_enable_splits, split1, split2,
                                         customer);
                    transfer_split_entry(m_enable_splits, split2, split1,
                                         neighbour);

                    const auto cost_after = paired_distance_on_route(
                        m_prob, split1, split2, m_tw_penalty, it1, it2);

                    const auto demand1_after =
                                   demand1_before - demand_it1 + demand_it2,
                               demand2_after =
                                   demand2_before - demand_it2 + demand_it1;

                    // aspiration criteria
                    bool impossible_move = (lists.exchange.has(customer, r2) ||
                                            lists.exchange.has(neighbour, r1) ||
                                            lists.pr_exchange.has(customer) ||
                                            lists.pr_exchange.has(neighbour)) &&
                                           cost_after >= best_ever_value;

                    const auto
                        route1_capacity =
                            m_prob.vehicles[sln.routes[r1].first].capacity,
                        route2_capacity =
                            m_prob.vehicles[sln.routes[r2].first].capacity;
                    impossible_move |= (demand1_after > route1_capacity &&
                                        demand1_after > demand1_before);
                    impossible_move |= (demand2_after > route2_capacity &&
                                        demand2_after > demand2_before);

                    impossible_move |= (!m_can_violate_tw &&
                                        (constraints::total_violated_time(
                                             m_prob, split1, route1.cbegin(),
                                             route1.cend()) != 0 ||
                                         constraints::total_violated_time(
                                             m_prob, split2, route2.cbegin(),
                                             route2.cend()) != 0));

                    // decide whether move is good
                    if (!impossible_move && cost_after < cost_before) {
                        // move is good
                        sln.customer_owners[customer].erase(r1);
                        sln.customer_owners[neighbour].erase(r2);
                        sln.customer_owners[customer][r2] = n_index;
                        sln.customer_owners[neighbour][r1] = c_index;
                        lists.exchange.emplace(customer, r1);
                        lists.exchange.emplace(neighbour, r2);
#if USE_PRESERVE_ENTRIES
                        lists.pr_exchange.emplace(customer);
                        lists.pr_exchange.emplace(neighbour);
#endif
                        best_ever_value = std::min(best_ever_value, cost_after);
                        improved = true;
                        skip_to_next_customer = true;
                    } else {
                        // move is bad - roll back the changes
                        std::swap(*it1, *it2);
                        transfer_split_entry(m_enable_splits, split2, split1,
                                             customer);
                        transfer_split_entry(m_enable_splits, split1, split2,
                                             neighbour);
                    }
                }
            }
        }
    }
    return improved;
}

bool LocalSearchMethods::two_opt(Solution& sln, TabuLists& lists) {
    static double best_ever_value = std::numeric_limits<double>::max();

    bool improved = false;

    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        auto& route = sln.routes[ri].second;

        // Note: no need to check if customer is split or not in case of 2-opt:
        //       can reverse route parts regardless of this information
        const SplitInfo& split = sln.route_splits[ri];

        // we can only improve routes that have 3+ nodes
        bool can_improve = route.size() > 2;
        while (can_improve) {
            bool found_new_best = false;
            // skip depots && beware of k = i + 1
            for (auto i = std::next(route.begin());
                 i != std::prev(route.end(), 2); ++i) {
                if (found_new_best)  // fast loop break
                    break;
                // skip depots && start from i + 1
                for (auto k = std::next(i); k != std::prev(route.end()); ++k) {
                    size_t ic_next = *std::next(i), kc_next = *std::next(k);

                    // cost before: (i-1)->i->(i+1) + (k-1)->k->(k+1)
                    const auto cost_before = paired_distance_on_route(
                        m_prob, split, split, m_tw_penalty, i, k);
                    // perform 2-opt
                    std::reverse(i, std::next(k));
                    // cost after: (i-1)->k->(i+1) + (k-1)->i->(k+1)
                    const auto cost_after = paired_distance_on_route(
                        m_prob, split, split, m_tw_penalty, i, k);

                    // aspiration
                    bool impossible_move = (lists.two_opt.has(*i, *k) ||
                                            lists.pr_two_opt.has(*i) ||
                                            lists.pr_two_opt.has(*k)) &&
                                           cost_after >= best_ever_value;

                    impossible_move |= (!m_can_violate_tw &&
                                        (constraints::total_violated_time(
                                             m_prob, split, route.cbegin(),
                                             route.cend()) != 0));

                    // decide whether move is good
                    if (!impossible_move && cost_after < cost_before) {
                        // move is good
                        found_new_best = true;
                        // forbid previously existing edges
                        lists.two_opt.emplace(*i, ic_next);
                        lists.two_opt.emplace(*k, kc_next);
#if USE_PRESERVE_ENTRIES
                        lists.pr_two_opt.emplace(*i);
                        lists.pr_two_opt.emplace(*k);
#endif
                        best_ever_value = std::min(best_ever_value, cost_after);
                        improved = false;
                        break;
                    } else {
                        // move is bad - roll back the changes
                        std::reverse(i, std::next(k));
                    }
                }
            }

            // if new best found: continue, else: stop
            can_improve = found_new_best;
        }
    }
    sln.update_customer_owners(m_prob);
    return improved;
}

bool LocalSearchMethods::cross(Solution& sln, TabuLists& lists) {
    static double best_ever_value = std::numeric_limits<double>::max();

    bool improved = false;

    const auto size = m_prob.n_customers();
    for (size_t customer = 1; customer < size; ++customer) {
        bool skip_to_next_customer = false;
        auto cfirst = sln.customer_owners[customer].cbegin(),
             clast = sln.customer_owners[customer].cend();
        for (; !skip_to_next_customer && cfirst != clast; ++cfirst) {
            size_t r1 = 0, c_index = 0;
            std::tie(r1, c_index) = *cfirst;
            validate_indices(r1, c_index, sln.routes);
            auto& route1 = sln.routes[r1].second;
            if (is_loop(route1)) {
                continue;
            }
            SplitInfo& split1 = sln.route_splits[r1];
            for (size_t neighbour = 1;
                 !skip_to_next_customer && neighbour < size; ++neighbour) {
                if (customer == neighbour) {
                    continue;
                }

                auto nfirst = sln.customer_owners[neighbour].cbegin(),
                     nlast = sln.customer_owners[neighbour].cend();
                for (; !skip_to_next_customer && nfirst != nlast; ++nfirst) {
                    size_t r2 = 0, n_index = 0;
                    std::tie(r2, n_index) = *nfirst;
                    validate_indices(r2, n_index, sln.routes);
                    // do not relocate inside the same route
                    if (r1 == r2) {
                        continue;
                    }

                    auto& route2 = sln.routes[r2].second;
                    if (is_loop(route2)) {
                        continue;
                    }

                    // check if both customers can be exchanged
                    if (!site_dependent(m_prob, sln.routes[r2].first,
                                        customer) ||
                        !site_dependent(m_prob, sln.routes[r1].first,
                                        neighbour)) {
                        // cannot exchange customers within forbidden route
                        continue;
                    }

                    SplitInfo& split2 = sln.route_splits[r2];

                    // perform exchange (just swap customer indices)
                    auto it1 = atit(route1, c_index),
                         it2 = atit(route2, n_index);

                    std::vector<size_t> customers1(std::next(it1),
                                                   route1.end());
                    std::vector<size_t> customers2(std::next(it2),
                                                   route2.end());
                    // FIXME: allow such moves?
                    if (m_enable_splits && split2.has_any(customers1)) {
                        continue;
                    }
                    if (m_enable_splits && split1.has_any(customers2)) {
                        continue;
                    }

                    // we assume we can exchange two iterators at this point
                    size_t c_next1 = *std::next(it1), c_next2 = *std::next(it2);

                    const auto cost_before =
                        distance_on_route(m_prob, split1, m_tw_penalty, it1,
                                          route1.end()) +
                        distance_on_route(m_prob, split2, m_tw_penalty, it2,
                                          route2.end());

                    const auto demand1_before =
                                   total_demand(m_prob, split1, route1.cbegin(),
                                                route1.cend()),
                               demand2_before =
                                   total_demand(m_prob, split2, route2.cbegin(),
                                                route2.cend());

                    // TODO: optimize
                    transfer_split_entry(m_enable_splits, split1, split2,
                                         customers1.cbegin(),
                                         customers1.cend());
                    transfer_split_entry(m_enable_splits, split2, split1,
                                         customers2.cbegin(),
                                         customers2.cend());

                    cross_routes(route1, std::next(it1), route2,
                                 std::next(it2));

                    const auto cost_after =
                        distance_on_route(m_prob, split1, m_tw_penalty, it1,
                                          route1.end()) +
                        distance_on_route(m_prob, split2, m_tw_penalty, it2,
                                          route2.end());

                    const auto demand1_after =
                                   total_demand(m_prob, split1, route1.cbegin(),
                                                route1.cend()),
                               demand2_after =
                                   total_demand(m_prob, split2, route2.cbegin(),
                                                route2.cend());

                    // aspiration criteria
                    bool impossible_move = (lists.cross.has(customer, r2) ||
                                            lists.cross.has(neighbour, r1) ||
                                            lists.pr_cross.has(customer) ||
                                            lists.pr_cross.has(neighbour)) &&
                                           cost_after >= best_ever_value;

                    const auto
                        route1_capacity =
                            m_prob.vehicles[sln.routes[r1].first].capacity,
                        route2_capacity =
                            m_prob.vehicles[sln.routes[r2].first].capacity;

                    impossible_move |= (demand1_after > route1_capacity &&
                                        demand1_after > demand1_before);
                    impossible_move |= (demand2_after > route2_capacity &&
                                        demand2_after > demand2_before);

                    impossible_move |= (!m_can_violate_tw &&
                                        (constraints::total_violated_time(
                                             m_prob, split1, route1.cbegin(),
                                             route1.cend()) != 0 ||
                                         constraints::total_violated_time(
                                             m_prob, split2, route2.cbegin(),
                                             route2.cend()) != 0));

                    // decide whether move is good
                    if (!impossible_move && cost_after < cost_before) {
                        // move is good
                        for (size_t c1 : customers1) {
                            sln.customer_owners[c1].erase(r1);
                        }
                        for (size_t c2 : customers2) {
                            sln.customer_owners[c2].erase(r2);
                        }
                        sln.update_customer_owners(m_prob, r1, c_index);
                        sln.update_customer_owners(m_prob, r2, n_index);
                        lists.cross.emplace(*it1, c_next1);
                        lists.cross.emplace(*it2, c_next2);
#if USE_PRESERVE_ENTRIES
                        lists.pr_cross.emplace(*it1);
                        lists.pr_cross.emplace(*it2);
#endif
                        best_ever_value = std::min(best_ever_value, cost_after);
                        improved = true;
                        skip_to_next_customer = true;
                    } else {
                        // move is bad - roll back the changes
                        transfer_split_entry(m_enable_splits, split2, split1,
                                             customers1.cbegin(),
                                             customers1.cend());
                        transfer_split_entry(m_enable_splits, split1, split2,
                                             customers2.cbegin(),
                                             customers2.cend());
                        cross_routes(route1, std::next(it1), route2,
                                     std::next(it2));
                    }
                }
            }
        }
    }
    return improved;
}

std::string LocalSearchMethods::str(size_t i) const {
    static const std::vector<std::string> methods = {"relocate", "exchange",
                                                     "two_opt", "cross"};
    return methods[i];
}

void LocalSearchMethods::route_save(Solution& sln, size_t threshold) {
    // _must_ relocate all customers, otherwise do not relocate anyone
    auto sln_copy = sln;

    std::list<size_t> small_routes;
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        auto& route = sln.routes[ri].second;
        if (route.size() > threshold) {
            continue;
        }
        small_routes.emplace_back(ri);
    }
    small_routes.sort([&sln](size_t i, size_t j) {
        return sln.routes[i].second.size() < sln.routes[j].second.size();
    });

    const auto size = m_prob.n_customers();
    while (!small_routes.empty()) {
        auto r_in = small_routes.front();
        small_routes.pop_front();
        auto& route_in = sln.routes[r_in].second;

        const size_t max_iters = route_in.size();
        for (size_t iter = 0; iter < max_iters && !is_loop(route_in); ++iter) {
            size_t customer = *std::next(route_in.cbegin());

            size_t _ = 0, c_index = 0;
            std::tie(_, c_index) = *sln.customer_owners[customer].begin();
            assert(_ == r_in);
            validate_indices(r_in, c_index, sln.routes);
            // check if current route (where customer was relocated) is small,
            // if not anymore, skip
            if (_ == r_in && route_in.size() > threshold) {
                break;
            }

            SplitInfo& split_in = sln.route_splits[r_in];

            for (size_t neighbour = 1; neighbour < size; ++neighbour) {
                if (customer == neighbour) {
                    continue;
                }

                size_t r_out = 0, n_index = 0;
                std::tie(r_out, n_index) =
                    *sln.customer_owners[neighbour].begin();
                validate_indices(r_out, n_index, sln.routes);
                // do not relocate inside the same route
                if (r_in == r_out) {
                    continue;
                }

                auto& route_out = sln.routes[r_out].second;
                if (is_loop(route_out)) {
                    continue;
                }
                if (!site_dependent(m_prob, sln.routes[r_out].first,
                                    customer)) {
                    // cannot insert customer in not allowed route
                    continue;
                }

                SplitInfo& split_out = sln.route_splits[r_out];
                if (m_enable_splits && split_out.has(customer)) {
                    continue;
                }

                // customer value represents the length of route i -> j, where:
                // ... -> (i -> customer -> j) -> ...
                const auto customer_value = distance_on_route(
                    m_prob, split_in, 0, route_in, c_index - 1, c_index + 2);
                const auto customer_neighbour_distance =
                    m_prob.costs[customer][neighbour];
                const auto customer_before_neighbour_value =
                    customer_neighbour_distance +
                    m_prob.costs[customer][at(route_out, n_index - 1)];
                const auto customer_after_neighbour_value =
                    customer_neighbour_distance +
                    m_prob.costs[customer][at(route_out, n_index + 1)];

                // if customer is closer to it's neighbours in __current__
                // route, do not relocate to neighbours in __new__ route
                if (customer_value < customer_before_neighbour_value &&
                    customer_value < customer_after_neighbour_value) {
                    continue;
                }

                // we assume there's a better place for our customer at this
                // point

                auto it_in_before = atit(route_in, c_index - 1),
                     it_in_after = atit(route_in, c_index + 1),
                     it_out_before = atit(route_out, n_index - 1),
                     it_out_after = atit(route_out, n_index + 1);

                const auto cost_before =
                    distance_on_route(m_prob, split_in, m_tw_penalty,
                                      it_in_before, std::next(it_in_after)) +
                    distance_on_route(m_prob, split_out, m_tw_penalty,
                                      it_out_before, std::next(it_out_after));

                Solution::RouteType::iterator inserted, erased;
                if (customer_before_neighbour_value <
                    customer_after_neighbour_value) {
                    inserted =
                        route_out.insert(std::next(it_out_before), customer);
                } else {
                    inserted = route_out.insert(it_out_after, customer);
                }

                transfer_split_entry(m_enable_splits, split_in, split_out,
                                     customer);

                erased = route_in.erase(std::next(it_in_before));

                const auto cost_after =
                    distance_on_route(m_prob, split_in, m_tw_penalty,
                                      it_in_before, std::next(it_in_after)) +
                    distance_on_route(m_prob, split_out, m_tw_penalty,
                                      it_out_before, std::next(it_out_after));

                const auto out_demand_after = total_demand(
                    m_prob, split_out, route_out.cbegin(), route_out.cend());

                const auto out_capacity =
                    m_prob.vehicles[sln.routes[r_out].first].capacity;
                bool impossible_move = (out_demand_after > out_capacity);
                impossible_move |=
                    (!m_can_violate_tw &&
                     (constraints::total_violated_time(m_prob, split_in,
                                                       route_in.cbegin(),
                                                       route_in.cend()) != 0 ||
                      constraints::total_violated_time(m_prob, split_out,
                                                       route_out.cbegin(),
                                                       route_out.cend()) != 0));

                // decide whether move is good
                if (!impossible_move && cost_after < cost_before) {
                    // move is good
                    sln.customer_owners[customer].erase(r_in);
                    sln.update_customer_owners(m_prob, r_in, c_index);
                    sln.update_customer_owners(m_prob, r_out, n_index - 1);
                    break;
                } else {
                    // move is bad - roll back the changes
                    route_in.insert(erased, customer);
                    route_out.erase(inserted);
                    transfer_split_entry(m_enable_splits, split_out, split_in,
                                         customer);
                }
            }
        }

        // get more recent copy of solution if route is emptied
        if (is_loop(route_in)) {
            sln_copy = sln;
        }
    }

    sln = std::move(sln_copy);
    delete_loops_after_relocate(sln);
    sln.update_customer_owners(m_prob);
}

void LocalSearchMethods::intra_relocate(Solution& sln) {
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        auto& route = sln.routes[ri].second;

        // Note: no need to check if customer is split or not in case of 2-opt:
        //       can reverse route parts regardless of this information
        const SplitInfo& split = sln.route_splits[ri];

        for (auto pos = std::next(route.begin()); pos != std::prev(route.end());
             ++pos) {
            for (auto new_pos = std::next(route.begin());
                 new_pos != std::prev(route.end()); ++new_pos) {
                if (pos == new_pos) {
                    continue;
                }

                const auto cost_before = distance_on_route(
                    m_prob, split, m_tw_penalty, route.begin(), route.end());
                // move customer to new position
                std::swap(*pos, *new_pos);
                const auto cost_after = distance_on_route(
                    m_prob, split, m_tw_penalty, route.begin(), route.end());

                const bool impossible_move =
                    (!m_can_violate_tw &&
                     (constraints::total_violated_time(
                          m_prob, split, route.cbegin(), route.cend()) != 0));

                // decide whether move is good
                if (!impossible_move && cost_after >= cost_before) {
                    // move is bad - roll back the changes
                    std::swap(*pos, *new_pos);
                }
            }
        }
    }
    sln.update_customer_owners(m_prob);
}

void LocalSearchMethods::penalize_tw(double value) { m_tw_penalty = value; }
void LocalSearchMethods::violate_tw(bool value) { m_can_violate_tw = value; }
}  // namespace tabu
}  // namespace vrp
