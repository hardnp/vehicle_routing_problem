#include "local_search.h"
#include "objective.h"

#include <algorithm>
#include <cassert>
#include <list>
#include <stack>
#include <stdexcept>
#include <unordered_set>

namespace vrp {
namespace tabu {
namespace {

// return if customer and vehicle are site-dependent. true if vehicle can
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

void delete_loops(Solution& sln) {
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

/// calculate distance from first to last-1 on route. this is an oversimplified
/// "objective function part"
inline double distance_on_route(const Problem& prob,
                                const Solution::RouteType& route, size_t first,
                                size_t last) {
    if (first > last || first > route.size()) {
        throw std::out_of_range("invalid indices");
    }
    --last;
    double distance = 0.0;
    for (; first != last; ++first) {
        distance += prob.costs[at(route, first)][at(route, first + 1)];
    }
    return distance;
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

// TODO: check constraints & tabu lists
void LocalSearchMethods::relocate(Solution& sln) {
    const auto vehicles_size = m_prob.n_vehicles();
    std::unordered_set<Solution::VehicleIndex> unused_vehicles;
    assert(vehicles_size >= sln.used_vehicles.size());
    unused_vehicles.reserve(vehicles_size - sln.used_vehicles.size());
    for (size_t v = 0; v < vehicles_size; ++v) {
        if (sln.used_vehicles.find(v) == sln.used_vehicles.cend()) {
            unused_vehicles.emplace(v);
        }
    }

    auto curr_best_value = objective(m_prob, sln);
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
            // not relocate
            if (customer_value < customer_before_neighbour_value &&
                customer_value < customer_after_neighbour_value) {
                continue;
            }

            // we assume there's a better place for our customer at this point

            Solution::RouteType::iterator inserted, erased;
            if (customer_before_neighbour_value <
                customer_after_neighbour_value) {
                inserted = route_out.insert(atit(route_out, n_index), customer);
            } else {
                inserted =
                    route_out.insert(atit(route_out, n_index + 1), customer);
            }
            erased = route_in.erase(atit(route_in, c_index));

            // decide whether move is good
            const auto value = objective(m_prob, sln);
            // move is good
            if (value < curr_best_value) {
                curr_best_value = value;
                sln.update_customer_owners(m_prob, r_in);
                sln.update_customer_owners(m_prob, r_out);
                // TODO: is break valid or we can improve further?
                break;
            }
            // move is bad - roll back the changes
            route_in.insert(erased, customer);
            route_out.erase(inserted);
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
            auto erased = route_in.erase(atit(route_in, c_index));

            // decide whether move is good
            const auto value = objective(m_prob, sln);
            // move is good
            if (value < curr_best_value) {
                curr_best_value = value;
                sln.update_customer_owners(m_prob, r_in);
                sln.update_customer_owners(m_prob, sln.routes.size() - 1);
                sln.used_vehicles.emplace(used_vehicle);
                continue;
            }
            // move is bad - roll back the changes
            route_in.insert(erased, customer);
            sln.routes.pop_back();
            unused_vehicles.emplace(used_vehicle);
        }
    }
    delete_loops(sln);
    sln.update_customer_owners(m_prob);
}

void LocalSearchMethods::relocate_split(Solution& sln) { return; }

void LocalSearchMethods::exchange(Solution& sln) { return; }

// TODO: check constraints & tabu lists
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
                    const auto value = objective(m_prob, sln);
                    // move is good
                    if (value < curr_best_value) {
                        curr_best_value = value;
                        route = std::move(route_copy);
                        found_new_best = true;
                        break;
                    }
                    // move is bad - roll back the changes
                    sln.routes[ri].second = std::move(old_route);
                }
            }

            // if new best found: continue, else: stop
            can_improve = found_new_best;
        }
    }
}
}  // namespace tabu
}  // namespace vrp
