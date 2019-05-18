#include "constraints.h"
#include "logging.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <set>
#include <type_traits>
#include <unordered_map>

namespace vrp {
namespace constraints {
namespace {
// return if customer and vehicle are site-dependent. true if vehicle can
// deliver to customer, false otherwise
bool site_dependent(const Problem& prob, size_t vehicle, size_t customer) {
    if (customer >= prob.allowed_vehicles_size()) {
        throw std::out_of_range("customer >= allowed vehicles size");
    }
    const auto& allowed = prob.allowed_vehicles(customer);
    if (vehicle >= allowed.size()) {
        throw std::out_of_range("vehicle >= allowed vehicles(customer) size");
    }
    return allowed[vehicle];
}
}  // namespace

int total_violated_time(const Problem& prob, const Solution& sln) {
    int violated_time = 0;

    for (size_t t = 0, size = sln.routes.size(); t < size; ++t) {
        const auto& route = sln.routes[t].second;
        violated_time += total_violated_time(prob, sln.route_splits.at(t),
                                             route.cbegin(), route.cend());
    }

    return violated_time;
}

TransportationQuantity total_violated_capacity(const Problem& prob,
                                               const Solution& sln) {
    TransportationQuantity violated_capacity = {};

    const auto& vehicles = prob.vehicles;
    for (size_t t = 0, size = sln.routes.size(); t < size; ++t) {
        const auto& vehicle_and_route = sln.routes[t];
        size_t v = vehicle_and_route.first;
        const auto& route = vehicle_and_route.second;
        violated_capacity += total_violated_capacity(
            prob, vehicles[v].capacity, sln.route_splits.at(t), route.cbegin(),
            route.cend());
    }

    return violated_capacity;
}

bool satisfies_site_dependency(const Problem& prob, const Solution& sln) {
    bool satisfies = true;
    for (const auto& vehicle_and_route : sln.routes) {
        const size_t v = vehicle_and_route.first;
        const auto& route = vehicle_and_route.second;
        satisfies &=
            std::all_of(route.cbegin(), route.cend(), [&v, &prob](size_t c) {
                return site_dependent(prob, v, c);
            });
    }
    return satisfies;
}

bool satisfies_vehicle_uniqueness(const Problem& prob, const Solution& sln) {
    // verify that each vehicle is used only once
    std::set<size_t> unique_vehicles;
    for (const auto& p : sln.routes) {
        unique_vehicles.emplace(p.first);
    }
    return unique_vehicles.size() == sln.routes.size();
}

bool satisfies_customers_service(const Problem& prob, const Solution& sln) {
    // verify that each customer is serviced and it is done only once
    std::unordered_map<size_t, size_t> customer_counts;
    customer_counts.reserve(prob.n_customers());
    for (const auto& p : sln.routes) {
        const auto& route = p.second;
        for (size_t c : route) {
            customer_counts[c]++;
        }
    }

    // each customer must be serviced
    if (customer_counts.size() != prob.n_customers()) {
        return false;
    }

    // depot is "serviced" 2x number of routes times
    if (customer_counts[0] != sln.routes.size() * 2) {
        return false;
    }

    // each customer (!= depot) is serviced only once
    for (const auto& p : customer_counts) {
        if (p.first == 0) {
            continue;
        }
        if (p.second != 1) {
            return false;
        }
    }

    return true;
}

namespace {
template<bool splits_enabled, typename RatioPredicate>
bool satisfies_split_delivery_impl(const Problem& prob, const Solution& sln,
                                   RatioPredicate pred) {
    std::unordered_map<size_t, SplitInfo::Ratio> ratios_per_customer;
    ratios_per_customer.reserve(prob.n_customers());

    for (size_t i = 0, size = sln.routes.size(); i < size; ++i) {
        const auto& route = sln.routes[i].second;
        const auto& split_info = sln.route_splits[i];
        for (size_t c : route) {
            // split info must have customer that exists in corresponding route
            if (!split_info.has(c)) {
                return false;
            }
            const auto& r = split_info.at(c);
            if (!pred(r)) {
                return false;
            }
            if (ratios_per_customer.find(c) == ratios_per_customer.cend()) {
                ratios_per_customer[c] = r;
            } else {
                ratios_per_customer[c] += r;
            }
        }
    }

    // if split delivery is enabled, must check that customer is fully serviced
    if (splits_enabled) {
        for (const auto& p : ratios_per_customer) {
            if (p.second != 1.0) {
                return false;
            }
        }
    }
    return true;
}
}  // namespace

bool satisfies_split_delivery(const Problem& prob, const Solution& sln) {
    // verify split info is correct
    if (prob.enable_splits()) {
        const auto ratio_checker = [](const SplitInfo::Ratio& r) {
            return r > 0.0 && r <= 1.0;
        };
        return satisfies_split_delivery_impl<true>(prob, sln, ratio_checker);
    } else {
        const auto ratio_checker = [](const SplitInfo::Ratio& r) {
            return r == 1.0;
        };
        return satisfies_split_delivery_impl<false>(prob, sln, ratio_checker);
    }
}

bool satisfies_all(const Problem& prob, const Solution& sln,
                   std::ostream* err) {
    static const std::vector<
        std::pair<std::string, bool (*)(const Problem&, const Solution&)>>
        checkers = {{"site dependency", satisfies_site_dependency},
                    {"capacity", satisfies_capacity},
                    {"time windows", satisfies_time_windows},
                    {"vehicle uniqueness", satisfies_vehicle_uniqueness},
                    {"customers service", satisfies_customers_service},
                    {"routes limit", satisfies_routes_limit},
                    {"split delivery", satisfies_split_delivery}};

    bool satisfied = true;
    for (const auto& checker : checkers) {
        bool res = checker.second(prob, sln);
        if (err != nullptr && res == false) {
            *err << "Unsatisfied: " << checker.first << EOL;
        }
        satisfied &= res;
    }

    return satisfied;
}
}  // namespace constraints
}  // namespace vrp
