#include "solution.h"

#include <cassert>

namespace vrp {
void Solution::update_times(const Problem& prob) {
    auto& sln = *this;

    sln.times.reserve(sln.routes.size());
    for (const auto& values : sln.routes) {
        sln.times.emplace_back(values.first, std::list<vrp::RoutePointTime>{});
    }
    // TODO: optimize
    static const auto at = [](const auto& list, size_t i) {
        if (list.size() <= i) {
            throw std::out_of_range("index out of list's range");
        }
        return *std::next(list.begin(), i);
    };
    const auto& customers = prob.customers;
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        const auto& route = sln.routes[ri].second;
        auto& time = sln.times[ri].second;
        int start_time = 0;
        for (size_t i = 0; i < route.size() - 1; ++i) {
            auto c = at(route, i);
            auto next_c = at(route, i + 1);

            RoutePointTime t;
            t.arrive = start_time;
            t.start = std::max(t.arrive, customers[c].hard_tw.first);
            t.finish = t.start + customers[c].service_time;

            start_time += t.finish + prob.times[c][next_c];

            time.emplace_back(std::move(t));
        }
        // last node is a depot
        time.emplace_back(start_time, start_time, start_time);
    }
}

void Solution::update_customer_owners(const Problem& prob) {
    customer_owners.resize(prob.n_customers());  // TODO: free on same size?

    const auto size = routes.size();
    for (size_t ri = 0; ri < size; ++ri) {
        update_customer_owners(prob, ri);
    }
}

// FIXME: use last parameter
void Solution::update_customer_owners(const Problem& prob, size_t route_index,
                                      size_t) {
    // expect allocated at this point
    assert(customer_owners.size() == prob.n_customers());

    const auto& route = routes[route_index].second;
    size_t i = 0;
    for (CustomerIndex customer : route) {
        customer_owners[customer] = std::make_pair(route_index, i);
        ++i;
    }
}

void Solution::update_used_vehicles() {
    used_vehicles.reserve(routes.size());

    for (const auto& vehicle_and_route : routes) {
        used_vehicles.emplace(vehicle_and_route.first);
    }
}

bool Solution::operator==(const Solution& other) const {
    if (this->routes.size() != other.routes.size()) {
        return false;
    }
    for (size_t i = 0, size = this->routes.size(); i < size; ++i) {
        if (this->routes[i] != other.routes[i]) {
            return false;
        }
    }
    return true;
}

Solution::operator bool() { return this->routes.empty(); }
}  // namespace vrp
