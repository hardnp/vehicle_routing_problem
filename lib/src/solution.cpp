#include "solution.h"

#include <cassert>

namespace vrp {
void transfer_split_entry(bool enable_splits, SplitInfo& src, SplitInfo& dst,
                          size_t key) {
    // do nothing if splits are disabled
    if (!enable_splits) {
        return;
    }

    auto& src_info = src.split_info;
    auto& dst_info = dst.split_info;

    // do nothing for depot case
    static constexpr const size_t depot = 0;
    if (key == depot) {
        return;
    }

    // the key shouldn't exist in dst
    if (dst_info.cend() != dst_info.find(key)) {
        throw std::runtime_error("given key exists in dst already");
    }

    auto src_it = src_info.find(key);
    if (src_info.cend() == src_it) {
        throw std::out_of_range("given key is not in src");
    }
    dst_info.emplace(src_it->first, src_it->second);
    src_info.erase(src_it);
}

void Solution::update_times(const Problem& prob) {
    this->times.clear();

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

            start_time = t.finish + prob.times[c][next_c];

            time.emplace_back(std::move(t));
        }
        // last node is a depot
        time.emplace_back(start_time, start_time, start_time);
    }
}

void Solution::update_customer_owners(const Problem& prob) {
    customer_owners.resize(prob.n_customers());  // TODO: free on same size?
    const auto empty_hash_map = std::unordered_map<size_t, size_t>{};
    std::fill(customer_owners.begin(), customer_owners.end(), empty_hash_map);

    const auto size = routes.size();
    for (size_t ri = 0; ri < size; ++ri) {
        update_customer_owners(prob, ri);
    }
}

void Solution::update_customer_owners(const Problem& prob, size_t route_index,
                                      size_t first_customer_index) {
    // expect allocated at this point
    assert(customer_owners.size() == prob.n_customers());

    const auto& route = routes[route_index].second;
    auto first = std::next(route.cbegin(), first_customer_index);
    for (size_t i = first_customer_index; first != route.cend(); ++first, ++i) {
        // skip depot
        if (*first == 0) {
            continue;
        }
        customer_owners[*first][route_index] = i;
        assert(customer_owners[*first].size() <=
               static_cast<size_t>(prob.max_splits));
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
}  // namespace vrp
