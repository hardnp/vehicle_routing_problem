#pragma once

#include "vehicle.h"
#include "customer.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace vrp {
/// Vehicle Routing Problem representation
class Problem {
public:
    std::vector<std::vector<double>> costs = {};    ///< cost matrix
    std::vector<Customer> customers = {};   ///< customer list
    std::vector<Vehicle> vehicles = {}; ///< vehicles list
    std::vector<std::vector<int>> times = {};   ///< time matrix
    int max_violated_soft_tw =
        std::numeric_limits<int>::max();    ///< max number of violated
                                            /// soft time windows
    inline size_t n_customers() const {
        return this->customers.size();
    }
    inline size_t n_vehicles() const {
        return this->vehicles.size();
    }
    inline size_t n_suitable_vehicles(size_t customer) const {
        auto n = this->customers[customer].suitable_vehicles.size();
        if (n) {
            return n;
        }
        return this->n_vehicles();
    }
};
}  // vrp
