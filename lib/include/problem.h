#pragma once

#include "vehicle.h"
#include "customer.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

namespace vrp {
/// Vehicle Routing Problem representation
class Problem {
    std::vector<std::vector<bool>> m_allowed_vehicles;  ///< allowed vehicles
                                                        /// for each customer:
                                                        /// true means allowed,
                                                        /// false otherwise
public:
    std::vector<std::vector<double>> costs = {};    ///< cost matrix
    std::vector<Customer> customers = {};   ///< customer list
    std::vector<Vehicle> vehicles = {}; ///< vehicles list
    std::vector<std::vector<int>> times = {};   ///< time matrix
    int max_violated_soft_tw =
        std::numeric_limits<int>::max();    ///< max number of violated
                                            /// soft time windows

    // TODO: should be part of ctor
    void set_up() {
        // Set up allowed vehicles
        const auto customers_size = n_customers();
        m_allowed_vehicles.resize(customers_size, {});
        for (size_t c = 0; c < customers_size; ++c) {
            const auto& suitable = this->customers[c].suitable_vehicles;
            auto& allowed_for_customer = m_allowed_vehicles[c];
            const auto vehicles_size = n_vehicles();
            if (suitable.size() == 0) {
                // if suitable is empty, consider this as "all inclusive"
                // customer: any vehicle can deliver to it
                allowed_for_customer.resize(vehicles_size, true);
            } else {
                // if t is in suitable vehicles, t is allowed for current
                // customer
                allowed_for_customer.resize(vehicles_size, false);
                for (size_t t = 0; t < vehicles_size; ++t) {
                    allowed_for_customer[t] = (suitable.cend() !=
                        std::find(suitable.cbegin(), suitable.cend(),
                            static_cast<int>(t)));
                }
            }
        }
    }

    inline size_t n_customers() const {
        return this->customers.size();
    }
    inline size_t n_vehicles() const {
        return this->vehicles.size();
    }
    /// Get allowed vehicles for customer. Expected `customer` param is 0-based
    inline const std::vector<bool>& allowed_vehicles(size_t customer) const {
        return m_allowed_vehicles[customer];
    }

};
}  // vrp
