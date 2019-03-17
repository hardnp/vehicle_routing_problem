#pragma once

#include "transportation_quantity.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace vrp {
/// Customer representation
class Customer {
public:
    int id = 0;                        ///< customer id. 0 is reserved for depot
    TransportationQuantity demand;     /// customer demand quantity
    std::pair<int, int> hard_tw = {};  ///< hard time window
    std::pair<int, int> soft_tw = {};  ///< soft time window
    int service_time = 0;              ///< customer service time

    // TODO: change to vector<bool> where index is the vehicle number, value
    // (bool) is whether the vehicle is allowed
    std::vector<int> suitable_vehicles = {};  ///< vehicles that can serve
                                              /// the customer
};
}  // namespace vrp
