#pragma once

#include <cstdint>
#include <utility>
#include <vector>

/// Customer representation
namespace vrp {
class Customer {
public:
    uint64_t id = 0;  ///< customer id
    uint64_t demand = 0;  ///< customer demand
    std::pair<uint64_t, uint64_t> hard_tw = {};  ///< hard time window
    std::pair<uint64_t, uint64_t> soft_tw = {};  ///< soft time window
    uint64_t service_time = 0;  ///< customer service time
    std::vector<uint64_t> suitable_vehicles = {};  ///< vehicles that can serve
                                                   /// the customer
};
}  // vrp
