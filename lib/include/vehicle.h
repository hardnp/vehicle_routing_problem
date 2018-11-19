#pragma once

#include <cstdint>

/// Vehicle representation
namespace vrp {
class Vehicle {
public:
    uint64_t id = 0;  ///< vehicle id
    uint64_t capacity = 0;  ///< vehicle capacity
    double fixed_cost = 0.;  ///< vehicle's fixed cost
    double variable_cost = 0.;  ///< vehicle's variable cost
};
}  // vrp
