#pragma once

#include <cstdint>

namespace vrp {
/// Vehicle representation
class Vehicle {
public:
    uint64_t id = 0;  ///< vehicle id
    uint64_t capacity = 0;  ///< vehicle capacity
    uint64_t max_weight = 0;  ///< max vehicle weight
    double fixed_cost = 0.;  ///< fixed vehicle cost
    double variable_cost = 0.;  ///< variable vehicle cost
};
}  // vrp
