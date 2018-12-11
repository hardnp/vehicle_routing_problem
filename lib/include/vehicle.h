#pragma once

#include <cstdint>
#include "transportation_unit.h"

namespace vrp {
/// Vehicle representation
class Vehicle {
public:
    int id = 0;  ///< vehicle id
/// int capacity = 0;  ///< vehicle capacity
/// int max_weight = 0;  ///< max vehicle weight
    TrUnit trunit_v;
    double fixed_cost = 0.;  ///< fixed vehicle cost
    double variable_cost = 0.;  ///< variable vehicle cost
};
}  // vrp
