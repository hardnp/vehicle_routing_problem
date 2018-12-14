#pragma once

#include <cstdint>
#include "transportation_quantity.h"

namespace vrp {
/// Vehicle representation
class Vehicle {
public:
    int id = 0;  ///< vehicle id
    TrQuant trquant_v; ///vehicle quantity
    double fixed_cost = 0.;  ///< fixed vehicle cost
    double variable_cost = 0.;  ///< variable vehicle cost
};
}  // vrp
