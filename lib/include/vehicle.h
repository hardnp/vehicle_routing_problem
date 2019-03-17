#pragma once

#include "transportation_quantity.h"
#include <cstdint>

namespace vrp {
/// Vehicle representation
class Vehicle {
public:
    int id = 0;                       ///< vehicle id
    TransportationQuantity capacity;  /// vehicle quantity
    double fixed_cost = 0.;           ///< fixed vehicle cost
    double variable_cost = 0.;        ///< variable vehicle cost
};
}  // namespace vrp
