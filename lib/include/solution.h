#pragma once

#include "vehicle.h"
#include "problem.h"

#include <utility>
#include <vector>

namespace vrp {
/// Solution representation
class Solution {
public:
    using VehicleIndex = size_t;  ///< vehicle index in problem's vector
                                  /// index != id
    using CustomerIndex = size_t;  ///< customer index in problem's vector
                                   /// index != id
    std::vector<std::pair<VehicleIndex, std::vector<CustomerIndex>>> routes;

	Solution() {};
	Solution(const Problem& prb) {};
};
}  // vrp
