#pragma once

#include <tuple>
#include <cstdint>
#include <vector>

namespace vrp {
/// Route point representation
class RoutePoint {
public:
	uint64_t id = 0;  ///< customer id
	bool limited = false;  ///< split delivery limiter

	/// vehicle id, arrive, start, finish
	std::vector<std::tuple<uint64_t, double, double, double>> times;
};
}  // vrp
