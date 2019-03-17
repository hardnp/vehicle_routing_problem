#pragma once

#include <cstdint>
#include <iostream>
#include <tuple>
#include <vector>

namespace vrp {
/// vehicle arrive, start, finish times for a route point
class RoutePointTime {
public:
    int arrive = 0;
    int start = 0;
    int finish = 0;

    RoutePointTime() = default;
    RoutePointTime(int a, int s, int f) : arrive(a), start(s), finish(f){};
};
}  // namespace vrp
