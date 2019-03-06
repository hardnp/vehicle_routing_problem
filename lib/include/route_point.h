#pragma once

#include <cstdint>
#include <iostream>
#include <tuple>
#include <vector>

namespace vrp {
/// vehicle arrive, start, finish times for a route point
class RoutePointTime {
public:
    RoutePointTime() : arrive(0), start(0), finish(0){};
    RoutePointTime(int a, int s, int f) : arrive(a), start(s), finish(f){};
    int arrive = 0;
    int start = 0;
    int finish = 0;
    friend std::ostream& operator<<(std::ostream& out,
                                    const RoutePointTime& t) {
        return out << t.arrive << ';' << t.start << ';' << t.finish << ';';
    }
};
}  // namespace vrp
