#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

namespace vrp{
namespace detail {
/// Splits string by delimiter
static std::vector<std::string> split(const std::string& src, char delimiter) {
    std::vector<std::string> dst;
    std::istringstream ss_src(src);
    std::string tmp;
    while(std::getline(ss_src, tmp, delimiter))
    {
        dst.push_back(tmp);
    }
    for (auto& value : dst) {
        auto space_pos = std::remove(value.begin(), value.end(), ' ');
        if (space_pos != value.end()) {
            value.erase(space_pos);
        }
    }
    return dst;
}
}  // detail
}  // vrp
