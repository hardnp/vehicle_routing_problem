#include "objective.h"

namespace vrp {

namespace {
// TODO: get rid of it later
std::vector<size_t> convert(const std::list<size_t>& customers) {
    std::vector<size_t> converted;
    converted.reserve(customers.size());
    for (const auto& c : customers) {
        converted.emplace_back(c);
    }
    return converted;
}
}  // namespace

double objective(const Problem& prob, const Solution& sln) {
    double A = 1.;  // TODO(andrgolubev): add to CSV
    double objective_value = 0.;
    for (const auto& route : sln.routes) {
        const auto& vehicle = prob.vehicles[route.first];
        const auto& customers = convert(route.second);
        for (size_t i = 0; i < customers.size() - 1; ++i) {
            objective_value += vehicle.variable_cost *
                               prob.costs[customers[i]][customers[i + 1]];
            objective_value += A * prob.times[customers[i]][customers[i + 1]];
        }
        objective_value += vehicle.fixed_cost;
    }
    return objective_value;
}
}  // namespace vrp
