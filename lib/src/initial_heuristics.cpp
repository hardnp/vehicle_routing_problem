#include "initial_heuristics.h"

#include "src/internal/cluster_first_route_second.h"
#include "src/internal/savings.h"

namespace vrp {
namespace {
// TODO: fix this somehow later - this introduces quite an overhead
std::vector<Solution> fill_splits(const Problem& prob,
                                  std::vector<Solution>&& slns, bool fill) {
    if (!fill) {
        return slns;
    }

    SplitInfo full_info = {};
    for (size_t c = 0, size = prob.n_customers(); c < size; ++c) {
        full_info.split_info[c] = 1.0;
    }
    for (auto& sln : slns) {
        sln.route_splits.resize(sln.routes.size(), full_info);
    }
    return slns;
}
}  // namespace

std::vector<Solution> create_initial_solutions(const Problem& prob,
                                               InitialHeuristic heuristic,
                                               size_t count) {
    const bool fill_with_default = !prob.enable_splits();
    switch (heuristic) {
    case InitialHeuristic::Savings:
        return fill_splits(prob, detail::savings(prob, count), true);
    case InitialHeuristic::Insertion:
        return {};
    case InitialHeuristic::ParallelInsertion:
        return {};
    case InitialHeuristic::ClusterFirstRouteSecond:
        return fill_splits(prob,
                           detail::cluster_first_route_second(prob, count),
                           fill_with_default);
    default:
        return {};
    }
}
}  // namespace vrp
