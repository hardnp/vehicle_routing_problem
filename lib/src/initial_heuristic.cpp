#include "initial_heuristics.h"

#include "src/internal/cluster_first_route_second.h"

namespace vrp {
std::vector<Solution> create_initial_solutions(const Problem& prob,
    InitialHeuristic heuristic, size_t count) {
    switch (heuristic) {
        case InitialHeuristic::Savings: return {};
        case InitialHeuristic::Insertion: return {};
        case InitialHeuristic::ParallelInsertion: return {};
        case InitialHeuristic::ClusterFirstRouteSecond:
            return detail::cluster_first_route_second(prob, heuristic, count);
        default: return {};
    }
}
}  // vrp
