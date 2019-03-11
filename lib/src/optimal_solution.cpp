#include "optimal_solution.h"

#include "src/internal/tabu_search.h"

namespace vrp {
Solution create_optimal_solution(const Problem& prob,
                                 const Solution& initial_sln,
                                 OptimalHeuristic heuristic) {
    switch (heuristic) {
    case OptimalHeuristic::Tabu:
        return detail::tabu_search(prob, initial_sln);
    default:
        return initial_sln;
    }
}
}  // namespace vrp
