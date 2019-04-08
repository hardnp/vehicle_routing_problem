#include "improvement_heuristics.h"

#include "src/internal/tabu_search.h"

namespace vrp {
Solution create_improved_solution(const Problem& prob,
                                  const Solution& initial_sln,
                                  ImprovementHeuristic heuristic) {
    switch (heuristic) {
    case ImprovementHeuristic::Tabu:
        return detail::tabu_search(prob, initial_sln);
    default:
        return initial_sln;
    }
}
}  // namespace vrp
