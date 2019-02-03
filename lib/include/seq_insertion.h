#include <problem.h>
#include <solution.h>

#ifndef VRP_SOLVER_SEQ_INSERTION_H
#define VRP_SOLVER_SEQ_INSERTION_H

namespace detail {
    std::vector<vrp::Solution> sequential_insertion(std::vector<vrp::Problem> const& problem);
}

#endif //VRP_SOLVER_SEQ_INSERTION_H
