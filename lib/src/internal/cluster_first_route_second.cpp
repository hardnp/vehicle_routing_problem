#include "cluster_first_route_second.h"

#if !NO_CPLEX_IMPL
#include "cfrs_cplex_inl.hpp"
#else
#include "stub_cfrs_cplex_inl.hpp"
#endif

/// Specify that the variable x is unused in the code
#define UNUSED(x) (void)x

namespace vrp {
namespace detail {

std::vector<Solution> cluster_first_route_second(const Problem& prob,
    size_t count) {
    return cfrs_impl(prob, count);
}
}  // detail
}  // vrp
