#include "cluster_first_route_second.h"

#if !NO_CPLEX_IMPL
#include "cfrs_cplex_inl.h"
#else
#include "stub_cfrs_cplex_inl.h"
#endif

namespace vrp {
namespace detail {

std::vector<Solution> cluster_first_route_second(const Problem& prob,
                                                 size_t count) {
    return cfrs_impl(prob, count);
}
}  // namespace detail
}  // namespace vrp
