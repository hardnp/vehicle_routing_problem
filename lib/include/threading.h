#pragma once

#include <cstdint>
#include <utility>

#if USE_TBB
#include "tbb/parallel_for.h"
#endif

namespace vrp {
namespace threading {

#if USE_TBB
template<typename T, typename Callable> void parallel_for(T iters, Callable f) {
    tbb::parallel_for(T(0), iters, [&](T i) { f(i); },
                      tbb::static_partitioner{});
}
#else
template<typename T, typename Callable> void parallel_for(T iters, Callable f) {
    for (T i = 0; i < iters; ++i) {
        f(i);
    }
}
#endif
}  // namespace threading
}  // namespace vrp
