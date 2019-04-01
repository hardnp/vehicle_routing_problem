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

template<typename T, typename Callable>
void parallel_range(T iters, Callable f) {
    tbb::parallel_for(
        tbb::blocked_range<T>(T(0), iters),
        [&](const auto& range) { f(range.begin(), range.end()); });
}
#else
template<typename T, typename Callable> void parallel_for(T iters, Callable f) {
    for (T i = 0; i < iters; ++i) {
        f(i);
    }
}

template<typename T, typename Callable>
void parallel_range(T iters, Callable f) {
    f(T(0), iters);
}
#endif
}  // namespace threading
}  // namespace vrp
