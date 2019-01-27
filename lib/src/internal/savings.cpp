#include "savings.h"
#include <algorithm>
#include <map>
#include <tuple>

namespace vrp {
namespace detail {
std::vector<Solution> savings(const Problem& prob, size_t count) {
	using VehicleIndex = size_t;
	using CustomerIndex = size_t;
	std::vector<std::pair<VehicleIndex, std::vector<CustomerIndex>>> routes;

	for (unsigned int i = 0; i < prob.customers.size(); ++i) {
		routes.push_back({ i, {0, i, 0} });
	}

	struct S {
		unsigned int i;
		unsigned int j;
		double save_ij;
	};

	std::vector<S> save(prob.customers.size() * prob.customers.size(), {0, 0, 0});

	for (unsigned int i = 0; i < prob.customers.size(); ++i) {
		for (unsigned int j = 0; j < prob.customers.size(); ++j) {
			save[i * prob.customers.size() + j] = {i, j, prob.costs[i][0] + prob.costs[0][j] - prob.costs[i][j] };
		}
	}

	std::sort(save.begin(), save.end(), [](const S & a, const S & b) {return a.save_ij < b.save_ij; });

	const int REGULATOR = std::min((size_t)3, save.size()); // check 3 best savings

	// times for customer
	struct Time {
		unsigned int start;
		unsigned int finish;
	};
    std::map<CustomerIndex, Time> times;

	for (auto i = 0; i < prob.customers.size(); ++i) {
		times[prob.customers[i].id] = {(unsigned int)prob.customers[i].hard_tw.first, (unsigned int)prob.customers[i].hard_tw.second};
	}

    std::vector<S> fine;

	while (save.size() > 0) { // main cycle

		for (int r = 0; r < REGULATOR; ++r) {
			if (times[save[save.size() - 1 - r].j].start - times[save[save.size() - 1 - r].i].finish
			        - prob.times[save[save.size() - 1 - r].j][save[save.size() - 1 - r].i] >= 0) {
				// fine
				fine.push_back(save[save.size() - 1 - r]);
			} else {
				int offset = times[save[save.size() - 1 - r].i].finish
				             + prob.times[save[save.size() - 1 - r].j][save[save.size() - 1 - r].i] - times[save[save.size() - 1 - r].j].start;
				// check the rest of root with offset
			}
		}

		//
		// put here all the solution staff
		//
	}

	return {};
}
}  // detail
}  // vrp
