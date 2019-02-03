#include "savings.h"
#include <algorithm>
#include <map>
#include <tuple>
#include <iostream>

namespace vrp {
namespace detail {
std::vector<Solution> savings(const Problem& prob, size_t count) {

    std::cout << "SAVINGS HEURISTIC" << std::endl;

    const auto cust_size = prob.customers.size();
    const auto points = cust_size + 1;

    // vehicle id <-> customets ids
	std::vector<std::pair<size_t, std::vector<size_t>>> routes;
    routes.reserve(cust_size);

    // start initialisation
	for (unsigned int i = 0; i < cust_size; ++i) {
		routes.push_back({ i, {0, i, 0} });
	}

    // saving for i and j vertices
	struct S {
		unsigned int i;
		unsigned int j;
		double save_ij;
	};

	std::vector<S> save(points * points, {0, 0, 0});


    // calculating matrix(vector) of savings
	for (unsigned int i = 0; i < points; ++i) {
		for (unsigned int j = 0; j < points; ++j) {
			save[i * points + j] = {i, j, prob.costs[i][0] + prob.costs[0][j] - prob.costs[i][j] };
		}
	}

    //DEBUGGING @DELETE ME@
    std::cout << "savings";
    for (unsigned int i = 0; i < points; ++i) {
        std::cout << std::endl;
        for (unsigned int j = 0; j < points; ++j) {
            std::cout << save[i * points + j].save_ij << ' ';
        }
    }

	std::sort(save.begin(), save.end(), [](const S & a, const S & b) {return a.save_ij < b.save_ij; });

    //DEBUGGING @DELETE ME@
    std::cout << std::endl << "SORTED SAVINGS" << std::endl;
    for (auto a:save) {
        std::cout << a.save_ij << ' ';
    }

    save.erase( std::remove_if(save.begin(), save.end(),
            [](const auto & o) { return o.i == o.j; }), save.end());

    //DEBUGGING @DELETE ME@
    std::cout << std::endl << "REMOVED SAVINGS I J" << std::endl;
    for (auto a : save) {
        std::cout << a.save_ij << ' ';
    }
    std::cout << std::endl;

	const int REGULATOR = std::min((size_t)3, save.size()); // check 3 best savings

	// times for customer
	struct Time {
		unsigned int start;
		unsigned int finish;
	};

    // customer id <-> hard tw
    std::map<size_t, Time> times;

    // DELETE ME
    std::cout << "TIMES (id start finish)" << std::endl;
	for (auto i = 0; i < cust_size; ++i) {
		times[prob.customers[i].id] = {(unsigned int)prob.customers[i].hard_tw.first, (unsigned int)prob.customers[i].hard_tw.second};
        std::cout << prob.customers[i].id << ' ' << times[prob.customers[i].id].start << ' ' << times[prob.customers[i].id].finish << std::endl;
	}

    std::vector<S> fine;
    //tmp variant. just 1 iteration
    int kk = 0;
	while (kk==0) { // main cycle
        kk = 1;
		for (int r = 0; r < REGULATOR; ++r) {
            // current best saving (one of r)
            auto best_save = save[save.size() - 1 - r];

            int time_diff = times[best_save.j].start - times[best_save.i].finish
                - prob.times[best_save.j][best_save.i];
			if (time_diff >= 0) {
				// fine

                std::cout << "going form i = " << best_save.i << " to j = "<< best_save.j << std::endl;

                std::cout <<  "j start: " << times[best_save.j].start << std::endl << "i finish: " << times[best_save.i].finish
                    << std::endl << "with time between: " << prob.times[best_save.j][best_save.i] << std::endl;

				fine.push_back(best_save);
			} else {
				int offset = times[best_save.i].finish
				             + prob.times[best_save.j][best_save.i] - times[best_save.j].start;
				// check the rest of root with offset
                std::cout << "offset "<< offset << std::endl;
                // offset is ok for j-route
                int flag = 0;
                // the ugliest multi-loop in the world //PLS KILL ME
                //
                //
                //RESOLVE 3 best routes possible - pick one according to cap/time
                //
                for (auto route : routes) {
                    for (auto cust : route.second) {
                        if (cust == best_save.j) {
                            // ckeck offset for that j-route
                            for (auto cust : route.second) {
                                if (times[cust].start + offset > times[cust].finish) {
                                    flag = 1;
                                    break;
                                }
                            }
                            goto exit;
                        }
                    }
                }
            exit:
            // offest is OK
                if (!flag) {
                    fine.push_back(best_save);
                }
			}
		}
        for (auto s : fine) {
            std::cout << "add edge for i to j with save" << std::endl;
            std::cout << s.i << ' ' << s.j << ' ' << s.save_ij << std::endl;
        }
        // TODO:
        // 1) check capacity
        // 2) calculate new times if cap is ok
		// 3) somehow remove i-j route
	}
    // after all
    // put here all the solution staff

	return {};
}
}  // detail
}  // vrp
