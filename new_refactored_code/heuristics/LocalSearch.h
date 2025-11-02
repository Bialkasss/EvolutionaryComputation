#pragma once
#include "../core/Instance.h"
#include <vector>
#include <random>

using namespace std;

namespace LocalSearch {

    enum class IntraMoveType { NODE_EXCHANGE, EDGE_EXCHANGE_2OPT };

    vector<int> local_search_steepest(const vector<int>& initial_tour, 
                                      const Instance& inst, 
                                      IntraMoveType intra_mode, 
                                      mt19937& rng);

    vector<int> local_search_greedy(const vector<int>& initial_tour, 
                                    const Instance& inst, 
                                    IntraMoveType intra_mode, 
                                    mt19937& rng);

} // namespace LocalSearch