#pragma once
#include "../core/Instance.h"
#include <vector>
#include <random>

using namespace std;

namespace GreedyHeuristics {

    // CH_RANDOM
    vector<int> random_solution(const Instance& inst, mt19937& rng, int start_anchor = -1);

    // CH_NN_END_ONLY
    vector<int> nn_end_only(const Instance& inst, int start_node, mt19937& rng);
    
    // CH_NN_INSERT_ANYWHERE_PATH
    vector<int> nn_path_insert_anywhere(const Instance& inst, int start_node, mt19937& rng);
    
    // CH_GREEDY_CYCLE_CHEAPEST_INSERTION
    vector<int> greedy_cycle_cheapest_insertion(const Instance& inst, int start_node, mt19937& rng);

} // namespace GreedyHeuristics