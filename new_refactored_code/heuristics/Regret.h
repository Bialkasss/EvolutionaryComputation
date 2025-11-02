#pragma once
#include "../core/Instance.h"
#include <vector>
#include <random>

using namespace std;

namespace RegretHeuristics {

    vector<int> nn_path_insert_anywhere_regret2(const Instance& inst, int start_node, mt19937& rng);

    vector<int> nn_path_insert_anywhere_regret2_weighted(const Instance& inst, int start_node, mt19937& rng, double w_reg, double w_cost);

    vector<int> greed_cycle_regret2(const Instance& inst, int start_node, mt19937& rng);

    vector<int> greed_cycle_regret2_weighted(const Instance& inst, int start_node, mt19937& rng, double w_reg, double w_cost);

} // namespace RegretHeuristics