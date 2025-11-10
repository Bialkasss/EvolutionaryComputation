#pragma once
#include "../core/Instance.h"
#include "LocalSearch.h" // For IntraMoveType enum
#include <vector>
#include <random>

using namespace std;

namespace LocalSearchCandidate {

    /**
     * @brief Precompute candidate lists for all nodes.
     * For each node i, finds k nearest neighbors based on D[i][j] + cost[j].
     * 
     * @param I Instance data
     * @param k Number of candidates per node (e.g., 5, 10, 20)
     * @return candidates[i] = vector of k nearest neighbor node IDs for node i
     */
    vector<vector<int>> precompute_candidate_list(const Instance& I, int k);

    /**
     * @brief Steepest local search using candidate moves only.
     * Only evaluates moves that introduce at least one candidate edge.
     * 
     * @param initial_tour Starting solution
     * @param I Instance data
     * @param intra_mode Type of intra-route move (only EDGE_EXCHANGE_2OPT supported)
     * @param candidates Precomputed candidate lists from precompute_candidate_list()
     * @param rng Random number generator (for potential tie-breaking)
     * @return Locally optimal tour
     */
    vector<int> local_search_steepest_candidate(
        const vector<int>& initial_tour,
        const Instance& I,
        LocalSearch::IntraMoveType intra_mode,
        const vector<vector<int>>& candidates,
        mt19937& rng
    );

} // namespace LocalSearchCandidate