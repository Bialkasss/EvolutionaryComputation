#include "LocalSearchCandidate.h"

#include <vector>
#include <algorithm>
#include <climits>
#include <iostream>
#include <stdlib.h>

#include "../core/Instance.h"
#include "../core/Objective.h"
#include "LocalSearch.h" // For IntraMoveType enum
#include "LSHelpers.h"   // <--- Added shared helpers

using namespace std;

// (Anonymous namespace with redundant helpers removed)

namespace LocalSearchCandidate {

/**
 * @brief Precompute candidate lists for all nodes.
 * For each node, store k nearest neighbors based on (distance + cost).
 */
vector<vector<int>> precompute_candidate_list(const Instance& I, int k) {
    vector<vector<int>> candidates(I.N);
    
    for (int i = 0; i < I.N; ++i) {
        vector<pair<long long, int>> neighbors;
        neighbors.reserve(I.N - 1);
        
        for (int j = 0; j < I.N; ++j) {
            if (i == j) continue;
            long long dist_cost = (long long)I.D[i][j] + I.cost[j];
            neighbors.push_back({dist_cost, j});
        }
        
        // Sort by distance + cost
        sort(neighbors.begin(), neighbors.end());
        
        // Take top k neighbors
        int num_candidates = min(k, (int)neighbors.size());
        candidates[i].reserve(num_candidates);
        for (int idx = 0; idx < num_candidates; ++idx) {
            candidates[i].push_back(neighbors[idx].second);
        }
    }
    
    return candidates;
}

/**
 * @brief Steepest local search using candidate moves only.
 * Only evaluates moves that introduce at least one candidate edge.
 */
vector<int> local_search_steepest_candidate(
    const vector<int>& initial_tour,
    const Instance& I,
    LocalSearch::IntraMoveType intra_mode,
    const vector<vector<int>>& candidates,
    mt19937& rng)
{
    vector<int> tour = initial_tour;
    
    // Use the shared helper struct
    LSHelpers h(I.N, I.K, tour);
    
    long long current_obj = Objective::calculate(tour, I);

    // Only support EDGE_EXCHANGE_2OPT for now (best from previous assignment)
    if (intra_mode != LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT) {
        cerr << "ERROR: Candidate list LS only supports EDGE_EXCHANGE_2OPT\n";
        return tour;
    }

    while (true) {
        long long best_delta = 0;
        int best_move_type = -1; // 0: inter, 1: intra-edge
        int best_u = -1, best_v = -1; // for inter
        int best_i = -1, best_j = -1; // for intra

        // ============================================================
        // INTER-ROUTE CANDIDATE MOVES
        // ============================================================
        
        for (int pos = 0; pos < I.K; ++pos) {
            int u = tour[pos]; // selected node
            
            // Try swapping u with each of its candidates
            for (int v : candidates[u]) {
                if (h.in_tour[v]) continue; // v must not be in tour
                
                // Use shared delta function
                long long delta = delta_inter_exchange(u, v, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 0;
                    best_u = u;
                    best_v = v;
                }
            }
        }

        // ALSO: For each unselected node v, try swapping with its
        // candidate neighbors u (if u is selected).
        
        for (int v = 0; v < I.N; ++v) {
            if (h.in_tour[v]) continue; // v must not be in tour
            
            // Try swapping v with each of its candidates
            for (int u : candidates[v]) {
                if (!h.in_tour[u]) continue; // u must be in tour
                
                // Use shared delta function
                long long delta = delta_inter_exchange(u, v, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 0;
                    best_u = u;
                    best_v = v;
                }
            }
        }

        // ============================================================
        // INTRA-ROUTE CANDIDATE MOVES (2-OPT)
        // ============================================================
        
        for (int i = 0; i < I.K; ++i) {
            int node_i = tour[i];
            
            // Case 1: New edge (tour[i], tour[j]) is a candidate edge
            for (int node_j : candidates[node_i]) {
                if (!h.in_tour[node_j]) continue; 
                
                int j = h.pos[node_j];
                
                if (abs(j - i) <= 1) continue; 
                if (i == 0 && j == I.K - 1) continue; 
                
                int i_adj = i, j_adj = j;
                if (i_adj > j_adj) swap(i_adj, j_adj);
                
                // Use shared delta function
                long long delta = delta_intra_exchange_edges_2opt(i_adj, j_adj, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 1;
                    best_i = i_adj;
                    best_j = j_adj;
                }
            }
            
            // Case 2: New edge (tour[i+1], tour[j+1]) is a candidate edge
            int node_i_next = tour[(i + 1) % I.K];
            for (int node_j_next : candidates[node_i_next]) {
                if (!h.in_tour[node_j_next]) continue;
                
                int j_next = h.pos[node_j_next];
                int j = (j_next - 1 + I.K) % I.K; 
                
                if (abs(j - i) <= 1) continue;
                if (i == 0 && j == I.K - 1) continue;
                
                int i_adj = i, j_adj = j;
                if (i_adj > j_adj) swap(i_adj, j_adj);
                
                // Use shared delta function
                long long delta = delta_intra_exchange_edges_2opt(i_adj, j_adj, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 1;
                    best_i = i_adj;
                    best_j = j_adj;
                }
            }
        }

        // ============================================================
        // APPLY BEST MOVE
        // ============================================================
        
        if (best_delta < -1e-9) {
            long long old_obj = current_obj;
            
            if (best_move_type == 0) { // inter
                apply_inter_exchange(h, best_u, best_v);
            } else if (best_move_type == 1) { // intra-edge
                apply_intra_exchange_edges_2opt(h, best_i, best_j);
            }
            
            current_obj += best_delta;
            
            // Debugging check
            long long new_obj_calc = Objective::calculate(tour, I);
            if (abs(new_obj_calc - current_obj) > 1) {
                cerr << "FATAL DELTA MISMATCH (CANDIDATE)"
                     << " Old: " << old_obj << " Delta: " << best_delta
                     << " New(calc): " << new_obj_calc
                     << " New(tracked): " << current_obj << endl;
                exit(1);
            }
            current_obj = new_obj_calc; // Resync
        } else {
            break; // Local optimum
        }
    }
    
    return tour;
}

} // namespace LocalSearchCandidate