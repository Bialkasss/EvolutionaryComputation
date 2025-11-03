#include "LocalSearchCandidate.h"

#include <vector>
#include <algorithm>
#include <climits>
#include <iostream>
#include <stdlib.h>

#include "../core/Instance.h"
#include "../core/Objective.h"
#include "LocalSearch.h" // For IntraMoveType enum

using namespace std;

namespace {

/**
 * @brief Helper struct for candidate-list local search.
 * Maintains tour state and fast lookups.
 */
struct LSCandidateHelpers {
    int K, N;
    vector<int>& tour;
    vector<int> pos;      // pos[node_id] -> index in tour, or -1
    vector<bool> in_tour; // in_tour[node_id] -> true if in tour

    LSCandidateHelpers(int n, int k, vector<int>& t) 
        : N(n), K(k), tour(t), pos(n, -1), in_tour(n, false) {
        build();
    }
    
    void build() {
        fill(pos.begin(), pos.end(), -1);
        fill(in_tour.begin(), in_tour.end(), false);
        for (int i = 0; i < K; ++i) {
            int node = tour[i];
            pos[node] = i;
            in_tour[node] = true;
        }
    }
};

// --- DELTA CALCULATIONS (same as LocalSearch.cpp) ---

static long long delta_inter_exchange(int u, int v, const LSCandidateHelpers& h, const Instance& I) {
    const auto& D = I.D;
    const auto& C = I.cost;
    int K = h.K;
    int i = h.pos[u];
    
    int prev_u = h.tour[(i - 1 + K) % K];
    int next_u = h.tour[(i + 1) % K];

    long long cost_delta = (long long)C[v] - C[u];
    long long edge_delta = (long long)D[prev_u][v] + D[v][next_u] 
                         - D[prev_u][u] - D[u][next_u];

    return cost_delta + edge_delta;
}

static long long delta_intra_exchange_edges_2opt(int i, int j, const LSCandidateHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (j < i) swap(i, j);

    int u = h.tour[i];
    int v = h.tour[(i + 1) % K];
    int x = h.tour[j];
    int y = h.tour[(j + 1) % K];

    if (v == x || u == y) return 0;

    long long delta = (long long)D[u][x] + D[v][y] - D[u][v] - D[x][y];
    return delta;
}

// --- MOVE APPLICATIONS (same as LocalSearch.cpp) ---

static void apply_inter_exchange(LSCandidateHelpers& h, int u, int v) {
    int i = h.pos[u];
    h.tour[i] = v;
    h.pos[u] = -1;
    h.in_tour[u] = false;
    h.pos[v] = i;
    h.in_tour[v] = true;
}

static void apply_intra_exchange_edges_2opt(LSCandidateHelpers& h, int i, int j) {
    int K = h.K;
    if (j < i) swap(i, j);
    reverse(h.tour.begin() + i + 1, h.tour.begin() + j + 1);
    h.build();
}

} // end anonymous namespace

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
    LSCandidateHelpers h(I.N, I.K, tour);
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
        // Strategy: For each selected node u, try swapping with its 
        // candidate neighbors v (if v is not selected).
        // This ensures at least one candidate edge is introduced.
        
        for (int pos = 0; pos < I.K; ++pos) {
            int u = tour[pos]; // selected node
            
            // Try swapping u with each of its candidates
            for (int v : candidates[u]) {
                if (h.in_tour[v]) continue; // v must not be in tour
                
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
        // This catches the "other direction" of candidate edges.
        
        for (int v = 0; v < I.N; ++v) {
            if (h.in_tour[v]) continue; // v must not be in tour
            
            // Try swapping v with each of its candidates
            for (int u : candidates[v]) {
                if (!h.in_tour[u]) continue; // u must be in tour
                
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
        // Strategy: For each node u in tour, try 2-opt moves where
        // one of the new edges involves a candidate of u.
        //
        // 2-opt removes edges (tour[i], tour[i+1]) and (tour[j], tour[j+1])
        // and adds edges (tour[i], tour[j]) and (tour[i+1], tour[j+1]).
        //
        // We want at least one new edge to be a candidate edge.
        
        for (int i = 0; i < I.K; ++i) {
            int node_i = tour[i];
            
            // Case 1: New edge (tour[i], tour[j]) is a candidate edge
            // So tour[j] must be a candidate of tour[i]
            for (int node_j : candidates[node_i]) {
                if (!h.in_tour[node_j]) continue; // must be in tour
                
                int j = h.pos[node_j];
                
                // Check validity of 2-opt move
                if (abs(j - i) <= 1) continue; // positions too close
                if (i == 0 && j == I.K - 1) continue; // wraps entire tour
                
                // Ensure i < j for delta calculation
                int i_adj = i, j_adj = j;
                if (i_adj > j_adj) swap(i_adj, j_adj);
                
                long long delta = delta_intra_exchange_edges_2opt(i_adj, j_adj, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 1;
                    best_i = i_adj;
                    best_j = j_adj;
                }
            }
            
            // Case 2: New edge (tour[i+1], tour[j+1]) is a candidate edge
            // So tour[j+1] must be a candidate of tour[i+1]
            int node_i_next = tour[(i + 1) % I.K];
            for (int node_j_next : candidates[node_i_next]) {
                if (!h.in_tour[node_j_next]) continue;
                
                int j_next = h.pos[node_j_next];
                int j = (j_next - 1 + I.K) % I.K; // j+1 = j_next, so j = j_next - 1
                
                // Check validity
                if (abs(j - i) <= 1) continue;
                if (i == 0 && j == I.K - 1) continue;
                
                int i_adj = i, j_adj = j;
                if (i_adj > j_adj) swap(i_adj, j_adj);
                
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