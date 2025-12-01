#include "LNS.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>
#include <iostream>
#include <unordered_set>

#include "../core/Instance.h"
#include "../core/Objective.h"
#include "../utils/Timer.h"
#include "../utils/Helpers.h"

// Assuming these headers exist based on your upload
#include "../heuristics/Greedy.h"      // For initial random solution
#include "../heuristics/LocalSearch.h" // For steepest LS

using namespace std;

namespace LNS {

    // =========================================================
    // HELPER: Roulette Wheel Selection
    // =========================================================
    // Selects an index from 'candidates' where probability is proportional to score
    template<typename T>
    int roulette_select(const vector<T>& candidates, const vector<double>& scores, double total_score, mt19937& rng) {
        if (total_score <= 1e-9) {
            // Fallback to uniform if all scores are 0
            uniform_int_distribution<int> dist(0, (int)candidates.size() - 1);
            return dist(rng);
        }

        uniform_real_distribution<double> dist(0.0, total_score);
        double r = dist(rng);
        double accumulator = 0.0;

        for (size_t i = 0; i < candidates.size(); ++i) {
            accumulator += scores[i];
            if (accumulator >= r) {
                return (int)i;
            }
        }
        return (int)candidates.size() - 1; // Should not happen ideally
    }

    // =========================================================
    // OPERATOR: Destroy Nodes (Roulette Wheel on Costs)
    // =========================================================
    // Removes 'n_remove' nodes. Higher Cost/Detour = Higher prob of removal.
    void destroy_nodes(vector<int>& tour, int n_remove, const Instance& I, mt19937& rng) {
        const auto& D = I.D;
        const auto& C = I.cost;

        for (int k = 0; k < n_remove; ++k) {
            if (tour.size() <= 2) break; // Don't destroy tiny tours

            vector<double> scores;
            scores.reserve(tour.size());
            double total_score = 0;
            int m = (int)tour.size();

            // Calculate badness score for each node
            for (int i = 0; i < m; ++i) {
                int u = tour[i];
                int prev = tour[(i - 1 + m) % m];
                int next = tour[(i + 1) % m];

                // Badness = NodeCost + Detour caused by this node
                // Detour = Dist(prev, u) + Dist(u, next) - Dist(prev, next)
                long long detour = (long long)D[prev][u] + D[u][next] - D[prev][next];
                double score = (double)(C[u] + detour); 
                
                // Ensure non-negative for roulette
                if (score < 0) score = 0; 
                
                scores.push_back(score);
                total_score += score;
            }

            // Select index to remove
            // Note: We pass indices [0..m-1] implicitly
            // We implement custom selection here to avoid creating a vector of indices every time
            if (total_score <= 1e-9) {
                // Uniform removal
                uniform_int_distribution<int> dist(0, m - 1);
                tour.erase(tour.begin() + dist(rng));
            } else {
                uniform_real_distribution<double> dist(0.0, total_score);
                double r = dist(rng);
                double acc = 0.0;
                int remove_idx = m - 1;
                for(int i=0; i<m; ++i) {
                    acc += scores[i];
                    if (acc >= r) {
                        remove_idx = i;
                        break;
                    }
                }
                tour.erase(tour.begin() + remove_idx);
            }
        }
    }

    // =========================================================
    // OPERATOR: Destroy Edges (Roulette Wheel on Distances)
    // =========================================================
    // Targets long edges. Removes one of the nodes attached to the edge.
    void destroy_edges(vector<int>& tour, int n_remove, const Instance& I, mt19937& rng) {
        const auto& D = I.D;
        
        for (int k = 0; k < n_remove; ++k) {
            if (tour.size() <= 2) break;

            vector<double> scores;
            int m = (int)tour.size();
            scores.reserve(m);
            double total_score = 0;

            // Calculate score for edge (i, i+1)
            for (int i = 0; i < m; ++i) {
                int u = tour[i];
                int v = tour[(i + 1) % m];
                double score = (double)D[u][v]; // Score = length of edge
                scores.push_back(score);
                total_score += score;
            }

            // Select edge index 'i' (edge from tour[i] to tour[i+1])
            int edge_idx = -1;
            
            if (total_score <= 1e-9) {
                uniform_int_distribution<int> dist(0, m - 1);
                edge_idx = dist(rng);
            } else {
                uniform_real_distribution<double> dist(0.0, total_score);
                double r = dist(rng);
                double acc = 0.0;
                edge_idx = m - 1;
                for(int i=0; i<m; ++i) {
                    acc += scores[i];
                    if (acc >= r) {
                        edge_idx = i;
                        break;
                    }
                }
            }

            // Decide which node to remove: u or v?
            // Heuristic: remove the one with higher node cost
            int u_idx = edge_idx;
            int v_idx = (edge_idx + 1) % m;
            int u = tour[u_idx];
            int v = tour[v_idx];

            if (I.cost[u] > I.cost[v]) {
                tour.erase(tour.begin() + u_idx);
            } else {
                tour.erase(tour.begin() + v_idx);
            }
        }
    }

    // =========================================================
    // OPERATOR: Repair (Greedy 2-Regret)
    // =========================================================
    // Adapted from Regret.cpp to work on a partial tour
    void repair_regret(vector<int>& tour, const Instance& I, mt19937& rng) {
        const auto& D = I.D; 
        const auto& C = I.cost; 
        int K = I.K; 
        int N = I.N;

        // 1. Identify used nodes
        vector<char> used(N, 0);
        for (int x : tour) used[x] = 1;

        // 2. Loop until size is K
        while ((int)tour.size() < K) {
            int m = (int)tour.size();

            long long best_score = LLONG_MIN;      // MAXIMIZE regret
            int best_j = -1;
            int best_edge_i = -1;     // insert between tour[i] and tour[(i+1)%m]
            long long best1_for_pick = LLONG_MAX;  // Tie-breaker (min absolute cost)

            // Iterate all candidates (nodes not in tour)
            for (int j = 0; j < N; ++j) if (!used[j]) {
                
                // Calculate insertion costs for all positions
                // We need to find Best Cost (c1) and 2nd Best Cost (c2)
                
                long long c1 = LLONG_MAX;
                long long c2 = LLONG_MAX;
                int best_pos_for_j = -1;

                // Check all edges in current tour
                for (int i = 0; i < m; ++i) {
                    int u = tour[i]; 
                    int v = tour[(i+1)%m];
                    
                    // Cost increase = dist(u,j) + dist(j,v) - dist(u,v) + cost(j)
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total = delta + C[j];

                    if (total < c1) {
                        c2 = c1;
                        c1 = total;
                        best_pos_for_j = i;
                    } else if (total < c2) {
                        c2 = total;
                    }
                }

                // If only 1 position exists (shouldn't happen if m >= 2, but safety)
                if (c2 == LLONG_MAX) c2 = c1;

                // Regret = 2nd Best - Best
                long long regret = c2 - c1;

                // Check if this candidate has the highest regret
                bool take = false;
                if (regret > best_score) {
                    take = true;
                } else if (regret == best_score) {
                    // Tie-break: pick the one with cheaper actual insertion
                    if (c1 < best1_for_pick) {
                        take = true;
                    } else if (c1 == best1_for_pick) {
                        // Random tie-break
                        if (uniform_int_distribution<int>(0,1)(rng)) take = true;
                    }
                }

                if (take) {
                    best_score = regret;
                    best_j = j;
                    best_edge_i = best_pos_for_j;
                    best1_for_pick = c1;
                }
            }

            // Insert the winner
            // Insert at best_edge_i + 1 (between i and i+1)
            if (best_j != -1) {
                tour.insert(tour.begin() + (best_edge_i + 1), best_j);
                used[best_j] = 1;
            } else {
                // Should not happen unless K > N
                break;
            }
        }
    }

    // =========================================================
    // MAIN LNS LOOP
    // =========================================================
    vector<int> run(const Instance& I, double T_max_ms, bool use_ls, mt19937& rng) {
        Stopwatch sw;
        
        // 1. Initial Solution: Random + Local Search
        // Using "random_solution" from Greedy.h if available, or manual shuffle
        vector<int> S_curr = GreedyHeuristics::random_solution(I, rng, -1);
        
        // Always apply LS to start
        // Using EDGE_EXCHANGE_2OPT (assumed typical enum value or pass via wrapper)
        // Assuming LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT exists
        S_curr = LocalSearch::local_search_steepest(S_curr, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);

        long long f_curr = Objective::calculate(S_curr, I);
        
        vector<int> S_best = S_curr;
        long long f_best = f_curr;

        // Parameters
        double removal_rate = 0.30; // 30% default

        // 2. Main Loop
        int iter = 0;
        while (sw.elapsed_ms() < T_max_ms) {
            iter++;
            vector<int> S_prime = S_curr; // Copy current

            // --- A. DESTROY ---
            int n_remove = (int)std::round(I.K * removal_rate);
            if (n_remove < 1) n_remove = 1;

            // Randomly choose destroy operator (50/50)
            if (uniform_int_distribution<int>(0, 1)(rng) == 0) {
                destroy_nodes(S_prime, n_remove, I, rng);
            } else {
                destroy_edges(S_prime, n_remove, I, rng);
            }

            // --- B. REPAIR ---
            // Fills S_prime back to size K using Greedy 2-Regret
            repair_regret(S_prime, I, rng);

            // --- C. OPTIONAL LOCAL SEARCH ---
            if (use_ls) {
                S_prime = LocalSearch::local_search_steepest(S_prime, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            }

            // --- D. ACCEPTANCE (Simple Descent) ---
            long long f_prime = Objective::calculate(S_prime, I);
            
            if (f_prime < f_curr) {
                S_curr = S_prime;
                f_curr = f_prime;

                if (f_prime < f_best) {
                    S_best = S_prime;
                    f_best = f_prime;
                    // Optional: Print progress
                    // cerr << "New best LNS: " << f_best << " Time: " << sw.elapsed_ms() << endl;
                }
            }
        }

        // Return best found
        return S_best;
    }
}