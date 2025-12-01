#include "LargeNeighbourhoodSearch.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>
#include <iostream>
#include <unordered_set>
#include <utility> // For std::pair

#include "../core/Instance.h"
#include "../core/Objective.h"
#include "../utils/Timer.h"
#include "Greedy.h"      
#include "LocalSearch.h" // For steepest LS (Standard)

using namespace std;

namespace LNS {

    // --- Destroy Utils ---
    void destroy_nodes(vector<int>& tour, int n_remove, const Instance& I, mt19937& rng) {
        const auto& D = I.D;
        const auto& C = I.cost;

        for (int k = 0; k < n_remove; ++k) {
            if (tour.size() <= 2) break; 
            vector<double> scores;
            scores.reserve(tour.size());
            double total_score = 0;
            int m = (int)tour.size();

            for (int i = 0; i < m; ++i) {
                int u = tour[i];
                int prev = tour[(i - 1 + m) % m];
                int next = tour[(i + 1) % m];
                long long detour = (long long)D[prev][u] + D[u][next] - D[prev][next];
                double score = (double)(C[u] + detour); 
                if (score < 0) score = 0; 
                scores.push_back(score);
                total_score += score;
            }

            int remove_idx = -1;
            if (total_score <= 1e-9) {
                uniform_int_distribution<int> dist(0, m - 1);
                remove_idx = dist(rng);
            } else {
                uniform_real_distribution<double> dist(0.0, total_score);
                double r = dist(rng);
                double acc = 0.0;
                remove_idx = m - 1;
                for(int i=0; i<m; ++i) {
                    acc += scores[i];
                    if (acc >= r) { remove_idx = i; break; }
                }
            }
            tour.erase(tour.begin() + remove_idx);
        }
    }

    void destroy_edges(vector<int>& tour, int n_remove, const Instance& I, mt19937& rng) {
        const auto& D = I.D;
        const auto& C = I.cost;
        
        for (int k = 0; k < n_remove; ++k) {
            if (tour.size() <= 2) break;
            vector<double> scores;
            int m = (int)tour.size();
            scores.reserve(m);
            double total_score = 0;

            for (int i = 0; i < m; ++i) {
                int u = tour[i];
                int v = tour[(i + 1) % m];
                double score = (double)D[u][v]; 
                scores.push_back(score);
                total_score += score;
            }

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
                    if (acc >= r) { edge_idx = i; break; }
                }
            }

            int u_idx = edge_idx;
            int v_idx = (edge_idx + 1) % m;
            int u = tour[u_idx];
            int v = tour[v_idx];
            if (C[u] > C[v]) tour.erase(tour.begin() + u_idx);
            else tour.erase(tour.begin() + v_idx);
        }
    }

    // =========================================================
    // REPAIR OPERATOR: Weighted Greedy 2-Regret
    // =========================================================
    void repair_regret_weighted(vector<int>& tour, const Instance& I, mt19937& rng, double w_reg = 1.0, double w_cost = 1.0) {
        const auto& D = I.D; 
        const auto& C = I.cost; 
        int K = I.K; 
        int N = I.N;

        // 1. Identify used nodes based on the PARTIAL tour passed in
        vector<char> used(N, 0);
        for (int x : tour) used[x] = 1;

        // 2. Loop until size is K
        while ((int)tour.size() < K) {
            int m = (int)tour.size();

            double best_score = -1e300;         
            long long best1_for_tie = LLONG_MAX;  
            int best_j = -1;
            int best_edge_i = -1;

            for (int j = 0; j < N; ++j) if (!used[j]) {
                
                long long cost_b1 = LLONG_MAX;
                long long cost_b2 = LLONG_MAX;
                int pos_b1 = -1;

                for (int i = 0; i < m; ++i) {
                    int u = tour[i]; 
                    int v = tour[(i+1)%m];
                    
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total_cost = delta + C[j];

                    if (total_cost < cost_b1) {
                        cost_b2 = cost_b1;
                        cost_b1 = total_cost;
                        pos_b1 = i;
                    } else if (total_cost < cost_b2) {
                        cost_b2 = total_cost;
                    }
                }

                if (cost_b2 == LLONG_MAX) cost_b2 = cost_b1;

                long long regret = cost_b2 - cost_b1;

                // Weighted Score
                double score = w_reg * (double)regret - w_cost * (double)cost_b1;

                if (score > best_score) {
                    best_score = score;
                    best_j = j;
                    best_edge_i = pos_b1;
                    best1_for_tie = cost_b1;
                } else if (std::abs(score - best_score) < 1e-12) {
                    if (cost_b1 < best1_for_tie ||
                        (cost_b1 == best1_for_tie && uniform_int_distribution<int>(0,1)(rng))) {
                        best_j = j;
                        best_edge_i = pos_b1;
                        best1_for_tie = cost_b1;
                    }
                }
            }

            if (best_j != -1) {
                tour.insert(tour.begin() + (best_edge_i + 1), best_j);
                used[best_j] = 1;
            } else {
                break; 
            }
        }
    }

    // UPDATED: Now returns a pair <BestTour, IterationCount>
    pair<vector<int>, int> run(const Instance& I, double T_max_ms, bool use_ls, mt19937& rng) {
        Stopwatch sw;
        
        // Initial Solution
        auto S_curr = GreedyHeuristics::random_solution(I, rng, -1);
        S_curr = LocalSearch::local_search_steepest(S_curr, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);

        long long f_curr = Objective::calculate(S_curr, I);
        
        vector<int> S_best = S_curr;
        long long f_best = f_curr;

        double removal_rate = 0.30; 
        int iterations = 0; // Move/Iteration Counter

        while (sw.elapsed_ms() < T_max_ms) {
            iterations++;
            vector<int> S_prime = S_curr; 

            // DESTROY
            int n_remove = (int)std::round(I.K * removal_rate);
            if (n_remove < 1) n_remove = 1;

            if (uniform_int_distribution<int>(0, 1)(rng) == 0) {
                destroy_nodes(S_prime, n_remove, I, rng);
            } else {
                destroy_edges(S_prime, n_remove, I, rng);
            }

            // REPAIR
            repair_regret_weighted(S_prime, I, rng, 1.0, 1.0);

            // OPTIONAL LS
            if (use_ls) {
                S_prime = LocalSearch::local_search_steepest(S_prime, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            }

            long long f_prime = Objective::calculate(S_prime, I);
            
            if (f_prime < f_curr) {
                S_curr = S_prime;
                f_curr = f_prime;

                if (f_prime < f_best) {
                    S_best = S_prime;
                    f_best = f_prime;
                }
            }
        }
        
        // Return both the tour and the number of iterations performed
        return {S_best, iterations};
    }
}