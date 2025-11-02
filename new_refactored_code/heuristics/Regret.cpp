#include "Regret.h"

#include <vector>
#include <random>
#include <algorithm> // For sort, swap
#include <climits>   // For LLONG_MAX, LLONG_MIN
#include <cmath>     // For fabs
#include <iostream>  // For cerr (if needed)

#include "../utils/Helpers.h" 

using namespace std;

namespace RegretHeuristics {

// CH_NN_PATH_INSERT_ANYWHERE_REGRET2
vector<int> nn_path_insert_anywhere_regret2(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);
    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        int m = (int)path.size();

        long long best_reg = LLONG_MIN;    // MAXIMIZE regret
        long long best1_tie = LLONG_MAX; // tie-break: prefer smaller best1
        int pick_j = -1, pick_pos = -1;

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<pair<long long,int>> ins; // (total_cost, position_index)
            ins.reserve(max(2, m+1));

            if (m == 1) {
                long long tot = (long long)D[j][path[0]] + C[j];
                ins.push_back({tot, 0}); // before 0
                ins.push_back({tot, 1}); // after 0
            } else {
                ins.push_back({(long long)D[j][path[0]] + C[j], 0}); // prepend
                for (int t = 0; t+1 < m; ++t) { // internal
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    ins.push_back({delta + C[j], t+1});
                }
                ins.push_back({(long long)D[path[m-1]][j] + C[j], m}); // append
            }

            sort(ins.begin(), ins.end());
            long long best1 = ins[0].first;
            int pos_best1 = ins[0].second;
            long long best2 = (ins.size() >= 2 ? ins[1].first : ins[0].first);
            long long reg = best2 - best1;

            bool take = false;
            if (reg > best_reg) take = true;
            else if (reg == best_reg) {
                if (best1 < best1_tie) take = true;
                else if (best1 == best1_tie && uniform_int_distribution<int>(0,1)(rng)) take = true;
            }
            if (take) {
                best_reg = reg;
                best1_tie = best1;
                pick_j = j;
                pick_pos = pos_best1;
            }
        }

        path.insert(path.begin() + pick_pos, pick_j);
        used[pick_j] = 1;
    }
    return path; 
}


// CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED
vector<int> nn_path_insert_anywhere_regret2_weighted(const Instance& I, int start, mt19937& rng,
                                                  double w_reg, double w_cost) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);
    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        int m = (int)path.size();

        double best_score = -1e300; // Maximize this score
        long long best1_tie = LLONG_MAX;
        int pick_j = -1, pick_pos = -1;

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<long long> costs;
            vector<int> pos;
            if (m == 1) {
                long long tot = (long long)D[j][path[0]] + C[j];
                costs.push_back(tot); pos.push_back(0); 
                costs.push_back(tot); pos.push_back(1); 
            } else {
                costs.push_back((long long)D[j][path[0]] + C[j]); pos.push_back(0); // prepend
                for (int t = 0; t+1 < m; ++t) { // internal
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    costs.push_back(delta + C[j]); pos.push_back(t+1);
                }
                costs.push_back((long long)D[path[m-1]][j] + C[j]); pos.push_back(m); // append
            }

            // Find best1 and best2 indices
            int b1 = 0, b2 = (int)costs.size()>1 ? 1 : 0;
            if (b2 && costs[b2] < costs[b1]) swap(b1, b2);
            for (int t = 2; t < (int)costs.size(); ++t) {
                if (costs[t] < costs[b1]) { b2 = b1; b1 = t; }
                else if (b2 == b1 || costs[t] < costs[b2]) { b2 = t; }
            }
            long long best1 = costs[b1];
            long long best2 = costs[b2];
            long long regret = best2 - best1;

            double score = w_reg * (double)regret - w_cost * (double)best1;

            if (score > best_score) { 
                best_score = score;
                best1_tie = best1;
                pick_j = j;
                pick_pos = pos[b1];
            } else if (fabs(score - best_score) < 1e-12) { // Tie in score
                if (best1 < best1_tie ||
                    (best1 == best1_tie && uniform_int_distribution<int>(0,1)(rng))) {
                    best1_tie = best1;
                    pick_j = j;
                    pick_pos = pos[b1];
                }
            }
        }

        path.insert(path.begin() + pick_pos, pick_j);
        used[pick_j] = 1;
    }
    return path; 
}


// CH_GREEDY_CYCLE_CHEAPEST_INSERTION
vector<int> greedy_cycle_cheapest_insertion(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;

    vector<char> used(N, 0);
    vector<int> cyc; cyc.reserve(K);
    cyc.push_back(start); used[start] = 1;

    while ((int)cyc.size() < K) {
        long long best_val = LLONG_MAX;
        int best_j = -1, best_insert_after = -1; // insert after cyc[i]

        int m = (int)cyc.size();
        for (int j = 0; j < N; ++j) if (!used[j]) { // for each unused node
            if (m == 1) { // special case: 1 node in cycle
                int s = cyc[0]; 
                long long delta = 2LL * D[s][j]; // s -> j -> s
                long long total = delta + C[j];
                if (total < best_val) { best_val = total; best_j = j; best_insert_after = 0; }
                else if (total == best_val && uniform_int_distribution<int>(0,1)(rng)) {
                    best_j = j; best_insert_after = 0;
                }
            } else { // general case: m >= 2
                for (int i = 0; i < m; ++i) { 
                    int u = cyc[i], v = cyc[(i+1)%m]; // edge u->v
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total = delta + C[j];
                    if (total < best_val) { best_val = total; best_j = j; best_insert_after = i; }
                    else if (total == best_val && uniform_int_distribution<int>(0,1)(rng)) {
                        best_j = j; best_insert_after = i;
                    }
                }
            }
        }
        // insert after position best_insert_after
        cyc.insert(cyc.begin() + (best_insert_after + 1), best_j);
        used[best_j] = 1;
    }
    return cyc;
}

// CH_GREEDY_CYCLE_REGRET2
vector<int> greed_cycle_regret2(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> cyc; cyc.reserve(K);
    cyc.push_back(start); used[start] = 1;

    while ((int)cyc.size() < K) {
        int m = (int)cyc.size();

        long long best_score = LLONG_MIN;      // MAXIMIZE regret
        int best_j = -1, best_edge_i = -1;     // insert between cyc[i] and cyc[(i+1)%m]
        long long best1_for_pick = LLONG_MAX;  // For tie-breaking

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<pair<long long,int>> ins; // (total_cost, edge_index)
            ins.reserve(max(1, m));
            if (m == 1) {
                int s = cyc[0];
                long long total = 2LL * D[s][j] + C[j]; // two edges + node cost
                ins.push_back({total, 0});
            } else {
                for (int i = 0; i < m; ++i) {
                    int u = cyc[i], v = cyc[(i+1)%m];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total = delta + C[j];
                    ins.push_back({total, i});
                }
            }
            
            sort(ins.begin(), ins.end());
            long long best1 = ins[0].first;
            int edge_for_best1 = ins[0].second;
            long long best2 = (ins.size() >= 2 ? ins[1].first : ins[0].first);
            long long regret = best2 - best1; 

            bool take = false;
            if (regret > best_score) take = true;
            else if (regret == best_score) {
                if (best1 < best1_for_pick) take = true;
                else if (best1 == best1_for_pick && uniform_int_distribution<int>(0,1)(rng)) take = true;
            }
            if (take) {
                best_score = regret;
                best_j = j;
                best_edge_i = edge_for_best1;
                best1_for_pick = best1;
            }
        }

        cyc.insert(cyc.begin() + (best_edge_i + 1), best_j);
        used[best_j] = 1;
    }
    return cyc;
}


// CH_GREEDY_CYCLE_REGRET2_WEIGHTED
vector<int> greed_cycle_regret2_weighted(const Instance& I, int start, mt19937& rng,
                                   double w_reg, double w_cost) {

    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> cyc; cyc.reserve(K);
    cyc.push_back(start); used[start] = 1;

    while ((int)cyc.size() < K) {
        int m = (int)cyc.size();

        double best_score = -1e300;         
        long long best1_for_tie = LLONG_MAX;  
        int best_j = -1, best_edge_i = -1;

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<long long> costs;
            vector<int> edges;
            if (m == 1) {
                int s = cyc[0];
                costs.push_back(2LL * D[s][j] + C[j]);
                edges.push_back(0);
            } else {
                costs.reserve(m);
                edges.reserve(m);
                for (int i = 0; i < m; ++i) {
                    int u = cyc[i], v = cyc[(i+1)%m];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    costs.push_back(delta + C[j]);
                    edges.push_back(i);
                }
            }

            // Find best1 and best2 indices
            int b1 = 0, b2 = (int)costs.size()>1 ? 1 : 0;
            if (b2 && costs[b2] < costs[b1]) swap(b1, b2);
            for (int t = 2; t < (int)costs.size(); ++t) {
                if (costs[t] < costs[b1]) { b2 = b1; b1 = t; }
                else if (b2 == b1 || costs[t] < costs[b2]) { b2 = t; }
            }
            long long best1 = costs[b1];
            long long best2 = costs[b2];
            long long regret = best2 - best1;

            double score = w_reg * (double)regret - w_cost * (double)best1;

            if (score > best_score) {
                best_score = score;
                best_j = j;
                best_edge_i = edges[b1];
                best1_for_tie = best1;
            } else if (fabs(score - best_score) < 1e-12) {
                if (best1 < best1_for_tie ||
                    (best1 == best1_for_tie && uniform_int_distribution<int>(0,1)(rng))) {
                    best_j = j;
                    best_edge_i = edges[b1];
                    best1_for_tie = best1;
                }
            }
        }

        cyc.insert(cyc.begin() + (best_edge_i + 1), best_j);
        used[best_j] = 1;
    }
    return cyc;
}

} // namespace RegretHeuristics
