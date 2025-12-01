#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include "../core/Instance.h"

using namespace std;

// Helper struct to maintain fast lookups for tour state
struct LSHelpers {
    int K, N;
    vector<int>& tour;
    vector<int> pos;
    vector<bool> in_tour;

    LSHelpers(int n, int k, vector<int>& t) : N(n), K(k), tour(t), pos(n, -1), in_tour(n, false) {
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

// --- DELTA FUNCTIONS (Inline to prevent linker errors) ---

inline long long delta_inter_exchange(int u, int v, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D; const auto& C = I.cost;
    int K = h.K;
    int i = h.pos[u];
    
    int prev_u = h.tour[(i - 1 + K) % K];
    int next_u = h.tour[(i + 1) % K];

    long long cost_delta = (long long)C[v] - C[u];
    long long edge_delta = (long long)D[prev_u][v] + D[v][next_u] 
                         - D[prev_u][u] - D[u][next_u];

    return cost_delta + edge_delta;
} 

inline long long delta_intra_exchange_nodes(int i, int j, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (i == j) return 0;
    if (j < i) swap(i, j); 

    int u = h.tour[i];
    int v = h.tour[j];

    int prev_i = h.tour[(i - 1 + K) % K];
    int next_i = h.tour[(i + 1) % K];
    int prev_j = h.tour[(j - 1 + K) % K];
    int next_j = h.tour[(j + 1) % K];

    long long delta = 0;

    if (j == i + 1) { // Adjacent
        delta = (long long)D[prev_i][v] + D[v][u] + D[u][next_j]
              - D[prev_i][u] - D[u][v] - D[v][next_j];
    } else if (i == 0 && j == K - 1) { // Adjacent wrap-around
        delta = (long long)D[prev_j][u] + D[u][v] + D[v][next_i]
              - D[prev_j][v] - D[v][u] - D[u][next_i];
    }
    else { 
        delta = (long long)D[prev_i][v] + D[v][next_i] + D[prev_j][u] + D[u][next_j]
              - D[prev_i][u] - D[u][next_i] - D[prev_j][v] - D[v][next_j];
    }
    return delta;
}

inline long long delta_intra_exchange_edges_2opt(int i, int j, const LSHelpers& h, const Instance& I) {
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

// --- APPLY FUNCTIONS ---

inline void apply_inter_exchange(LSHelpers& h, int u, int v) {
    int i = h.pos[u];
    h.tour[i] = v;
    h.pos[u] = -1;
    h.in_tour[u] = false;
    h.pos[v] = i;
    h.in_tour[v] = true;
}

inline void apply_intra_exchange_nodes(LSHelpers& h, int i, int j) {
    int u = h.tour[i];
    int v = h.tour[j];
    swap(h.tour[i], h.tour[j]);
    h.pos[u] = j;
    h.pos[v] = i;
}

inline void apply_intra_exchange_edges_2opt(LSHelpers& h, int i, int j) {
    int K = h.K;
    if (j < i) swap(i, j);
    int start = (i + 1) % K;
    int end = j;
    while (start != end) {
        int u = h.tour[start];
        int v = h.tour[end];
        swap(h.tour[start], h.tour[end]);
        h.pos[u] = end;
        h.pos[v] = start;
        start = (start + 1) % K;
        if (start == end) break;
        end = (end - 1 + K) % K;
    }
}