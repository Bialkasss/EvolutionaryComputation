#pragma once
#include <vector>
#include <random> // For mt19937, uniform_int_distribution

using namespace std;

/**
 * @brief Finds the index of the minimum value in a vector.
 * Breaks ties randomly.
 */
template<class T>
int argmin_random_tie(const vector<T>& vals, mt19937& rng) {
    T best = vals[0];
    vector<int> idx{0};
    for (int i = 1; i < (int)vals.size(); ++i) {
        if (vals[i] < best) { best = vals[i]; idx.assign(1, i); }
        else if (vals[i] == best) idx.push_back(i);
    }
    uniform_int_distribution<int> dist(0, (int)idx.size()-1);
    return idx[dist(rng)];
}

/**
 * @brief Inserts a value into a vector at a specific position.
 */
template<typename T>
void insert_at(vector<T>& v, int pos, const T& val) {
    v.insert(v.begin()+pos, val);
}
