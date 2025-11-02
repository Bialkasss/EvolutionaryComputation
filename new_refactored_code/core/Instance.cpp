#pragma once

#include <vector>
#include <string>
#include <filesystem> // For filesystem::path
#include <cmath>        // For llround, sqrt
#include <fstream>      // For ifstream
#include <iostream>     // For cerr
#include <cctype>       // For isspace

#include "Common.h" // For Point struct

using namespace std;

/**
 * @brief Holds all data for a single problem instance (nodes, costs, distances).
 * Responsible for loading itself from a file.
 */
class Instance {
public:
    string name;
    vector<Point> pts;         // Coordinates (for SVG)
    vector<int> cost;          // Node costs
    vector<vector<int>> D;     // Distance matrix
    int N = 0;                 // Total nodes
    int K = 0;                 // Nodes to select (ceil(N/2))

    /**
     * @brief Loads instance data from the specified file path.
     * This was formerly the 'read_instance' function.
     * @return true on success, false on failure.
     */
    bool loadFromFile(const string& path); 

private:
    /**
     * @brief (Copied from main.cpp) Helper for parsing lines.
     */
    static vector<string> split_tokens(const string& line);

    /**
     * @brief (Copied from main.cpp) Helper for rounding distances.
     */
    static inline int iround(double v);
};
