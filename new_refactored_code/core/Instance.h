#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include "Common.h"

using namespace std;

class Instance {
public:
    string name;
    vector<Point> pts;
    vector<int> cost;
    vector<vector<int>> D; // Distance matrix
    int N = 0;
    int K = 0;

    // Replaces the global read_instance function
    bool loadFromFile(const string& path); 

private:
    // These functions are now private helpers for loadFromFile
    static vector<string> split_tokens(const string& line);
    static inline int iround(double v);
};