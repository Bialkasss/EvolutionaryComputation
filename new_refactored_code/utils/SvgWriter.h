#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>      // For setprecision
#include <algorithm>    // For min_element, max_element
#include <unordered_set>
#include <cmath>        // For sqrt, round
#include <climits>      // For INT_MAX, INT_MIN
#include <tuple>        // For std::make_tuple

#include "../core/Instance.h" // Needs Instance data
// Note: core/Common.h is included via core/Instance.h

using namespace std;

// Helper for SVG (currently a no-op, but good to keep)
static inline string esc(const string& s){ return s; }

/**
 * @brief Saves a tour as an SVG visualization.
 * @param I The problem instance (for coordinates and costs).
 * @param tour The solution tour vector.
 * @param out_path The file path to save to.
 */
inline void save_svg(const Instance& I, const vector<int>& tour, const string& out_path) {
    if (tour.empty()) return;
    int W = 1000, H = 1000, margin = 40;

    // Find bounding box
    int minx = INT_MAX, maxx = INT_MIN, miny = INT_MAX, maxy = INT_MIN;
    for (auto&p: I.pts) { minx=min(minx,p.x); maxx=max(maxx,p.x); miny=min(miny,p.y); maxy=max(maxy,p.y); } 
    
    // Calculate scaling
    double sx = (maxx==minx) ? 1.0 : (double)(W-2*margin)/(maxx-minx);
    double sy = (maxy==miny) ? 1.0 : (double)(H-2*margin)/(maxy-miny);

    // Scaling functions
    auto X = [&](int x){ return margin + (x - minx) * sx; };
    auto Y = [&](int y){ return margin + (y - miny) * sy; };

    // Node radius scaling
    int minC = *min_element(I.cost.begin(), I.cost.end());
    int maxC = *max_element(I.cost.begin(), I.cost.end());
    auto rad = [&](int c){
        if (maxC==minC) return 6.0;
        double t = (double)(c - minC) / (maxC - minC);
        return 4.0 + 10.0 * t; // 4..14 px
    };
    
    // Node color scaling (Green to Red)
    auto green = [&](int c){
        if (maxC == minC) return make_tuple(0, 200, 0);
        double t = (double)(c - minC) / (maxC - minC);
        int r = (int)(255 * t);
        int g = (int)(255 * (1 - t));
        int b = 50;
        return make_tuple(r, g, b);
    };

    // --- Write SVG File ---
    ofstream out(out_path);
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='"<<W<<"' height='"<<H<<"'>\n"; 
    out << "<rect x='0' y='0' width='100%' height='100%' fill='white'/>\n";

    // Drawing tour edges
    out << "<g stroke='black' stroke-width='1' fill='none'>\n";
    for (int i = 0; i < (int)tour.size(); ++i) {
        int u = tour[i], v = tour[(i+1)%tour.size()];
        out << "<line x1='"<<X(I.pts[u].x)<<"' y1='"<<Y(I.pts[u].y)
            <<"' x2='"<<X(I.pts[v].x)<<"' y2='"<<Y(I.pts[v].y)<<"'/>\n";
    }
    out << "</g>\n";

    // Drawing nodes
    out << "<g>\n";
    unordered_set<int> sel(tour.begin(), tour.end());
    for (int i = 0; i < I.N; ++i) {
        double cx = X(I.pts[i].x), cy = Y(I.pts[i].y);
        auto [r, g, b] = green(I.cost[i]);
        if (sel.count(i)) {
            // Draw selected node
            out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='"<<rad(I.cost[i])
                <<"' fill='rgb("<<r<<","<<g<<","<<b<<")' stroke='black' stroke-width='1'/>\n";
            // Draw label for selected node
            out << "<text x='"<<cx+6<<"' y='"<<cy-6<<"' font-size='10' fill='black'>"<<i<<"</text>\n";
        } else {
            // Draw unselected node
            out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='3' fill='none' stroke='black' />\n";  
        }
    }
    out << "</g>\n";
    out << "</svg>\n";
}
