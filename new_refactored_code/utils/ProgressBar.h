#pragma once
#include <iostream>
#include <string>

using namespace std;

/**
 * @brief Simple in-terminal progress bar.
 * @param progress A value between 0.0 and 1.0.
 * @param label A label to print next to the bar.
 */
inline void print_progress(double progress, const string& label) {
    int bar_width = 40; // width of the bar
    // Ensure progress is capped at 1.0
    if (progress > 1.0) progress = 1.0;
    if (progress < 0.0) progress = 0.0;
    int pos = (int)(bar_width * progress);
    
    // Print the bar
    cerr << "  [";
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) cerr << "=";
        else if (i == pos) cerr << ">";
        else cerr << " ";
    }
    cerr << "] " << (int)(progress * 100.0 + 0.5) << "% "
         << label << "                \r"; // Add padding and \r
    cerr.flush();
}
