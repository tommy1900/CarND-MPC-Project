#ifndef PLT_H
#define PLT_H

#include <fstream>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void DrawThePath() {
  std::string file = "../lake_track_waypoints.csv";
  std::ifstream sin(file);
  std::string line;
  // Discard the first line
  std::getline(sin, line);
  vector<double> wpx;
  vector<double> wpy;
  while (std::getline(sin, line)) {
    wpx.push_back(std::stod(line.substr(0, line.find(',') - 1)));
    wpy.push_back(std::stod(line.substr(line.find(',') + 1)));
    //if (i++ % 10 == 0) {
    //  temp.x.push_back(waypoints.x.back());
    //  temp.y.push_back(waypoints.y.back());
    //}
  }
  plt::named_plot("The Map", wpx, wpy, "b--");
 }

#endif
