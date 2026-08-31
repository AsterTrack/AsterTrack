#define EZC3D_API_EXPORTS
///
/// \file Points.cpp
/// \brief Implementation of Points class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Points.h"
#include "ezc3d/Header.h"
#include "ezc3d/ezc3d.h"
#include <stdexcept>

// Point3d data
ezc3d::DataNS::Points3dNS::Points::Points() {}

ezc3d::DataNS::Points3dNS::Points::Points(
    ezc3d::c3d &c3d, std::fstream &file,
    const ezc3d::DataNS::Points3dNS::Info &info) {
  points.reserve(c3d.header().nb3dPoints());
  for (size_t i = 0; i < c3d.header().nb3dPoints(); ++i) {
    points.emplace_back(c3d, file, info, i);
  }
}

ezc3d::DataNS::Points3dNS::Points
ezc3d::DataNS::Points3dNS::Points::clone() const {
  Points copy;
  for (const auto &point : points)
    copy.points.push_back(point.clone());
  return copy;
}

void ezc3d::DataNS::Points3dNS::Points::print() const {
  for (size_t i = 0; i < points.size(); ++i)
    points[i].print();
}

void ezc3d::DataNS::Points3dNS::Points::write(
    std::fstream &f, const ezc3d::DataNS::Points3dNS::Info &pointsInfo) const {
  for (size_t i = 0; i < points.size(); ++i)
    points[i].write(f, pointsInfo, i);
}

bool ezc3d::DataNS::Points3dNS::Points::isEmpty() const {
  for (const auto &point : points)
    if (!point.isEmpty())
      return false;
  return true;
}
