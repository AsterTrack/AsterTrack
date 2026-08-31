#define EZC3D_API_EXPORTS
///
/// \file Rotations.cpp
/// \brief Implementation of Rotations class
/// \author Pariterre
/// \version 1.0
/// \date April 30th, 2022
///

#include "ezc3d/Rotations.h"
#include "ezc3d/Header.h"
#include "ezc3d/Parameters.h"
#include "ezc3d/RotationsInfo.h"
#include "ezc3d/RotationsSubframe.h"
#include "ezc3d/ezc3d.h"
#include <iostream>
#include <stdexcept>

// Rotations data
ezc3d::DataNS::RotationNS::Rotations::Rotations() {}

ezc3d::DataNS::RotationNS::Rotations::Rotations(
    ezc3d::c3d &c3d, std::fstream &file,
    const ezc3d::DataNS::RotationNS::Info &info) {
  if (!c3d.header().hasRotationalData())
    return;

  subframes.reserve(info.ratio());
  for (size_t k = 0; k < subframes.capacity(); ++k) {
    subframes.emplace_back(c3d, file, info);
  }
}

ezc3d::DataNS::RotationNS::Rotations
ezc3d::DataNS::RotationNS::Rotations::clone() const {
  Rotations copy;
  for (const auto &subframe : subframes)
    copy.subframes.push_back(subframe);
  return copy;
}

void ezc3d::DataNS::RotationNS::Rotations::print() const {
  for (size_t i = 0; i < subframes.size(); ++i) {
    std::cout << "Subframe = " << i << "\n";
    subframes.at(i).print();
    std::cout << "\n";
  }
}

void ezc3d::DataNS::RotationNS::Rotations::write(std::fstream &f) const {
  for (const auto &subframe : subframes) {
    subframe.write(f);
  }
}

bool ezc3d::DataNS::RotationNS::Rotations::isEmpty() const {
  for (const auto &subframe : subframes) {
    if (!subframe.isEmpty()) {
      return false;
    }
  }
  return true;
}
