#define EZC3D_API_EXPORTS
///
/// \file RotationsSubframe.cpp
/// \brief Implementation of Subframe class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/RotationsSubframe.h"
#include "ezc3d/Header.h"
#include "ezc3d/RotationsInfo.h"
#include <iostream>
#ifdef _WIN32
#include <string>
#endif
#include <stdexcept>

ezc3d::DataNS::RotationNS::SubFrame::SubFrame() {}

ezc3d::DataNS::RotationNS::SubFrame::SubFrame(
    ezc3d::c3d &c3d, std::fstream &file,
    const ezc3d::DataNS::RotationNS::Info &info) {
  rotations.reserve(info.used());

  // Read the rotations
  for (size_t i = 0; i < rotations.capacity(); ++i) {
    rotations.emplace_back(c3d, file, info);
  }
}

ezc3d::DataNS::RotationNS::SubFrame
ezc3d::DataNS::RotationNS::SubFrame::clone() const {
  SubFrame copy;
  for (const auto &rotation : rotations)
    copy.rotations.push_back(rotation.clone());
  return copy;
}

void ezc3d::DataNS::RotationNS::SubFrame::print() const {
  for (size_t j = 0; j < rotations.size(); ++j) {
    std::cout << "Rotation: " << j << "\n";
    rotations.at(j).print();
  }
}

void ezc3d::DataNS::RotationNS::SubFrame::write(std::fstream &f) const {
  for (const auto &rotation : rotations)
    rotation.write(f);
}

bool ezc3d::DataNS::RotationNS::SubFrame::isEmpty() const {
  for (const auto &rotation : rotations)
    if (!rotation.isEmpty())
      return false;
  return true;
}
