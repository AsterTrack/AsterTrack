#define EZC3D_API_EXPORTS
///
/// \file Analogs.cpp
/// \brief Implementation of Analogs class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Analogs.h"
#include "ezc3d/Header.h"
#include "ezc3d/ezc3d.h"
#include <iostream>
#include <stdexcept>

ezc3d::DataNS::AnalogsNS::Analogs::Analogs() {}

ezc3d::DataNS::AnalogsNS::Analogs::Analogs(ezc3d::c3d &c3d, std::fstream &file,
                                           const AnalogsNS::Info &info) {
  subframes.reserve(c3d.header().nbAnalogByFrame());
  for (size_t k = 0; k < c3d.header().nbAnalogByFrame(); ++k) {
    subframes.emplace_back(c3d, file, info);
  }
}

ezc3d::DataNS::AnalogsNS::Analogs
ezc3d::DataNS::AnalogsNS::Analogs::clone() const {
  Analogs copy;
  for (const auto &subframe : subframes)
    copy.subframes.push_back(subframe.clone());
  return copy;
}

void ezc3d::DataNS::AnalogsNS::Analogs::print() const {
  for (size_t i = 0; i < subframes.size(); ++i) {
    std::cout << "Subframe = " << i << "\n";
    subframes[i].print();
    std::cout << "\n";
  }
}

void ezc3d::DataNS::AnalogsNS::Analogs::write(
    std::fstream &f, const ezc3d::DataNS::AnalogsNS::Info &analogsInfo) const {
  for (const auto &subframe : subframes)
    subframe.write(f, analogsInfo);
}

bool ezc3d::DataNS::AnalogsNS::Analogs::isEmpty() const {
  for (const auto &subframe : subframes) {
    if (!subframe.isEmpty()) {
      return false;
    }
  }
  return true;
}
