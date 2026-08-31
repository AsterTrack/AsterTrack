#define EZC3D_API_EXPORTS
///
/// \file AnalogsSubframe.cpp
/// \brief Implementation of Subframe class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/AnalogsSubframe.h"
#include "ezc3d/Header.h"
#include "ezc3d/ezc3d.h"
#include <stdexcept>

ezc3d::DataNS::AnalogsNS::SubFrame::SubFrame() {}

ezc3d::DataNS::AnalogsNS::SubFrame::SubFrame(
    ezc3d::c3d &c3d, std::fstream &file,
    const ezc3d::DataNS::AnalogsNS::Info &info) {
  channels.reserve(c3d.header().nbAnalogs());
  for (size_t i = 0; i < channels.capacity(); ++i) {
    channels.emplace_back(c3d, file, info, i);
  }
}

ezc3d::DataNS::AnalogsNS::SubFrame
ezc3d::DataNS::AnalogsNS::SubFrame::clone() const {
  SubFrame copy;
  for (const auto &channel : channels)
    copy.channels.push_back(channel.clone());
  return copy;
}

void ezc3d::DataNS::AnalogsNS::SubFrame::print() const {
  for (const auto &channel : channels)
    channel.print();
}

void ezc3d::DataNS::AnalogsNS::SubFrame::write(
    std::fstream &f, const ezc3d::DataNS::AnalogsNS::Info &analogsInfo) const {
  for (size_t i = 0; i < channels.size(); ++i)
    channels[i].write(f, analogsInfo, i);
}

bool ezc3d::DataNS::AnalogsNS::SubFrame::isEmpty() const {
  for (const auto &channel : channels)
    if (!channel.isEmpty())
      return false;
  return true;
}
