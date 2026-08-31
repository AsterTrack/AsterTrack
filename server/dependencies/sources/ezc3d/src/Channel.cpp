#define EZC3D_API_EXPORTS
///
/// \file Channel.cpp
/// \brief Implementation of Channel class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Channel.h"
#include "ezc3d/AnalogsInfo.h"
#include "ezc3d/Header.h"
#include "ezc3d/ezc3d.h"
#include <iostream>

ezc3d::DataNS::AnalogsNS::Channel::Channel() {}

ezc3d::DataNS::AnalogsNS::Channel::Channel(
    const ezc3d::DataNS::AnalogsNS::Channel &channel)
    : data(channel.data) {}

ezc3d::DataNS::AnalogsNS::Channel::Channel(
    ezc3d::c3d &c3d, std::fstream &file,
    const ezc3d::DataNS::AnalogsNS::Info &info, size_t channelIndex) {
  if (c3d.header().scaleFactor() < 0) // if it is float
    data = (c3d.readFloat(info.processorType(), file) -
          info.zeroOffset()[channelIndex]) *
         info.scaleFactors()[channelIndex] * info.generalFactor();
  else
    data = (static_cast<float>(
              c3d.readInt(info.processorType(), file, ezc3d::DATA_TYPE::WORD)) -
          info.zeroOffset()[channelIndex]) *
         info.scaleFactors()[channelIndex] * info.generalFactor();
}

ezc3d::DataNS::AnalogsNS::Channel
ezc3d::DataNS::AnalogsNS::Channel::clone() const {
  return Channel(*this);
}

void ezc3d::DataNS::AnalogsNS::Channel::print() const {
  std::cout << "Analog = " << data << "\n";
}

void ezc3d::DataNS::AnalogsNS::Channel::write(
    std::fstream &f, const ezc3d::DataNS::AnalogsNS::Info &analogsInfo,
    size_t channelIndex) const {

  double scaleFactor = analogsInfo.scaleFactors().size() < channelIndex + 1
                           ? analogsInfo.scaleFactors()[0]
                           : analogsInfo.scaleFactors()[channelIndex];
  double generalFactor = analogsInfo.generalFactor();
  double zeroOffset = analogsInfo.zeroOffset()[channelIndex];

  float out(
      static_cast<float>(((data / generalFactor) / scaleFactor) + zeroOffset));
  f.write(reinterpret_cast<const char *>(&out), ezc3d::DATA_TYPE::FLOAT);
}

bool ezc3d::DataNS::AnalogsNS::Channel::isEmpty() const {
  if (static_cast<double>(data) == 0.0) {
    return true;
  } else {
    return false;
  }
}
