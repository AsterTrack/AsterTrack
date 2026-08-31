#define EZC3D_API_EXPORTS
///
/// \file ezc3d.cpp
/// \brief Implementation of ezc3d class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/ezc3d.h"
#include "ezc3d/AnalogsInfo.h"
#include "ezc3d/Data.h"
#include "ezc3d/DataStartInfo.h"
#include "ezc3d/Header.h"
#include "ezc3d/Options.h"
#include "ezc3d/Parameters.h"
#include "ezc3d/PointsInfo.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

void ezc3d::removeTrailingSpaces(std::string &s) {
  // Remove the spaces at the end of the strings
  for (int i = static_cast<int>(s.size()); i >= 0; --i)
    if (s.size() > 0 && s[s.size() - 1] == ' ')
      s.pop_back();
    else
      break;
}

std::string ezc3d::toUpper(const std::string &str) {
  std::string new_str = str;
  std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::toupper);
  return new_str;
}

ezc3d::c3d::c3d()
    : options(Options()), _filePath(""),
      m_nByteToRead_float(4 * ezc3d::DATA_TYPE::BYTE),
      m_nByteToReadMax_int(100) {
  c_float = std::vector<char>(m_nByteToRead_float + 1);
  c_float_tp = std::vector<char>(m_nByteToRead_float + 1);
  c_int = std::vector<char>(m_nByteToReadMax_int + 1);
  c_int_tp = std::vector<char>(m_nByteToReadMax_int + 1);

  _header = std::make_shared<ezc3d::Header>();
  _parameters = std::make_shared<ezc3d::ParametersNS::Parameters>();
  data = std::make_shared<ezc3d::DataNS::Data>();
}

ezc3d::c3d::c3d(const std::string &filePath, const Options &options)
    : options(options), _filePath(filePath),
      m_nByteToRead_float(4 * ezc3d::DATA_TYPE::BYTE),
      m_nByteToReadMax_int(100) {
  std::fstream stream(_filePath, std::ios::in | std::ios::binary);
  c_float = std::vector<char>(m_nByteToRead_float + 1);
  c_float_tp = std::vector<char>(m_nByteToRead_float + 1);
  c_int = std::vector<char>(m_nByteToReadMax_int + 1);
  c_int_tp = std::vector<char>(m_nByteToReadMax_int + 1);

  if (!stream.is_open())
    throw std::ios_base::failure(
        "The c3d file could not be opened, please verify the path");

  // Read all the section
  _header = std::make_shared<ezc3d::Header>(*this, stream);
  _parameters = std::make_shared<ezc3d::ParametersNS::Parameters>(*this, stream);

  // header may be inconsistent with the parameters, so it must be
  // update to make sure sizes are consistent
  updateHeader();

  // Now read the data
  data = std::make_shared<ezc3d::DataNS::Data>(*this, stream);

  // Parameters and header may be inconsistent with data,
  // so reprocess them if needed
  fixupParameters();

  // Close the file
  stream.close();
}

ezc3d::c3d ezc3d::c3d::clone() const {
  c3d copy;
  copy._filePath = _filePath;
  copy.options = options.clone();
  copy._header = std::make_shared<ezc3d::Header>(_header->clone());
  copy._parameters = std::make_shared<ezc3d::ParametersNS::Parameters>(_parameters->clone());
  copy.data = std::make_shared<ezc3d::DataNS::Data>(data->clone());
  return copy;
}

void ezc3d::c3d::print() const {
  header().print();
  parameters().print();
  data->print();
}

void ezc3d::c3d::write(const std::string &filePath,
                       const WriteOptions &writeOptions) const {

  std::fstream f(filePath, std::ios::out | std::ios::binary);

  ezc3d::DataStartInfo dataStartInfoToFill;

  // Write the header
  header().write(writeOptions, f, dataStartInfoToFill);

  // Write the parameters
  ezc3d::ParametersNS::Parameters p(
      parameters().write(writeOptions, f, dataStartInfoToFill, header()));

  // Write the data (Should the scales be taken from p?)
  ezc3d::DataNS::Points3dNS::Info pointsInfo(*this);
  ezc3d::DataNS::AnalogsNS::Info analogsInfo(*this);
  data->write(header(), f, pointsInfo, analogsInfo, dataStartInfoToFill);

  // Go back and write all the required data start
  writeDataStart(f, dataStartInfoToFill);

  f.close();
}

void ezc3d::c3d::resizeCharHolder(unsigned int nByteToRead) {
  m_nByteToReadMax_int = nByteToRead;
  c_int = std::vector<char>(m_nByteToReadMax_int + 1);
  c_int_tp = std::vector<char>(m_nByteToReadMax_int + 1);
}

void ezc3d::c3d::readFile(std::fstream &file, unsigned int nByteToRead,
                          std::vector<char> &c, int nByteFromPrevious,
                          const std::ios_base::seekdir &pos) {
  if (pos != 1)
    file.seekg(nByteFromPrevious, pos); // Move to number analogs
  file.read(&c[0], nByteToRead);
  c[nByteToRead] = '\0'; // Make sure last char is NULL
}

unsigned int ezc3d::c3d::hex2uint(const std::vector<char> &val,
                                  unsigned int len) {
  unsigned int ret(0);
  // Discard any extra bytes to avoid overflow of int
  unsigned int max_bytes = std::min(len, 4u);
  for (unsigned int i = 0; i < max_bytes; ++i)
    ret |= static_cast<unsigned int>(static_cast<unsigned char>(val[i]))
           << (8 * i);
  return ret;
}

int ezc3d::c3d::hex2int(const std::vector<char> &val, unsigned int len) {
  unsigned int tp(hex2uint(val, len));

  // convert to signed int
  // Find max int value
  // Discard any extra bytes to avoid overflow of int
  unsigned int max(0);
  unsigned int max_bytes = std::min(len, 4u);
  for (unsigned int i = 0; i < max_bytes; ++i)
    max |= 0xFFu << (8 * i);

  // If the value is over uint_max / 2 then it is a negative number
  int out;
  if (tp > max / 2)
    out = static_cast<int>(tp - max - 1);
  else
    out = static_cast<int>(tp);

  return out;
}

void ezc3d::c3d::writeDataStart(
    std::fstream &f, const ezc3d::DataStartInfo &dataStartPosition) const {

  if (dataStartPosition.hasHeaderPointDataStart()) {
    f.seekg(dataStartPosition.headerPointDataStart());
    // DATA_START is 1-based
    int nBlocksToNext = int(dataStartPosition.pointDataStart()) / 512 + 1;
    f.write(reinterpret_cast<const char *>(&nBlocksToNext),
            dataStartPosition.headerPointDataStartSize());
  }

  if (dataStartPosition.hasParameterPointDataStart()) {
    f.seekg(dataStartPosition.parameterPointDataStart());
    // DATA_START is 1-based
    int nBlocksToNext = int(dataStartPosition.pointDataStart()) / 512 + 1;
    f.write(reinterpret_cast<const char *>(&nBlocksToNext),
            dataStartPosition.parameterPointDataStartSize());
  }

  if (dataStartPosition.hasParameterRotationsDataStart()) {
    f.seekg(dataStartPosition.parameterRotationsDataStart());
    // DATA_START is 1-based
    int nBlocksToNext = int(dataStartPosition.rotationsDataStart()) / 512 + 1;
    f.write(reinterpret_cast<const char *>(&nBlocksToNext),
            dataStartPosition.parameterRotationsDataStartSize());
  }
}

int ezc3d::c3d::readInt(PROCESSOR_TYPE processorType, std::fstream &file,
                        unsigned int nByteToRead, int nByteFromPrevious,
                        const std::ios_base::seekdir &pos) {
  if (nByteToRead > m_nByteToReadMax_int)
    resizeCharHolder(nByteToRead);

  readFile(file, nByteToRead, c_int, nByteFromPrevious, pos);

  int out;
  if (processorType == PROCESSOR_TYPE::MIPS) {
    // This is more or less good. Sometimes, it should not reverse...
    for (size_t i = 0; i < nByteToRead; ++i) {
      c_int_tp[i] = c_int[nByteToRead - 1 - i];
    }
    c_int_tp[nByteToRead] = '\0';
    out = hex2int(c_int_tp, nByteToRead);
  } else {
    // make sure it is an int and not an unsigned int
    out = hex2int(c_int, nByteToRead);
  }

  return out;
}

size_t ezc3d::c3d::readUint(PROCESSOR_TYPE processorType, std::fstream &file,
                            unsigned int nByteToRead, int nByteFromPrevious,
                            const std::ios_base::seekdir &pos) {
  if (nByteToRead > m_nByteToReadMax_int)
    resizeCharHolder(nByteToRead);

  readFile(file, nByteToRead, c_int, nByteFromPrevious, pos);

  size_t out;
  if (processorType == PROCESSOR_TYPE::MIPS) {
    // This is more or less good. Sometimes, it should not reverse...
    for (size_t i = 0; i < nByteToRead; ++i) {
      c_int_tp[i] = c_int[nByteToRead - 1 - i];
    }
    c_int_tp[nByteToRead] = '\0';
    // make sure it is an int and not an unsigned int
    out = hex2uint(c_int_tp, nByteToRead);
  } else {
    // make sure it is an int and not an unsigned int
    out = hex2uint(c_int, nByteToRead);
  }

  return out;
}

float ezc3d::c3d::readFloat(PROCESSOR_TYPE processorType, std::fstream &file,
                            int nByteFromPrevious,
                            const std::ios_base::seekdir &pos) {
  readFile(file, m_nByteToRead_float, c_float, nByteFromPrevious, pos);
  float out;
  if (processorType == PROCESSOR_TYPE::INTEL) {
    out = *reinterpret_cast<float *>(&c_float[0]);
  } else if (processorType == PROCESSOR_TYPE::DEC) {
    c_float_tp[0] = c_float[2];
    c_float_tp[1] = c_float[3];
    c_float_tp[2] = c_float[0];
    if (c_float[1] != 0)
      c_float_tp[3] = c_float[1] - 1;
    else
      c_float_tp[3] = c_float[1];
    c_float_tp[4] = '\0';
    out = *reinterpret_cast<float *>(&c_float_tp[0]);
  } else if (processorType == PROCESSOR_TYPE::MIPS) {
    for (unsigned int i = 0; i < m_nByteToRead_float; ++i)
      c_float_tp[i] = c_float[m_nByteToRead_float - 1 - i];
    c_float_tp[m_nByteToRead_float] = '\0';
    out = *reinterpret_cast<float *>(&c_float_tp[0]);
  } else {
    throw std::runtime_error("Wrong type of processor for floating points");
  }
  return out;
}

std::string ezc3d::c3d::readString(std::fstream &file, unsigned int nByteToRead,
                                   int nByteFromPrevious,
                                   const std::ios_base::seekdir &pos) {
  if (nByteToRead > m_nByteToReadMax_int)
    resizeCharHolder(nByteToRead);

  std::vector<char> c = std::vector<char>(nByteToRead + 1);
  readFile(file, nByteToRead, c, nByteFromPrevious, pos);
  std::string out(&c[0]);
  return out;
}

void ezc3d::c3d::readParam(PROCESSOR_TYPE processorType, std::fstream &file,
                           unsigned int dataLenghtInBytes,
                           const std::vector<size_t> &dimension,
                           std::vector<int> &param_data, size_t currentIdx) {
  for (size_t i = 0; i < dimension[currentIdx]; ++i)
    if (currentIdx == dimension.size() - 1)
      param_data.push_back(readInt(processorType, file,
                                   dataLenghtInBytes * ezc3d::DATA_TYPE::BYTE));
    else
      readParam(processorType, file, dataLenghtInBytes, dimension, param_data,
                currentIdx + 1);
}

void ezc3d::c3d::readParam(PROCESSOR_TYPE processorType, std::fstream &file,
                           const std::vector<size_t> &dimension,
                           std::vector<double> &param_data, size_t currentIdx) {
  for (size_t i = 0; i < dimension[currentIdx]; ++i)
    if (currentIdx == dimension.size() - 1)
      param_data.push_back(readFloat(processorType, file));
    else
      readParam(processorType, file, dimension, param_data, currentIdx + 1);
}

void ezc3d::c3d::readParam(std::fstream &file,
                           const std::vector<size_t> &dimension,
                           std::vector<std::string> &param_data_string) {
  std::vector<std::string> param_data_string_tp;
  _readMatrix(file, dimension, param_data_string_tp);

  // Vicon c3d stores text length on first dimension, I am not sure if
  // this is a standard or a custom made stuff.
  // I implemented it like that for now
  if (dimension.size() == 1) {
    if (dimension[0] != 0) {
      std::string tp;
      for (size_t j = 0; j < dimension[0]; ++j) {
        tp += param_data_string_tp[j];
      }
      if (!options.getKeepParametersTrailingSpaces())
        ezc3d::removeTrailingSpaces(tp);
      param_data_string.push_back(tp);
    }
  } else
    _dispatchMatrix(dimension, param_data_string_tp, param_data_string);
}

void ezc3d::c3d::moveCursorToANewBlock(std::fstream &f) {
  // Move the cursor to the beginning of a block as rotations should start at a
  // new block
  int blankValue(0);
  std::streampos currentPos(f.tellg());
  for (int i = 0; i < 512 - static_cast<int>(currentPos) % 512; ++i) {
    f.write(reinterpret_cast<const char *>(&blankValue), ezc3d::BYTE);
  }
}

size_t
ezc3d::c3d::_dispatchMatrix(const std::vector<size_t> &dimension,
                            const std::vector<std::string> &param_data_in,
                            std::vector<std::string> &param_data_out,
                            size_t idxInParam, size_t currentIdx) {
  for (size_t i = 0; i < dimension[currentIdx]; ++i)
    if (currentIdx == dimension.size() - 1) {
      std::string tp;
      for (size_t j = 0; j < dimension[0]; ++j) {
        tp += param_data_in[idxInParam];
        ++idxInParam;
      }
      if (!options.getKeepParametersTrailingSpaces())
        ezc3d::removeTrailingSpaces(tp);
      param_data_out.push_back(tp);
    } else
      idxInParam = _dispatchMatrix(dimension, param_data_in, param_data_out,
                                   idxInParam, currentIdx + 1);
  return idxInParam;
}

void ezc3d::c3d::_readMatrix(std::fstream &file,
                             const std::vector<size_t> &dimension,
                             std::vector<std::string> &param_data,
                             size_t currentIdx) {
  for (size_t i = 0; i < dimension[currentIdx]; ++i)
    if (currentIdx == dimension.size() - 1)
      param_data.push_back(readString(file, ezc3d::DATA_TYPE::BYTE));
    else
      _readMatrix(file, dimension, param_data, currentIdx + 1);
}

const ezc3d::Header &ezc3d::c3d::header() const { return *_header; }

const ezc3d::ParametersNS::Parameters &ezc3d::c3d::parameters() const {
  return *_parameters;
}

ezc3d::ParametersNS::Parameters &ezc3d::c3d::parameters() {
  return *_parameters;
}

std::vector<ezc3d::DataNS::Frame> &ezc3d::c3d::frames() const {
  return data->frames;
}

const std::vector<std::string> ezc3d::c3d::pointNames() const {
  std::vector<std::string> labels =
      parameters().group("POINT").parameter("LABELS").valuesAsString();
  int i = 2;
  while (
      parameters().group("POINT").isParameter("LABELS" + std::to_string(i))) {
    const std::vector<std::string> &labels_tp =
        parameters()
            .group("POINT")
            .parameter("LABELS" + std::to_string(i))
            .valuesAsString();
    labels.insert(labels.end(), labels_tp.begin(), labels_tp.end());
    ++i;
  }
  return labels;
}

size_t ezc3d::c3d::pointIdx(const std::string &pointName) const {
  const std::vector<std::string> &currentNames(pointNames());
  for (size_t i = 0; i < currentNames.size(); ++i)
    if (!currentNames[i].compare(pointName))
      return i;
  throw std::invalid_argument("ezc3d::pointIdx could not find " + pointName +
                              " in the points data set.");
}

const std::vector<std::string> ezc3d::c3d::channelNames() const {

  std::vector<std::string> labels =
      parameters().group("ANALOG").parameter("LABELS").valuesAsString();
  int i = 2;
  while (
      parameters().group("ANALOG").isParameter("LABELS" + std::to_string(i))) {
    const std::vector<std::string> &labels_tp =
        parameters()
            .group("ANALOG")
            .parameter("LABELS" + std::to_string(i))
            .valuesAsString();
    labels.insert(labels.end(), labels_tp.begin(), labels_tp.end());
    ++i;
  }
  return labels;
}

size_t ezc3d::c3d::channelIdx(const std::string &channelName) const {
  const std::vector<std::string> &currentNames(channelNames());
  for (size_t i = 0; i < currentNames.size(); ++i)
    if (!currentNames[i].compare(channelName))
      return i;
  throw std::invalid_argument("ezc3d::channelIdx could not find " +
                              channelName + " in the analogous data set");
}

const std::vector<std::string> ezc3d::c3d::poseNames() const {
  std::vector<std::string> labels =
      parameters().group("ROTATION").parameter("LABELS").valuesAsString();
  int i = 2;
  while (
      parameters().group("ROTATION").isParameter("LABELS" + std::to_string(i))) {
    const std::vector<std::string> &labels_tp =
        parameters()
            .group("ROTATION")
            .parameter("LABELS" + std::to_string(i))
            .valuesAsString();
    labels.insert(labels.end(), labels_tp.begin(), labels_tp.end());
    ++i;
  }
  return labels;
}

size_t ezc3d::c3d::poseIdx(const std::string &poseName) const {
  const std::vector<std::string> &currentNames(poseNames());
  for (size_t i = 0; i < currentNames.size(); ++i)
    if (!currentNames[i].compare(poseName))
      return i;
  throw std::invalid_argument("ezc3d::poseIdx could not find " +
                              poseName + " in the rotation data set");
}

void ezc3d::c3d::setFirstFrame(size_t firstFrame) {
  _header->firstFrame(firstFrame);
}

void ezc3d::c3d::setGroupMetadata(const std::string &groupName,
                                  const std::string &description,
                                  bool isLocked) {
  size_t idx;
  try {
    idx = parameters().groupIdx(groupName);
  } catch (const std::invalid_argument &) {
    _parameters->group(ezc3d::ParametersNS::GroupNS::Group(groupName));
    idx = parameters().groupIdx(groupName);
  }

  _parameters->group(idx).description(description);
  if (isLocked) {
    _parameters->group(idx).lock();
  } else {
    _parameters->group(idx).unlock();
  }
}

void ezc3d::c3d::parameter(const std::string &groupName,
                           const ezc3d::ParametersNS::GroupNS::Parameter &p) {
  if (!p.name().compare("")) {
    throw std::invalid_argument("Parameter must have a name");
  }

  size_t idx;
  try {
    idx = parameters().groupIdx(groupName);
  } catch (const std::invalid_argument &) {
    _parameters->group(ezc3d::ParametersNS::GroupNS::Group(groupName));
    idx = parameters().groupIdx(groupName);
  }

  _parameters->group(idx).parameter(p);

  // Do a sanity check on the header if important stuff like number
  // of frames or number of elements is changed
  updateHeader();
}

void ezc3d::c3d::remove(const std::string &groupName,
                        const std::string &parameterName) {
  if (_parameters->isMandatory(groupName, parameterName)) {
    throw std::invalid_argument("You can't remove a mandatory parameter");
  }

  _parameters->group(groupName).remove(parameterName);
}

void ezc3d::c3d::remove(const std::string &groupName) {
  if (_parameters->isMandatory(groupName)) {
    throw std::invalid_argument("You can't remove a mandatory parameter");
  }

  _parameters->remove(groupName);
}

void ezc3d::c3d::lockGroup(const std::string &groupName) {
  _parameters->group(groupName).lock();
}

void ezc3d::c3d::unlockGroup(const std::string &groupName) {
  _parameters->group(groupName).unlock();
}

void ezc3d::c3d::finaliseFrames(bool validate)
{
  auto &grpPoint = parameters().group("POINT");
  size_t nFrames(data->frames.size());
  if (nFrames != grpPoint.parameter("FRAMES").valuesConvertedAsInt()[0]) {
    grpPoint.parameter("FRAMES").set(nFrames);
  }

  if (!validate)
    return;

  auto &grpAnalog = parameters().group("ANALOG");
  size_t nPoints = grpPoint.parameter("USED").valuesAsInt()[0];
  size_t nAnalogs = grpAnalog.parameter("USED").valuesAsInt()[0];
  float pointRate = grpPoint.parameter("RATE").valuesAsDouble()[0];
  float analogRate = grpAnalog.parameter("RATE").valuesAsDouble()[0];
  float poseRate = 0.0f;
  if (parameters().isGroup("ROTATION"))
    poseRate = parameters().group("ROTATION").parameter("RATE").valuesAsDouble()[0];

  for (const auto &f : data->frames)
  {
    // Make sure f.points->points() is the same as data.f[ANY].points()
    if (nPoints != 0 && f.points->points.size() != nPoints) {
      throw std::runtime_error(
          "Number of points in POINT:USED parameter must equal to "
          "the number of points sent in the frame");
    }

    if (f.points->points.size() > 0 && pointRate == 0.0) {
      throw std::runtime_error(
          "Point frame rate must be specified if you add some");
    }
    if (f.analogs->subframes.size() > 0 && analogRate == 0.0) {
      throw std::runtime_error(
          "Analog frame rate must be specified if you add some");
    }
    if (f.rotations->subframes.size() > 0 && poseRate == 0.0) {
      throw std::runtime_error(
          "Analog frame rate must be specified if you add some");
    }

    size_t subSize(f.analogs->subframes.size());
    if (subSize != 0) {
      size_t nChannel(f.analogs->subframes.front().channels.size());
      size_t nAnalogByFrames(header().nbAnalogByFrame());
      if (!(nAnalogs == 0 && nAnalogByFrames == 0) && nChannel != nAnalogs)
        throw std::runtime_error(
            "Number of analogs in ANALOG:USED parameter must equal "
            "the number of analogs sent in the frame");
    }
  }
}

void ezc3d::c3d::addPoint(const std::string &pointName)
{
  std::vector<std::string> names;
  names.push_back(pointName);
  addPoint(names);
}

void ezc3d::c3d::addPoint(const std::vector<std::string> &PointNames)
{
  const std::vector<std::string> &currentNames(pointNames());
  for (size_t idx = 0; idx < PointNames.size(); ++idx) {
    for (size_t i = 0; i < currentNames.size(); ++i)
      if (!PointNames[idx].compare(currentNames[i]))
        throw std::invalid_argument(
            "The point you try to create already exists "
            "in the data set");
  }

  if (data->frames.size() > 0)
  {
    size_t nbPoints = parameters().group("POINT").parameter("USED").valuesAsInt()[0] + PointNames.size();
    for (auto &frame : data->frames) {
      frame.points->points.resize(nbPoints);
    }
  }

  appendParameters(PointNames);
}

void ezc3d::c3d::addAnalog(const std::string &channelName) {
  std::vector<std::string> names;
  names.push_back(channelName);
  addAnalog(names);
}

void ezc3d::c3d::addAnalog(const std::vector<std::string> &ChannelNames)
{
  const std::vector<std::string> &currentNames(channelNames());
  for (size_t idx = 0; idx < ChannelNames.size(); ++idx) {
    for (size_t i = 0; i < currentNames.size(); ++i)
      if (!ChannelNames[idx].compare(currentNames[i]))
        throw std::invalid_argument("The channel you try to create already "
                                    "exists in the data set");
  }

  if (data->frames.size() > 0)
  {
    size_t nbChannels = parameters().group("ANALOG").parameter("USED").valuesAsInt()[0] + ChannelNames.size();
    for (auto &frame : data->frames) {
      for (auto &sf : frame.analogs->subframes) {
        sf.channels.resize(nbChannels);
      }
    }
  }

  appendParameters({}, ChannelNames);
}

void ezc3d::c3d::addPose(const std::string &poseName) {
  std::vector<std::string> names;
  names.push_back(poseName);
  addPose(names);
}

void ezc3d::c3d::addPose(const std::vector<std::string> &PoseNames)
{
  const std::vector<std::string> &currentNames(poseNames());
  for (size_t idx = 0; idx < PoseNames.size(); ++idx) {
    for (size_t i = 0; i < currentNames.size(); ++i)
      if (!PoseNames[idx].compare(currentNames[i]))
        throw std::invalid_argument("The channel you try to create already "
                                    "exists in the data set");
  }

  if (data->frames.size() > 0)
  {
    size_t nbPoses = parameters().group("ROTATION").parameter("USED").valuesAsInt()[0] + PoseNames.size();
    for (auto &frame : data->frames) {
      for (auto &sf : frame.rotations->subframes) {
        sf.rotations.resize(nbPoses);
      }
    }
  }

  appendParameters({}, {}, PoseNames);
}

void ezc3d::c3d::updateHeader() {
  // Parameter is always consider as the right value.
  const auto &points(parameters().group("POINT"));
  size_t nbFrames(static_cast<size_t>(
      points.parameter("FRAMES").valuesConvertedAsInt()[0]));
  if (nbFrames != 0 && nbFrames != header().nbFrames()) {
    // The nbFrames != 0 is to account for Kistler implementation which does not
    // declare points If there is a discrepancy between them, change the header,
    // while keeping the firstFrame value
    _header->lastFrame(nbFrames + _header->firstFrame() - 1);
  }
  double pointRate(points.parameter("RATE").valuesAsDouble()[0]);
  float buffer(10000); // For decimal truncature
  if (static_cast<int>(pointRate * buffer) !=
      static_cast<int>(header().frameRate() * buffer)) {
    // If there are points but the rate don't match keep the one from header
    if (points.parameter("RATE").valuesAsDouble()[0] == 0.0 &&
        points.parameter("USED").valuesAsInt()[0] != 0) {
      ezc3d::ParametersNS::GroupNS::Parameter rate("RATE");
      rate.set(header().frameRate());
      parameter("POINT", rate);
    } else
      _header->frameRate(static_cast<float>(pointRate));
  }
  if (static_cast<size_t>(points.parameter("USED").valuesAsInt()[0]) !=
      header().nb3dPoints()) {
    _header->nb3dPoints(
        static_cast<size_t>(points.parameter("USED").valuesAsInt()[0]));
  }

  // Compare the subframe with data when possible, otherwise go with the
  // parameters
  const auto &analog(parameters().group("ANALOG"));
  if (data != nullptr && data->frames.size() > 0 &&
      data->frames.front().analogs->subframes.size() != 0) {
    if (data->frames.front().analogs->subframes.size() != header().nbAnalogByFrame())
      _header->nbAnalogByFrame(data->frames.front().analogs->subframes.size());
  } else if (static_cast<size_t>(pointRate) != 0 &&
             static_cast<size_t>(analog.parameter("RATE").valuesAsDouble()[0] /
                                 pointRate) != header().nbAnalogByFrame()) {
    if (header().nbAnalogByFrame() == 1 && parameters().isGroup("SHADOW")) {
      // The SHADOW company is not following the standard so they did not
      // set analog rate ezc3d automatically sets it to zero which results
      // in a discrepancy
      ezc3d::ParametersNS::GroupNS::Parameter &analogNonConst =
          _parameters->group("ANALOG").parameter("RATE");
      analogNonConst.set(static_cast<float>(header().nbAnalogByFrame()));
    } else {
      _header->nbAnalogByFrame(static_cast<size_t>(
          analog.parameter("RATE").valuesAsDouble()[0] / pointRate));
    }
  }

  if (static_cast<size_t>(analog.parameter("USED").valuesAsInt()[0]) !=
      header().nbAnalogs())
    _header->nbAnalogs(
        static_cast<size_t>(analog.parameter("USED").valuesAsInt()[0]));

  if (parameters().isGroup("ROTATION"))
    _header->hasRotationalData(true);
}

void ezc3d::c3d::appendParameters(const std::vector<std::string> &newPoints,
                                  const std::vector<std::string> &newAnalogs,
                                  const std::vector<std::string> &newPoses) {
  // If frames has been added
  auto &grpPoint(_parameters->group("POINT"));
  size_t oldPointUsed(grpPoint.parameter("USED").valuesAsInt()[0]);
  if (!newPoints.empty()) {
    grpPoint.parameter("USED").set(oldPointUsed + newPoints.size());

    std::vector<std::string> newDescriptions(newPoints.size(), "");
    std::vector<std::string> newUnits(newPoints.size(), "mm");

    // Dispatch names in LABELS, LABELS2, etc.
    size_t first_idx = 0, last_idx = 0, i = 0;
    while (last_idx < newPoints.size()) {
      std::string mod = i != 0? std::to_string(i + 1) : "";

      auto &labelsParam = grpPoint.parameter("LABELS" + mod, ezc3d::DATA_TYPE::CHAR);
      auto &descriptionsParam = grpPoint.parameter("DESCRIPTIONS" + mod, ezc3d::DATA_TYPE::CHAR);
      auto &unitsParam = grpPoint.parameter("UNITS" + mod, ezc3d::DATA_TYPE::CHAR);

      auto &labels = labelsParam.valuesAsString();
      auto &descriptions = descriptionsParam.valuesAsString();
      auto &units = unitsParam.valuesAsString();

      if (labels.size() != 255) {
        last_idx = std::min(first_idx + 255 - labels.size(), newPoints.size());

        labels.insert(labels.end(), newPoints.begin() + first_idx, newPoints.begin() + last_idx);
        descriptions.insert(descriptions.end(), newDescriptions.begin() + first_idx, newDescriptions.begin() + last_idx);
        units.insert(units.end(), newUnits.begin() + first_idx, newUnits.begin() + last_idx);

        labelsParam.updateSingleDimension();
        descriptionsParam.updateSingleDimension();
        unitsParam.updateSingleDimension();

        first_idx = last_idx;
      }
      ++i;
    }
  }

  // If analogous data has been added
  auto &grpAnalog(_parameters->group("ANALOG"));
  size_t oldAnalogUsed(grpAnalog.parameter("USED").valuesAsInt()[0]);
  // Should always be greater than 0..., but we have to take in
  // account Optotrak lazyness
  if (grpAnalog.nbParameters() && !newAnalogs.empty()) {
    grpAnalog.parameter("USED").set(oldAnalogUsed + newAnalogs.size());

    std::vector<std::string> newDescriptions(newAnalogs.size(), "");
    std::vector<double> newScale(newAnalogs.size(), 1.0);
    std::vector<int> newOffset(newAnalogs.size(), 0);
    std::vector<std::string> newUnits(newAnalogs.size(), "");

    // Dispatch names in LABELS, LABELS2, etc.
    size_t first_idx = 0, last_idx = 0, i = 0;
    while (last_idx < newAnalogs.size()) {
      std::string mod = i != 0? std::to_string(i + 1) : "";

      auto &labelsParam = grpAnalog.parameter("LABELS" + mod, ezc3d::DATA_TYPE::CHAR);
      auto &descriptionsParam = grpAnalog.parameter("DESCRIPTIONS" + mod, ezc3d::DATA_TYPE::CHAR);
      auto &scalesParam = grpAnalog.parameter("SCALE" + mod, ezc3d::DATA_TYPE::FLOAT);
      auto &offsetsParam = grpAnalog.parameter("OFFSET" + mod, ezc3d::DATA_TYPE::INT);
      auto &unitsParam = grpAnalog.parameter("UNITS" + mod, ezc3d::DATA_TYPE::CHAR);

      auto &labels = labelsParam.valuesAsString();
      auto &descriptions = descriptionsParam.valuesAsString();
      auto &scale = scalesParam.valuesAsDouble();
      auto &offset = offsetsParam.valuesAsInt();
      auto &units = unitsParam.valuesAsString();

      if (labels.size() != 255) {
        last_idx = std::min(first_idx + 255 - labels.size(), newAnalogs.size());

        labels.insert(labels.end(), newAnalogs.begin() + first_idx, newAnalogs.begin() + last_idx);
        descriptions.insert(descriptions.end(), newDescriptions.begin() + first_idx, newDescriptions.begin() + last_idx);
        scale.insert(scale.end(), newScale.begin() + first_idx, newScale.begin() + last_idx);
        offset.insert(offset.end(), newOffset.begin() + first_idx, newOffset.begin() + last_idx);
        units.insert(units.end(), newUnits.begin() + first_idx,  newUnits.begin() + last_idx);

        labelsParam.updateSingleDimension();
        descriptionsParam.updateSingleDimension();
        scalesParam.updateSingleDimension();
        offsetsParam.updateSingleDimension();
        unitsParam.updateSingleDimension();

        first_idx = last_idx;
      }
      ++i;
    }
  }

  if (_parameters->isGroup("ROTATION"))
  { // If rotation data has been added
    auto &grpRotation(_parameters->group("ROTATION"));
    int oldPosesUsed(grpRotation.parameter("USED").valuesAsInt()[0]);
    if (grpRotation.nbParameters() && !newPoses.empty()) {
      grpRotation.parameter("USED").set(oldPosesUsed + newPoses.size());

      std::vector<std::string> newDescriptions(newPoses.size(), "");

      // Dispatch names in LABELS, LABELS2, etc.
      size_t first_idx = 0, last_idx = 0, i = 0;
      while (last_idx < newPoses.size()) {
        std::string mod = i != 0? std::to_string(i + 1) : "";

        auto &labelsParam = grpRotation.parameter("LABELS" + mod, ezc3d::DATA_TYPE::CHAR);
        auto &descriptionsParam = grpRotation.parameter("DESCRIPTIONS" + mod, ezc3d::DATA_TYPE::CHAR);

        auto &labels = labelsParam.valuesAsString();
        auto &descriptions = descriptionsParam.valuesAsString();

        if (labels.size() != 255) {
          last_idx = std::min(first_idx + 255 - labels.size(), newPoses.size());

          labels.insert(labels.end(), newPoses.begin() + first_idx, newPoses.begin() + last_idx);
          descriptions.insert(descriptions.end(), newDescriptions.begin() + first_idx, newDescriptions.begin() + last_idx);

          labelsParam.updateSingleDimension();
          descriptionsParam.updateSingleDimension();

          first_idx = last_idx;
        }
        ++i;
      }
    }
  }
}


void ezc3d::c3d::fixupParameters() {

  // Deal with ACTUAL_START_FIELD and ACTUAL_END_FIELD from VICON, if they are
  // present
  bool isVicon = parameters().isGroup("MANUFACTURER") &&
                 parameters().group("MANUFACTURER").isParameter("COMPANY") &&
                 parameters()
                         .group("MANUFACTURER")
                         .parameter("COMPANY")
                         .valuesAsString()
                         .front()
                         .find("Vicon") != std::string::npos;
  if (isVicon &&
      parameters().group("TRIAL").isParameter("ACTUAL_START_FIELD")) {
    // Make sure "ACTUAL_START_FIELD" is of type INT
    _parameters->group("TRIAL")
        .parameter("ACTUAL_START_FIELD")
        .staticCastType(ezc3d::DATA_TYPE::INT);
  }
  if (isVicon && parameters().group("TRIAL").isParameter("ACTUAL_END_FIELD")) {
    // Make sure "ACTUAL_END_FIELD" is of type INT
    _parameters->group("TRIAL")
        .parameter("ACTUAL_END_FIELD")
        .staticCastType(ezc3d::DATA_TYPE::INT);
  }

  if (data->frames.size() == 0) {
    updateHeader();
    return;
  }

  std::vector<std::string> newPoints, newAnalogs, newPoses;

  // If frames has been added
  auto &grpPoint(_parameters->group("POINT"));
  size_t oldPointUsed(grpPoint.parameter("USED").valuesAsInt()[0]);
  size_t nPoints = data->frames.front().points->points.size();
  if (nPoints != oldPointUsed) {
    newPoints.reserve(nPoints - oldPointUsed);
    for (size_t i = oldPointUsed; i < nPoints; ++i) {
      newPoints.push_back("Marker " + std::to_string(i));
    }
  }

  // If analogous data has been added
  auto &grpAnalog(_parameters->group("ANALOG"));
  size_t oldAnalogUsed(grpAnalog.parameter("USED").valuesAsInt()[0]);
  size_t nAnalogs = 0;
  if (data->frames.front().analogs->subframes.size() > 0)
    nAnalogs = data->frames.front().analogs->subframes.front().channels.size();

  // Should always be greater than 0..., but we have to take in
  // account Optotrak lazyness
  if (grpAnalog.nbParameters() && nAnalogs != oldAnalogUsed) {
    newAnalogs.reserve(nAnalogs - oldAnalogUsed);
    for (size_t i = oldAnalogUsed; i < nAnalogs; ++i) {
      newAnalogs.push_back("Analog " + std::to_string(i));
    }
  }

  if (_parameters->isGroup("ROTATION"))
  { // If rotation data has been added
    auto &grpRotation(_parameters->group("ROTATION"));
    int oldPosesUsed(grpRotation.parameter("USED").valuesAsInt()[0]);
    size_t nbPoses = 0;
    if (data->frames.front().rotations->subframes.size() > 0)
      nbPoses = data->frames.front().rotations->subframes.front().rotations.size();

    if (grpRotation.nbParameters() && nbPoses != oldPosesUsed) {
      newPoses.reserve(nbPoses - oldPosesUsed);
      for (size_t i = oldPosesUsed; i < nbPoses; ++i) {
        newPoses.push_back("Pose " + std::to_string(i));
      }
    }
  }

  if (newPoints.empty() && newAnalogs.empty() && newPoses.empty())
    return;

  // Add any missing labels in parameters for existing data 
  appendParameters(newPoints, newAnalogs, newPoses);

  updateHeader();
}