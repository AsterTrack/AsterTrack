#ifndef EZC3D_DATA_H
#define EZC3D_DATA_H
///
/// \file Data.h
/// \brief Declaration of data class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Frame.h"

///
/// \brief Data of the C3D file
///
/// The class stores all the data frames of a given or create C3D into a STL
/// vector of frame.
///
class EZC3D_VISIBILITY ezc3d::DataNS::Data {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create a ready to fill Data class
  ///
  EZC3D_API Data();

  ///
  /// \brief Create a filled Data class from a given file
  /// \param c3d Reference to the c3d to copy the data in
  /// \param file File to copy the data from
  ///
  EZC3D_API Data(ezc3d::c3d &c3d, std::fstream &file);

  ///
  /// \brief Create a deep copy of a Data
  /// \return A deep copy of the Data
  ///
  EZC3D_API Data clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the data
  ///
  /// Print all the data to the console by calling sequentially all the print
  /// method for all the frames
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write all the data to an opened file
  /// \param The header of a c3d
  /// \param f Already opened fstream file with write access
  /// \param pointsInfo The points info to write the data with
  /// \param analogsInfo The analogs info to write the data with
  /// \param dataStartInfoToFill The start position to fill
  ///
  /// Write all the data to a file by calling sequentially all the write method
  /// for all the frames
  ///
  EZC3D_API void write(const ezc3d::Header &header, std::fstream &f,
                       const ezc3d::DataNS::Points3dNS::Info &pointsInfo,
                       const ezc3d::DataNS::AnalogsNS::Info &analogsInfo,
                       ezc3d::DataStartInfo &dataStartInfoToFill) const;

  //---- FRAME ----//

  std::vector<ezc3d::DataNS::Frame> frames; ///< Storage of the data

public:
  ///
  /// \brief Add/replace a frame to the data set
  /// \param frame the frame to add/replace
  /// \param idx the index of the frame
  ///
  /// Add or replace a particular frame to the data set.
  ///
  /// If no idx is sent, then the frame is appended to the data set.
  /// If the idx correspond to a specific frame, it replaces it.
  /// If idx is outside the data set, it resize the data set accordingly and add
  /// the frame where it belongs but leaves the other created frames empty.
  ///
  EZC3D_API void frame(ezc3d::DataNS::Frame &&frame,
                       size_t idx = SIZE_MAX);
};

#endif
