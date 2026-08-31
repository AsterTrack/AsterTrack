#ifndef EZC3D_FRAME_H
#define EZC3D_FRAME_H
///
/// \file Frame.h
/// \brief Declaration of Frame class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Analogs.h"
#include "ezc3d/Points.h"
#include "ezc3d/Rotations.h"

///
/// \brief Frame holder for C3D data
///
class EZC3D_VISIBILITY ezc3d::DataNS::Frame {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create an empty frame
  ///
  EZC3D_API Frame();

  ///
  /// \brief Create a deep copy of a frame
  /// \param frame The frame to copy
  ///
  EZC3D_API Frame clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the frame
  ///
  /// Print the frame to the console by calling sequentially the print method
  /// for points and analogs
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write a frame to an opened file
  /// \param f Already opened fstream file with write access
  /// \param pointsInfo The points info to write the data with
  /// \param analogsInfo The analogs info to write the data with
  /// \param dataTypeToWrite The type of data block (0 points/analogs, 1
  /// rotations)
  ///
  /// Write the frame to a file by calling sequentially the write method for
  /// points and analogs
  ///
  EZC3D_API void write(std::fstream &f,
                       const ezc3d::DataNS::Points3dNS::Info &pointsInfo,
                       const ezc3d::DataNS::AnalogsNS::Info &analogsInfo,
                       int dataTypeToWrite) const;

  std::shared_ptr<ezc3d::DataNS::Points3dNS::Points> points; ///< All the points for this frame

  std::shared_ptr<ezc3d::DataNS::AnalogsNS::Analogs> analogs; ///< All the subframes for all the analogs

  std::shared_ptr<ezc3d::DataNS::RotationNS::Rotations> rotations; ///< All the rotations for this frame

  ///
  /// \brief Return if the frame is empty
  /// \return if the frame is empty
  ///
  EZC3D_API bool isEmpty() const;
};

#endif
