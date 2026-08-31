#ifndef EZC3D_POINTS_H
#define EZC3D_POINTS_H
///
/// \file Points.h
/// \brief Declaration of Points class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Point.h"
#include <cstdint>

///
/// \brief Points holder for C3D data 3D points data
///
class EZC3D_VISIBILITY ezc3d::DataNS::Points3dNS::Points {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create an empty holder for 3D points
  ///
  EZC3D_API Points();

  ///
  /// \brief Create an empty holder for 3D points preallocating the size of it
  /// \param nbPoints Number of 3D points to be in the holder
  ///
  EZC3D_API Points(size_t nbPoints);

  ///
  /// \brief Create a filled Points class at a given frame from a given file
  /// \param c3d Reference to the c3d to copy the data in
  /// \param file File to copy the data from
  /// \param info The information about the points
  ///
  EZC3D_API Points(ezc3d::c3d &c3d, std::fstream &file,
                   const Points3dNS::Info &info);

  ///
  /// \brief Create a deep copy of a Points class
  /// \param points The Points class to copy
  /// \return A deep copy of the Points class
  ///
  EZC3D_API Points clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the points
  ///
  /// Print the points to the console by calling sequentially the print method
  /// for all the points
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write points to an opened file
  /// \param f Already opened fstream file with write access
  /// \param pointsInfo The points info to write the data with
  ///
  /// Write all the points to a file by calling sequentially the write method of
  /// each point
  ///
  EZC3D_API void write(std::fstream &f,
                       const ezc3d::DataNS::Points3dNS::Info &pointsInfo) const;

  //---- POINT ----//

  std::vector<ezc3d::DataNS::Points3dNS::Point> points; ///< Holder of the 3D points
public:

  ///
  /// \brief Return if the points are empty
  /// \return if the points are empty
  ///
  EZC3D_API bool isEmpty() const;
};

#endif
