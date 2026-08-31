#ifndef EZC3D_ROTATIONS_SUBFRAME_H
#define EZC3D_ROTATIONS_SUBFRAME_H
///
/// \file RotationsSubframe.h
/// \brief Declaration of Subframe class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Rotation.h"
#include <cstdint>

///
/// \brief Subframe for the rotation data
///
class EZC3D_VISIBILITY ezc3d::DataNS::RotationNS::SubFrame {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create an empty subframe for rotation data
  ///
  EZC3D_API SubFrame();

  ///
  /// \brief Create a filled SubFrame class at a given frame from a given file
  /// \param c3d Reference to the c3d to copy the data in
  /// \param file File to copy the data from
  /// \param info The information about the rotations
  ///
  EZC3D_API SubFrame(ezc3d::c3d &c3d, std::fstream &file,
                     const RotationNS::Info &info);

  ///
  /// \brief Create a deep copy of a SubFrame
  /// \param subframe The SubFrame to copy
  /// \return A deep copy of the SubFrame
  ///
  EZC3D_API SubFrame clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the subframe
  ///
  /// Print the subframe to the console by calling sequentially the print method
  /// of all of the rotation
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write the subframe to an opened file
  /// \param f Already opened fstream file with write access
  ///
  /// Write the subframe to a file by calling sequentially the write method of
  /// all of the rotation
  ///
  EZC3D_API void write(std::fstream &f) const;

  //---- ROTATIONS ----//

  std::vector<ezc3d::DataNS::RotationNS::Rotation> rotations; ///< Holder for the rotations
public:
  ///
  /// \brief Return if the subframe is empty
  /// \return if the subframe is empty
  ///
  EZC3D_API bool isEmpty() const;
};

#endif
