#ifndef EZC3D_ROTATIONS_H
#define EZC3D_ROTATIONS_H
///
/// \file Rotations.cpp
/// \brief Implementation of Rotations class
/// \author Pariterre
/// \version 1.0
/// \date April 30th, 2022
///

#include "ezc3d/RotationsSubframe.h"

///
/// \brief Rotation holder for C3D Rotations data
/// base on documentation from
/// https://www.c-motion.com/v3dwiki/index.php?title=ROTATION_DATA_TYPE
///
class EZC3D_VISIBILITY ezc3d::DataNS::RotationNS::Rotations {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create an empty holder for Rotation data
  ///
  EZC3D_API Rotations();

  ///
  /// \brief Create an empty holder for Rotation data preallocating the size of
  /// it \param c3d Reference to the c3d to copy the data in \param file File to
  /// copy the data from \param info The information about the rotations
  ///
  EZC3D_API Rotations(ezc3d::c3d &c3d, std::fstream &file,
                      const RotationNS::Info &info);

  ///
  /// \brief Create a deep copy of a Rotations
  /// \return A deep copy of the Rotations
  ///
  EZC3D_API Rotations clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the rotations
  ///
  /// Print the Rotations to the console by calling sequentially the print
  /// method for all the rotations
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write rotations to an opened file (scaleFactor is necessarily -1)
  /// \param f Already opened fstream file with write access
  ///
  /// Write all the rotations to a file by calling sequentially the write method
  /// of each rotation
  ///
  EZC3D_API void write(std::fstream &f) const;

  //---- ROTATION ----//

  std::vector<ezc3d::DataNS::RotationNS::SubFrame> subframes; ///< Holder of the 3D rotations at each frame

public:

  ///
  /// \brief Return if the rotations are empty
  /// \return if the rotations are empty
  ///
  EZC3D_API bool isEmpty() const;
};

#endif
