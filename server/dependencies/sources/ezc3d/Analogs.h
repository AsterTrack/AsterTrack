#ifndef EZC3D_ANALOGS_H
#define EZC3D_ANALOGS_H
///
/// \file Analogs.h
/// \brief Declaration of Analogs class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/AnalogsSubframe.h"

///
/// \brief Analog holder for C3D analogous data
///
class EZC3D_VISIBILITY ezc3d::DataNS::AnalogsNS::Analogs {
  //---- CONSTRUCTORS ----//
public:
  ///
  /// \brief Create an empty holder for the analogous data
  ///
  EZC3D_API Analogs();

  ///
  /// \brief Create a filled Analogs class at a given frame from a given file
  /// \param c3d Reference to the c3d to copy the data in
  /// \param file File to copy the data from
  /// \param info The information about the analogs
  ///
  EZC3D_API Analogs(ezc3d::c3d &c3d, std::fstream &file,
                    const AnalogsNS::Info &info);

  ///
  /// \brief Create a deep copy of an Analogs class
  /// \return A deep copy of the Analogs class
  ///
  EZC3D_API Analogs clone() const;

  //---- STREAM ----//
public:
  ///
  ///
  /// \brief Print the subframes
  ///
  /// Print the subframes to the console by calling sequentially the print
  /// method of each subframes
  ///
  EZC3D_API void print() const;

  ///
  /// \brief Write the subframes to an opened file
  /// \param f Already opened fstream file with write access
  /// \param analogsInfo The information about the analogs
  ///
  /// Write all the subframes to a file by calling sequentially the write method
  /// of each subframe
  ///
  EZC3D_API void write(std::fstream &f,
                       const ezc3d::DataNS::AnalogsNS::Info &analogsInfo) const;

  //---- SUBFRAME ----//

  std::vector<ezc3d::DataNS::AnalogsNS::SubFrame> subframes; ///< Holder for the subframes
public:
  ///
  /// \brief Return if the analogs are empty
  /// \return if the analogs are empty
  ///
  EZC3D_API bool isEmpty() const;
};

#endif
