#define EZC3D_API_EXPORTS
///
/// \file Frame.cpp
/// \brief Implementation of Frame class
/// \author Pariterre
/// \version 1.0
/// \date October 17th, 2018
///

#include "ezc3d/Frame.h"
#include "ezc3d/DataStartInfo.h"
#include <stdexcept>

ezc3d::DataNS::Frame::Frame() {
	points = std::make_shared<ezc3d::DataNS::Points3dNS::Points>();
	analogs = std::make_shared<ezc3d::DataNS::AnalogsNS::Analogs>();
	rotations = std::make_shared<ezc3d::DataNS::RotationNS::Rotations>();
}

ezc3d::DataNS::Frame ezc3d::DataNS::Frame::clone() const {
	Frame copy;
	copy.points = std::make_shared<ezc3d::DataNS::Points3dNS::Points>(points->clone());
	copy.analogs = std::make_shared<ezc3d::DataNS::AnalogsNS::Analogs>(analogs->clone());
	copy.rotations = std::make_shared<ezc3d::DataNS::RotationNS::Rotations>(rotations->clone());
	return copy;
}

void ezc3d::DataNS::Frame::print() const {
	points->print();
	analogs->print();
	rotations->print();
}

void ezc3d::DataNS::Frame::write(
		std::fstream &f, const ezc3d::DataNS::Points3dNS::Info &pointsInfo,
		const ezc3d::DataNS::AnalogsNS::Info &analogsInfo,
		int dataTypeToWrite) const {
	if (dataTypeToWrite == 0) { // Points and analogs
		points->write(f, pointsInfo);
		analogs->write(f, analogsInfo);
	} else if (dataTypeToWrite == 1) { // Rotations
		rotations->write(f);
	} else {
		throw std::runtime_error("Data type not implemented yet");
	}
}

bool ezc3d::DataNS::Frame::isEmpty() const {
	return points->isEmpty() && analogs->isEmpty();
}
