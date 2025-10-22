/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ui/ui.hpp"

#include "unsupported/Eigen/NonLinearOptimization"

#include <cmath>

void InterfaceState::UpdateLensSelectionTool(InterfaceWindow &window)
{
	if (!window.open)
	{
		visState.calib.showDesignCalib = false;
		visState.calib.designCalibs.clear();
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		visState.calib.showDesignCalib = false;
		visState.calib.designCalibs.clear();
		ImGui::End();
		return;
	}

	// Sensor parameters
	static Eigen::Vector2i sensorPixels, activePixels;
	static Eigen::Vector2f sensorSize, activeSize;
	static float sensorCRA = 9;
	static bool OV9281Input = true;

	// Blob Detection parameters
	static Eigen::Vector2i usedPixels;
	static Eigen::Vector2f usedSize;

	// Lens parameters
	struct LensParams
	{
		float focalLen = 2.5f;
		float fnumber = 2.3f;
		float CRA = 19;
		Eigen::Vector2f designSize = { 3.6, 2.7 }; // 1/4" 4:3 sensor
		std::array<float, 3> measuredFOV_HVD = { 78.2, 45.5, 91.5 };
		Eigen::Vector3f distCoeff = {  0, 0, 0 };
	};
	static std::vector<LensParams> lenses(1);
	static int lensSlot = 1;

	// Setup parameters
	static float wavelength = 0.850;
	static float desiredContrast = 0.2f;
	static float testResolution = 200;

	ImGui::PushTextWrapPos();

	BeginSection("Sensor");
	CheckboxInput("Default OV9281", &OV9281Input);
	if (OV9281Input)
	{
		sensorSize = Eigen::Vector2f(3.896f, 2.453f);
		sensorPixels = Eigen::Vector2i(1296, 816);
		activePixels = Eigen::Vector2i(1280, 800);
		usedPixels = Eigen::Vector2i(1152, 800);
		sensorCRA = 9;
	}
	ImGui::BeginDisabled(OV9281Input);
	ScalarInput2<float>("Sensor Size", "um", &sensorSize.x(), &sensorSize.y(), 0, 10000, 1, 1000, "%.0f");
	ScalarInput2<int>("Sensor Pixels", "px", &sensorPixels.x(), &sensorPixels.y(), 0, 10000);
	ImGui::SetItemTooltip("Pixel Size that corresponds to sensor size - for some sensors, larger than what is actually useable.");
	ScalarInput2<int>("Active Pixels", "px", &activePixels.x(), &activePixels.y(), 0, 10000);
	ImGui::SetItemTooltip("Pixel Size of the best mode, used to evaluate full sensor.");
	ImGui::EndDisabled();
	ImGui::BeginDisabled(true);
	activeSize = sensorSize.cwiseProduct(activePixels.cast<float>().cwiseQuotient(sensorPixels.cast<float>()));
	ScalarInput2<float>("Active Size", "um", &activeSize.x(), &activeSize.y(), 0, 10000, 1, 1000, "%.0f");
	float fullImageCircle = activeSize.norm();
	ScalarInput<float>("Active Image Circle", "mm", &fullImageCircle, 0, 10000);
	Eigen::Vector2f pixelSizes = activeSize.cwiseQuotient(activePixels.cast<float>());
	float pixelSizeUM = pixelSizes.x()*1000;
	if (std::abs(pixelSizes.x() - pixelSizes.y()) > 0.000001)
		ImGui::Text("Sensor has a Pixel Size of %.3fum x %.3f - Warning: Non-Square!", pixelSizes.x()*1000, pixelSizes.y()*1000);
	else
		ImGui::Text("Sensor has a Pixel Size of %.3fum!", pixelSizes.x()*1000);
	ImGui::EndDisabled();
	ImGui::BeginDisabled(OV9281Input);
	ScalarInput<float>("CRA", "°", &sensorCRA, 0, 40, 1, 1, "%.1f");
	ImGui::SetItemTooltip("Chief Ray Angle, mismatch to lens CRA can result in poor relative illumination,\n"
		"and even vignetting (different from vignetting image circle).");
	ImGui::EndDisabled();
	EndSection();

	BeginSection("Used Area");
	ScalarInput2<int>("Used Pixels", "px", &usedPixels.x(), &usedPixels.y(), 0, 10000);
	ImGui::SetItemTooltip("Pixel Size of the used mode, used to evaluate a relevant ROI of the sensor.");
	ImGui::BeginDisabled(true);
	usedSize = pixelSizes.cwiseProduct(usedPixels.cast<float>());
	ScalarInput2<float>("Used Size", "um", &usedSize.x(), &usedSize.y(), 0, 10000, 1, 1000, "%.0f");
	float usedImageCircle = usedSize.norm();
	ScalarInput<float>("Used Image Circle", "mm", &usedImageCircle, 0, 10000);
	ImGui::EndDisabled();
	EndSection();

	BeginSection("Lens");
	SliderInput<int>("Selected Lens Slot", &lensSlot, 1, 3);
	if (lenses.size() < lensSlot) lenses.resize(lensSlot);
	auto &lens = lenses[lensSlot-1];
	SliderInput<float>("Focal Length", &lens.focalLen, 1, 5, 1, "%.3fmm");
	ScalarInput<float>("f/# (F-Number)", "x", &lens.fnumber, 0, 10, 0.1f);
	ImGui::SetItemTooltip("Ratio of focal len to aperture diameter.\n"
		"Lower f/# means more light throughput and more potential resolution due to diffraction.\n"
		"However, it also means lower Depth of Field and worse relative illumination (still more light in total).");
	ImGui::BeginDisabled(true);
	float apertureDiameter = lens.focalLen/lens.fnumber;
	ScalarInput<float>("Aperture Diameter", "mm", &apertureDiameter, 0, 100, 1);
	ImGui::EndDisabled();
	ScalarInput2<float>("Design Format", "um", &lens.designSize.x(), &lens.designSize.y(), 0, 10000, 1, 1000, "%.0f");
	ImGui::SetItemTooltip("The format (or better yet, exact sensor size) the lens has been designed for (3600x2700 for a 4:3 1/4\").\n"
		"Without knowing this, many specs are practically useless, including FoV, distortions and relative illumination.");
	ScalarInput<float>("CRA", "°", &lens.CRA, 0, 40, 1, 1, "%.1f");
	ImGui::SetItemTooltip("Chief Ray Angle, mismatch to sensor CRA can result in poor relative illumination,\n"
		"and even vignetting (different from vignetting image circle).");
	EndSection();

	BeginSection("Distortion");
	static bool fov2Dist = true;
	ImGui::Checkbox("Estimate Distortion from FoV", &fov2Dist);
	ImGui::SetItemTooltip("This will attempt to estimate the distortion profile from the following values:\n"
		"1. The real focal length of the lens (not EFL)\n"
		"2. Three FoV measurements in degree, H, V, D\n"
		"3. Exact sensor size used to measure that FoV - if only format is specified, guess.");

	ImGui::BeginDisabled(!fov2Dist);
	ImGui::InputFloat3("h/v/d", lens.measuredFOV_HVD.data(), "%.2f°");
	ImGui::EndDisabled();
	if (fov2Dist)
	{
		struct ErrorTerm
		{
			enum
			{
				InputsAtCompileTime = Eigen::Dynamic,
				ValuesAtCompileTime = Eigen::Dynamic
			};
			typedef float Scalar;
			typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

			std::array<float, 3> fovHVD;
			Eigen::Vector2f size;
			float fInv;

			int operator()(const Eigen::VectorXf &coeffs, Eigen::VectorXf &errors) const
			{
				CameraCalib calib = {};
				calib.fInv = fInv;
				calib.f = 1.0f/fInv;
				calib.distortion.k1 = coeffs(0);
				calib.distortion.k2 = coeffs(1);
				calib.distortion.k3 = coeffs(2);
				float effFInvH = std::tan(fovHVD[0] / 180 * PI / 2);
				float effFInvV = std::tan(fovHVD[1] / 180 * PI / 2);
				float effFInvD = std::tan(fovHVD[2] / 180 * PI / 2);
				float distH = undistortPoint(calib, Eigen::Vector2f(size.x(), 0)).x();
				float distV = undistortPoint(calib, Eigen::Vector2f(size.y(), 0)).x();
				float distD = undistortPoint(calib, Eigen::Vector2f(size.norm(), 0)).x();
				errors(0) = distH - effFInvH/fInv;
				errors(1) = distV - effFInvV/fInv;
				errors(2) = distD - effFInvD/fInv;
				return 0;
			}

			int inputs() const { return 3; }
			int values() const { return 3; }
		};
		ErrorTerm errorTerm = {};
		errorTerm.fovHVD = lens.measuredFOV_HVD;
		errorTerm.size = lens.designSize / lens.designSize.x();
		errorTerm.fInv = lens.designSize.x()/2/lens.focalLen;
		Eigen::NumericalDiff<ErrorTerm> errorGradient(errorTerm);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ErrorTerm>, float> lm(errorGradient);
		Eigen::VectorXf projCoeff(lens.distCoeff);
		auto status = lm.minimize(projCoeff);
		lens.distCoeff = Eigen::Vector3f(projCoeff);
	}

	ImGui::BeginDisabled(fov2Dist);
	ImGui::InputFloat3("K1/K2/K3", lens.distCoeff.data(), "%.5f");
	ImGui::SetItemTooltip("Distortion coefficients, never found in specs, but can be estimated or calibrated.");
	ImGui::EndDisabled();

	// Estimated calibration for the lens + design sensor combination
	CameraCalib calib = {};
	calib.fInv = lens.designSize.x()/2/lens.focalLen;
	calib.f = 1.0f/calib.fInv;
	calib.distortion.k1 = lens.distCoeff[0];
	calib.distortion.k2 = lens.distCoeff[1];
	calib.distortion.k3 = lens.distCoeff[2];
	float scale = 1.0f/lens.designSize.x();

	ImGui::Text("Distortions of %.2f%% h, %.2f%% v, %.2f%% d for design sensor.",
		(undistortPoint(calib, Eigen::Vector2f(lens.designSize.x()*scale,0)).x()-1)*100,
		(undistortPoint(calib, Eigen::Vector2f(lens.designSize.y()*scale,0)).x()-1)*100,
		(undistortPoint(calib, Eigen::Vector2f(lens.designSize.norm()*scale,0)).x()-1)*100);
	ImGui::Text("Warning: Don't use EFL with distortion values! Read more.");
	ImGui::SetItemTooltip("If you use distortion values, make sure you supply the real focal length, not the effective focal length (EFL).\n"
		"There is no way to get the real focal length if not supplied by the lens specification.\n"
		"EFL already tries to account for distortion, but can only do so for one point,\n"
		"usually the hFoV of the format (or specific sensor) it is designed for.");

	ImGui::Checkbox("Visualise design calibration", &visState.calib.showDesignCalib);

	{ // Translate to active sensor for visualisation
		if (visState.calib.designCalibs.size() < lensSlot) visState.calib.designCalibs.resize(lensSlot);
		auto &activeCalib = visState.calib.designCalibs[lensSlot-1];
		activeCalib = calib;
		activeCalib.fInv = activeSize.x()/2/lens.focalLen;
		activeCalib.f = 1.0f/activeCalib.fInv;
	}

	EndSection();

	BeginSection("Field of View");
	ImGui::Text("Design Sensor: %.1fdg hFOV, %.1fdg vFOV, %.1fdg dFOV",
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(lens.designSize.x()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(lens.designSize.y()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(lens.designSize.norm()*scale,0)).x()*calib.fInv) * 180/PI);
	ImGui::Text("Active Area: %.1fdg hFOV, %.1fdg vFOV, %.1fdg dFOV",
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(activeSize.x()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(activeSize.y()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(activeSize.norm()*scale,0)).x()*calib.fInv) * 180/PI);
	ImGui::Text("Used Area: %.1fdg hFOV, %.1fdg vFOV, %.1fdg dFOV",
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(usedSize.x()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(usedSize.y()*scale,0)).x()*calib.fInv) * 180/PI,
		2 * std::atan(undistortPoint(calib, Eigen::Vector2f(usedSize.norm()*scale,0)).x()*calib.fInv) * 180/PI);
	ImGui::Text("Warning: Estimation inaccurate without exact distortion profile! Read more.");
	ImGui::SetItemTooltip("This estimation tries to account for distortion using the distortion coefficients supplied.\n"
		"Without those, FoV estimations are entirely hypothetical, since distortion affects hFOV and vFOV differently.\n"
		"Usually hFOV tends to be higher than sensor aspect suggests, while vFOV tends to be lower.\n"
		"Effective Focal Length (EFL) usually accounts for that distortion in some way, but only for one FOV value, usually hFOV.");
	EndSection();

	BeginSection("Optical Resolution");
	ScalarInput<float>("Wavelength", "nm", &wavelength, 100, 1000, 10, 1000, "%.0f");
	float airyDisk = 2.44 * wavelength * lens.fnumber;
	float diffractionLimit = (1000.0f) / (lens.fnumber * wavelength);
	ImGui::Text("Airy Disk (Min Detail Size) of %.3fum is %c pixel size of %.3fum! "
		"Diffraction limit is %.3flp/mm - Read more.",
		airyDisk, airyDisk < pixelSizeUM? '<' : '>', pixelSizeUM, diffractionLimit);
	ImGui::SetItemTooltip("Airy Disk is the minimum size an infinitely fine detail would have on the sensor due to diffraction.\n"
		"At the resolution specified by the diffraction limit, even if the lens was perfect, details would be indistinguishable.\n"
		"Detail here means two pixel details with one pixel in between, the contrast of that in-between pixel is the relevant measure.\n"
		"Generally, contrast of 10-20%% is the minimum, below that the details are considered lost.");
	float sensorResolution = 1000/pixelSizeUM/2;
	auto getContrast = [&diffractionLimit](float resolution)
	{
		float phi = std::acos(resolution / diffractionLimit);
		return 2/PI * (phi - std::cos(phi) * std::sin(phi));
	};
	ImGui::Text("The sensor resolution is %.0flp/mm, a perfect lens could reproduce details with %.2f%% contast",
		sensorResolution, getContrast(sensorResolution)*100);
	SliderInput<float>("Test Lens Resolution", &testResolution, 100, 500, 1, "%.0flp/mm");
	ImGui::Text("A perfect lens could resolve such details with %.1f%% contrast - Read more.",
		getContrast(testResolution)*100);
	ImGui::SetItemTooltip("Note: In the end, this is a lens spec that should be specified for the relevant\n"
		"wavelength and with a specific contrast, or better as a full MTF-graph!");
	//SliderInput<float>("Desired Contrast", &desiredContrast, nullptr, 0, 100, 100, "%.2f%%");
	EndSection();

	BeginSection("Relative Illumination");
	float virtualApertureHeight = lens.designSize.norm() / std::sin(lens.CRA/180*PI);
	float CRAinUsedArea = std::asin(usedSize.norm()/virtualApertureHeight) /PI*180;
	float CRAmaxDev = sensorCRA < 10? 10 : (sensorCRA < 20? 7 : 4); // according to https://commonlands.com/blogs/technical/lens-chief-ray-angle-and-mismatch
	ImGui::Text("Max CRA in used sensor area: %.2f° (sensor CRA %.2f°)", CRAinUsedArea, sensorCRA);
	float CRADeviation = std::abs(CRAinUsedArea-sensorCRA);
	ImGui::Text("Deviation of %.2f° is %c recommended limit of %.2f°! Read more.", CRADeviation, CRADeviation < CRAmaxDev? '<' : '>', CRAmaxDev);
	ImGui::SetItemTooltip("Note: Too high of a CRA deviation can yield strong drops in relative illumination (and discoloration for color sensors)!");
	EndSection();

	ImGui::PopTextWrapPos();

	ImGui::End();
}