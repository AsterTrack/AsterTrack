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

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <stdint.h>

struct ThresholdingParameters
{
	int absolute = 100;
	int edge = 15;
};

struct ClassificationParameters 
{
	// Classification 1: Whether to resegment first, or just accept cluster as-is
	// Goal: Smaller clusters that might have multiple merged blobs should be resegmented
	//       Larger ones likely to only contain one blob should not 
	bool resegmentSingleClusters = true;
	int resegmentationThreshold = 1000;

	// Classification 2: Whether to refine the cluster or not
	int blobRefinementThreshold = 200;

	int blobTinyThreshold = 1;
};

struct BaseParameters
{
	bool blur = true;
	int radius = 1;
	float sigma = 0.5f;
	// TODO: Add box blur? Doesn't seem to perform any better at these low radii though
};

struct SSRParameters 
{
	int sigmaSteps = 4;
	float sigmaMin = 0.4f;
	float sigmaMax = 1.0f;
	float sigmaCurve = 0.8f;
	float sigmaTrunc = 2.5f;
};

struct MaximaHintParameters 
{
	int plateauThreshold = 245;
	int plateauFillOffset = 20;
	int minStable = 2;
	int minScale = 1;
};

struct FloodfillingParameters 
{
	struct
	{
		int min = 15;
		int step = 40, minSubStep = 5;
		int acceptableLoss = 5;
	} threshold;

	struct
	{
		float peakMinimumRatio = 8.0f;
		float limitExpansionFactor = 1.2f;
		float limitExpansionBase = 0.3f;
		int allowBoundsExpansion = 1;
	} blob;
};

struct BlobFilteringParameters 
{
	float minContrastValue = 10;
	int minContributingPixels = 5;
};

struct RefinementParameters 
{
	int targetEdgeVal = 100;
	float maxEdgeOffsetPX = 2.0f;
};

struct BlobProcessingParameters 
{
	ThresholdingParameters thresholds;
	ClassificationParameters classification;
	BaseParameters base;
	SSRParameters ssr;
	MaximaHintParameters maximaHints;
	FloodfillingParameters floodfilling;
	BlobFilteringParameters filtering;
	RefinementParameters refinement;
};

#endif // PARAMETERS_H