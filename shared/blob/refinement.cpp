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

#include "refinement.hpp"

#define LOG_MAX_LEVEL LInfo
#include "util/log.hpp"

#include <cmath>
#include <cstring>
#include <cstdint>

void refineBlobEdge(const std::vector<Vector2<uint16_t>> &edge, const uint8_t *frame, uint32_t stride, std::vector<Vector2<float>> &refinedEdge, int target, float maxOffsetPX)
{
	// Parameters for gradient calibration
	const int n = 162, c = 47; // Neighbours and corner numbers for sobel filter - chosen to be close to true derivative
	float maxOffsetPXSq = maxOffsetPX*maxOffsetPX;

	// Refine edge point positions using a few iterations over the gradient
	refinedEdge.resize(edge.size());
	for (int i = 0; i < edge.size(); i++)
	{
		Vector2<uint16_t> edgePt = edge[i];
		int index = edgePt.y()*stride+edgePt.x();
		Vector2<int> sobelSum = Vector2<int>::Zero();
		if (edgePt.y() > 0)
		{ // Except for the top, there is a 2 pixel margin left by QPU blob detection itself
			// For top, with 0px margin - assume 0 gradient, e.g. don't move edge
			sobelSum.x() = c*frame[index-stride-1] + n*frame[index-1] + c*frame[index+stride-1]
				- (c*frame[index-stride+1] + n*frame[index+1] + c*frame[index+stride+1]);
			sobelSum.y() = c*frame[index-stride-1] + n*frame[index-stride] + c*frame[index-stride+1]
				- (c*frame[index+stride-1] + n*frame[index+stride] + c*frame[index+stride+1]);
		}
		Vector2<float> gradient = sobelSum.cast<float>() / (n+c+c);
		float steepnessSq = gradient.squaredNorm();
		if (steepnessSq > 0)
		{
			float travelFac = (frame[index] - target)/steepnessSq;
			Vector2<float> offset = gradient * travelFac;
			float travelSq = offset.squaredNorm();
			if (travelSq > maxOffsetPXSq)
			{ // Limit travel (roughly)
				float norm = maxOffsetPX / std::sqrt(travelSq);
				offset = offset*norm;
			}
			refinedEdge[i] = edgePt.cast<float>() + offset;
		}
		else
			refinedEdge[i] = edgePt.cast<float>();
		//refinedEdge[i].x() -= bounds.minX;
		//refinedEdge[i].y() -= bounds.minY;
	}
}

bool getRefinementEstimate(const std::vector<Vector2<float>> &edge, const HoughParameters &params, int &radiusStep, Vector2<float> &center)
{
	radiusStep = -1;
	center = Vector2<float>(0,0);
	if (params.positionStep < 0.001) return false; // no 0 allowed

	static std::vector<uint8_t> votes;
	int max = 0, bestSum = 0;
	int r;
	for (r = 0; r < params.radiusRange; r++)
	{
		Vector2<float> positionBoundsSize;
		if (params.boundsPX.x() > params.boundsPX.y())
		{
			//positionBoundsSize.x() = params.boundsPX.y();
			positionBoundsSize.x() = params.boundsPX.x();
			//positionBoundsSize.y() = params.boundsPX.x()*2-params.boundsPX.y();
			positionBoundsSize.y() = params.boundsPX.x();
		}
		else
		{
			//positionBoundsSize.x() = params.boundsPX.y()*2-params.boundsPX.x();
			positionBoundsSize.x() = params.boundsPX.y();
			//positionBoundsSize.y() = params.boundsPX.x();
			positionBoundsSize.y() = params.boundsPX.y();
		}
		Vector2<int> positionRange = (positionBoundsSize/params.positionStep).cast<int>();
		Vector2<float> positionMin = params.boundsMid - positionRange.cast<float>()*params.positionStep/2;
		Vector2<float> positionMax = params.boundsMid + positionRange.cast<float>()*params.positionStep/2;

		votes.resize(positionRange.x()*positionRange.y());
		memset(votes.data(), 0, positionRange.x()*positionRange.y());

		float radius = params.radiusMin + r * params.radiusStep;
		float radiusSq = radius*radius;
		float circleWidthSq = (radius+params.circleWidth)*(radius+params.circleWidth) - radiusSq;
		LOG(LCameraBlob, LDebug, "Checking for circle of radius %f (%d) in bounds (%f, %f) - (%f, %f) with map (%d, %d)",
			radius, r, positionMin.x(), positionMin.y(), positionMax.x(), positionMax.y(), positionRange.x(), positionRange.y());
		for (int i = 0; i < edge.size(); i++)
		{
			Vector2<float> edgePt = edge[i];
			for (int y = 0; y < positionRange.y(); y++)
			{
				float yPos = positionMin.y()+params.positionStep*y;
				float yDiffSq = (yPos-edgePt.y());
				yDiffSq = yDiffSq*yDiffSq;
				for (int x = 0; x < positionRange.x(); x++)
				{
					float xPos = positionMin.x()+params.positionStep*x;
					float xDiffSq = (xPos-edgePt.x());
					xDiffSq = xDiffSq*xDiffSq;
					// Get distance
					float diffSq = yDiffSq + xDiffSq;
					if (std::abs(diffSq - radiusSq) < circleWidthSq)
						votes[y*positionRange.x()+x] += 1;
				}
			}
		}

		int rMax = -1;
		Vector2<int> rBest;
		for (int y = 1; y < positionRange.y()-1; y++)
		{
			for (int x = 1; x < positionRange.y()-1; x++)
			{
				int index = y*positionRange.y()+x;
				if (votes[index] > rMax)
				{
					rMax = votes[index];
					rBest = Vector2<int>(x, y);

					Eigen::Vector2f centerTmp = positionMin+params.positionStep*rBest.cast<float>();
					LOG(LCameraBlob, LTrace, "Found new best circle for radius %f (%d), center (%f, %f), %d votes, %d with surroundings",
						radius, r, centerTmp.x(), centerTmp.y(), max, bestSum);
				}
			}
		}
		Eigen::Vector2f centerTmp = positionMin+params.positionStep*rBest.cast<float>();
		LOG(LCameraBlob, LDebug, "Best circle for radius %f (%d) is center (%f, %f), %d votes", radius, r, centerTmp.x(), centerTmp.y(), rMax);

		if (rMax > 0)
		{
			int index = rBest.y()*positionRange.x()+rBest.x();
			int rBestSum = votes[index] + votes[index-1] + votes[index+1] + votes[index-positionRange.x()] + votes[index+positionRange.x()];

			float sizeFac = 1.0f + 0.2f*r/params.radiusRange;
			if (rBestSum*sizeFac > bestSum)
			{
				max = rMax;
				radiusStep = r;
				bestSum = rBestSum * sizeFac;
				center = positionMin+params.positionStep*rBest.cast<float>();
				LOG(LCameraBlob, LDebug, "Found new temp best circle for radius %f (%d) is center (%f, %f), %d votes, sum of %d", radius, r, center.x(), center.y(), rMax, rBestSum);
			}
			else
				LOG(LCameraBlob, LDebug, "Is not new temp best circle for radius %f (%d) with %d votes, sum of %d", radius, r, rMax, rBestSum);
			if (bestSum > (edge.size()*5)/((float)1.5f + ((float)r)/params.radiusRange)) // TODO: Check with cnl, division is different, might need remainder
			{ // Average over 5
				LOG(LCameraBlob, LTrace, "Selected circle for radius %f (%d) with %d votes, sum of %d as best with %d > %f",
					radius, r, max, bestSum, bestSum, (edge.size()*5)/((float)1.2f + ((float)r)/params.radiusRange));
				break;
			}
			else
				LOG(LCameraBlob, LDebug, "Did NOT select temp best circle for radius %f (%d) with %d votes, sum of %d as best with %f <= %f",
					radius, r, max, bestSum, bestSum/5.0f, edge.size()/(1.5f + (((float)r)/params.radiusRange*1.0f)));
		}
	}

	LOG(LCameraBlob, LDebug, "Checked %d / %d radius levels!", r, params.radiusRange);

	return max > 0 && bestSum > 0;
}

int iterativeRefinement(Vector2<float> &center, float &radius, const std::vector<Vector2<float>> &edge, std::vector<bool> &outliers, int iterations)
{
	// Iteratively adjust to average distance, discard edge points that significantly increased in distance

	static std::vector<float> edgeDistSq;
	edgeDistSq.clear();
	edgeDistSq.resize(edge.size(), 0.0f);

	float radiusSq = 0.0;
	int perimeterCnt = 0;
	for (int it = 0; it < iterations; it++)
	{
		// Get new best estimate for the radius
		float sumRSq = 0.0;
		perimeterCnt = 0;
		for (int i = 0; i < edge.size(); i++)
		{
			if (!outliers[i])
			{
				edgeDistSq[i] = (edge[i]-center).squaredNorm();
				sumRSq += edgeDistSq[i];
				perimeterCnt++;
			}
		}
		radiusSq = sumRSq/perimeterCnt;

		// Find limit (based on standard deviation)
		float varRSq = 0.0;
		for (int i = 0; i < edge.size(); i++)
		{
			if (!outliers[i])
			{
				float dR = edgeDistSq[i]-radiusSq;
				varRSq += dR * dR;
			}
		}
		varRSq = varRSq/perimeterCnt;
		float limitSq = varRSq*2;

		// Find new outliers and improve subpixel position based on new perimeter
		Vector2<float> move(0, 0);
		perimeterCnt = 0;
		for (int i = 0; i < edge.size(); i++)
		{
			if (!outliers[i] && std::abs(edgeDistSq[i] - radiusSq) < limitSq)
			{
				float lerp = edgeDistSq[i]/radiusSq - 1;
				move += (edge[i]-center) * lerp;
				perimeterCnt++;
			}
			else
				outliers[i] = true;
		}
		move /= perimeterCnt;

		LOG(LCameraBlob, LDebug, "Iteration: Radius is %f +- %f, position is corrected by %fpx (%f, %f), using %d/%d edge points!",
			radius, std::sqrt(varRSq), move.norm(), move.x(), move.y(), perimeterCnt, (int)edge.size());

		center += move;
	}
	radius = std::sqrt(radiusSq);
	return perimeterCnt;
}