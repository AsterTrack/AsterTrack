/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "ui/ui.hpp"


void updateEmulationVis(std::shared_ptr<BlobEmulationVis> &vis, const std::shared_ptr<BlobEmulationResults> &result, std::vector<SceneLabel> &labels);

void updateEmulationVisUI(CameraVisState &visCamera);

void updateEmulationVisualisation(const TrackingCameraState &camera, CameraVisState &visCamera, const CameraFrameRecord &frame, Eigen::Vector2i viewSize);

Bounds2i getValidMaskRect(uint32_t width, uint32_t height);


#endif // INTEGRATION_H