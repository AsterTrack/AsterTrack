/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef VISUALISATION_H
#define VISUALISATION_H

#include "util/eigendef.hpp"

#include <vector>


/* Structures */

struct Color;

#pragma pack(push, 1)
struct Color8
{
	uint8_t r, g, b, a = 255;

	inline operator Color() const;
};
struct Color
{
	float r, g, b, a = 1.0f;

	inline operator Color8() const;
};
#pragma pack(pop)

inline Color8::operator Color() const { return Color{ r/255.0f, g/255.0f, b/255.0f, a/255.0f }; }
inline Color::operator Color8() const { return Color8{ (uint8_t)(r*255), (uint8_t)(g*255), (uint8_t)(b*255), (uint8_t)(a*255) }; }


static inline Color lerp(Color a, Color b, float val)
{
	Color col;
	if (val < 0.0f) val = 0.0f;
	else if (val > 1.0f) val = 1.0f;
	col.r = b.r*val + a.r*(1.0f-val);
	col.g = b.g*val + a.g*(1.0f-val);
	col.b = b.b*val + a.b*(1.0f-val);
	col.a = b.a*val + a.a*(1.0f-val);
	return col;
}

#pragma pack(push, 1)
struct VisPoint
{
	Eigen::Vector3f pos;
	Color8 color;
	float size;
};
#pragma pack(pop)

// OpenGL Points are weirdly larger than they ought to be
const static float pointSizeCorrection = 1.0f/1.2f;

// For use as a scene overlay
struct SceneLabel
{
	Eigen::Vector3f position;
	float radius;
	std::string text;
	Color color;
	bool toggle = false;
};
struct SceneButton
{
	Eigen::Vector3f position;
	float radius;
	Color color;
	void* context;
	std::function<void(void*)> callback;
};


/* Functions */

void initVisualisation();
void cleanVisualisation();

std::array<float,2> getPointSizeRange();

// Setup projection space
void visSetupView(const Eigen::Projective3f &projection, const Eigen::Isometry3f &view, Eigen::Vector2i viewport);
void visSetupCamera(const Eigen::Isometry3f &postProjection, const CameraCalib &calib, const CameraMode &mode, Eigen::Vector2i viewport);
void visSetupProjection(const Eigen::Isometry3f &projection, Eigen::Vector2i viewport);

/*
 * Visualisation functions for both 2D and 3D views
 */

/**
 * Render skybox with natural sky shader
 */
void visualiseSkybox(float time = 50.0f);

/**
 * Render xy-floor plane at z = 0
 */
void visualiseFloor(Color color = { 0.15f, 0.4f, 0.50f, 1.0f });

/**
 * Update VBO with data of points
 */
void updatePointsVBO(unsigned int &VBO, const std::vector<VisPoint> &points);

/**
 * Render vertices in VBO as round or square sprite points
 */
void visualisePointsVBOSprites(unsigned int VBO, unsigned int count, bool round = true, float sizeFactor = 1.0f);

/**
 * Render points as round or square 2D sprites
 */
void visualisePointsSprites(const std::vector<VisPoint> &points, bool round = true);

/**
 * Render points as round 3D meshes
 */
void visualisePointsSpheres(const std::vector<VisPoint> &points);

/**
 * Render points while accounting for depth for correct alpha blending
 */
void visualisePointsSpheresDepthSorted(const std::vector<VisPoint> &points);

/**
 * Render segmented 3D lines
 */
void visualiseLines(const std::vector<std::pair<VisPoint, VisPoint>> &lines, float size = 2.0f);

/**
 * Render a continuous 3D line
 */
void visualiseLines(const std::vector<VisPoint> &lines, float size = 2.0f);

/**
 * Render a mesh with in given OpenGL mode
 */
void visualiseMesh(const std::vector<VisPoint> &vertices, unsigned int mode);

/**
 * Render representative camera model
 */
void visualiseCamera(Eigen::Isometry3f camera, Color color = { 0.3f, 0.3f, 0.3f, 1.0f });

/**
 * Render coordinate origin at pose
 */
void visualisePose(const Eigen::Isometry3f &pose, Color color, float scale, float lineWidth);


/*
 * Visualisation functions for 2D views exclusively
 */

/**
 * Upload a grayscale frame into GPU memory for rendering. Pass 0 to frame to create a new handle.
 */
void loadGrayscaleFrame(unsigned int &frame, const uint8_t *data, int width, int height);

/**
 * Upload a 2D vector field into GPU memory, e.g. for quick undistortion in the shader. Pass 0 to frame to create a new handle.
 */
void loadVectorField(unsigned int &frame, const std::vector<Eigen::Vector2f> &data, int width, int height);

/**
 * Delete a frame texture denoted by its handle
 */
void deleteFrameTexture(unsigned int frame);

/**
 * Render a previously uploaded grayscale frame denoted by its handle
 */
void showGrayscaleFrame(unsigned int frame, const Eigen::Affine3f &projection,
	Color color = {1,1,1,0}, float alpha = 1.0f, float brightness = 0.0f, float contrast = 1.0f);

/**
 * Render a previously uploaded grayscale frame denoted by its handle
 */
void showGrayscaleFrameUndistorted(unsigned int frame, unsigned int undistortionTex,
	const Bounds2f &bounds, const CameraMode &mode,
	Eigen::Vector2f viewportSize = Eigen::Vector2f(1.0f, 1.0f),
	Color color = {1,1,1,0}, float alpha = 1.0f, float brightness = 0.0f, float contrast = 1.0f);

/**
 * Render a previously uploaded grayscale frame denoted by its handle
 */
void showGrayscaleFrameUndistorted(unsigned int frame,
	const Bounds2f &bounds, const CameraMode &mode, const CameraCalib &calib,
	Eigen::Vector2f viewportSize = Eigen::Vector2f(1.0f, 1.0f),
	Color color = {1,1,1,0}, float alpha = 1.0f, float brightness = 0.0f, float contrast = 1.0f);


/**
 * Render blob circle with cross in the middle
 */
void showCircleWithCenter(Eigen::Vector2f pos, float size, Color color, float crossSize = 1/100.0f);

/**
 * Render a solid ellipse
 */
void showSolidEllipse(Eigen::Vector2f pos, Eigen::Vector2f size, Color color);

/**
 * Render 2D points
 */
void visualisePoints2D(const std::vector<Eigen::Vector2f> &points2D, Color color, float size = 6.0f, float depth = 0.9f, bool round = true);

/**
 * Render 2D points
 */
template<class it_type>
void visualisePoints2D(it_type pts_begin, it_type pts_end, Color color, float size = 6.0f, float depth = 0.9f, bool round = true);

/**
 * Update VBO with a grid of pixels mapping from colorA to colorB around center for square point rendering
 */
void updatePixelVBO(unsigned int &VBO, const std::vector<uint8_t> &pixels2D, Eigen::Vector2i sizeImg, Eigen::Vector2f center, float pixelStride, float pixelSize, Color colorA, Color colorB, float depth = 0.9f);


#endif // VISUALISATION_H