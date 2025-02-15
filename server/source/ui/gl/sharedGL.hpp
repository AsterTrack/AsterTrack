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

#ifndef SHARED_GL_H
#define SHARED_GL_H

#include "shader.hpp"
#include "mesh.hpp"

#include <vector>

extern Mesh *coordinateOriginMesh, *xyPlaneMesh, *cameraMesh, *cubePointMesh, *icosahedronMesh, *spherePointMesh, *smoothSphereMesh;
extern std::vector<float> icosphereVerts;
extern std::vector<unsigned int> icosphereTris;
extern ShaderProgram *flatUniformColorShader, *flatVertColorShader, *flatTexShader;
extern ShaderProgram *flatRoundPointShader, *flatSquarePointShader;
extern ShaderProgram *imageShader, *undistortTexShader, *undistortAlgShader;
extern ShaderProgram *skyShader;

extern GLint skyTimeAdr, skySunAdr; // For skyShader
extern GLint roundSizeAdr; // For flatRoundPointShader
extern GLint squareSizeAdr; // For flatSquarePointShader

void initSharedGL();
void cleanSharedGL();

void subdivide_icosphere(std::vector<float> &vertices, std::vector<unsigned int> &triangles);

#endif // SHARED_GL_H