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

#ifndef MESH_H
#define MESH_H

struct wl_display;
struct wl_resource;
#include "GL/glew.h"

#include <vector>

/**
 * An OOP-implementation of a flexible GL mesh object
 * Can flexibly set any vertex data and draw in many modes
 */


/* Structures and Definitions */

// Type of packed data for vertex attributes
enum VertexType
{
	NONE = 0,
	POS = 1,
	COL = 2,
	TEX = 4,
	NRM = 8,
};
inline VertexType operator | (VertexType a, VertexType b)
{
	return static_cast<VertexType>(static_cast<int>(a) | static_cast<int>(b));
}

/*
 * Generic Mesh structure allowing for arbitrary vertex data packing
 */
class Mesh
{
	public:
	GLuint VBO_ID, EBO_ID;
	VertexType type;
	GLenum mode;
	std::vector<VertexType> packing;
	unsigned int FpV, vertexCount, elementCount;

	Mesh(const std::vector<VertexType> &Packing, const std::vector<float> &Vertices, const std::vector<unsigned int> &Elements, GLenum mode = GL_TRIANGLES);
	~Mesh(void);
	void updateVertexData(const std::vector<float> &Vertices, bool stream = false);
	void updateElementData(const std::vector<unsigned int> &Elements, bool stream = false);
	void prepare(void);
	void cleanup(void);
	void drawPart(void);
	void draw(void);
	void setMode(GLenum mode);
};

#endif // MESH_H
