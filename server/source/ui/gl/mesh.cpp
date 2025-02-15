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

#include "mesh.hpp"

#include <cmath>
#include <cstdio>

#include "util/log.hpp"


static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3; // Sync with shader.cpp

inline unsigned int PackedFloats (VertexType type)
{
	unsigned int val = 0;
	if ((type & POS) != 0) val += 3;
	if ((type & COL) != 0) val += 3;
	if ((type & TEX) != 0) val += 2;
	if ((type & NRM) != 0) val += 3;
	return val;
}

Mesh::Mesh(const std::vector<VertexType> &Packing, const std::vector<float> &Vertices, const std::vector<unsigned int> &Elements, GLenum Mode)
{
	// Calculate Floats per vertex as specified by packing
	packing = Packing;
	type = NONE;
	for (int i = 0; i < packing.size(); i++)
		type = type | packing[i];
	FpV = PackedFloats(type);
	vertexCount = (unsigned int)std::floor((double)Vertices.size()/FpV);
	elementCount = (unsigned int)Elements.size();
//	std::cout << "Mesh (type " << type << ", FpV " << FpV << "): " << vertexCount << " vertices (" << Vertices.size() << " floats) / " << elementCount << " elements\n";

	// Setup vertex buffer
	glGenBuffers(1, &VBO_ID);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_ID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Setup elements buffer
	EBO_ID = 0;
	if (elementCount > 0)
	{
		glGenBuffers(1, &EBO_ID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_ID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * elementCount, &Elements[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	mode = Mode;
}

Mesh::~Mesh(void)
{
	glDeleteBuffers(1, &VBO_ID);
	glDeleteBuffers(1, &EBO_ID);
}

void Mesh::updateVertexData(const std::vector<float> &Vertices, bool stream)
{
	vertexCount = (unsigned int)std::floor((double)Vertices.size()/FpV);

	// Setup vertex buffer
	glBindBuffer(GL_ARRAY_BUFFER, VBO_ID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * Vertices.size(), &Vertices[0], stream? GL_STREAM_DRAW : GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Mesh::updateElementData(const std::vector<unsigned int> &Elements, bool stream)
{
	elementCount = (unsigned int)Elements.size();

	// Setup element buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_ID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * Elements.size(), &Elements[0], stream? GL_STREAM_DRAW : GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::prepare(void)
{
	glBindBuffer(GL_ARRAY_BUFFER, VBO_ID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_ID);

	// Setup vertex attributes
	unsigned int offset = 0;
	for (int i = 0; i < packing.size(); i++)
	{
		unsigned int packFloats = PackedFloats(packing[i]);
		GLint adr = 0;
		if (packing[i] == POS) adr = vPosAdr;
		else if (packing[i] == COL) adr = vColAdr;
		else if (packing[i] == TEX) adr = vUVAdr;
		else if (packing[i] == NRM) adr = vNrmAdr;
		else {
			LOG(LGUI, LError, "Unknown vertex packing %d!\n", packing[i]);
			continue;
		}
		glVertexAttribPointer(adr, packFloats, GL_FLOAT, GL_FALSE, sizeof(float) * FpV, (void *)(sizeof(float) * offset));
		glEnableVertexAttribArray(adr);
		offset += packFloats;
	}
}
void Mesh::cleanup(void)
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::drawPart(void)
{
	if (EBO_ID == 0)
		glDrawArrays(mode, 0, vertexCount);
	else
		glDrawElements(mode, elementCount, GL_UNSIGNED_INT, 0);
}

void Mesh::draw(void)
{
	prepare();

	if (EBO_ID == 0)
		glDrawArrays(mode, 0, vertexCount);
	else
		glDrawElements(mode, elementCount, GL_UNSIGNED_INT, 0);

	cleanup();
}

void Mesh::setMode (GLenum Mode)
{
	mode = Mode;
}