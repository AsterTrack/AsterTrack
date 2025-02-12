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

#include "shader.hpp"
#include "util/log.hpp"

static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3; // Sync with mesh.cpp

/*
 * Load shader text from file and compile
 */
static GLuint loadShader(const std::string &source, int type)
{
	GLuint shader = glCreateShader(type);
	if (shader == 0)
	{
		LOG(LGUI, LError, "Failed to create shader! Check context!\n");
		return 0;
	}
	glCompileShader(shader);
	const char *src = source.c_str();
	glShaderSource(shader, 1, &src, NULL);
	glCompileShader(shader);
	int success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		char infoLog[512];
		glGetShaderInfoLog(shader, 512, NULL, infoLog);
		LOG(LGUI, LError, "Failed to compile shader:\n%s\n", infoLog);
		LOG(LGUI, LError, "Source:\n%s\n", src);
		return 0;
	}
	return shader;
}

ShaderProgram::ShaderProgram(const std::string &vertSource, const std::string &fragSource)
{
	ID = 0;
	GLuint vertShader = loadShader(vertSource, GL_VERTEX_SHADER);
	GLuint fragShader = loadShader(fragSource, GL_FRAGMENT_SHADER);
	if (vertShader != 0 && fragShader != 0)
	{
		ID = glCreateProgram();
		glAttachShader(ID, vertShader);
		glAttachShader(ID, fragShader);
		// Vertex data, bound to fixed channels
		glBindAttribLocation(ID, vPosAdr, "vPos");
		glBindAttribLocation(ID, vColAdr, "vCol");
		glBindAttribLocation(ID, vUVAdr, "vTex");
		glBindAttribLocation(ID, vNrmAdr, "vNrm");
		// Link
		glLinkProgram(ID);
		int success;
		glGetProgramiv(ID, GL_LINK_STATUS, &success);
		if (!success)
		{
			char infoLog[512];
			glGetProgramInfoLog(ID, 512, NULL, infoLog);
			LOG(LGUI, LError, "Failed to link shader program:\n%s\n", infoLog);
			ID = 0;
		}
		else
		{
			uImageAdr = glGetUniformLocation(ID, "image");
			uColorAdr = glGetUniformLocation(ID, "col");
			uProjAdr = glGetUniformLocation(ID, "proj");
			uModelAdr = glGetUniformLocation(ID, "model");
		}
	}
	if (vertShader != 0) glDeleteShader(vertShader);
	if (fragShader != 0) glDeleteShader(fragShader);
}

ShaderProgram::~ShaderProgram ()
{
	glDeleteProgram(ID);
}

void ShaderProgram::use (void)
{
	glUseProgram(ID);
}