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

#include "sharedGL.hpp"

#include <map>
#include <array>
#include <cmath>

#define GLSL(str) (const char*)"#version 330 core\n" #str


/* Variables */

static int initCount = 0;

Mesh *coordinateOriginMesh, *xyPlaneMesh, *cameraMesh, *cubePointMesh, *icosahedronMesh, *spherePointMesh;

ShaderProgram *flatUniformColorShader, *flatVertColorShader, *flatTexShader;
ShaderProgram *flatRoundPointShader, *flatSquarePointShader;
ShaderProgram *imageShader, *undistortTexShader, *undistortAlgShader;
ShaderProgram *skyShader;

GLint skyTimeAdr, skySunAdr; // For skyShader
GLint roundSizeAdr; // For flatRoundPointShader
GLint squareSizeAdr; // For flatSquarePointShader


/* Functions */

static void subdivide_icosphere(std::vector<float> &vertices, std::vector<unsigned int> &triangles);

static void initSharedMeshes()
{
	coordinateOriginMesh = new Mesh({ POS, COL }, {
		0, 0, 0, 1, 0, 0,
		1, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 1, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1,
		0, 0, 1, 0, 0, 1,
	}, {}, GL_LINES);

	xyPlaneMesh = new Mesh({ POS, TEX }, {
		-1,  1, 0, 0, 1,
		 1,  1, 0, 1, 1,
		-1, -1, 0, 0, 0,
		 1,  1, 0, 1, 1,
		 1, -1, 0, 1, 0,
		-1, -1, 0, 0, 0,
	}, {}, GL_TRIANGLES);

	cameraMesh = new Mesh({ POS }, {
		-0.0264,  0.02, 0,
		-0.0264, -0.02, 0,
		 0.0264,  0.02, 0,
		 0.0264, -0.02, 0,
		-0.04224,  0.032, 0.06,
		-0.04224, -0.032, 0.06,
		 0.04224,  0.032, 0.06,
		 0.04224, -0.032, 0.06,
		-0.1056,  0.08, 0.16,
		-0.1056, -0.08, 0.16,
		 0.1056,  0.08, 0.16,
		 0.1056, -0.08, 0.16
	}, {
		0, 1, 3, 2,

		0, 1, 5, 4,
		2, 3, 7, 6,
		0, 2, 6, 4,
		1, 3, 7, 5,

		4, 5, 9, 8,
		6, 7, 11, 10,
		4, 6, 10, 8,
		5, 7, 11, 9,

		8, 9, 11, 10
	}, GL_QUADS);

	cubePointMesh = new Mesh({ POS }, {
		0, 0, 1,
		0, 0, -1,
		1, 0, 0,
		-1, 0, 0,
		0, 1, 0,
		0, -1, 0
	}, {
		0, 2, 4,
		0, 2, 5,
		0, 3, 5,
		0, 3, 4,
		1, 2, 4,
		1, 2, 5,
		1, 3, 5,
		1, 3, 4,
	}, GL_TRIANGLES);

	const float X = 0.525731112119133606f;
	const float Z = 0.850650808352039932f;
	const float O = 0.0f;
	std::vector<float> icosphereVerts = {
		-X,  O,  Z,
		 X,  O,  Z,
		-X,  O, -Z,
		 X,  O, -Z,
		 O,  Z,  X,
		 O,  Z, -X,
		 O, -Z,  X,
		 O, -Z, -X,
		 Z,  X,  O,
		-Z,  X,  O,
		 Z, -X,  O,
		-Z, -X,  O
	};
	std::vector<unsigned int> icosphereTris = {
		0,4,1,
		0,9,4,
		9,5,4,
		4,5,8,
		4,8,1,
		8,10,1,
		8,3,10,
		5,3,8,
		5,2,3,
		2,7,3,
		7,10,3,
		7,6,10,
		7,11,6,
		11,0,6,
		0,1,6,
		6,1,10,
		9,0,11,
		9,11,2,
		9,2,5,
		7,2,11
	};
	icosahedronMesh = new Mesh({ POS }, icosphereVerts, icosphereTris, GL_TRIANGLES);

	subdivide_icosphere(icosphereVerts, icosphereTris);
	subdivide_icosphere(icosphereVerts, icosphereTris);
	spherePointMesh = new Mesh({ POS }, icosphereVerts, icosphereTris, GL_TRIANGLES);
}

static void subdivide_icosphere(std::vector<float> &vertices, std::vector<unsigned int> &triangles)
{
	std::map<std::pair<int, int>, int> lookup;
	auto vertex_for_edge = [&lookup, &vertices](int v0, int v1)
	{
		std::pair<int, int> key(v0, v1);
		if (key.first > key.second)
			std::swap(key.first, key.second);

		auto inserted = lookup.insert({key, (int)vertices.size()/3});
		if (inserted.second)
		{
			float d1 = vertices[v0*3+0] + vertices[v1*3+0];
			float d2 = vertices[v0*3+1] + vertices[v1*3+1];
			float d3 = vertices[v0*3+2] + vertices[v1*3+2];
			float fac = 1.0f/std::sqrt(d1*d1 + d2*d2 + d3*d3);
			vertices.push_back(d1*fac);
			vertices.push_back(d2*fac);
			vertices.push_back(d3*fac);
		}

		return inserted.first->second;
	};

	std::vector<unsigned int> newTriangles;
	for (int t = 0; t < triangles.size(); t += 3)
	{
		std::array<int, 3> mid;
		for (int edge = 0; edge < 3; ++edge)
			mid[edge] = vertex_for_edge(triangles[t+edge], triangles[t + (edge+1)%3]);

		newTriangles.push_back(triangles[t+0]);
		newTriangles.push_back(mid[0]);
		newTriangles.push_back(mid[2]);

		newTriangles.push_back(triangles[t+1]);
		newTriangles.push_back(mid[1]);
		newTriangles.push_back(mid[0]);

		newTriangles.push_back(triangles[t+2]);
		newTriangles.push_back(mid[2]);
		newTriangles.push_back(mid[1]);

		newTriangles.push_back(mid[0]);
		newTriangles.push_back(mid[1]);
		newTriangles.push_back(mid[2]);
	}
	std::swap(triangles, newTriangles);
}

static void initSharedShaders()
{
	flatUniformColorShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
		}
	),
	GLSL(
		uniform vec4 col;
		out vec4 FragColor;
		void main(){
			FragColor = col;
		}
	));

	flatVertColorShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 1) in vec4 vCol;
		out vec4 vertCol;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
			vertCol = vCol;
		}
	),
	GLSL(
		in vec4 vertCol;
		out vec4 FragColor;
		void main(){
			FragColor = vertCol;
		}
	));

	flatTexShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 2) in vec2 vTex;
		out vec2 uv;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
			uv = vTex;
		}
	),
	GLSL(
		uniform vec4 col;
		uniform sampler2D image;
		in vec2 uv;
		out vec4 FragColor;
		void main(){
			FragColor = texture(image, uv).r * col;
		}
	));

	imageShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 2) in vec2 vTex;
		out vec2 uv;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
			uv = vTex;
		}
	),
	GLSL(
		uniform vec4 col;
		uniform vec4 adjust;
		uniform sampler2D image;
		in vec2 uv;
		out vec4 FragColor;
		void main(){
			float value = texture(image, uv).r;
			value = (value + adjust.r) * adjust.g;
			FragColor = value * col;
			FragColor.a += adjust.a; // Optional - allows for full control over alpha
		}
	));

	undistortTexShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 2) in vec2 vTex;
		out vec2 uv;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
			uv = vTex;
		}
	),
	GLSL(
		uniform vec4 col;
		uniform vec4 adjust;
		uniform sampler2D image;
		uniform sampler2D undistortTex;
		in vec2 uv;
		uniform vec4 bounds;
		uniform vec2 size;
		out vec4 FragColor;

		void main(){
			vec2 pt = texture(undistortTex, uv).rg;
			pt.y = -pt.y;
			pt = (pt / size + vec2(1,1)) / 2;
			pt.x = (pt.x - bounds.x) / (bounds.z-bounds.x);
			pt.y = (pt.y - bounds.y) / (bounds.w-bounds.y);
			//if (pt.x < 0 || pt.y < 0 || pt.x > 1 || pt.y > 1)
			//	discard; // Not necessary right now
			float value = texture(image, pt).r;
			value = (value + adjust.r) * adjust.g;
			FragColor = value * col;
			FragColor.a += adjust.a; // Optional - allows for full control over alpha
		}
	));

	undistortAlgShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 2) in vec2 vTex;
		out vec2 uv;
		uniform mat4 proj;
		uniform mat4 model;
		void main(){
			gl_Position = proj * model * vec4(vPos.xyz, 1.0);
			uv = vTex;
		}
	),
	GLSL(
		uniform vec4 col;
		uniform vec4 adjust;
		uniform sampler2D image;
		in vec2 uv;
		uniform vec2 uvScale = vec2(1.0, 1.0);
		uniform vec4 bounds;
		uniform vec2 size;
		uniform vec2 principal;
		uniform vec3 distK;
		uniform vec2 distP;
		out vec4 FragColor;

		vec2 undistort(vec2 pt){
			float x = pt.x-principal.x;
			float y = pt.y-principal.y;
			float xsq = x*x;
			float ysq = y*y;
			float rsq = xsq + ysq;
			float rd = 1 + rsq * (distK.x + rsq * (distK.y + (rsq*distK.z)));
			float dx = 2*distP.x*x*y + distP.y*(rsq+2*xsq);
			float dy = distP.x*(rsq+2*ysq) + 2*distP.y*x*y;
			float xd = x*rd + dx;
			float yd = y*rd + dy;
			return vec2(xd+principal.x, yd+principal.y);
		}

		vec2 distort(vec2 pt){
			float x = pt.x-principal.x;
			float y = pt.y-principal.y;
			float xu = x;
			float yu = y;
			for (int i = 0; i < 100; i++)
			{ // This iteration may not converge for some distortion values
				float xsq = x*x;
				float ysq = y*y;
				float rsq = xsq + ysq;
				float rd = 1 + rsq * (distK.x + rsq * (distK.y + (rsq*distK.z)));
				float dx = 2*distP.x*x*y + distP.y*(rsq+2*xsq);
				float dy = distP.x*(rsq+2*ysq) + 2*distP.y*x*y;
				x = (xu - dx) / rd;
				y = (yu - dy) / rd;
			}
			return vec2(x+principal.x, y+principal.y);
		}

		void main(){
			vec2 pt = (uv * 2 - vec2(1,1)) * uvScale;
			pt = distort(pt);
			pt.y = -pt.y;
			pt = (pt / size + vec2(1,1)) / 2;
			pt.x = (pt.x - bounds.x) / (bounds.z-bounds.x);
			pt.y = (pt.y - bounds.y) / (bounds.w-bounds.y);
			//if (pt.x < 0 || pt.y < 0 || pt.x > 1 || pt.y > 1)
			//	discard; // Not necessary right now
			float value = texture(image, pt).r;
			value = (value + adjust.r) * adjust.g;
			FragColor = value * col;
			FragColor.a += adjust.a; // Optional - allows for full control over alpha
		}
	));

	// Shaders for use with VisPoint

	flatSquarePointShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 1) in vec4 vCol;
		layout (location = 3) in float vSize;
		out vec4 vertCol;
		uniform mat4 proj;
		uniform float sizeFactor;
		void main(){
			gl_Position = proj * vec4(vPos.xyz, 1.0);
			gl_PointSize = vSize*sizeFactor;
			vertCol = vCol;
		}
	),
	GLSL(
		in vec4 vertCol;
		out vec4 FragColor;
		void main(){
			FragColor = vertCol;
		}
	));
	squareSizeAdr = glGetUniformLocation(flatSquarePointShader->ID, "sizeFactor");

	flatRoundPointShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		layout (location = 1) in vec4 vCol;
		layout (location = 3) in float vSize;
		out vec4 vertCol;
		uniform mat4 proj;
		uniform float sizeFactor;
		void main(){
			gl_Position = proj * vec4(vPos.xyz, 1.0);
			gl_PointSize = vSize*sizeFactor;
			vertCol = vCol;
		}
	),
	GLSL(
		in vec4 vertCol;
		out vec4 FragColor;
		void main(){
			if (length(gl_PointCoord - vec2(0.5)) > 0.5)
				discard;
			FragColor = vertCol;
		}
	));
	roundSizeAdr = glGetUniformLocation(flatRoundPointShader->ID, "sizeFactor");

	skyShader = new ShaderProgram(
	GLSL(
		layout (location = 0) in vec3 vPos;
		out vec3 pos;
		out vec3 fsun;
		uniform mat4 proj;
		uniform mat4 model; // actually view
		uniform float time = 0.0;
		uniform vec2 sunPos;

		void main()
		{
			gl_Position = vec4(vPos.xy, -1.0f, 1.0);
			mat3 view = mat3(model);
			pos = transpose(view) * (inverse(proj) * vec4(vPos.xyz, 1.0)).xyz;
			pos = pos.xzy;
			fsun = vec3(0.0, sunPos.xy);
		}
	),
	GLSL(
		in vec3 pos;
		in vec3 fsun;
		out vec4 FragColor;
		uniform float time = 0.0;
		uniform float cirrus = 0.5;
		uniform float cumulus = 0.5;

		const float Br = 0.0004;
		const float Bm = 0.0004;
		const float g =  0.9995;
		const vec3 nitrogen = vec3(0.650, 0.570, 0.475);
		const vec3 Kr = Br / pow(nitrogen, vec3(4.0));
		const vec3 Km = Bm / pow(nitrogen, vec3(0.84));

		float hash(float n)
		{
			return fract(sin(n) * 43758.5453123);
		}

		float noise(vec3 x)
		{
			vec3 f = fract(x);
			float n = dot(floor(x), vec3(1.0, 157.0, 113.0));
			return mix(mix(mix(hash(n +   0.0), hash(n +   1.0), f.x),
						mix(hash(n + 157.0), hash(n + 158.0), f.x), f.y),
					mix(mix(hash(n + 113.0), hash(n + 114.0), f.x),
						mix(hash(n + 270.0), hash(n + 271.0), f.x), f.y), f.z);
		}

		const mat3 m = mat3(0.0, 1.60,  1.20, -1.6, 0.72, -0.96, -1.2, -0.96, 1.28);
		float fbm(vec3 p)
		{
			float f = 0.0;
			f += noise(p) / 2; p = m * p * 1.1;
			f += noise(p) / 4; p = m * p * 1.2;
			f += noise(p) / 6; p = m * p * 1.3;
			f += noise(p) / 12; p = m * p * 1.4;
			f += noise(p) / 24;
			return f;
		}

		void main()
		{
			float horizonStart = -0.4;
			float horizonEnd = 0.1;
			if (pos.y < horizonStart)
				discard;

			FragColor.a = smoothstep(0, 1, clamp((pos.y-horizonStart)/(horizonEnd-horizonStart), 0, 1));

			// Atmosphere Scattering
			float mu = dot(normalize(pos), normalize(fsun));
			float rayleigh = 3.0 / (8.0 * 3.14) * (1.0 + mu * mu);
			vec3 mie = (Kr + Km * (1.0 - g * g) / (2.0 + g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5)) / (Br + Bm);

			vec3 day_extinction = exp(-exp(-((pos.y + fsun.y * 4.0) * (exp(-pos.y * 16.0) + 0.1) / 80.0) / Br) * (exp(-pos.y * 16.0) + 0.1) * Kr / Br) * exp(-pos.y * exp(-pos.y * 8.0 ) * 4.0) * exp(-pos.y * 2.0) * 4.0;
			vec3 night_extinction = vec3(1.0 - exp(fsun.y)) * 0.2;
			vec3 extinction = mix(day_extinction, night_extinction, -fsun.y * 0.2 + 0.5);
			FragColor.rgb = rayleigh * mie * clamp(extinction, 0, 2.3);

			// Cirrus Clouds
			float density = smoothstep(1.0 - cirrus, 1.0, fbm(pos.xyz / pos.y * 2.0 + time * 0.05)) * 0.3;
			FragColor.rgb = mix(FragColor.rgb, extinction * 4.0, density * max(pos.y, 0.0));

			// Cumulus Clouds
			for (int i = 0; i < 3; i++)
			{
				float density = smoothstep(1.0 - cumulus, 1.0, fbm((0.7 + float(i) * 0.01) * pos.xyz / pos.y + time * 0.3));
				FragColor.rgb = mix(FragColor.rgb, extinction * density * 5.0, min(density, 1.0) * max(pos.y, 0.0));
			}

			// Dithering Noise
			FragColor.rgb += noise(pos * 1000) * 0.01;
		}
	));
	skyTimeAdr = glGetUniformLocation(skyShader->ID, "time");
	skySunAdr = glGetUniformLocation(skyShader->ID, "sunPos");
}

void initSharedGL()
{
	initCount++;
	if (initCount > 1)
		return;

	initSharedMeshes();
	initSharedShaders();
}

void cleanSharedGL()
{
	initCount--;
	if (initCount > 0)
		return;

	delete flatUniformColorShader;
	delete flatVertColorShader;
	delete flatTexShader;
	delete flatSquarePointShader;
	delete flatRoundPointShader;

	delete coordinateOriginMesh;
	delete xyPlaneMesh;
	delete cameraMesh;
	delete cubePointMesh;
	delete icosahedronMesh;
	delete spherePointMesh;
}