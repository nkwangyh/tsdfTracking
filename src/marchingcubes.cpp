/*
 * marchingcubes.cpp
 *
 *  Created on: Feb 3, 2013
 *      Author: erikbylow
 */
#define GL_GLEXT_PROTOTYPES

#include <QGLViewer/qglviewer.h>
#include <qapplication.h>
#include <qobject.h>
#include <QKeyEvent>
#include "marchingcubes.h"
#include "Tables.h"
#include <iostream>
//#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QPainter>
#include <QGLWidget>
#include <stdio.h>
#include <stdlib.h>
//#include <Eigen/Geometry>

using namespace std;
using namespace qglviewer;

Marchingcubes::Marchingcubes() {
}
;
Marchingcubes::Marchingcubes(float *Distance, float *Red, float *Green,
		float *Blue) {
	this->Distance = Distance;
	this->Red = Red;
	this->Green = Green;
	this->Blue = Blue;
	this->drawGround = false;
	this->drawPoseEstimation = false;
	this->drawSurface = false;
	this->drawthePoints = false;
}

Vector4d getGlobal(int i, int j, int k) {
	return Vector4d(-x_s + i * xstep, -y_s + j * ystep, -z_s + k * zstep, 1);
}

Vector3d calcNormal(int x1, int y1, int z1, const Vector3d &p, float *Distance) {
	Vector3d normal;
	if (x1 > 0 && y1 > 0 && z1 > 0 && x1 < RESOLUTION - 1 && y1 < RESOLUTION
			- 1 && z1 < RESOLUTION - 1) {
		normal(0) = Distance[(x1 - 1) + y1 * RESOLUTION + z1 * RESOLUTION
				* RESOLUTION] - Distance[(x1 + 1) + y1 * RESOLUTION + z1
				* RESOLUTION * RESOLUTION];

		normal(1) = Distance[x1 + (y1 - 1) * RESOLUTION + z1 * RESOLUTION
				* RESOLUTION] - Distance[x1 + (y1 + 1) * RESOLUTION + z1
				* RESOLUTION * RESOLUTION];

		normal(2) = Distance[x1 + y1 * RESOLUTION + (z1 - 1) * RESOLUTION
				* RESOLUTION] - Distance[x1 + y1 * RESOLUTION + (z1 + 1)
				* RESOLUTION * RESOLUTION];
		double norm = normal.norm();
		if (norm > 0) {
			return (1 / norm) * normal;
		}
		return normal;
	}
	normal = p;
	//cout << "Boundary when calculating normal, returning p instead" << endl;
	double norm = sqrt(sqr(normal(0)) + sqr(normal(1)) + sqr(normal(2)));
	if (norm > 0) {
		normal = (1 / norm) * normal;
	}
	return normal;

}

Vector3d getNormal(int x1, int y1, int z1, int x2, int y2, int z2,
		const Vector3d& p, float *Distance) {

	Vector3d normal;

	Vector4d p1 = getGlobal(x1, y1, z1);

	Vector3d n1 = calcNormal(x1, y1, z1, p, Distance);
	Vector3d n2 = calcNormal(x2, y2, z2, p, Distance);

	float w = sqrt(sqr(p(0)-p1(0)) + sqr(p(1)-p1(1)) + sqr(p(2)-p1(2)));

	normal(0) = (xstep - w) * n1(0) + (w) * n2(0);
	normal(1) = (ystep - w) * n1(1) + (w) * n2(1);
	normal(2) = (zstep - w) * n1(2) + (w) * n2(2);
	normal.normalize();

	return normal;

}

Vector3d interpolate(int i1, int j1, int k1, int i2, int j2, int k2,
		float *Distance) {
	Vector3d p;

	float v1 = Distance[k1 * RESOLUTION * RESOLUTION + j1 * RESOLUTION + i1];
	float v2 = Distance[k2 * RESOLUTION * RESOLUTION + j2 * RESOLUTION + i2];

	Vector4d tmp1 = getGlobal(i1, j1, k1); // Global coordinates for (i1,j1,k1)
	Vector4d tmp2 = getGlobal(i2, j2, k2);

	Vector3d p1(tmp1(0), tmp1(1), tmp1(2)); // Same as tmp1
	Vector3d p2(tmp2(0), tmp2(1), tmp2(2));

	if (fabs(ISOVALUE - v1) < 0.000001) {
		return p1;
	}
	if (fabs(ISOVALUE - v2) < 0.000001) {
		return p2;
	}
	if (fabs(v1 - v2) < 0.0000001) {
		return p1;
	}

	double mu = (ISOVALUE - v1) / (v2 - v1);

	p(0) = p1(0) + mu * (p2(0) - p1(0));
	p(1) = p1(1) + mu * (p2(1) - p1(1));
	p(2) = p1(2) + mu * (p2(2) - p1(2));

	//getchar();
	return p;
}

int Marchingcubes::cVertices(int i, int j, int k) {
	int cubeindex = 0;

	if (Distance[k * RESOLUTION * RESOLUTION + (j + 1) * RESOLUTION + (i + 1)]
			> ISOVALUE) {
		cubeindex |= 1;
	}
	if (Distance[k * RESOLUTION * RESOLUTION + j * RESOLUTION + (i + 1)]
			> ISOVALUE) {
		cubeindex |= 2;
	}
	if (Distance[k * RESOLUTION * RESOLUTION + j * RESOLUTION + i] > ISOVALUE) {
		cubeindex |= 4;
	}
	if (Distance[k * RESOLUTION * RESOLUTION + (j + 1) * RESOLUTION + i]
			> ISOVALUE) {
		cubeindex |= 8;
	}
	if (Distance[(k + 1) * RESOLUTION * RESOLUTION + (j + 1) * RESOLUTION + i
			+ 1] > ISOVALUE) {
		cubeindex |= 16;
	}
	if (Distance[(k + 1) * RESOLUTION * RESOLUTION + j * RESOLUTION + i + 1]
			> ISOVALUE) {
		cubeindex |= 32;
	}
	if (Distance[(k + 1) * RESOLUTION * RESOLUTION + j * RESOLUTION + i]
			> ISOVALUE) {
		cubeindex |= 64;
	}
	if (Distance[(k + 1) * RESOLUTION * RESOLUTION + (j + 1) * RESOLUTION + i]
			> ISOVALUE) {
		cubeindex |= 128;
	}

	return cubeindex;
}

void Marchingcubes::computeTriangles(int index) {
	Vector3d p1, p2, p3, normal1, normal2, normal3;
	Vector3d c1, c2, c3;
	for (int i = 0; triTable[index][i] != -1; i += 3) {

		p1 = edge[triTable[index][i]].point;
		p2 = edge[triTable[index][i + 1]].point;
		p3 = edge[triTable[index][i + 2]].point;

		normal1 = edge[triTable[index][i]].normal;
		normal2 = edge[triTable[index][i + 1]].normal;
		normal3 = edge[triTable[index][i + 2]].normal;

		c1 = edge[triTable[index][i]].color;
		c2 = edge[triTable[index][i + 1]].color;
		c3 = edge[triTable[index][i + 2]].color;
		Triangle tmp(p1, p2, p3, normal1, normal2, normal3, c1, c2, c3);
		triangles.push_back(tmp);
	}

}

void Marchingcubes::loadColors() {
	Colors = (float*) malloc(sizeof(float) * 9 * triangles.size());
	int j = 0;

	for (int i = 0; i < this->triangles.size(); ++i) {
		Colors[j] = (triangles[i].colors[0])[0];
		Colors[j + 1] = (triangles[i].colors[0])[1];
		Colors[j + 2] = (triangles[i].colors[0])[2];

		Colors[j + 3] = (triangles[i].colors[1])[0];
		Colors[j + 4] = (triangles[i].colors[1])[1];
		Colors[j + 5] = (triangles[i].colors[1])[2];

		Colors[j + 6] = (triangles[i].colors[2])[0];
		Colors[j + 7] = (triangles[i].colors[2])[1];
		Colors[j + 8] = (triangles[i].colors[2])[2];

		j += 9;
	}
}

void Marchingcubes::loadVertices() {

	Vertices = (float*) malloc(sizeof(float) * 9 * triangles.size());

	int j = 0;

	for (int i = 0; i < this->triangles.size(); i++) {

		Vertices[j] = (triangles[i].points[0])[0];
		Vertices[j + 1] = (triangles[i].points[0])[1];
		Vertices[j + 2] = (triangles[i].points[0])[2];

		Vertices[j + 3] = (triangles[i].points[1])[0];
		Vertices[j + 4] = (triangles[i].points[1])[1];
		Vertices[j + 5] = (triangles[i].points[1])[2];

		Vertices[j + 6] = (triangles[i].points[2])[0];
		Vertices[j + 7] = (triangles[i].points[2])[1];
		Vertices[j + 8] = (triangles[i].points[2])[2];
		j += 9;

	}

}
void Marchingcubes::loadNormals() {

	Normals = (float*) malloc(sizeof(float) * 9 * triangles.size());

	int j = 0;
	for (int i = 0; i < this->triangles.size(); i++) {

		Normals[j] = (triangles[i].normals[0])[0];
		Normals[j + 1] = (triangles[i].normals[0])[1];
		Normals[j + 2] = (triangles[i].normals[0])[2];

		Normals[j + 3] = (triangles[i].normals[1])[0];
		Normals[j + 4] = (triangles[i].normals[1])[1];
		Normals[j + 5] = (triangles[i].normals[1])[2];

		Normals[j + 6] = (triangles[i].normals[2])[0];
		Normals[j + 7] = (triangles[i].normals[2])[1];
		Normals[j + 8] = (triangles[i].normals[2])[2];
		j += 9;

	}

}

Vector3d getIndices(const Vector4d &p) {
	Vector3d tmp;
	tmp(0) = (p(0) + x_s) / xstep;
	tmp(1) = (p(1) + y_s) / ystep;
	tmp(2) = (p(2) + z_s) / zstep;
	return tmp;
}

Vector3d Marchingcubes::getColor(int x1, int y1, int z1, int x2, int y2,
		int z2, const Vector3d& p) {
	//	cout << "p:\n " << p << endl;
	//	cout << "x1, x2, x3: " << x1 << " " << y1 << " " << z1 << endl;
	//	getchar();
	Vector3d tmp(0, 0, 0);
	//	const Vector4d pos = getGlobal(x1,y1,z1);
	const float scale = 1.0;
	const Vector4d F(p(0), p(1), p(2), 1);
	const Vector3d Ind = getIndices(F);

	float w = sqrt(sqr(Ind(0)- x1) + sqr(Ind(1) - y1) + sqr(Ind(2) - z1));

	float red = scale * ((1 - w) * Red[x1 + RESOLUTION * y1 + RESOLUTION
			* RESOLUTION * z1] + w * Red[x2 + y2 * RESOLUTION + z2 * RESOLUTION
			* RESOLUTION]);

	float green = scale * ((1 - w) * Green[x1 + RESOLUTION * y1 + RESOLUTION
			* RESOLUTION * z1] + w * Green[x2 + y2 * RESOLUTION + z2
			* RESOLUTION * RESOLUTION]);
	float blue = scale * ((1 - w) * Blue[x1 + RESOLUTION * y1 + RESOLUTION
			* RESOLUTION * z1] + w * Blue[x2 + y2 * RESOLUTION + z2
			* RESOLUTION * RESOLUTION]);
	tmp[0] = red;
	tmp[1] = green;
	tmp[2] = blue;

	return tmp;
}

void Marchingcubes::marchingcubes() {
	cout << "Start Marchingcubes" << endl;
	//		control(Distance);
	//		getchar();
	this->triangles.clear();

	for (int i = 1; i < RESOLUTION - 1; i++) {
		for (int j = 1; j < RESOLUTION - 1; j++) {
			for (int k = 1; k < RESOLUTION - 1; k++) {

				int cubeindex = cVertices(i, j, k);

				if (cubeindex != 0 && cubeindex != 255) {

					if (edgeTable[cubeindex] & 1) {

						edge[0].point = interpolate(i + 1, j + 1, k, i + 1, j,
								k, this->Distance);

						edge[0].index[0] = i + 1;
						edge[0].index[1] = j + 1;
						edge[0].index[2] = k;
						edge[0].index[3] = i + 1;
						edge[0].index[4] = j;
						edge[0].index[5] = k;
						edge[0].normal = getNormal(i + 1, j + 1, k, i + 1, j,
								k, edge[0].point, this->Distance);
						this->edge[0].color = getColor(i + 1, j + 1, k, i + 1,
								j, k, edge[0].point);

					}

					if (edgeTable[cubeindex] & 2) {
						//Interpolate between vertices 1 and 2

						edge[1].point = interpolate(i + 1, j, k, i, j, k,
								this->Distance);
						edge[1].index[0] = i + 1;
						edge[1].index[1] = j;
						edge[1].index[2] = k;
						edge[1].index[3] = i;
						edge[1].index[4] = j;
						edge[1].index[5] = k;
						edge[1].normal = getNormal(i + 1, j, k, i, j, k,
								edge[1].point, Distance);
						edge[1].color = getColor(i + 1, j, k, i, j, k,
								edge[1].point);

					}

					if (edgeTable[cubeindex] & 4) {
						//Interpolate between vertices 2 and 3

						edge[2].point = interpolate(i, j, k, i, j + 1, k,
								this->Distance);
						edge[2].index[0] = i;
						edge[2].index[1] = j;
						edge[2].index[2] = k;
						edge[2].index[3] = i;
						edge[2].index[4] = j + 1;
						edge[2].index[5] = k;
						edge[2].normal = getNormal(i, j + 1, k, i, j, k,
								edge[2].point, this->Distance); //Opposite direction
						edge[2].color = getColor(i, j + 1, k, i, j, k,
								edge[2].point);

					}

					if (edgeTable[cubeindex] & 8) {
						//Interpolate between vertices 3 and 0

						edge[3].point = interpolate(i, j + 1, k, i + 1, j + 1,
								k, this->Distance);
						edge[3].index[0] = i;
						edge[3].index[1] = j + 1;
						edge[3].index[2] = k;
						edge[3].index[3] = i + 1;
						edge[3].index[4] = j + 1;
						edge[3].index[5] = k;
						edge[3].normal = getNormal(i + 1, j + 1, k, i, j + 1,
								k, edge[3].point, this->Distance); // Opposite direction
						edge[3].color = getColor(i + 1, j + 1, k, i, j + 1, k,
								edge[3].point);

					}

					if (edgeTable[cubeindex] & 16) {
						//Interpolate between vertices 4 and 5

						edge[4].point = (interpolate(i + 1, j + 1, k + 1,
								i + 1, j, k + 1, this->Distance));
									computeTriangles(cubeindex);		edge[4].index[0] = i + 1;
						edge[4].index[1] = j + 1;
						edge[4].index[2] = k + 1;
						edge[4].index[3] = i + 1;
						edge[4].index[4] = j;
						edge[4].index[5] = k + 1;

						edge[4].normal = getNormal(i + 1, j + 1, k + 1, i + 1,
								j, k + 1, edge[4].point, this->Distance);
						edge[4].color = getColor(i + 1, j + 1, k + 1, i + 1, j,
								k + 1, edge[4].point);

					}

					if (edgeTable[cubeindex] & 32) {

						//Interpolate between vertices 5 and 6

						edge[5].point = interpolate(i + 1, j, k + 1, i, j,
								k + 1, this->Distance);
						edge[5].index[0] = i + 1;
						edge[5].index[1] = j;
						edge[5].index[2] = k + 1;
						edge[5].index[3] = i;
						edge[5].index[4] = j;
						edge[5].index[5] = k + 1;
						edge[5].normal = getNormal(i + 1, j, k + 1, i, j,
								k + 1, edge[5].point, this->Distance);
						edge[5].color = getColor(i + 1, j, k + 1, i, j, k + 1,
								edge[5].point);

					}

					if (edgeTable[cubeindex] & 64) {
						//Interpolate between vertices 6 and 7

						edge[6].point = interpolate(i, j, k + 1, i, j + 1,
								k + 1, this->Distance);
						edge[6].index[0] = i;
						edge[6].index[1] = j;
						edge[6].index[2] = k + 1;
						edge[6].index[3] = i;
						edge[6].index[4] = j + 1;
						edge[6].index[5] = k + 1;

						edge[6].normal = getNormal(i, j + 1, k + 1, i, j,
								k + 1, edge[6].point, this->Distance);

						edge[6].color = getColor(i, j + 1, k + 1, i, j, k + 1,
								edge[6].point);

					}

					if (edgeTable[cubeindex] & 128) {
						//Interpolate between vertices 7 and 4

						edge[7].point = interpolate(i, j + 1, k + 1, i + 1,
								j + 1, k + 1, this->Distance);
						edge[7].index[0] = i;
						edge[7].index[1] = j + 1;
						edge[7].index[2] = k + 1;
						edge[7].index[3] = i + 1;
						edge[7].index[4] = j + 1;
						edge[7].index[5] = k + 1;
						edge[7].normal = getNormal(i + 1, j + 1, k + 1, i,
								j + 1, k + 1, edge[7].point, this->Distance);

						edge[7].color = getColor(i + 1, j + 1, k + 1, i, j + 1,
								k + 1, edge[7].point);

					}

					if (edgeTable[cubeindex] & 256) {
						//Interpolate between vertices 0 and 4

						edge[8].point = interpolate(i + 1, j + 1, k, i + 1,
								j + 1, k + 1, this->Distance);
						edge[8].index[0] = i + 1;
						edge[8].index[1] = j + 1;
						edge[8].index[2] = k;
						edge[8].index[3] = i + 1;
						edge[8].index[4] = j + 1;
						edge[8].index[5] = k + 1;
						edge[8].normal = getNormal(i + 1, j + 1, k, i + 1,
								j + 1, k + 1, edge[8].point, this->Distance);

						edge[8].color = getColor(i + 1, j + 1, k, i + 1, j + 1,
								k + 1, edge[8].point);

					}

					if (edgeTable[cubeindex] & 512) {
						//Interpolate between vertices 1 and 5

						edge[9].point = interpolate(i + 1, j, k, i + 1, j,
								k + 1, this->Distance);
						edge[9].index[0] = i + 1;
						edge[9].index[1] = j;
						edge[9].index[2] = k;
						edge[9].index[3] = i + 1;
						edge[9].index[4] = j;
						edge[9].index[5] = k + 1;
						edge[9].normal = getNormal(i + 1, j, k, i + 1, j,
								k + 1, edge[9].point, this->Distance);

						edge[9].color = getColor(i + 1, j, k, i + 1, j, k + 1,
								edge[9].point);

					}

					if (edgeTable[cubeindex] & 1024) {
						//Interpolate between vertices 2 and 6

						edge[10].point = interpolate(i, j, k, i, j, k + 1,
								this->Distance);
						edge[10].index[0] = i;
						edge[10].index[1] = j;
						edge[10].index[2] = k;
						edge[10].index[3] = i;
						edge[10].index[4] = j;
						edge[10].index[5] = k + 1;
						edge[10].normal = getNormal(i, j, k, i, j, k + 1,
								edge[10].point, this->Distance);

						edge[10].color = getColor(i, j, k, i, j, k + 1,
								edge[10].point);

					}

					if (edgeTable[cubeindex] & 2048) {
						//Interpolate between vertices 3 and 7

						edge[11].point = interpolate(i, j + 1, k, i, j + 1,
								k + 1, this->Distance);
						edge[11].index[0] = i;
						edge[11].index[1] = j + 1;
						edge[11].index[2] = k;
						edge[11].index[3] = i;
						edge[11].index[4] = j + 1;
						edge[11].index[5] = k + 1;
						edge[11].normal = getNormal(i, j + 1, k, i, j + 1,
								k + 1, edge[11].point, this->Distance);

						edge[11].color = getColor(i, j + 1, k, i, j + 1, k + 1,
								edge[11].point);

					}

					computeTriangles(cubeindex);

				}

			}

		}
	}
	cout << "Done Marchingcubes" << endl;
	loadVertices();
	loadNormals();
	loadColors();
}

GLuint bufferPoints;
GLuint bufferNormals;
GLuint bufferColors;
void Marchingcubes::drawTriangles() {

	glGenBuffers(1, &bufferPoints);
	glGenBuffers(1, &bufferNormals);

	glBindBuffer(GL_ARRAY_BUFFER, bufferPoints);

	//Upload vertex data to the video device
	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
			Vertices, GL_STATIC_DRAW);

	//Draw Triangle from VBO - do each time window, view point or data changes
	//Establish its 3 coordinates per vertex with zero stride in this array; necessary here
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	//Establish array contains vertices (not normals, colours, texture coords etc)
	glEnableClientState( GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, bufferNormals);

	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
			Normals, GL_STATIC_DRAW);

	glNormalPointer(GL_FLOAT, 0, NULL);// One Buffer: Normals (address)

	glEnableClientState( GL_NORMAL_ARRAY);

	glGenBuffers(1, &bufferColors);
	////

	if (DO_COLOR == 0) {
		glColor3f(1.0, 1.0, 1.0);
	}
	if (DO_COLOR == 1) {

		glBindBuffer(GL_ARRAY_BUFFER, bufferColors);

		glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
				Colors, GL_STATIC_DRAW);

		glColorPointer(3, GL_FLOAT, 0, NULL);

		glEnableClientState( GL_COLOR_ARRAY);
	}
	//
	glDrawArrays(GL_TRIANGLES, 0, 3 * triangles.size());

	glDeleteBuffers(1, &bufferNormals);
	glDeleteBuffers(1, &bufferPoints);
	glDeleteBuffers(1, &bufferColors);
	//	cout << "Time to draw the triangles: " << t << endl;


}

void drawSlowTriangles(int stop, const vector<Triangle> triangles) {
	for (int nbr = 0; nbr <= stop; nbr++) {
		glBegin( GL_TRIANGLES);

		//		glColor3f(1, 1, 1);
		glNormal3f((triangles[nbr].normals[0])[0],
				(triangles[nbr].normals[0])[1], (triangles[nbr].normals[0])[2]);
		glColor3f((triangles[nbr].colors[0])[0], (triangles[nbr].colors[0])[1],
				(triangles[nbr].colors[0])[2]);
		glVertex3f((triangles[nbr].points[0])[0],
				(triangles[nbr].points[0])[1], (triangles[nbr].points[0])[2]);

		glNormal3f((triangles[nbr].normals[1])[0],
				(triangles[nbr].normals[1])[1], (triangles[nbr].normals[1])[2]);
		glColor3f((triangles[nbr].colors[1])[0], (triangles[nbr].colors[1])[1],
				(triangles[nbr].colors[1])[2]);
		glVertex3f((triangles[nbr].points[1])[0],
				(triangles[nbr].points[1])[1], (triangles[nbr].points[1])[2]);

		glNormal3f((triangles[nbr].normals[2])[0],
				(triangles[nbr].normals[2])[1], (triangles[nbr].normals[2])[2]);
		glColor3f((triangles[nbr].colors[2])[0], (triangles[nbr].colors[2])[1],
				(triangles[nbr].colors[2])[2]);
		glVertex3f((triangles[nbr].points[2])[0],
				(triangles[nbr].points[2])[1], (triangles[nbr].points[2])[2]);

		glEnd();

	}

}

void Marchingcubes::setPose(const vector<Matrix4d>& pose) {
	this->pose = pose;
}

void drawPose(const vector<Matrix4d>& pose) {
	Vector4d e_x(0.1, 0, 0, 1);
	Vector4d e_y(0, 0.1, 0, 1);
	Vector4d e_z(0, 0, 0.1, 1);

	Vector4d tmp;
	Matrix4d Camera;
	for (int i = 0; i < pose.size(); ++i) {
		Camera = pose[i];
		tmp = Camera * e_x;
		glBegin( GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));

		tmp = Camera * e_y;
		glColor3f(0, 1, 0);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));

		tmp = Camera * e_z;
		glColor3f(0, 0, 1);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));
		glEnd();
	}

}
void Marchingcubes::InitQGL() {
	Vector4d pos;
	glColor3f(0, 0, 1);
	for (int i = 0; i < RESOLUTION; ++i) {
		for (int j = 0; j < RESOLUTION; ++j) {
			for (int k = 0; k < RESOLUTION; ++k) {
				pos = getGlobal(i, j, k);
				if (this->Distance[k * RESOLUTION * RESOLUTION + j * RESOLUTION
						+ i] > 0) {
					glBegin( GL_POINTS);
					glVertex3f(pos(0), pos(1), pos(2));
					glEnd();
				}
			}
		}
	}

}

void drawCloud(const vector<Vector4d>& cloud) {
	int stop = cloud.size();
	glBegin( GL_POINTS);
	glColor3f(1, 0, 0);
	for (int i = 0; i < stop; ++i) {
		glVertex3f(cloud[i](0), cloud[i](1), cloud[i](2));
	}
	glEnd();
}

void Marchingcubes::setGround(const vector<Matrix4d>& ground) {
	this->ground = ground;
}

void Marchingcubes::drawGroundtruth() {

	Vector4d e_x(0.1, 0, 0, 1);
	Vector4d e_y(0, 0.1, 0, 1);
	Vector4d e_z(0, 0, 0.1, 1);

	Vector4d tmp;
	Matrix4d Camera;
	for (int i = 0; i < this->ground.size(); ++i) {
		Camera = ground[i];
		tmp = Camera * e_x;
		glBegin( GL_LINES);
		glColor3f(1, 1, 0);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));

		tmp = Camera * e_y;
		glColor3f(0, 1, 1);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));

		tmp = Camera * e_z;
		glColor3f(1, 0, 1);
		glVertex3f(Camera(0, 3), Camera(1, 3), Camera(2, 3));
		glVertex3f(tmp(0), tmp(1), tmp(2));
		glEnd();
	}

}
void Marchingcubes::draw() {

	//	InitQGL();
	if (drawSurface) {
		drawTriangles();
	}
	//	int stop = this->triangles.size() - 1;
	//	drawSlowTriangles(stop, this->triangles);
	if (drawPoseEstimation) {
		drawPose(this->pose);
	}
	if (drawGround) {
		drawGroundtruth();
	}
	if (drawthePoints) {
		drawPoints();
	}

}

void Marchingcubes::setCloud(const vector<Vector4d>& cloud) {
	this->cloud = cloud;
}

void Marchingcubes::init() {
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f); // Black --- White Background
	glClearDepth(1.0f);// Depth Buffer Setup
	glEnable( GL_DEPTH_TEST); // Enables Depth Testing
	glDepthFunc( GL_LEQUAL);
	glShadeModel( GL_SMOOTH);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	setAxisIsDrawn();
	this->camera()->setZNearCoefficient(0.001);
	this->camera()->setZClippingCoefficient(20);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		GLfloat light_position[] = { 0, 0, 20, 1 };
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	//	updateGL();
}

void Marchingcubes::keyPressEvent(QKeyEvent *e) {

	if (e->key() == Qt::Key_S) {
		drawSurface = !drawSurface;
		updateGL();
	}
	if (e->key() == Qt::Key_T) {
		drawGround = !drawGround;
		updateGL();
	}
	if (e->key() == Qt::Key_E) {
		drawPoseEstimation = !drawPoseEstimation;
		updateGL();
	}
	if (e->key() == Qt::Key_P){
		drawthePoints = !drawthePoints;
		updateGL();
	}
	if (e->key() == Qt::Key_Escape) {
		exit(0);
	}
}

void Marchingcubes::drawNormals(const vector<Vector3d>& n,
		const vector<Vector3d>& p) {

	const int stop = n.size();

	for (int i = 0; i < stop; ++i) {
		glBegin( GL_LINES);
		glColor3f(1, 0, 0);

		glVertex3f(p[i](0), p[i](1), p[i](2));
		glVertex3f(p[i](0) + n[i](0), p[i](1) + n[i](1), p[i](2) + n[i](2));
		glEnd();
	}
}
void Marchingcubes::setPoints(const vector<Vector3d>& p){
	this->points = p;
}
void Marchingcubes::drawPoints(){
	const int stop = points.size();
	glBegin(GL_POINTS);
	for (int i = 0; i < stop; ++i){
		glColor3f(1,0,0);
		glVertex3f(points[i](0), points[i](1), points[i](2));
	}
	glEnd();
}
