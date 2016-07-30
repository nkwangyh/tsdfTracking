/*
 * visualDebug.cpp
 *
 *  Created on: Sep 13, 2013
 *      Author: erikbylow
 */

#include <GL/glew.h>
#include "Viewer.h"

using namespace std;

void drawPoints(vector<point> &points) {

	if (points.size() == 0) {
		cout << "No points to draw!" << endl;
	} else {
		for (int i = 0; i < points.size(); ++i) {
			glPointSize(0.50);
			glBegin(GL_POINTS);
			point tmp = points[i];
			glVertex3f(tmp.x, tmp.y, tmp.z);
			glColor3f(1, 0, 0);
			glEnd();
		}
	}
}
point getCoord(Index &tmp, float flength, point &start) {
	point p = { start.x + tmp.i * flength, start.y + tmp.j * flength, start.z
			+ tmp.k * flength };
	return p;
}
void drawFinegrid(GridDisc &disc, point &start, float flength) {
	for (int i = 0; i < fResolution; ++i) {
		for (int j = 0; j < fResolution; ++j) {
			for (int k = 0; k < fResolution; ++k) {
				Index t1 = { i, j, k };
				Index t2 = { i + 1, j, k };
				Index t3 = { i + 1, j + 1, k };
				Index t4 = { i, j + 1, k };

				Index t5 = { i, j, k + 1 };
				Index t6 = { i + 1, j, k + 1 };
				Index t7 = { i + 1, j + 1, k + 1 };
				Index t8 = { i, j + 1, k + 1 };

				point c1 = getCoord(t1, flength, start);
				point c2 = getCoord(t2, flength, start);
				point c3 = getCoord(t3, flength, start);
				point c4 = getCoord(t4, flength, start);

				point c5 = getCoord(t5, flength, start);
				point c6 = getCoord(t6, flength, start);
				point c7 = getCoord(t7, flength, start);
				point c8 = getCoord(t8, flength, start);

				glBegin(GL_LINES);
				glColor3f(0, 1, 0);
				glVertex3f(c1.x, c1.y, c1.z);
				glVertex3f(c2.x, c2.y, c2.z);

				glVertex3f(c2.x, c2.y, c2.z);
				glVertex3f(c3.x, c3.y, c3.z);

				glVertex3f(c3.x, c3.y, c3.z);
				glVertex3f(c4.x, c4.y, c4.z);

				glVertex3f(c4.x, c4.y, c4.z);
				glVertex3f(c1.x, c1.y, c1.z);

				glVertex3f(c5.x, c5.y, c5.z);
				glVertex3f(c6.x, c6.y, c6.z);

				glVertex3f(c6.x, c6.y, c6.z);
				glVertex3f(c7.x, c7.y, c7.z);

				glVertex3f(c7.x, c7.y, c7.z);
				glVertex3f(c8.x, c8.y, c8.z);

				glVertex3f(c8.x, c8.y, c8.z);
				glVertex3f(c5.x, c5.y, c5.z);

				glVertex3f(c1.x, c1.y, c1.z);
				glVertex3f(c5.x, c5.y, c5.z);

				glVertex3f(c2.x, c2.y, c2.z);
				glVertex3f(c6.x, c6.y, c6.z);

				glVertex3f(c3.x, c3.y, c3.z);
				glVertex3f(c7.x, c7.y, c7.z);

				glVertex3f(c4.x, c4.y, c4.z);
				glVertex3f(c8.x, c8.y, c8.z);
				glEnd();

			}
		}
	}
}

void Viewer::drawCoarseGrid(point &start, float length) {
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);

	glVertex3f(start.x, start.y, start.z);
	glVertex3f(start.x + length, start.y, start.z);

	glVertex3f(start.x + length, start.y, start.z);
	glVertex3f(start.x + length, start.y + length, start.z);

	glVertex3f(start.x + length, start.y + length, start.z);
	glVertex3f(start.x, start.y + length, start.z);

	glVertex3f(start.x, start.y + length, start.z);
	glVertex3f(start.x, start.y, start.z);
	///////////////////////////////////////////////////////////////////
	glVertex3f(start.x, start.y, start.z + length);
	glVertex3f(start.x + length, start.y, start.z + length);

	glVertex3f(start.x + length, start.y, start.z + length);
	glVertex3f(start.x + length, start.y + length, start.z + length);

	glVertex3f(start.x + length, start.y + length, start.z + length);
	glVertex3f(start.x, start.y + length, start.z + length);

	glVertex3f(start.x, start.y + length, start.z + length);
	glVertex3f(start.x, start.y, start.z + length);
	/////////////////////////////////////////////////////////////////
	glVertex3f(start.x, start.y, start.z + length);
	glVertex3f(start.x, start.y, start.z);

	glVertex3f(start.x + length, start.y, start.z + length);
	glVertex3f(start.x + length, start.y, start.z);

	glVertex3f(start.x + length, start.y + length, start.z + length);
	glVertex3f(start.x + length, start.y + length, start.z);

	glVertex3f(start.x, start.y + length, start.z + length);
	glVertex3f(start.x, start.y + length, start.z);
	glEnd();

}
void Viewer::drawVoxelGrid() {
	glBegin(GL_LINE_LOOP);
	glColor3f(1, 0, 0);
	glVertex3f(-x_s, -y_s, -z_s);
	glVertex3f(x_s, -y_s, -z_s);
	glVertex3f(x_s, y_s, -z_s);
	glVertex3f(-x_s, y_s, -z_s);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glColor3f(1, 0, 0);
	glVertex3f(-x_s, -y_s, z_s);
	glVertex3f(x_s, -y_s, z_s);
	glVertex3f(x_s, y_s, z_s);
	glVertex3f(-x_s, y_s, z_s);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glColor3f(1, 0, 0);
	glVertex3f(-x_s, -y_s, -z_s);
	glVertex3f(-x_s, y_s, -z_s);
	glVertex3f(-x_s, y_s, z_s);
	glVertex3f(-x_s, -y_s, z_s);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glColor3f(1, 0, 0);
	glVertex3f(x_s, -y_s, -z_s);
	glVertex3f(x_s, y_s, -z_s);
	glVertex3f(x_s, y_s, z_s);
	glVertex3f(x_s, -y_s, z_s);
	glEnd();

	for (int i = 0; i < cResolution; ++i) {
		for (int j = 0; j < cResolution; ++j) {
			for (int k = 0; k < cResolution; ++k) {
				GridDisc tmp = grid->getDisc(i, j, k);
				point start = { -x_s + i * grid->getclength(), -y_s + j
						* grid->getclength(), -z_s + k * grid->getclength() };
				if (tmp.isOccupied) {

					//										drawFinegrid(tmp, start, grid->getflength());
					//					glPointSize(5.0);
					//					glBegin(GL_POINTS);
					//					glColor3f(0, 0, 1);
					//					glVertex3f(start.x, start.y, start.z);
					//					glEnd();
				}
				if (tmp.isOccupied) {
					drawCoarseGrid(start, grid->getclength());
				}
			}
		}
	}
}
void drawNegFine(const GridDisc &disc, point &start, float flength) {
	for (int i = 0; i < fResolution; ++i) {
		for (int j = 0; j < fResolution; ++j) {
			for (int k = 0; k < fResolution; ++k) {
				if (abs(disc.operator ()(i, j, k)) < DELTA - 0.01
						&& disc.getWeight(i, j, k) > 0) {
					point p = { start.x + i * flength, start.y + j * flength,
							start.z + k * flength };
					glBegin(GL_POINTS);
					glColor3f(1, 0, 0);
					glVertex3f(p.x, p.y, p.z);
					glEnd();
				}
			}
		}
	}
}

//void drawNegCoarse(const GridDisc &disc, point &start, float clength) {
//	for (int i = 0; i < 2; ++i) {
//		for (int j = 0; j < 2; ++j) {
//			for (int k = 0; k < 2; ++k) {
//				if (disc.cdata[i + 2 * j + 4 * k].distance > 0 && disc.getWeight(i,j,k) > 0) {
//
//					point p = { start.x + i * clength, start.y + j * clength,
//							start.z + k * clength };
//					glBegin(GL_POINTS);
//					glVertex3f(p.x, p.y, p.z);
//					glEnd();
//				}
//			}
//		}
//	}
//}
void Viewer::drawNegative() {
	float clength = grid->getclength();
	float flength = grid->getflength();
	for (int i = 0; i < cResolution; ++i) {
		for (int j = 0; j < cResolution; ++j) {
			for (int k = 0; k < cResolution; ++k) {
				point start = { -x_s + i * clength, -y_s + j * clength, -z_s
						+ k * clength };
				if (grid->getDisc(i, j, k).isOccupied) {
					drawNegFine(grid->getDisc(i, j, k), start, flength);
				} else {
					//					drawNegCoarse(grid->getDisc(i, j, k), start, clength);
				}
			}
		}
	}
}

void Viewer::drawSlowTriangles(std::vector<Triangle> &triangle) {
	for (int i = 0; i < triangle.size(); ++i) {
		Eigen::Vector3d p1 = triangle[i].points[0];
		Eigen::Vector3d p2 = triangle[i].points[1];
		Eigen::Vector3d p3 = triangle[i].points[2];

		Eigen::Vector3d n1 = triangle[i].normals[0];
		Eigen::Vector3d n2 = triangle[i].normals[1];
		Eigen::Vector3d n3 = triangle[i].normals[2];

		float color = 0.8;
		Eigen::Vector3d c1(color,color,color);
		Eigen::Vector3d c2(color, color, color);
		Eigen::Vector3d c3(color, color, color);

		if (DOCOLOR){
			Eigen::Vector3d c1 = triangle[i].colors[0];
			Eigen::Vector3d c2 = triangle[i].colors[1];
			Eigen::Vector3d c3 = triangle[i].colors[2];
		}

		glBegin(GL_TRIANGLES);
		glColor3f(c1(0), c1(1), c1(2));
		glNormal3f(n1(0), n1(1), n1(2));
		glVertex3f(p1(0), p1(1), p1(2));

		glColor3f(c2(0), c2(1), c2(2));
		glNormal3f(n2(0), n2(1), n2(2));
		glVertex3f(p2(0), p2(1), p2(2));

		glColor3f(c3(0), c3(1), c3(2));
		glNormal3f(n3(0), n3(1), n3(2));
		glVertex3f(p3(0), p3(1), p3(2));
		glEnd();
	}
}

/*GLuint bufferPoints;
GLuint bufferNormals;
GLuint bufferColors;
void Viewer::drawTriangles() {

	glGenBuffers(1, &bufferPoints);
	glGenBuffers(1, &bufferNormals);
	glGenBuffers(1, &bufferColors);

	glBindBuffer(GL_ARRAY_BUFFER, bufferPoints);

	//Upload vertex data to the video device
	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
			Vertices, GL_STATIC_DRAW);

	//Draw Triangle from VBO - do each time window, view point or data changes
	//Establish its 3 coordinates per vertex with zero stride in this array; necessary here
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	//Establish array contains vertices (not normals, colours, texture coords etc)
	glEnableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, bufferNormals);

	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
			Normals, GL_STATIC_DRAW);

	glNormalPointer(GL_FLOAT, 0, NULL);// One Buffer: Normals (address)

	glEnableClientState(GL_NORMAL_ARRAY);

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

		glEnableClientState(GL_COLOR_ARRAY);

	}
	//
	glDrawArrays(GL_TRIANGLES, 0, 3 * triangles.size());

	glDeleteBuffers(1, &bufferNormals);
	glDeleteBuffers(1, &bufferPoints);
	glDeleteBuffers(1, &bufferColors);
	//	cout << "Time to draw the triangles: " << t << endl;
}*/

void drawPose(const std::vector<Eigen::Matrix4d>& pose) {
	Eigen::Vector4d e_x(0.1, 0, 0, 1);
	Eigen::Vector4d e_y(0, 0.1, 0, 1);
	Eigen::Vector4d e_z(0, 0, 0.1, 1);

	Eigen::Vector4d tmp;
	Eigen::Matrix4d Camera;
	for (int i = 0; i < pose.size(); ++i) {
		Camera = pose[i];
		tmp = Camera * e_x;
		glBegin(GL_LINES);
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

void drawGroundtruth(std::vector<Eigen::Matrix4d> &ground) {

	Eigen::Vector4d e_x(0.1, 0, 0, 1);
	Eigen::Vector4d e_y(0, 0.1, 0, 1);
	Eigen::Vector4d e_z(0, 0, 0.1, 1);

	Eigen::Vector4d tmp;
	Eigen::Matrix4d Camera;
	for (int i = 0; i < ground.size(); ++i) {
		Camera = ground[i];
		tmp = Camera * e_x;
		glBegin(GL_LINES);
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

void drawNormals(const std::vector<Eigen::Vector3d>& normals,
		const std::vector<Eigen::Vector3d> &basepoint) {
	for (int i = 0; i < normals.size(); ++i) {
		glBegin(GL_LINES);
		glVertex3f(basepoint[i](0), basepoint[i](1), basepoint[i](2));
		glVertex3f(basepoint[i](0) + 0.1 * normals[i](0),
				basepoint[i](1) + 0.1 * normals[i](1),
				basepoint[i](2) + 0.1 * normals[i](2));
		glEnd();
	}
}
GLuint bufferPoints;
GLuint bufferNormals;
GLuint bufferColors;
void Viewer::drawTriangles() {

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
	glEnableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, bufferNormals);

	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float) * triangles.size(),
			Normals, GL_STATIC_DRAW);

	glNormalPointer(GL_FLOAT, 0, NULL); // One Buffer: Normals (address)

	glEnableClientState(GL_NORMAL_ARRAY);

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

		glEnableClientState(GL_COLOR_ARRAY);
	}
	//
	glDrawArrays(GL_TRIANGLES, 0, 3 * triangles.size());

	glDeleteBuffers(1, &bufferNormals);
	glDeleteBuffers(1, &bufferPoints);
	glDeleteBuffers(1, &bufferColors);
	//	cout << "Time to draw the triangles: " << t << endl;

}
void Viewer::draw() {
	if (drawpoints) {
		drawPoints(points);
	}
	if (voxelGrid) {
		drawVoxelGrid();
	}
	if (drawnegative) {
		drawNegative();
	}
	//
	if (surface) {
		drawTriangles();

		//drawSlowTriangles(this->triangles);
	}
	if (drawpose) {
		drawPose(this->pose);
	}
	if (drawtruth) {
		drawGroundtruth(this->groundtruth);
	}
	if (drawnormals) {
		drawNormals(this->normals, this->basepoint);
	}
}

void Viewer::setGrid(Voxelgrid *grid) {
	this->grid = grid;
}
void Viewer::init() {
	// Restore previous viewer state.
	this->surface = false;
	this->voxelGrid = false;
	this->drawpose = false;
	this->drawtruth = false;
	this->drawnegative = false;
	this->drawpoints = false;
	this->drawnormals = false;
	restoreStateFromFile();
	this->camera()->setZClippingCoefficient(30);
	glClearColor(0, 0, 0, 1.0f);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
//	glEnable(GL_CULL_FACE);
//	glCullFace(GL_BACK);

}

void Viewer::setPoints(const vector<point> &points) {
	this->points = points;
}

void Viewer::setTriangles(std::vector<Triangle> tmp) {
	this->triangles = tmp;
	loadVertices();
	loadNormals();
	loadColors();
}

void Viewer::loadVertices() {
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

void Viewer::loadNormals() {
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

void Viewer::loadColors() {
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

void Viewer::setPose(std::vector<Eigen::Matrix4d> &pose) {
	this->pose = pose;
}

void Viewer::setGroundtruth(std::vector<Eigen::Matrix4d> &ground) {
	this->groundtruth = ground;
}

void Viewer::keyPressEvent(QKeyEvent *e) {
	if (e->key() == Qt::Key_V) {
		voxelGrid = !voxelGrid;
		updateGL();
	}
	if (e->key() == Qt::Key_S) {
		surface = !surface;
		updateGL();
	}
	if (e->key() == Qt::Key_E) {
		drawpose = !drawpose;
		updateGL();
	}
	if (e->key() == Qt::Key_T) {
		drawtruth = !drawtruth;
		updateGL();
	}
	if (e->key() == Qt::Key_N) {
		drawnormals = !drawnormals;
		updateGL();
	}
	if (e->key() == Qt::Key_P) {
		drawpoints = !drawpoints;
		updateGL();
	}
}

float* Viewer::getVertices() {
	return Vertices;
}

float* Viewer::getColors() {
	return Colors;
}

void Viewer::setNormals(const std::vector<Eigen::Vector3d> &normals,
		const std::vector<Eigen::Vector3d> &basepoint) {
	this->basepoint = basepoint;
	this->normals = normals;
}
