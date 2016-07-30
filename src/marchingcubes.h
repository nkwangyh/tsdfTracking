/*
 * marchingcubes.h
 *
 *  Created on: Feb 3, 2013
 *      Author: erikbylow
 */

#ifndef MARCHINGCUBES_H_
#define MARCHINGCUBES_H_

#include <QGLViewer/qglviewer.h>
#include <Eigen/Dense>
#include <stdlib.h>
#include <vector>
#include "parameters.h"

using namespace std;
using namespace Eigen;
using namespace qglviewer;

#ifndef _TRIANGLES_
#define _TRIANGLES_
class Triangle {
public:
	Eigen::Vector3d points[3];
	Eigen::Vector3d normals[3];
	Eigen::Vector3d colors[3];

	Triangle() {
	}
	;
	Triangle(const Vector3d& p1, const Vector3d &p2, const Vector3d &p3,
			const Vector3d& n1, const Vector3d &n2, const Vector3d& n3,
			const Vector3d& c1, const Vector3d& c2, const Vector3d &c3) {
		this->points[0] = p1;
		this->points[1] = p2;
		this->points[2] = p3;

		this->normals[0] = n1;
		this->normals[1] = n2;
		this->normals[2] = n3;

		this->colors[0] = c1;
		this->colors[1] = c2;
		this->colors[2] = c3;

	}
};

#endif


#ifndef _EDGES_
#define _EDGES_
struct Edges {
	int index[6];
	Vector3d point;
	Vector3d normal;
	Vector3d color;
};
#endif
class Marchingcubes : public QGLViewer {
protected:
	virtual void draw();
private:
	vector<Triangle> triangles;
	vector<Matrix4d> pose;
	vector<Matrix4d> ground;
	vector<Vector4d> cloud;
	vector<Vector3d> points;
	Edges edge[12];
	float *Distance, *Red, *Green,*Blue, *Normals, *Vertices, *Colors;
	void computeTriangles(int index);
	void loadNormals();
	void loadVertices();
	void loadColors();
	void clear();
	void drawGroundtruth();
	Vector3d getColor(int x1, int y1, int z1, int x2, int y2, int z2,
			const Vector3d& p);
	bool drawGround;
	bool drawPoseEstimation;
	bool drawSurface;
	bool drawthePoints;
public:
	int cVertices(int i, int j, int k);
	Marchingcubes();
	Marchingcubes(float* Distance, float *Red, float *Green, float *Blue);
	void marchingcubes();
	void drawTriangles();
	void InitQGL();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	void setPose(const vector<Matrix4d>& pose);
	void setCloud(const vector<Vector4d>& cloud);
	void setGround(const vector<Matrix4d>& ground);
	void drawNormals(const vector<Vector3d>& n, const vector<Vector3d>& p );
	void drawPoints();
	void setPoints(const vector<Vector3d>& points);

};

#endif /* MARCHINGCUBES_H_ */
