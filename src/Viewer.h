/*
 * visualDebug.h
 *
 *  Created on: Sep 13, 2013
 *      Author: erikbylow
 */

#ifndef VISUALDEBUG_H_
#define VISUALDEBUG_H_
#define GL_GLEXT_PROTOTYPES

#include <QGLViewer/qglviewer.h>
#include <QKeyEvent>
#include "Voxelgrid.h"
#include "RenderSurface.h"
#include <vector>
class Viewer: public QGLViewer {
private:
	float *Vertices, *Normals, *Colors;
	Voxelgrid *grid;
	std::vector<point> points;
	std::vector<Triangle> triangles;
	void loadVertices();
	void loadNormals();
	void loadColors();
	void drawTriangles();
	std::vector<Eigen::Matrix4d> pose, groundtruth;
	bool voxelGrid, surface, drawpose, drawtruth, drawpoints, drawnegative, drawnormals;
	std::vector<Eigen::Vector3d> normals, basepoint;
	bool DOCOLOR = true;
	void drawSlowTriangles(std::vector<Triangle> &triangle);
	void drawNegative();
	void drawVoxelGrid();
	void drawCoarseGrid(point &start, float length);
protected:
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);

public:
	void setPoints(const std::vector<point> &points);
	void setGrid(Voxelgrid *grid);
	void setTriangles(const std::vector<Triangle> tmp);
	void setPose(std::vector<Eigen::Matrix4d> &pose);
	void setGroundtruth(std::vector<Eigen::Matrix4d> &ground);
	float* getVertices();
	float* getColors();
	void setNormals(const std::vector<Eigen::Vector3d> &normals, const std::vector<Eigen::Vector3d> &basepoint);

};

#endif /* VISUALDEBUG_H_ */
