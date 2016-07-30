/*
 * RenderSurface.h
 *
 *  Created on: Sep 23, 2013
 *      Author: erikbylow
 */

#ifndef RENDERSURFACE_H_
#define RENDERSURFACE_H_
#include "Voxelgrid.h"
#include "Tables.h"
#include "parameters.h"

struct Triangle {
	Eigen::Vector3d points[3];
	Eigen::Vector3d normals[3];
	Eigen::Vector3d colors[3];
};

#ifndef _EDGES_
#define _EDGES_
struct Edges {
	int index[6];
	Eigen::Vector3d point;
	Eigen::Vector3d normal;
	Eigen::Vector3d color;
};
#endif

class RenderSurface {

	struct gradient {
		float d1, dx1, dy1, dz1;
		float d2, dx2, dy2, dz2;
	};
	struct corners {
		Index I1, I2;
	};
	struct derivative {
		Index Idx, Idy, Idz;
	};
private:
	bool DOCOLOR = true;
	std::vector<Eigen::Vector3d> normals, basepoint;
	Voxelgrid *grid;
	float *Vertices, *Normals, *Colors;
	void loadVertices();
	void loadNormals();
	int cVertices();
	gradient getDistances(corners &c, derivative &d1, derivative &d2, GridDisc &disc);
	std::vector<Triangle> triangles;
	Edges edge[12];
	void loadTriangles(GridDisc &disc, point &start);
	Eigen::Vector3d	interpolate(Index &I1, Index &I2, Data *voxel, point &start);
	void computeTriangles(int cubeindex, Edges *edge);
	Eigen::Vector3d getColor(const point &start, const Index &I0, const Index &I1, const GridDisc &disc, const Eigen::Vector3d &p);
public:
	RenderSurface(Voxelgrid *grid);
	virtual ~RenderSurface();
	void ExtractSurface();
	std::vector<Triangle> getMesh();
	std::vector<Eigen::Vector3d> getNormals();
	std::vector<Eigen::Vector3d> getBasepoints();

};

#endif /* RENDERSURFACE_H_ */
