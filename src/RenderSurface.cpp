/*
 * RenderSurface.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: erikbylow
 */

#include "RenderSurface.h"
using namespace std;

RenderSurface::RenderSurface(Voxelgrid *grid) {
	this->grid = grid;
}
RenderSurface::~RenderSurface() {
}

int cvertices(Data *voxel, int i, int j, int k) {
	int cubeindex = 0;
	Index I0 = { i + 1, j + 1, k };
	Index I1 = { i + 1, j, k };
	Index I2 = { i, j, k };
	Index I3 = { i, j + 1, k };

	Index I4 = { i + 1, j + 1, k + 1 };
	Index I5 = { i + 1, j, k + 1 };
	Index I6 = { i, j, k + 1 };
	Index I7 = { i, j + 1, k + 1 };

	if (voxel[I0.i + I0.j * fResolution + I0.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 1;
	}
	if (voxel[I1.i + I1.j * fResolution + I1.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 2;
	}
	if (voxel[I2.i + I2.j * fResolution + I2.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 4;
	}
	if (voxel[I3.i + I3.j * fResolution + I3.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 8;
	}
	if (voxel[I4.i + I4.j * fResolution + I4.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 16;
	}
	if (voxel[I5.i + I5.j * fResolution + I5.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 32;
	}
	if (voxel[I6.i + I6.j * fResolution + I6.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 64;
	}
	if (voxel[I7.i + I7.j * fResolution + I7.k * fResolution * fResolution].distance
			> 0) {
		cubeindex |= 128;
	}
	return cubeindex;
}

Eigen::Vector3d RenderSurface::interpolate(Index &I1, Index &I2, Data *voxel,
		point &start) {
	Eigen::Vector3d p;
	const float v1 = voxel[I1.i + I1.j * fResolution + I1.k * fResolution
			* fResolution].distance;
	const float v2 = voxel[I2.i + I2.j * fResolution + I2.k * fResolution
			* fResolution].distance;

	Eigen::Vector3d p1(start.x + I1.i * grid->getflength(),
			start.y + I1.j * grid->getflength(),
			start.z + I1.k * grid->getflength());
	Eigen::Vector3d p2(start.x + I2.i * grid->getflength(),
			start.y + I2.j * grid->getflength(),
			start.z + I2.k * grid->getflength());

	if (fabs(ISOVALUE - v1) < 0.000001) {
		return p1;
	}
	if (fabs(ISOVALUE - v2) < 0.000001) {
		return p2;
	}
	if (fabs(v1 - v2) < 0.0000001) {
		return p1;
	}

	float mu = (ISOVALUE - v1) / (v2 - v1);
	p(0) = p1(0) + mu * (p2(0) - p1(0));
	p(1) = p1(1) + mu * (p2(1) - p1(1));
	p(2) = p1(2) + mu * (p2(2) - p1(2));

	//getchar();
	return p;

}

void RenderSurface::computeTriangles(int cubeindex, Edges *edge) {

	for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
		Triangle tmp;
		tmp.points[0] = edge[triTable[cubeindex][i]].point;
		tmp.points[1] = edge[triTable[cubeindex][i + 1]].point;
		tmp.points[2] = edge[triTable[cubeindex][i + 2]].point;

		tmp.normals[0] = edge[triTable[cubeindex][i]].normal;
		tmp.normals[1] = edge[triTable[cubeindex][i + 1]].normal;
		tmp.normals[2] = edge[triTable[cubeindex][i + 2]].normal;

		//		tmp.colors[0] = tmp.colors[1] = tmp.colors[2] = {1, 1, 1};
		tmp.colors[0] = edge[triTable[cubeindex][i]].color;
		tmp.colors[1] = edge[triTable[cubeindex][i + 1]].color;
		tmp.colors[2] = edge[triTable[cubeindex][i + 2]].color;
		triangles.push_back(tmp);
	}

}
// Estimating the normals for the zerocrossing
RenderSurface::gradient RenderSurface::getDistances(corners &c,
		derivative &dv1, derivative &dv2, GridDisc &disc) {
	Index I1 = c.I1;
	Index I2 = c.I2;
	float d1 = disc.operator ()(I1.i, I1.j, I1.k);
	float d2 = disc.operator()(I2.i, I2.j, I2.k);

	Index Idx1 = dv1.Idx;
	Index Idy1 = dv1.Idy;
	Index Idz1 = dv1.Idz;

	Index Idx2 = dv2.Idx;
	Index Idy2 = dv2.Idy;
	Index Idz2 = dv2.Idz;

	float dx1 = disc.operator()(Idx1.i, Idx1.j, Idx1.k);
	float dy1 = disc.operator()(Idy1.i, Idy1.j, Idy1.k);
	float dz1 = disc.operator()(Idz1.i, Idz1.j, Idz1.k);

	float dx2 = disc.operator()(Idx2.i, Idx2.j, Idx2.k);
	float dy2 = disc.operator()(Idy2.i, Idy2.j, Idy2.k);
	float dz2 = disc.operator()(Idz2.i, Idz2.j, Idz2.k);

	gradient tmp = { d1, dx1, dy1, dz1, d2, dx2, dy2, dz2 };
	return tmp;
	//	Eigen::Vector3d n1((dx1 - d1)/step, (dy1 - d1)/step, (dz1 - d1)/step);
	//	Eigen::Vector3d n2((dx2 - d2)/step, (dy2 - d2)/ step, (dz2 - d2)/step);
	//
	//	float weight = sqrt(sqr(c.p.x - c.pc.x) + sqr(c.p.y - c.pc.y) + sqr(c.p.z - c.pc.z));
	//	Eigen::Vector3d normal(
	//			(1 - weight) * n1(0) + (weight) * n2(0),
	//			(1 - weight) * n1(1) + (weight) * n2(1),
	//			(1 - weight) * n1(2) + (weight) * n2(2));
	//	return -normal;
}

void RenderSurface::loadVertices() {
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

Eigen::Vector3d RenderSurface::getColor(const point& start, const Index &I0,
		const Index &I1, const GridDisc &disc, const Eigen::Vector3d &p) {
	Eigen::Vector3d tmp(1, 1, 1);

	if (DO_COLOR == 1) {
		float step = this->grid->getflength();
		const point p1 = { start.x + I0.i * step, start.y + I0.j * step,
				start.z + I0.k * step };
		const float weight = sqrt(
				sqr(p(0) - p1.x) + sqr(p(1) - p1.y)
						+ sqr(p(2) - p1.z)) / step;

		const float red = disc.getRed(I0.i, I0.j, I0.k) * (1 - weight)
				+ disc.getRed(I1.i, I1.j, I1.k) * weight;
		const float green = disc.getGreen(I0.i, I0.j, I0.k) * (1 - weight)
				+ disc.getGreen(I1.i, I1.j, I1.k) * weight;
		const float blue = disc.getBlue(I0.i, I0.j, I0.k) * (1 - weight)
				+ disc.getBlue(I1.i, I1.j, I1.k) * weight;

		tmp[0] = red;
		tmp[1] = green;
		tmp[2] = blue;
	}

	return tmp;
}

// Running the Marchingcubes algorithm for the fine grids
void RenderSurface::loadTriangles(GridDisc &disc, point &start) {
	for (int i = 0; i < fResolution - 1; ++i) {
		for (int j = 0; j < fResolution - 1; ++j) {
			for (int k = 0; k < fResolution - 1; ++k) {
				if (disc.fdata[i + j * fResolution + k * fResolution
						* fResolution].weight < 1) {
										//continue;
				}
				int cubeindex = cvertices(disc.fdata, i, j, k);
				if (cubeindex > 0) {
					Edges edge[12];
					Index I0 = { i + 1, j + 1, k };
					Index I1 = { i + 1, j, k };
					Index I2 = { i, j, k };
					Index I3 = { i, j + 1, k };

					Index I4 = { i + 1, j + 1, k + 1 };
					Index I5 = { i + 1, j, k + 1 };
					Index I6 = { i, j, k + 1 };
					Index I7 = { i, j + 1, k + 1 };
					const float flength = grid->getflength();
					// Indices I0-I1
					if (edgeTable[cubeindex] & 1) {

						edge[0].point = interpolate(I0, I1, disc.fdata, start);
						int index1 = I0.i + I0.j * fResolution + I0.k
								* fResolution * fResolution;

						int indexdx1 = I3.i + I3.j * fResolution + I3.k
								* fResolution * fResolution;
						int indexdy1 = I1.i + I1.j * fResolution + I1.k
								* fResolution * fResolution;
						int indexdz1 = I4.i + I4.j * fResolution + I4.k
								* fResolution * fResolution;

						float d1 = disc.fdata[index1].distance;
						float dx1 = disc.fdata[indexdx1].distance;
						float dy1 = disc.fdata[indexdy1].distance;
						float dz1 = disc.fdata[indexdz1].distance;

						int index2 = I1.i + I1.j * fResolution + I1.k
								* fResolution * fResolution;
						int indexdx2 = I2.i + I2.j * fResolution + I2.k
								* fResolution * fResolution;
						int indexdy2 = I0.i + I0.j * fResolution + I0.k
								* fResolution * fResolution;
						int indexdz2 = I5.i + I5.j * fResolution + I5.k
								* fResolution * fResolution;

						float d2 = disc.fdata[index2].distance;
						float dx2 = disc.fdata[indexdx2].distance;
						float dy2 = disc.fdata[indexdy2].distance;
						float dz2 = disc.fdata[indexdz2].distance;

						Eigen::Vector3d p = edge[0].point;
						point p1 = { start.x + I0.i * flength, start.y + I0.j
								* flength, start.z + I0.k * flength };

						Eigen::Vector3d n1((d1 - dx1) / flength,
								(d1 - dy1) / flength, (dz1 - d1) / flength);

						Eigen::Vector3d n2((d2 - dx2) / flength,
								(dy2 - d2) / flength, (dz2 - d2) / flength);
						float weight = sqrt(
								sqr(p1.x - p(0)) + sqr(p1.y - p(1))
										+ sqr(p1.z - p(2))) / flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[0].normal = -normal;
							edge[0].color = getColor(start, I0, I1, disc,
								edge[0].point);
						
				
					}

					// Index I1- I2
					if (edgeTable[cubeindex] & 2) {

						edge[1].point = interpolate(I1, I2, disc.fdata, start);

						int index1 = I1.i + I1.j * fResolution + I1.k
								* fResolution * fResolution;

						int indexdx1 = I2.i + I2.j * fResolution + I2.k
								* fResolution * fResolution;
						int indexdy1 = I0.i + I0.j * fResolution + I0.k
								* fResolution * fResolution;
						int indexdz1 = I5.i + I5.j * fResolution + I5.k
								* fResolution * fResolution;

						float d1 = disc.fdata[index1].distance;
						float dx1 = disc.fdata[indexdx1].distance;
						float dy1 = disc.fdata[indexdy1].distance;
						float dz1 = disc.fdata[indexdz1].distance;

						int index2 = I2.i + I2.j * fResolution + I2.k
								* fResolution * fResolution;

						int indexdx2 = I1.i + I1.j * fResolution + I1.k
								* fResolution * fResolution;
						int indexdy2 = I3.i + I3.j * fResolution + I3.k
								* fResolution * fResolution;
						int indexdz2 = I6.i + I6.j * fResolution + I6.k
								* fResolution * fResolution;

						float d2 = disc.fdata[index2].distance;
						float dx2 = disc.fdata[indexdx2].distance;
						float dy2 = disc.fdata[indexdy2].distance;
						float dz2 = disc.fdata[indexdz2].distance;
						Eigen::Vector3d p = edge[1].point;

						point p1 = { start.x + I1.i * flength, start.y + I1.j
								* flength, start.z + I1.k * flength };

						Eigen::Vector3d n1((d1 - dx1) / flength,
								(dy1 - d1) / flength, (dz1 - d1) / flength);

						Eigen::Vector3d n2((dx2 - d2) / flength,
								(dy2 - d2) / flength, (dz2 - d2) / flength);
						float weight = sqrt(
								sqr(p1.x - p(0)) + sqr(p1.y - p(1))
										+ sqr(p1.z - p(2))) / flength;
						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[1].normal = -normal;
						edge[1].color = getColor(start, I1, I2, disc,
								edge[1].point);

					}
					if (edgeTable[cubeindex] & 4) {

						edge[2].point = interpolate(I2, I3, disc.fdata, start);

						int index1 = I2.i + I2.j * fResolution + I2.k
								* fResolution * fResolution;

						int indexdx1 = I1.i + I1.j * fResolution + I1.k
								* fResolution * fResolution;
						int indexdy1 = I3.i + I3.j * fResolution + I3.k
								* fResolution * fResolution;
						int indexdz1 = I6.i + I6.j * fResolution + I6.k
								* fResolution * fResolution;

						float d1 = disc.fdata[index1].distance;
						float dx1 = disc.fdata[indexdx1].distance;
						float dy1 = disc.fdata[indexdy1].distance;
						float dz1 = disc.fdata[indexdz1].distance;

						int index2 = I3.i + I3.j * fResolution + I3.k
								* fResolution * fResolution;

						int indexdx2 = I0.i + I0.j * fResolution + I0.k
								* fResolution * fResolution;
						int indexdy2 = I2.i + I2.j * fResolution + I2.k
								* fResolution * fResolution;
						int indexdz2 = I7.i + I7.j * fResolution + I7.k
								* fResolution * fResolution;
						Eigen::Vector3d p = edge[2].point;
						point p1 = { start.x + I2.i * flength, start.y + I2.j
								* flength, start.z + I2.k * flength };
						float d2 = disc.fdata[index2].distance;
						float dx2 = disc.fdata[indexdx2].distance;
						float dy2 = disc.fdata[indexdy2].distance;
						float dz2 = disc.fdata[indexdz2].distance;

						Eigen::Vector3d n1((dx1 - d1) / flength,
								(dy1 - d1) / flength, (dz1 - d1) / flength);

						Eigen::Vector3d n2((dx2 - d2) / flength,
								(d2 - dy2) / flength, (dz2 - d2) / flength);
						float weight = sqrt(
								sqr(p1.x - p(0)) + sqr(p1.y - p(1))
										+ sqr(p1.z - p(2))) / flength;
						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[2].normal = -normal;
						edge[2].color = getColor(start, I2, I3, disc,
								edge[2].point);

					}
					if (edgeTable[cubeindex] & 8) {

						edge[3].point = interpolate(I3, I0, disc.fdata, start);
						corners c = { I3, I0 };
						derivative dv1 = { I0, I2, I7 };
						derivative dv2 = { I3, I1, I4 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.dx1 - g.d1) / flength,
								(g.d1 - g.dy1) / flength,
								(g.dz1 - g.d1) / flength);
						Eigen::Vector3d n2((g.d2 - g.dx2) / flength,
								(g.d2 - g.dy2) / flength,
								(g.dz2 - g.d2) / flength);

						point s = { start.x + I3.i * flength, start.y + I3.j
								* flength, start.z + I3.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[3].point(0))
										+ sqr(edge[3].point(1) - s.y)
										+ sqr(edge[3].point(2) - s.z))
								/ flength;

//						cout<<edge[3].point<<endl;
//						getchar();
						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[3].normal = -normal;
						edge[3].color = getColor(start, I3, I0, disc,
								edge[3].point);


					}
					if (edgeTable[cubeindex] & 16) {

						edge[4].point = interpolate(I4, I5, disc.fdata, start);
//						getchar();
						corners c = { I4, I5 };
						derivative dv1 = { I7, I5, I0 };
						derivative dv2 = { I6, I4, I1 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.d1 - g.dx1) / flength,
								(g.d1 - g.dy1) / flength,
								(g.d1 - g.dz1) / flength);
						Eigen::Vector3d n2((g.d2 - g.dx2) / flength,
								(g.dy2 - g.d2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I4.i * flength, start.y + I4.j
								* flength, start.z + I4.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[4].point(0))
										+ sqr(edge[4].point(1) - s.y)
										+ sqr(edge[4].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[4].normal = -normal;
						edge[4].color = getColor(start, I4, I5, disc,
								edge[4].point);


					}
					if (edgeTable[cubeindex] & 32) {

						edge[5].point = interpolate(I5, I6, disc.fdata, start);

						corners c = { I5, I6 };
						derivative dv1 = { I6, I4, I1 };
						derivative dv2 = { I5, I7, I2 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.d1 - g.dx1) / flength,
								(g.dy1 - g.d1) / flength,
								(g.d1 - g.dz1) / flength);
						Eigen::Vector3d n2((g.dx2 - g.d2) / flength,
								(g.dy2 - g.d2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I5.i * flength, start.y + I5.j
								* flength, start.z + I5.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[5].point(0))
										+ sqr(edge[5].point(1) - s.y)
										+ sqr(edge[5].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[5].normal = -normal;
						edge[5].color = getColor(start, I5, I6, disc,
								edge[5].point);



					}
					if (edgeTable[cubeindex] & 64) {

						edge[6].point = interpolate(I6, I7, disc.fdata, start);

						corners c = { I6, I7 };
						derivative dv1 = { I5, I7, I2 };
						derivative dv2 = { I4, I6, I3 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.dx1 - g.d1) / flength,
								(g.dy1 - g.d1) / flength,
								(g.d1 - g.dz1) / flength);
						Eigen::Vector3d n2((g.dx2 - g.d2) / flength,
								(g.d2 - g.dy2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I6.i * flength, start.y + I6.j
								* flength, start.z + I6.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[6].point(0))
										+ sqr(edge[6].point(1) - s.y)
										+ sqr(edge[6].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[6].normal = -normal;
						edge[6].color = getColor(start, I6, I7, disc,
								edge[6].point);
					}
					if (edgeTable[cubeindex] & 128) {

						edge[7].point = interpolate(I7, I4, disc.fdata, start);

						corners c = { I7, I4 };
						derivative dv1 = { I4, I6, I3 };
						derivative dv2 = { I7, I5, I0 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.dx1 - g.d1) / flength,
								(g.d1 - g.dy1) / flength,
								(g.d1 - g.dz1) / flength);
						Eigen::Vector3d n2((g.d2 - g.dx2) / flength,
								(g.d2 - g.dy2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I7.i * flength, start.y + I7.j
								* flength, start.z + I7.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[7].point(0))
										+ sqr(edge[7].point(1) - s.y)
										+ sqr(edge[7].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[7].normal = -normal;
						edge[7].color = getColor(start, I7, I4, disc,
								edge[7].point);
					}
					if (edgeTable[cubeindex] & 256) {

						edge[8].point = interpolate(I0, I4, disc.fdata, start);

						corners c = { I0, I4 };
						derivative dv1 = { I3, I1, I4 };
						derivative dv2 = { I7, I5, I0 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.d1 - g.dx1) / flength,
								(g.d1 - g.dy1) / flength,
								(g.dz1 - g.d1) / flength);
						Eigen::Vector3d n2((g.d2 - g.dx2) / flength,
								(g.d2 - g.dy2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I0.i * flength, start.y + I0.j
								* flength, start.z + I0.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[8].point(0))
										+ sqr(edge[8].point(1) - s.y)
										+ sqr(edge[8].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[8].normal = -normal;
						edge[8].color = getColor(start, I0, I4, disc,
								edge[8].point);
					}
					if (edgeTable[cubeindex] & 512) {

						edge[9].point = interpolate(I1, I5, disc.fdata, start);

						corners c = { I1, I5 };
						derivative dv1 = { I2, I0, I5 };
						derivative dv2 = { I6, I4, I1 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.d1 - g.dx1) / flength,
								(g.dy1 - g.d1) / flength,
								(g.dz1 - g.d1) / flength);
						Eigen::Vector3d n2((g.d2 - g.dx2) / flength,
								(g.dy2 - g.d2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I1.i * flength, start.y + I1.j
								* flength, start.z + I1.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[9].point(0))
										+ sqr(edge[9].point(1) - s.y)
										+ sqr(edge[9].point(2) - s.z))
								/ flength;
						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[9].normal = -normal;
						edge[9].color = getColor(start, I1, I5, disc,
								edge[9].point);
					}
					if (edgeTable[cubeindex] & 1024) {

						edge[10].point = interpolate(I2, I6, disc.fdata, start);

						corners c = { I2, I6 };

						derivative dv1 = { I1, I3, I6 };
						derivative dv2 = { I5, I7, I2 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.dx1 - g.d1) / flength,
								(g.dy1 - g.d1) / flength,
								(g.dz1 - g.d1) / flength);
						Eigen::Vector3d n2((g.dx2 - g.d2) / flength,
								(g.dy2 - g.d2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I2.i * flength, start.y + I2.j
								* flength, start.z + I2.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[10].point(0))
										+ sqr(edge[10].point(1) - s.y)
										+ sqr(edge[10].point(2) - s.z))
								/ flength;
						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[10].normal = -normal;
						edge[10].color = getColor(start, I2, I6, disc,
								edge[10].point);
					}
					if (edgeTable[cubeindex] & 2048) {
						edge[11].point = interpolate(I3, I7, disc.fdata, start);
						corners c = { I3, I7 };
						derivative dv1 = { I0, I2, I7 };
						derivative dv2 = { I4, I6, I3 };

						gradient g = getDistances(c, dv1, dv2, disc);

						Eigen::Vector3d n1((g.dx1 - g.d1) / flength,
								(g.d1 - g.dy1) / flength,
								(g.dz1 - g.d1) / flength);
						Eigen::Vector3d n2((g.dx2 - g.d2) / flength,
								(g.d2 - g.dy2) / flength,
								(g.d2 - g.dz2) / flength);

						point s = { start.x + I3.i * flength, start.y + I3.j
								* flength, start.z + I3.k * flength };
						float weight = sqrt(
								sqr(s.x - edge[11].point(0))
										+ sqr(edge[11].point(1) - s.y)
										+ sqr(edge[11].point(2) - s.z))
								/ flength;

						Eigen::Vector3d normal(
								(1 - weight) * n1(0) + (weight) * n2(0),
								(1 - weight) * n1(1) + (weight) * n2(1),
								(1 - weight) * n1(2) + (weight) * n2(2));
						normal.normalize();
						edge[11].normal = -normal;
						edge[11].color = getColor(start, I3, I7, disc,
								edge[11].point);
						basepoint.push_back(edge[11].point);
						normals.push_back(normal);

					}
					computeTriangles(cubeindex, edge);
				}
			}
		}
	}
}
// Find the zero-crossing of the distance function and saving the triangles
void RenderSurface::ExtractSurface() {
	for (int i = 0; i < cResolution; ++i) {
		for (int j = 0; j < cResolution; ++j) {
			for (int k = 0; k < cResolution; ++k) {
				GridDisc disc = grid->getDisc(i, j, k);
				point start = { -x_s + i * grid->getclength(), -y_s + j
						* grid->getclength(), -z_s + k * grid->getclength() };
				if (disc.isOccupied) {
					loadTriangles(disc, start);
				}
			}
		}
	}
}

std::vector<Triangle> RenderSurface::getMesh() {
	return triangles;
}

std::vector<Eigen::Vector3d> RenderSurface::getNormals(){
	return this->normals;
}

std::vector<Eigen::Vector3d> RenderSurface::getBasepoints(){
	return this->basepoint;
}
