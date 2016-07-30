/*
 * Voxelgrid.h
 *
 *  Created on: Sep 12, 2013
 *      Author: erikbylow
 */

#ifndef VOXELGRID_H_
#define VOXELGRID_H_
#include <cstdlib>
#include <iostream>
#include <opencv/cvaux.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "parameters.h"
#include <omp.h>

// Represents a 3D-coordinate
struct point {
	float x, y, z;
};
// Represents indices in a voxelgrid.
struct Index {
	int i,j,k;
};
struct Data {
	float distance, weight,red,green,blue,colorweight;

};
struct GridDisc {
	bool isOccupied;
	Data *fdata;
//	Data cdata[8];
	inline float operator()(int i, int j, int k) const{
		return fdata[i + j*fResolution + k*fResolution*fResolution].distance;
	}
	inline float getWeight(int i, int j, int k) const {
		return fdata[i + j*fResolution + k*fResolution*fResolution].weight;
	}
	inline float getRed(int i , int j, int k) const {
		return fdata[i + j*fResolution + k*fResolution*fResolution].red;
	}
	inline float getBlue(int i, int j, int k) const {
		return fdata[i + j*fResolution + k*fResolution*fResolution].blue;
	}
	inline float getGreen(int i, int j, int k) const {
		return fdata[i + j*fResolution + k*fResolution*fResolution].green;
	}
	inline float getCWeight(int i, int j, int k) const {
		return fdata[i + j*fResolution + k*fResolution*fResolution].colorweight;
	}

};

struct pixel {
	float x,y;
};
class Voxelgrid {
private:


	Index getIndex(const point &p);

	void updateFineGrid(const cv::Mat &depth, const Eigen::Matrix4d &camera,
			const int INDEX, const point &START, const cv::Mat& color);
	void updateCoarseGrid(const cv::Mat &depth, const Eigen::Matrix4d &camera,
			const int INDEX, const point &START);
	GridDisc *grid;
	float clength, flength, DELTAC,SIGMAC;
	std::vector<point> points;
	pixel getPixel(const point &p);
	void printSDF();
	float getColorWeight(const float d);
	void split(int i, int j, int k, Index &index, size_t fsize);
public:
	Voxelgrid(){};
	Voxelgrid(const float size, const int cResolution, const int fResolution, const float DELTAC);
	virtual ~Voxelgrid();
	void splitGrid(const cv::Mat &img, const Eigen::Matrix4d &Camera);
	std::vector<point> getPoints();
	struct GridDisc getDisc(int i, int j, int k);
	float getclength();
	float getflength();
	void UpdateSDF(const cv::Mat &depth, const Eigen::Matrix4d &camera, const cv::Mat &color);
	point getLocal(float x, float y, float z);
	point Multiply(const Eigen::Matrix4d &camera, const point &p);
	long int getMemory();
};

#endif /* VOXELGRID_H_ */
