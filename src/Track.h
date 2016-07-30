/*
 * Track.h
 *
 *  Created on: Sep 30, 2013
 *      Author: erikbylow
 */

#ifndef TRACK_H_
#define TRACK_H_
#include "Voxelgrid.h"
#include "se3.hpp"
#include "so3.hpp"
#include <fstream>
#include <ostream>
class Track {
private:
	Voxelgrid *grid;
	float getEnergy(const point &p);
	Index getIndex(const point &p, const point &start,float step);
	float interpolate(const Index &p, const GridDisc &disc, const point &weight);
	GridDisc getDisc(const Index &p);
	struct Index getFineIndex(const Index &Ic, const point &p);
	struct point getWeights(const Index &If, const point &p, const Index &Ic);
	std::vector<point> trackpoints;
	float getRGBError(const point &p, const pixel &pix, const cv::Mat &colorImg);
	void gridRGB(const point &weights, const Index &I, const GridDisc &disc, float *rgb);
	Sophus::Vector6d getRGBGradient(const point& p0, const pixel& pix0, const point& pdx,
			const point& pdy, const point& pdz, const point& prx, const point& pry,
			const point& prz, const cv::Mat& colorImg, const float V0);
	float getTotalError(const point &p, const pixel &pix,
			const cv::Mat &colorImg);
	float alpha;
	Sophus::Vector6d getGradient(const point& pdx, const point &pdy, const point &pdz, const point &prx, const point &pry, const point &prz, const float V0);

	Eigen::Matrix4d getGlobalMatrix(Sophus::Vector6d &Xi);
	Eigen::MatrixXd getDerivative(Eigen::Vector3d &w, Eigen::Vector3d &x);
	Eigen::VectorXd getAnalyticGradient(Eigen::Vector3d &w, Eigen::Vector3d &t, Eigen::Vector3d &pLocal,
		const double V0Ray, const double V0Dx, const double V0Dy, const double V0Dz, const double delta);
	void getAnalyticGradientComb(Eigen::Vector3d& w,
		Eigen::Vector3d& t, Eigen::Vector3d& xGlobal, Eigen::Vector3d& xLocal,
		float V0, float V0_RGB, const cv::Mat& colorImg, pixel& pix,
		float delta, Sophus::Vector6d &gradStruct, Sophus::Vector6d &gradRGB);

public:
	float sumEnergy(const cv::Mat &depthImg, const Eigen::Matrix4d &camera);
	void controlEnergy(const cv::Mat &depthImg, const Eigen::Matrix4d &camera);
	Track(Voxelgrid *grid, const float alpha);
	virtual ~Track();
	Sophus::Vector6d getNewPose(const Sophus::Vector6d &Xi_0, const cv::Mat &depthImg);
	Sophus::Vector6d getPose(const Sophus::Vector6d &Xi_0, const cv::Mat &depthImg);
	std::vector<point> getTrackPoints();
	float sumColorEnergy(const Eigen::Matrix4d &camera, const cv::Mat &depth_img, const cv::Mat &colorImg);
	void controlRGBEnergy(const cv::Mat &depthImg, const cv::Mat &colorImg,
			const Eigen::Matrix4d &camera);
	Sophus::Vector6d GetColorPose(const Sophus::Vector6d &Xi_0,
			const cv::Mat &depthImg, const cv::Mat &colorImg);
	Sophus::Vector6d GetRGB_DPose(const Sophus::Vector6d &Xi_0,
			const cv::Mat &depthImg, const cv::Mat &colorImg);
	Sophus::Vector6d SteepestDescent(const Sophus::Vector6d &Xi_0,
			const cv::Mat &depthImg, const cv::Mat &colorImg);
	Sophus::Vector6d getAnalyticPose(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg);
	Sophus::Vector6d GetAnalyticRGB_DPose(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg, const cv::Mat &colorImg);
};

#endif /* TRACK_H_ */
