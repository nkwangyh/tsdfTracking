/*
 * groundtruth.h
 *
 *  Created on: Feb 3, 2013
 *      Author: erikbylow
 */

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <iostream>
#include "parameters.h"
//#include "marchingcubes.h"

class Groundtruth {
private:
	std::string file;
	int nbrOfImages;
	std::vector<std::string> depth_names;
	std::vector<std::string> color_names;
	std::vector<std::string> time_stamp;
	std::vector<Eigen::Matrix4d> trajectory;
	Eigen::Matrix4d move;

public:
	Groundtruth(){};
	Groundtruth(std::string path, int nbrofimages);
	Eigen::Matrix4d GetPose(int i) const;
	cv::Mat GetDepthImage(int i) const;
	cv::Mat GetColorImage(int i) const;
	int Number_Of_Images() const;
	std::string getTimeStamp(int) const;
	Eigen::Matrix4d GetMove() const;
};

#endif /* GROUNDTRUTH_H_ */
