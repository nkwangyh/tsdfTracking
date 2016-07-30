/*
 * groundtruth.cpp
 *
 *  Created on: Feb 3, 2013
 *      Author: erikbylow
 */
#include "groundtruth.h"
#include <fstream>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace std;


std::vector<Eigen::Matrix4d> CameraMatrices(string filename, int columns_skip,
		int rows_step, int stop) {

	fstream in(filename.c_str());
	if (!in) {
		cerr << "Could not open file: " << filename << endl;

	}

	std::vector<Eigen::Matrix4d> transformations;
	int frames;
	string s;
	for (frames = 0; frames < stop; frames++) {
		getline(in, s);
		if (frames % rows_step != 0) {
			continue;
		}

		stringstream line(s);
		for (int i = 0; i < columns_skip; i++) {
			string tmp;
			line >> tmp;
		}
		double stamp, tx, ty, tz, qx, qy, qz, qw;
		line >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
		if (qw * qw + qx * qx + qy * qy + qz * qz < 0.99) {
			cerr << "pose " << frames << " has invalid rotation" << endl;
		}

		Eigen::Vector3d translation(tx, ty, tz);
		Eigen::Quaterniond quaternion(qw, qx, qy, qz);
		Eigen::Matrix3d rotmatrix(quaternion);

		Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();

		transf(0, 3) = translation(0);
		transf(1, 3) = translation(1);
		transf(2, 3) = translation(2);
		transf.block(0, 0, 3, 3) = rotmatrix;

		transformations.push_back(transf);

	}
	cerr << "read " << frames << " frames" << endl;

	return transformations;
}


std::vector<string> loadName(string filename, int columns_skip, int row_step,
		int stop) {

	fstream in(filename.c_str());
	if (!in) {
		cerr << "Could not open file: " << filename << endl;

	}

	std::vector<string> names;
	int frames;
	for (frames = 0; frames < stop; frames++) {
		string s;
		getline(in, s);
		if (frames % row_step != 0) {
			continue;
		}

		stringstream line(s);

		for (int i = 0; i < columns_skip; i++) {
			string tmp;
			line >> tmp;

		}
		string name;
		line >> name;
		names.push_back(name);

	}

	return names;

}

Eigen::Matrix4d Move(const Eigen::Matrix4d& M){
	Eigen::Matrix4d INIT = Eigen::Matrix4d::Zero();

	INIT(0, 2) = 1;
	INIT(0, 3) = START_X;
	INIT(1, 0) = -1;
	INIT(2, 1) = -1;
	INIT(1, 3) = START_Y;
	INIT(2,3) = START_Z;
	INIT(3, 3) = 1;

	return INIT * M.inverse();
}

Groundtruth::Groundtruth(string path, int nbrOfImages){
	this->file = path;
	this->nbrOfImages = nbrOfImages;
	this->depth_names = loadName(path + "associated.txt", 11, 1,
			nbrOfImages);
	this->color_names = loadName(path + "associated.txt", 9, 1,
			nbrOfImages);
	this->time_stamp = loadName(path + "associated.txt", 0, 1,
			nbrOfImages);
	this->trajectory = CameraMatrices(path + "associated.txt", 0,1, nbrOfImages);
	this->move = Move(this->trajectory[0]);
}



cv::Mat Groundtruth::GetDepthImage(int i) const {
	cv::Mat tmp = cv::imread(this->file + this->depth_names[i],-1);
	tmp.convertTo(tmp, CV_32FC1, 1.0 / 5000.0, 0);
    return tmp;
}

cv::Mat Groundtruth::GetColorImage(int i) const{
	cv::Mat tmp = cv::imread(this->file + this->color_names[i],-1);
	return tmp;

}


Eigen::Matrix4d Groundtruth::GetPose(int i) const {
	return this->move*this->trajectory[i];
}

int Groundtruth::Number_Of_Images() const{
	return this->nbrOfImages;
}

string Groundtruth::getTimeStamp(int i) const{
	return this->time_stamp[i];
}

Eigen::Matrix4d Groundtruth::GetMove() const{
	return this->move;
}
