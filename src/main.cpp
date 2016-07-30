//============================================================================
// Name        : SparseVoxelgrid.cpp
// Author      : Erik Bylow
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <qapplication.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>
#include <ostream>
//#include <cxcore.h>
#include "Viewer.h"
#include "groundtruth.h"
#include "Voxelgrid.h"
#include "RenderSurface.h"
#include "Track.h"

float alpha = (getenv("alpha") ? atof(getenv("alpha")) / 100.0 : 0.6);
const float DELTAC = (getenv("DELTAC") ? atof(getenv("delta_c")) : 0.3);
const int NBROFIMAGES = (getenv("NUM_IMAGES") ? atoi(getenv("NUM_IMAGES"))
		: 10);
using namespace std;
using namespace cv;
std::vector<Eigen::Matrix4d> pose, groundPose;
//string path = "/home/erikbylow/DataSets/rgbd_dataset_freiburg1_teddy/";
//string path = "C:\\Users\\erikbylow\\OneDrive\\DataSets\\rgbd_dataset_freiburg1_teddy\\";
string path = "D:\\wangyh\\git\\DataSets\\rgbd_dataset_freiburg1_teddy\\";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Distance_Weight(int c_x, int c_y, int i, int j, float sigma_d,
		const Mat& kernel) {

	if (c_x + i <= 0 || c_y + j <= 0 || c_x + i > ROWS || c_y + j > COLS) {
		return 0;
	}
	//	cout<<setprecision(8)<<exp(-0.5 * sqr(sqrt(sqr(i) + sqr(j))/sigma_d))- kernel.at<float>(abs(i),abs(j))<<endl;
	//	return exp(-0.5 * sqr(sqrt(sqr(i) + sqr(j))/sigma_d));

	return kernel.at<float> (abs(i), abs(j));
}

float Similarity_Weight(int c_x, int c_y, int i, int j, const Mat& img,
		float sigma_s, float c_val, float val) {

	if (c_x + i <= 0 || c_y + j <= 0 || c_x + i > ROWS || c_y + j > COLS) {
		return 0;
	}

	if (val < 0.1) {
		return 0;
	}

	return exp(-0.5 * sqr(abs(c_val - val)/sigma_s));
}

float filter(int center_x, int center_y, const Mat& img, float sigma_s,
		float sigma_d, int half_size, const Mat& kernel) {

	float sum = 0;
	float weight_total = 0;

	float weight_c = 0;
	float weight_d = 0;

	float z_center = img.at<float> (center_x, center_y);

	if (z_center < 0.1) {
		return 0;
	}

	//#pragma omp parallel for shared(sum,weight_total)

	for (int i = 0; i <= half_size; ++i) {
		for (int j = 0; j <= half_size; ++j) {

			weight_d = Distance_Weight(center_x, center_y, i, j, sigma_d,
					kernel); //  The weight for the distance between the center pixel and the pixel (i,j)

			// If weight_d > 0 we now (center_x + i, center_y + j) are not outside the bound of the image.
			if (weight_d > 0) {
				float val = img.at<float> (center_x + i, center_y + j);
				weight_c = Similarity_Weight(center_x, center_y, i, j, img,
						sigma_s, z_center, val);
				//
				// If the value at img.at<float>(center_x, center_y + j) = 0, weight_c = 0 so it does not influence the filtering.
				//
				sum += weight_c * weight_d * val;
				weight_total += weight_c * weight_d;
			}
			if (i != 0 && j != 0) {
				weight_d = Distance_Weight(center_x, center_y, -i, -j, sigma_d,
						kernel);
				if (weight_d > 0) {
					float val = img.at<float> (center_x - i, center_y - j);
					weight_c = Similarity_Weight(center_x, center_y, -i, -j,
							img, sigma_s, z_center, val);
					sum += weight_c * weight_d * val;
					weight_total += weight_c * weight_d;
				}
			}

		}
	}

	if (weight_total > 0) {
		return sum / weight_total;
	}
	return 0;
}

// Kernel size is 2*half_size + 1

Mat Kernel(int half_size, float sigma_d) {
	Mat kernel = Mat::zeros(half_size + 1, half_size + 1, CV_32F);
#pragma omp parallel for shared(kernel)
	for (int i = 0; i < half_size + 1; ++i) {
		for (int j = 0; j < half_size + 1; ++j) {
			kernel.at<float> (i, j) = exp(
					-0.5 * sqr((sqrt(sqr(i) + sqr(j))/sigma_d)));
		}
	}

	return kernel;
}

Mat Bilateral(const Mat& input, int half_size, float sigma_d, float sigma_s) {

	Mat kernel = Kernel(half_size, sigma_d);

	Mat target = Mat::zeros(input.rows, input.cols, input.type());
#pragma omp parallel for shared(target)

	for (int i = 0; i < ROWS; ++i) {
		for (int j = 0; j < COLS; ++j) {

			target.at<float> (i, j) = filter(i, j, input, sigma_s, sigma_d,
					half_size, kernel);

		}
	}

	return target;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


string getErikDepthName(int frame) {
	ostringstream o;
	o << frame;
	string name = path + "depth" + o.str() + ".png";
	return name;
}

string getErikColorName(int frame) {
	ostringstream o;
	o << frame;
	string name = path + "color" + o.str() + ".png";
	return name;
}
string getStanfordName(int frame) {
	char buffer[12];
	sprintf(buffer, "frame_%04d", frame);
	string tmp = buffer;
	return tmp;
}
string getStanfordName5(int frame) {
	char buffer[12];
	sprintf(buffer, "frame_%05d", frame);
	string tmp = buffer;
	return tmp;
}

cv::Mat getDepth(const string &path) {
	cv::Mat tmp = cv::imread(path, -1);
	tmp.convertTo(tmp, CV_32FC1, 1.0 / 1000.0);
	return tmp;
}
cv::Mat getColor(const string &path) {
	return cv::imread(path, 1);
}

void insertImage(const Groundtruth& groundtruth, Voxelgrid &grid) {

	double t1 = omp_get_wtime();
	for (int i = 0; i < NBROFIMAGES; i += 1) {

		grid.splitGrid(groundtruth.GetDepthImage(i), groundtruth.GetPose(i));
		cv::Mat colorImg = groundtruth.GetColorImage(i);
		//		colorImg.convertTo(colorImg, CV_32FC3);
		grid.UpdateSDF(groundtruth.GetDepthImage(i),
				groundtruth.GetPose(i).inverse(), colorImg);
		groundPose.push_back(groundtruth.GetPose(i));
		cout << i << endl;
	}
	double t2 = omp_get_wtime();
	cout << "Time for spliting the grid: " << (t2 - t1) << " s" << endl;
	cout << "Frame rate: " << NBROFIMAGES / (t2 - t1) << endl;
}

void getEnergyFunctions(Track& track, const Groundtruth &ground) {
	int frame = 0;
	string colorName = getErikColorName(frame);
	string depthName = getErikDepthName(frame);

	cv::Mat depthImg = getDepth(depthName);
	if (depthImg.data == NULL) {
		cout << "Could not open image" << endl;
		exit(1);
	}

	cv::Mat colorImg = getColor(colorName);
	//	track.controlRGBEnergy(ground.GetDepthImage(frame),
	//			ground.GetColorImage(frame), ground.GetPose(frame));
	track.controlRGBEnergy(depthImg, colorImg, ground.GetPose(frame));

}

void startTracking(Track &track, Groundtruth &ground, Voxelgrid &grid, float sigma_s, float sigma_d, int half_size) {
	int start = 0;
	int frame = start;


	//	string path2 = getStanfordName(frame);
	//	string depthName = path + path2 + "_depth.png";
	//	string colorName = path + path2 + "_rgb.png";
	//	cv::Mat depthImg = getStanfordDepth(depthName);
	//	cv::Mat colorImg = getStanfordColor(colorName);
	//	if (!depthImg.data || !colorImg.data) // Check for invalid input
	//	{
	//		cout << "Could not open or find the image, trying new path"
	//				<< std::endl;
	//		string path2 = getStanfordName5(frame);
	//
	//		depthName = path + path2 + "_depth.png";
	//		colorName = path + path2 + "_rgb.png";
	//
	//		depthImg = getStanfordDepth(depthName);
	//		colorImg = getStanfordColor(colorName);
	//
	//		if (!depthImg.data || !colorImg.data) {
	//			cout << "Could not open any file, exiting" << endl;
	//			exit(1);
	//		}
	//	}
	//
	cv::Mat colorImg, depthImg;
	string depthName, colorName;
	if (SET == 2) {
		colorName = getErikColorName(frame);
		depthName = getErikDepthName(frame);

		depthImg = getDepth(depthName);
		if (depthImg.data == NULL) {
			cout << "Could not open image" << endl;
			exit(1);
		}

		colorImg = getColor(colorName);
	}

	if (SET == 1) {
		depthImg = ground.GetDepthImage(frame);
		colorImg = ground.GetColorImage(frame);
		// Here one can filter the depth image if wanted
		//depthImg = Bilateral(depthImg, half_size, sigma_d, sigma_s);
		//	colorImg.convertTo(colorImg,CV_32FC3);
	}
	Eigen::Matrix4d INIT = Eigen::Matrix4d::Zero();
	INIT(0, 2) = 1;
	INIT(0, 3) = START_X;
	INIT(1, 0) = -1;
	INIT(2, 1) = -1;
	INIT(1, 3) = START_Y;
	INIT(2, 3) = START_Z;
	INIT(3, 3) = 1;
	Eigen::Matrix4d camera = INIT;
	// Integrate the first image into the empty grid
	grid.splitGrid(depthImg, camera);
	grid.UpdateSDF(depthImg, camera.inverse(), colorImg);
	//	std::cout<<camera<<std::endl;
	//	getchar();
	//	pose.push_back(camera);

	groundPose.push_back(ground.GetPose(frame));
	// Create the Lie-Algebra representation

	Sophus::Affine3d tmp(camera);
	Sophus::SE3 P(tmp.rotation(), tmp.translation());
	Sophus::Vector6d Xi = P.log();

	for (frame = start+1; frame < NBROFIMAGES; ++frame) {
		if (SET == 2) {
			depthName = getErikDepthName(frame);
			colorName = getErikColorName(frame);
			depthImg = getDepth(depthName);
			colorImg = getColor(colorName);
			if (depthImg.data == NULL) {
				cout << "Could not open image" << endl;
				exit(1);
			}
		}
		if (SET == 1) {
			depthImg = ground.GetDepthImage(frame);
			colorImg = ground.GetColorImage(frame);
		}
		//		cv::Mat depthTmp = Bilateral(depthImg, half_size, sigma_d,sigma_s);
		//		cv::Mat colorTmp;
		//		cv::GaussianBlur(colorImg,colorTmp,cv::Size(9,9),1,1);
		//		imshow("Test",colorTmp);
		//		cv::waitKey();
		//		string path2 = getStanfordName(frame);
		//		string depthName = path + path2 + "_depth.png";
		//		string colorName = path + path2 + "_rgb.png";
		//		depthImg = getStanfordDepth(depthName);
		//		colorImg = getStanfordColor(colorName);
		//		if (!depthImg.data || !colorImg.data) // Check for invalid input
		//		{
		//			cout << "Could not open or find the image, trying new path"
		//					<< std::endl;
		//			string path2 = getStanfordName5(frame);
		//
		//			depthName = path + path2 + "_depth.png";
		//			colorName = path + path2 + "_rgb.png";
		//
		//			depthImg = getStanfordDepth(depthName);
		//			colorImg = getStanfordColor(colorName);
		//
		//			if (!depthImg.data || !colorImg.data) {
		//				cout << "Could not open any file, exiting" << endl;
		//				exit(1);
		//			}
		//		}

		double t1 = omp_get_wtime();
		//		Xi = track.getNewPose(Xi, depthImg);
		//		Xi = track.SteepestDescent(Xi, depthImg, colorImg);
		//		Xi = track.getPose(Xi, depthImg);
		//		Xi = track.GetColorPose(Xi, depthImg, colorImg);   // Here you can choose between using just geometry or include color in tracking
		//Xi = track.GetRGB_DPose(Xi, depthImg, colorImg);
		//Xi = track.getAnalyticPose(Xi, depthImg);
		Xi = track.GetAnalyticRGB_DPose(Xi, depthImg, colorImg);
		camera = Sophus::SE3::exp(Xi).matrix();
		grid.splitGrid(depthImg, camera);
		grid.UpdateSDF(depthImg, camera.inverse(), colorImg);
		double t2 = omp_get_wtime();
		std::cout << "Tracking rate: " << 1 / (t2 - t1) << endl;
		pose.push_back(camera);
		groundPose.push_back(ground.GetPose(frame));
		cout << frame << endl;
	}
	cout << "Done Tracking" << endl;
}

void Write_Pose_To_Quaternions(const Groundtruth& ground, string name) {
	stringstream s;
	int nbr = (int)(alpha * 10);
	s << name << nbr<<".txt";
	string fileName = s.str();
	cout << fileName << endl;
	ofstream myfile;
	myfile.open(fileName.c_str());

	string time;
	for (int i = 0; i < pose.size(); ++i) {

		Eigen::Affine3d tmp(pose[i]);
		Eigen::Quaterniond q = Eigen::Quaterniond(tmp.rotation());
		// writing the quaternions and the translation to the file
		time = ground.getTimeStamp(i);
		myfile << time << " " << tmp.translation()(0) << " "
				<< tmp.translation()(1) << " " << tmp.translation()(2) << " "
				<< q.w() << " " << q.x() << " " << q.y() << " " << q.z()
				<< endl;
	}
	myfile.close();
}

/*string meshToString(double scale, vector<Triangle> &triangles, float *Vertices,
		float *Colors) {
	stringstream p;
	stringstream f;
	int i2 = 0;
	for (int i = 0; i < (int) triangles.size(); i++) {
		bool valid = true;
		for (int t = 0; t < 3; t++) {
			for (int t2 = t + 1; t2 < 3; t2++) {
				if (boost::str(
						boost::format("%0.5f %0.5f %0.5f") % Vertices[i * 9 + t
								* 3 + 0] % Vertices[i * 9 + t * 3 + 1]
								% Vertices[i * 9 + t * 3 + 2]) == boost::str(
						boost::format("%0.5f %0.5f %0.5f") % Vertices[i * 9
								+ t2 * 3 + 0] % Vertices[i * 9 + t2 * 3 + 1]
								% Vertices[i * 9 + t2 * 3 + 2])) {
					valid = false;
				}
			}
		}
		if (!valid)
			continue;
		for (int t = 0; t < 3; t++) {
			p << boost::format("v %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n")
					% (Vertices[i * 9 + t * 3 + 0] * scale) % (Vertices[i * 9
					+ t * 3 + 1] * scale) % (Vertices[i * 9 + t * 3 + 2]
					* scale) % Colors[i * 9 + t * 3 + 0] % Colors[i * 9 + t * 3
					+ 1] % Colors[i * 9 + t * 3 + 2];
			//			 % Normals[i * 9 + t * 3 + 0] % Normals[i * 9 + t * 3 + 1] % Normals[i * 9 + t * 3 + 2]

		}
		f << boost::format("f %d %d %d\n") % (i2 * 3 + 1) % (i2 * 3 + 2) % (i2
				* 3 + 3);
		i2++;
	}
	cout << "triangles: " << triangles.size() << " valid: " << i2 << endl;
	return p.str() + f.str();
}

void saveSDF(vector<Triangle> &triangles, float *Vertices, float *Colors) {
	cout << "Saving SDF and mesh.." << endl;

	double scale = 1.0;

	ofstream objfile("mesh.obj", ios::out);
	objfile << meshToString(scale, triangles, Vertices, Colors);
	objfile.close();

	cout << "..done!" << endl;
}*/
int main(int argc, char **argv) {
	cout << "Program starts" << endl;

	if (argc > 1) {
		path = argv[1];
	}
	cout << path << endl;
	cout << "alpha: " << alpha << endl;
	omp_set_dynamic(0);
	omp_set_num_threads(7);

	float sigma_s = 0.3;
	float sigma_d = 2;
	int half_size = 5;
	/*cout << "Enter sigma_s: ";
	cin >> sigma_s;
	cout << endl;
	cout << "Enter sigma_d: ";
	cin >> sigma_d;
	cout << endl;
	cout << "Enter half_size: ";
	cin >> half_size;
	cout << endl;*/

	//#pragma omp parallel
	//
	//#pragma omp master
	//		{
	//			std::cout<<omp_get_num_threads()<<std::endl;
	//		}
	//
	//	getchar();
	Groundtruth groundtruth(path, NBROFIMAGES); // Initializing an object of groundtruth
	//	calc_mean(groundtruth);
	//	Write_Pose_To_Quaternions(groundtruth);
	//
	//	getchar();
//	Voxelgrid grid(griddim, cResolution, fResolution, DELTAC); // Initializing the grid
	std::vector<point> trackpoints;
	//	insertImage(groundtruth, grid);
	//for (float i = 0; i <= 1; i += 0.1){
		alpha = 0.5;
		Voxelgrid grid(griddim, cResolution, fResolution, DELTAC); // Initializing the grid

		Track track(&grid, alpha); // initializing the track object

		startTracking(track, groundtruth, grid,sigma_s,sigma_d,half_size); // Start the tracking from offline data
		cout << "Pose length: " << pose.size() << endl;
		string name = "Teddy";
		Write_Pose_To_Quaternions(groundtruth, name);
	//	pose.clear();
	//}
	//	getEnergyFunctions(track, groundtruth);
	//cout << grid.getMemory() / 1000000 << " MegaBytes" << endl;
	//	getchar();
	//return 0;
	RenderSurface render(&grid);
	double t1 = omp_get_wtime();
	render.ExtractSurface();
	double t2 = omp_get_wtime();
	std::cout << "Time for Marchingcubes: " << t2 - t1 << std::endl;
	std::vector<Triangle> tmp = render.getMesh();
	std::vector<point> points = grid.getPoints();
	
	// Read command lines arguments.


	QApplication application(argc, argv);

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setPoints(trackpoints);
	viewer.setGrid(&grid);
	viewer.setTriangles(tmp);
	viewer.setPose(pose);
	viewer.setGroundtruth(groundPose);
	viewer.setNormals(render.getNormals(),render.getBasepoints());
	// Write mesh to .obj file
	//	saveSDF(tmp, viewer.getVertices(), viewer.getColors());
	// Make the viewer window visible on screen.
	viewer.show();

	// Run main loop.
	return application.exec();
}
