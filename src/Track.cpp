/*
 * Track.cpp
 *
 *  Created on: Sep 30, 2013
 *      Author: erikbylow
 */

#include "Track.h"

Track::Track(Voxelgrid *grid, const float alpha) {
	this->grid = grid;
	this->alpha = alpha;
}

Track::~Track() {
	std::cout << "In destructor for Track" << std::endl;
}

struct point getPoint(float x, float y, float z) {
	point tmp = { (x - cx) * z / fy, (y - cy) * z / fy, z };
	return tmp;
}

Index Track::getIndex(const point &p, const point &start, float step) {
	Index tmp = { (int) floor((p.x - start.x) / step), (int) floor(
			(p.y - start.y) / step), (int) floor((p.z - start.z) / step) };
	return tmp;
}
float Track::interpolate(const Index &I, const GridDisc &disc,
		const point &weight) {
	if (disc.getWeight(I.i, I.j, I.k) <= 0) {
		return STOP;
	}
	const float d1 = disc.operator ()(I.i, I.j, I.k);
	//
	if (disc.getWeight(I.i + 1, I.j, I.k) <= 0) {
		return STOP;
	}
	const float d2 = disc.operator ()(I.i + 1, I.j, I.k);

	if (disc.getWeight(I.i + 1, I.j + 1, I.k) <= 0) {
		return STOP;
	}
	const float d3 = disc.operator ()(I.i + 1, I.j + 1, I.k);

	if (disc.getWeight(I.i, I.j + 1, I.k) <= 0) {
		return STOP;
	}
	const float d4 = disc.operator ()(I.i, I.j + 1, I.k);
	//
	//
	if (disc.getWeight(I.i, I.j, I.k + 1) <= 0) {
		return STOP;
	}
	const float d5 = disc.operator ()(I.i, I.j, I.k + 1);

	if (disc.getWeight(I.i + 1, I.j, I.k + 1) <= 0) {
		return STOP;
	}
	const float d6 = disc.operator ()(I.i + 1, I.j, I.k + 1);

	if (disc.getWeight(I.i + 1, I.j + 1, I.k + 1) <= 0) {
		return STOP;
	}
	const float d7 = disc.operator ()(I.i + 1, I.j + 1, I.k + 1);

	if (disc.getWeight(I.i, I.j + 1, I.k + 1) <= 0) {
		return STOP;
	}
	const float d8 = disc.operator ()(I.i, I.j + 1, I.k + 1);

	//	if (fabs(d1) >= DELTA || fabs(d2) >= DELTA || fabs(d3) >= DELTA || fabs(d4)
	//			>= DELTA || fabs(d5) >= DELTA || fabs(d6) >= DELTA || fabs(d7)
	//			>= DELTA || fabs(d8) >= DELTA) {
	//		return STOP;
	//	}
	//

	const float d12 = (1 - weight.x) * d1 + (weight.x) * d2;
	const float d34 = (1 - weight.x) * d4 + (weight.x) * d3;
	const float d56 = (1 - weight.x) * d5 + (weight.x) * d6;
	const float d78 = (1 - weight.x) * d8 + (weight.x) * d7;

	const float d1234 = (1 - weight.y) * d12 + (weight.y) * d34;
	const float d5678 = (1 - weight.y) * d56 + (weight.y) * d78;

	return ((1 - weight.z) * d1234 + (weight.z) * d5678);
}
GridDisc Track::getDisc(const Index &I) {
	return grid->getDisc(I.i, I.j, I.k);
}

struct Index Track::getFineIndex(const Index &Ic, const point &p) {
	point start = { -x_s + Ic.i * grid->getclength(), -y_s + Ic.j
			* grid->getclength(), -z_s + Ic.k * grid->getclength() };
	return getIndex(p, start, grid->getflength());
}

struct point Track::getWeights(const Index &If, const point &p, const Index &Ic) {
	// Global position for the coarse voxel
	point start = { -x_s + Ic.i * grid->getclength(), -y_s + Ic.j
			* grid->getclength(), -z_s + Ic.k * grid->getclength() };
	// The global position for the coarse voxel is the starting point for the fine grid
	const point voxel = { start.x + If.i * grid->getflength(), start.y + If.j
			* grid->getflength(), start.z + If.k * grid->getflength() };
	//	if (p.x < voxel.x || p.y < voxel.y || p.z < voxel.z){
	//		std::cout<<"Something is wrong: "<<std::endl;
	//		std::cout<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
	//		std::cout<<voxel.x<<" "<<voxel.y<<" "<<voxel.z<<std::endl;
	//		getchar();
	//	}
	float w1 = (p.x - voxel.x) / grid->getflength();
	float w2 = (p.y - voxel.y) / grid->getflength();
	float w3 = (p.z - voxel.z) / grid->getflength();

	point weight = { w1, w2, w3 };

	//	point weight = {fabs(floatindex.x - If.i), fabs(floatindex.y - If.j), fabs(floatindex.z - If.k)};
	return weight;
}
bool checkIndex(const Index &I, int R) {
	if (I.i < 0 || I.j < 0 || I.k < 0 || I.i >= R || I.j >= R || I.k >= R) {
		return false;
	}
	return true;
}
float Track::getEnergy(const point& p) {
	const point start = { -x_s, -y_s, -z_s };
	const Index Ic = getIndex(p, start, grid->getclength());
	bool valid = checkIndex(Ic, cResolution);
	if (!valid) {
		return STOP;
	}
	const GridDisc disc = getDisc(Ic);
	if (!disc.isOccupied) {
		return STOP;
	}
	const Index If = getFineIndex(Ic, p);
	valid = checkIndex(If, fResolution - 1); // In interpolate we add 1 to the indices, so also If.i,j,k must be less than fResolution - 1
	if (!valid) {
		return STOP;
	}
	const point weights = getWeights(If, p, Ic);

	return (interpolate(If, disc, weights));
}
float Track::sumEnergy(const cv::Mat &depthImg, const Eigen::Matrix4d &camera) {
	float sum = 0;
	float energy = 0;
	for (int i = 0; i < depthImg.rows; i++) {
		for (int j = 0; j < depthImg.cols; j++) {
			const float z = depthImg.at<float> (i, j);
			if (z > 0) {
				const point local = grid->getLocal(j, i, z);
				const point global = grid->Multiply(camera, local);
				energy = getEnergy(global);
				if (fabs(energy) < (STOP2)) {
					sum += sqr(energy);
				}
			}
		}
	}
	return sum;
}

void Track::controlEnergy(const cv::Mat &depthImg,
		const Eigen::Matrix4d &camera) {
	Sophus::Affine3d tmp(camera);
	Sophus::SE3 G(tmp.rotation(), tmp.translation());
	Sophus::Vector6d Xi = G.log();
	const Sophus::Vector6d XI_OLD = Xi;
	float step = 0.001;
	float width = 0.3;

	std::ofstream myfile;
	myfile.open("Energy.txt");
	int index = 4;
	for (float i = -width; i <= width; i += step) {
		Xi(index) = XI_OLD(index) + i;
		const float energy = sumEnergy(depthImg, Sophus::SE3::exp(Xi).matrix());
		myfile << i << " " << energy << std::endl;
		std::cout << i << std::endl;
	}
	myfile.close();
}

bool checkGradient(const Sophus::Vector6d &g) {
	if (fabs(g(0)) > abs(STOP2) || fabs(g(1)) > abs(STOP2) || fabs(g(2))
			> abs(STOP2) || fabs(g(3)) > abs(STOP2) || fabs(g(4)) > abs(
			STOP2) || fabs(g(5)) > abs(STOP2)) {
		return false;
	}
	return true;
}

Sophus::Vector6d Track::getPose(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg) {
	Sophus::Vector6d Xi = Xi_0;
	bool check = false;
	int itr = 0;
	while (!check) {
		Sophus::Vector6d tdx, tdy, tdz, rdx, rdy, rdz;
		Sophus::Vector6d prev = Xi;
		//	for (int itr = 0; itr < MAX_ITR; ++itr){

		prev = Xi;

		tdx = tdy = tdz = rdx = rdy = rdz = Xi;

		// This is for the numerical derivatives
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;

		rdx(3) += epsilon;
		rdy(4) += epsilon;
		rdz(5) += epsilon;

		const Eigen::Matrix4d G0 = Sophus::SE3::exp(Xi).matrix();

		const Eigen::Matrix4d Gdx = Sophus::SE3::exp(tdx).matrix();
		const Eigen::Matrix4d Gdy = Sophus::SE3::exp(tdy).matrix();
		const Eigen::Matrix4d Gdz = Sophus::SE3::exp(tdz).matrix();
		const Eigen::Matrix4d Grx = Sophus::SE3::exp(rdx).matrix();
		const Eigen::Matrix4d Gry = Sophus::SE3::exp(rdy).matrix();
		const Eigen::Matrix4d Grz = Sophus::SE3::exp(rdz).matrix();

		Sophus::Matrix6d A = Sophus::Matrix6d::Zero();
		Sophus::Vector6d b = Sophus::Vector6d::Zero();
		int nbrofpoints = 0;
		for (int i = 0; i < depthImg.rows; ++i) {
			for (int j = 0; j < depthImg.cols; ++j) {
				const float z = depthImg.at<float> (i, j);
				if (z > 0) {
					const point local = grid->getLocal(j, i, z);
					const point g0 = grid->Multiply(G0, local);
					const float V0 = getEnergy(g0);
					if (V0 < (STOP2)) {
						//						float weight = getLSWeight(V0);
						Sophus::Vector6d gradient = Sophus::Vector6d::Zero();
						gradient(0) = getEnergy(grid->Multiply(Gdx, local));
						gradient(1) = getEnergy(grid->Multiply(Gdy, local));
						gradient(2) = getEnergy(grid->Multiply(Gdz, local));
						gradient(3) = getEnergy(grid->Multiply(Grx, local));
						gradient(4) = getEnergy(grid->Multiply(Gry, local));
						gradient(5) = getEnergy(grid->Multiply(Grz, local));
						//						std::cout<<"V0: "<<V0<<std::endl;
						//						std::cout<<gradient<<std::endl;
						//						getchar();


						bool valid = checkGradient(gradient);
						if (valid) {
							for (int i = 0; i < 6; ++i) {
								gradient(i) -= V0;
							}
							//						std::cout<<gradient<<std::endl;
							//						getchar();

							gradient /= epsilon;
							//							gradient *= weight;

							//							std::cout<<gradient<<std::endl;
							//							getchar();
							b -= V0 * gradient;
							A += gradient * gradient.transpose();
						}
					}
				}
			}
		}
		//		std::cout<<nbrofpoints<<std::endl;
		//		getchar();
		//		std::cout<<A<<std::endl;
		//		getchar();
		//		A /= sqr(epsilon);
		//		b /= epsilon;
		itr++;
		//		std::cout<<A<<std::endl;
		//		getchar();
		Xi = A.ldlt().solve(A * Xi + b);

		if ((fabs(Xi(0) - prev(0)) < epsilon_stop && fabs(Xi(1) - prev(1))
				< epsilon_stop && fabs(Xi(2) - prev(2)) < epsilon_stop && fabs(
				Xi(3) - prev(3)) < epsilon_stop && fabs(Xi(4) - prev(4))
				< epsilon_stop && fabs(Xi(5) - prev(5)) < epsilon_stop) || itr
				== MAX_ITR) {
			check = true;
		}
	}
	std::cout << "Iterations  " << itr << std::endl;
	return Xi;

}

std::vector<point> Track::getTrackPoints() {
	return trackpoints;
}

void Track::gridRGB(const point &weight, const Index &I, const GridDisc &disc,
		float *rgb) {
	rgb[0] = rgb[1] = rgb[2] = STOP;
	//	std::cout<<"distance: "<<distance<<std::endl;
	//	std::cout<<"THRES: "<<THRES<<std::endl;
	//	getchar();
	if (disc.getCWeight(I.i, I.j, I.k) > 0 /*&& distance < THRES*/) {

		const float r1 = disc.getRed(I.i, I.j, I.k);
		const float g1 = disc.getGreen(I.i, I.j, I.k);
		const float b1 = disc.getBlue(I.i, I.j, I.k);
		//
		if (disc.getCWeight(I.i + 1, I.j, I.k) > 0 /*&& distance < THRES*/) {

			const float r2 = disc.getRed(I.i + 1, I.j, I.k);
			const float g2 = disc.getGreen(I.i + 1, I.j, I.k);
			const float b2 = disc.getBlue(I.i + 1, I.j, I.k);

			if (disc.getCWeight(I.i + 1, I.j + 1, I.k) > 0 /*&& distance < THRES*/) {

				const float r3 = disc.getRed(I.i + 1, I.j + 1, I.k);
				const float g3 = disc.getGreen(I.i + 1, I.j + 1, I.k);
				const float b3 = disc.getBlue(I.i + 1, I.j + 1, I.k);
				if (disc.getCWeight(I.i, I.j + 1, I.k) > 0 /*&& distance < THRES*/) {

					const float r4 = disc.getRed(I.i, I.j + 1, I.k);
					const float g4 = disc.getGreen(I.i, I.j + 1, I.k);
					const float b4 = disc.getBlue(I.i, I.j + 1, I.k);
					//
					//
					if (disc.getCWeight(I.i, I.j, I.k + 1) > 0 /*&& distance
					 < THRES*/) {

						const float r5 = disc.getRed(I.i, I.j, I.k + 1);
						const float g5 = disc.getGreen(I.i, I.j, I.k + 1);
						const float b5 = disc.getBlue(I.i, I.j, I.k + 1);

						if (disc.getCWeight(I.i + 1, I.j, I.k + 1) > 0
						/*&& distance < THRES*/) {

							const float r6 = disc.getRed(I.i + 1, I.j, I.k + 1);
							const float g6 = disc.getGreen(I.i + 1, I.j,
									I.k + 1);
							const float b6 =
									disc.getBlue(I.i + 1, I.j, I.k + 1);
							if (disc.getCWeight(I.i + 1, I.j + 1, I.k + 1) > 0
							/*&& distance < THRES*/) {

								const float r7 = disc.getRed(I.i + 1, I.j + 1,
										I.k + 1);
								const float g7 = disc.getGreen(I.i + 1,
										I.j + 1, I.k + 1);
								const float b7 = disc.getBlue(I.i + 1, I.j + 1,
										I.k + 1);

								if (disc.getCWeight(I.i, I.j + 1, I.k + 1) > 0
								/*&& distance < THRES*/) {

									const float r8 = disc.getRed(I.i, I.j + 1,
											I.k + 1);
									const float g8 = disc.getGreen(I.i,
											I.j + 1, I.k + 1);
									const float b8 = disc.getBlue(I.i, I.j + 1,
											I.k + 1);

									const float r12 = (1 - weight.x) * r1
											+ weight.x * r2;
									const float g12 = (1 - weight.x) * g1
											+ weight.x * g2;
									const float b12 = (1 - weight.x) * b1
											+ weight.x * b2;

									const float r43 = (1 - weight.x) * r4
											+ weight.x * r3;
									const float g43 = (1 - weight.x) * g4
											+ weight.x * g3;
									const float b43 = (1 - weight.x) * b4
											+ weight.x * b3;

									const float r56 = (1 - weight.x) * r5
											+ weight.x * r6;
									const float g56 = (1 - weight.x) * g5
											+ weight.x * g6;
									const float b56 = (1 - weight.x) * b5
											+ weight.x * b6;

									const float r87 = (1 - weight.x) * r8
											+ weight.x * r7;
									const float g87 = (1 - weight.x) * g8
											+ weight.x * g7;
									const float b87 = (1 - weight.x) * b8
											+ weight.x * b7;

									const float r1243 = (1 - weight.y) * r12
											+ weight.y * r43;
									const float g1243 = (1 - weight.y) * g12
											+ weight.y * g43;
									const float b1243 = (1 - weight.y) * b12
											+ weight.y * b43;

									const float r5687 = (1 - weight.y) * r56
											+ weight.y * r87;
									const float g5687 = (1 - weight.y) * g56
											+ weight.y * g87;
									const float b5687 = (1 - weight.y) * b56
											+ weight.y * b87;

									const float r = (1 - weight.z) * r1243
											+ weight.z * r5687;
									const float b = (1 - weight.z) * b1243
											+ weight.z * b5687;
									const float g = (1 - weight.z) * g1243
											+ weight.z * g5687;

									rgb[0] = r;
									rgb[1] = g;
									rgb[2] = b;
								}
							}
						}
					}
				}
			}
		}
	}
}

float Track::getRGBError(const point &p, const pixel &pix,
		const cv::Mat &colorImg) {
	// Find which grid the point lies in
	const point start = { -x_s, -y_s, -z_s };
	const Index Ic = getIndex(p, start, grid->getclength());
	bool valid = checkIndex(Ic, cResolution);
	if (!valid) {
		return STOP;
	}
	// Get the correct disc
	const GridDisc disc = getDisc(Ic);
	if (!disc.isOccupied) {
		return STOP;
	}
	// Get the index in the disc
	const Index If = getFineIndex(Ic, p);
	valid = checkIndex(If, fResolution - 1); // In interpolate we add 1 to the indices, so also If.i,j,k must be less than fResolution - 1
	if (!valid) {
		return STOP;
	}
	// The wieghts for interpolation
	const point weights = getWeights(If, p, Ic);

	float RGB[3] = { 0, 0, 0 };
	gridRGB(weights, If, disc, RGB);
	if (RGB[0] == STOP || RGB[1] == STOP || RGB[2] == STOP) {
		return STOP;
	}
	const cv::Vec3b rgb = colorImg.at<cv::Vec3b> (pix.x, pix.y);
	const float scale = 255;
	const float projRed = rgb(2) / scale;
	const float projGreen = rgb(1) / scale;
	const float projBlue = rgb(0) / scale;

	if (isnan(projRed) || isnan(projGreen) || isnan(projBlue)) {
		return STOP;
	}
	//		std::cout << RGB[0] << "  " << projRed << std::endl;
	//		std::cout << RGB[1] << "  " << projGreen << std::endl;
	//		std::cout << RGB[2] << "  " << projBlue << std::endl;
	//		getchar();

	const float redError = sqr((projRed - RGB[0])*0.2990);
	const float greenError = sqr((projGreen - RGB[1])*0.5870);
	const float blueError = sqr((projBlue - RGB[2])*0.114);

	return sqrt(redError + greenError + blueError);
}

float Track::sumColorEnergy(const Eigen::Matrix4d &camera,
		const cv::Mat &depth_img, const cv::Mat &colorImg) {

	float energy = 0;

	for (int i = 0; i < depth_img.rows; ++i) {
		for (int j = 0; j < depth_img.cols; ++j) {
			float z = depth_img.at<float> (i, j);
			if (z > 0) {
				point local = this->grid->getLocal(j, i, z);
				point global = this->grid->Multiply(camera, local);
				pixel pix = { i, j };
				//				float rgbError = getRGBError(global, pix, colorImg);

				float rgbError = getTotalError(global, pix, colorImg);
				//				float rgbError = getEnergy(global);
				if (abs(rgbError < STOP2)) {

					energy += sqr(rgbError);
				}
			}
		}
	}
	return energy;
}

void Track::controlRGBEnergy(const cv::Mat &depthImg, const cv::Mat &colorImg,
		const Eigen::Matrix4d &camera) {
	Sophus::Affine3d tmp(camera);
	Sophus::SE3 G(tmp.rotation(), tmp.translation());
	Sophus::Vector6d Xi = G.log();
	const Sophus::Vector6d XI_OLD = Xi;
	float step = 0.001;
	float width = 0.3;

	std::ofstream myfile;
	myfile.open("Energy.txt");
	std::cout << "here" << std::endl;
	int index = 3;
	for (float i = -width; i <= width; i += step) {
		Xi(index) = XI_OLD(index) + i;
		const float energy = sumColorEnergy(Sophus::SE3::exp(Xi).matrix(),
				depthImg, colorImg);
		myfile << i << " " << energy << std::endl;
		std::cout << i << std::endl;
	}
	myfile.close();
}

Sophus::Vector6d Track::getRGBGradient(const point& p0, const pixel& pix0,
		const point& pdx, const point& pdy, const point& pdz, const point& prx,
		const point& pry, const point& prz, const cv::Mat& colorImg,
		const float V0) {

	Sophus::Vector6d tmp = Sophus::Vector6d::Zero();

	// Calculating the partial derivatives for the different coordinated in Xi-vector
	tmp(0) = getRGBError(pdx, pix0, colorImg) - V0;
	tmp(1) = getRGBError(pdy, pix0, colorImg) - V0;
	tmp(2) = getRGBError(pdz, pix0, colorImg) - V0;

	tmp(3) = getRGBError(prx, pix0, colorImg) - V0;
	tmp(4) = getRGBError(pry, pix0, colorImg) - V0;
	tmp(5) = getRGBError(prz, pix0, colorImg) - V0;

	// Check so that no point is reprojected outside the grid or where the weight is zero (we have no information)
	if (fabs(tmp(0)) > abs(STOP2) || fabs(tmp(1)) > abs(STOP2)
			|| fabs(tmp(2)) > abs(STOP2) || fabs(tmp(3)) > abs(STOP2)
			|| fabs(tmp(4)) > abs(STOP2) || fabs(tmp(5)) > abs(STOP2)) {
		tmp(0) = tmp(1) = tmp(2) = tmp(3) = tmp(4) = tmp(5) = 0;
	}
	tmp /= epsilon;

	return tmp;
}
Sophus::Vector6d Track::getGradient(const point& pdx, const point &pdy, const point &pdz, const point &prx, const point &pry, const point &prz, const float V0){

	Sophus::Vector6d tmp = Sophus::Vector6d::Zero();

	// Calculating the partial derivatives for the different coordinated in Xi-vector
	tmp(0) = getEnergy(pdx) - V0;
	tmp(1) = getEnergy(pdy) - V0;
	tmp(2) = getEnergy(pdz) - V0;

	tmp(3) = getEnergy(prx) - V0;
	tmp(4) = getEnergy(pry) - V0;
	tmp(5) = getEnergy(prz) - V0;

	// Check so that no point is reprojected outside the grid or where the weight is zero (we have no information)
	if (fabs(tmp(0)) > abs(STOP2) || fabs(tmp(1)) > abs(STOP2)
			|| fabs(tmp(2)) > abs(STOP2) || fabs(tmp(3)) > abs(STOP2)
			|| fabs(tmp(4)) > abs(STOP2) || fabs(tmp(5)) > abs(STOP2)) {
		tmp(0) = tmp(1) = tmp(2) = tmp(3) = tmp(4) = tmp(5) = 0;
	}
	tmp /= epsilon;

	return tmp;
}
Sophus::Vector6d Track::GetColorPose(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg, const cv::Mat &colorImg) {

	Sophus::Vector6d Xi = Xi_0;
	bool check = false;
	int itr = 0;

	while (!check) {
		Sophus::Vector6d tdx, tdy, tdz, rdx, rdy, rdz;
		Sophus::Vector6d prev = Xi;
		//	for (int itr = 0; itr < MAX_ITR; ++itr){

		prev = Xi;

		tdx = tdy = tdz = rdx = rdy = rdz = Xi;

		// This is for the numerical derivatives
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;

		rdx(3) += epsilon;
		rdy(4) += epsilon;
		rdz(5) += epsilon;

		const Eigen::Matrix4d G0 = Sophus::SE3::exp(Xi).matrix();

		const Eigen::Matrix4d Gdx = Sophus::SE3::exp(tdx).matrix();
		const Eigen::Matrix4d Gdy = Sophus::SE3::exp(tdy).matrix();
		const Eigen::Matrix4d Gdz = Sophus::SE3::exp(tdz).matrix();
		const Eigen::Matrix4d Grx = Sophus::SE3::exp(rdx).matrix();
		const Eigen::Matrix4d Gry = Sophus::SE3::exp(rdy).matrix();
		const Eigen::Matrix4d Grz = Sophus::SE3::exp(rdz).matrix();

		Sophus::Matrix6d A = Sophus::Matrix6d::Zero();
		Sophus::Vector6d b = Sophus::Vector6d::Zero();
		Sophus::Matrix6d A_col,A_dist;
		Sophus::Vector6d b_col,b_dist;
		A_col=A_dist=Sophus::Matrix6d::Zero();
		b_col=b_dist=Sophus::Vector6d::Zero();
		for (int i = 0; i < depthImg.rows; ++i) {
			for (int j = 0; j < depthImg.cols; ++j) {
				const float z = depthImg.at<float> (i, j);
				A_col=A_dist=Sophus::Matrix6d::Zero();
				b_col=b_dist=Sophus::Vector6d::Zero();
				if (z > 0) {
					const pixel pix = { i, j };
					const point local = grid->getLocal(j, i, z);
					const point g0 = grid->Multiply(G0, local);
					const float V0 = getRGBError(g0, pix, colorImg);

					point pdx = grid->Multiply(Gdx, local);
					point pdy = grid->Multiply(Gdy, local);
					point pdz = grid->Multiply(Gdz, local);

					point prx = grid->Multiply(Grx, local);
					point pry = grid->Multiply(Gry, local);
					point prz = grid->Multiply(Grz, local);

					if (V0 < STOP2) {

						// Get the global coordinates


						Sophus::Vector6d gradient = getRGBGradient(g0, pix,
								pdx, pdy, pdz, prx, pry, prz, colorImg, V0);
						bool valid = checkGradient(gradient);
						if (valid) {

							 b_col = alpha*V0 * gradient;
							A_col = (alpha)*gradient
									* gradient.transpose();
						}

					}
					const float V0_d = getEnergy(g0);
					if (V0_d < STOP2) {
						Sophus::Vector6d g_dist = getGradient(pdx, pdy, pdz, prx, pry, prz,V0_d);
						bool valid = checkGradient(g_dist);
						if (valid){
							b_dist = V0_d*g_dist;
							A_dist = g_dist*g_dist.transpose();
						}
					}

					b-=(b_col+b_dist);
					A += (A_col+A_dist);

				}
			}
		}

		itr++;
		//		std::cout<<A<<std::endl;
		//		getchar();
		Xi = A.ldlt().solve(A * Xi + b);

		if ((fabs(Xi(0) - prev(0)) < epsilon_stop && fabs(Xi(1) - prev(1))
				< epsilon_stop && fabs(Xi(2) - prev(2)) < epsilon_stop && fabs(
				Xi(3) - prev(3)) < epsilon_stop && fabs(Xi(4) - prev(4))
				< epsilon_stop && fabs(Xi(5) - prev(5)) < epsilon_stop) || itr
				== MAX_ITR) {
			check = true;
		}
	}
	std::cout << "Iterations  " << itr << std::endl;
	return Xi;
}

float Track::getTotalError(const point &p, const pixel &pix,
		const cv::Mat &colorImg) {
	float distError = fabs(getEnergy(p));

	float RGBError = fabs(getRGBError(p, pix, colorImg));
	if (RGBError > STOP2 && distError > STOP2) {
		return STOP;
	}
	if (distError < STOP2 && RGBError > STOP2) {
		return distError / DELTA;
	}
	if (distError > STOP2 && RGBError < STOP2) {
		return STOP;
	}
	//	if (RGBError < 0.05) {
	//		std::cout << "rgb: " << RGBError << std::endl;
	//		std::cout << "dist: " << distError / DELTA << std::endl;
	//		getchar();
	//	}
	return sqrt((sqr(distError / DELTA)) + (alpha) * sqr(RGBError));
}
Sophus::Vector6d Track::GetRGB_DPose(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg, const cv::Mat &colorImg) {
	Sophus::Vector6d Xi = Xi_0;
	bool check = false;
	int itr = 0;
	float energy_new, energy_old;
//	energy_new = sumColorEnergy(Sophus::SE3::exp(Xi).matrix(), depthImg,
//			colorImg);

	while (!check) {
		energy_old = energy_new;
		Sophus::Vector6d tdx, tdy, tdz, rdx, rdy, rdz;
		Sophus::Vector6d prev = Xi;
		//	for (int itr = 0; itr < MAX_ITR; ++itr){

		prev = Xi;

		tdx = tdy = tdz = rdx = rdy = rdz = Xi;

		// This is for the numerical derivatives
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;

		rdx(3) += epsilon;
		rdy(4) += epsilon;
		rdz(5) += epsilon;

		const Eigen::Matrix4d G0 = Sophus::SE3::exp(Xi).matrix();

		const Eigen::Matrix4d Gdx = Sophus::SE3::exp(tdx).matrix();
		const Eigen::Matrix4d Gdy = Sophus::SE3::exp(tdy).matrix();
		const Eigen::Matrix4d Gdz = Sophus::SE3::exp(tdz).matrix();
		const Eigen::Matrix4d Grx = Sophus::SE3::exp(rdx).matrix();
		const Eigen::Matrix4d Gry = Sophus::SE3::exp(rdy).matrix();
		const Eigen::Matrix4d Grz = Sophus::SE3::exp(rdz).matrix();

		Sophus::Matrix6d A = Sophus::Matrix6d::Zero();
		Sophus::Vector6d b = Sophus::Vector6d::Zero();

		int i, j;
		float z, V0, V0_rgb;

		point local, global;
		pixel pix;
		Sophus::Vector6d g_dist, g_rgb;
		bool valid;
		float A00, A01, A02, A03, A04, A05, A11, A12, A13, A14, A15, A22, A23,
				A24, A25, A33, A34, A35, A44, A45, A55, b0, b1, b2, b3, b4, b5;

		A00 = A01 = A02 = A03 = A04 = A05 = A11 = A12 = A13 = A14 = A15 = A22
				= A23 = A24 = A25 = A33 = A34 = A35 = A44 = A45 = A55 = b0 = b1
						= b2 = b3 = b4 = b5 = 0;



		point pdx,pdy,pdz,prx,pry,prz,p0;
		Sophus::Vector6d b_col,b_dist;
		Sophus::Matrix6d A_col,A_dist;

#pragma omp parallel for shared(depthImg, colorImg) private(V0_rgb,b_col,b_dist,A_col,A_dist,p0,pdx,pdy,pdz,prx,pry,prz,i,j,z,local,global,V0,g_dist,g_rgb,pix, valid) reduction(+:A00,A01,A02,A03,A04,A05,A11,A12,A13,A14,A15,A22,A23,A24,A25,A33,A34,A35,A44,A45,A55) reduction(-:b0,b1,b2,b3,b4,b5)

		for (i = 0; i < depthImg.rows; i ++) {
			for (j = 0; j < depthImg.cols; j ++) {
				A_col=A_dist=Sophus::Matrix6d::Zero();
				b_col=b_dist=Sophus::Vector6d::Zero();

				z = depthImg.at<float> (i, j);
				if (z > 0) {
					pix = {i,j};
					local = grid->getLocal(j, i, z);
					point p0 = grid->Multiply(G0, local);

					point pdx = grid->Multiply(Gdx, local);
					point pdy = grid->Multiply(Gdy, local);
					point pdz = grid->Multiply(Gdz, local);

					point prx = grid->Multiply(Grx, local);
					point pry = grid->Multiply(Gry, local);
					point prz = grid->Multiply(Grz, local);
					V0 = getEnergy(p0);

					if (fabs(V0) < STOP2) {

						g_dist = Sophus::Vector6d::Zero();
						//						g(0) = getTotalError(grid->Multiply(Gdx, local),pix,colorImg) - V0;
							//						g(1) = getTotalError(grid->Multiply(Gdy, local),pix,colorImg) - V0;
							//						g(2) = getTotalError(grid->Multiply(Gdz, local),pix,colorImg) - V0;
							//
							//						g(3) = getTotalError(grid->Multiply(Grx, local),pix,colorImg) - V0;
							//						g(4) = getTotalError(grid->Multiply(Gry, local),pix,colorImg) - V0;
							//						g(5) = getTotalError(grid->Multiply(Grz, local),pix,colorImg) - V0;

							g_dist = getGradient(pdx,pdy,pdz,prx,pry,prz,V0);
//							std::cout<<pdx.x<<std::endl;
//										getchar();
							valid = checkGradient(g_dist);
							if (valid) {

								b_dist = V0*g_dist;
								A_dist = g_dist*g_dist.transpose();
							}
					}

					V0_rgb = getRGBError(p0,pix,colorImg);
					if (V0_rgb < STOP2){
						g_rgb = getRGBGradient(p0, pix, pdx,pdy,pdz,prx,pry,prz,colorImg,V0_rgb);
						valid = checkGradient(g_rgb);
						if (valid){
							b_col = alpha*V0_rgb*g_rgb;
							A_col = alpha*g_rgb*g_rgb.transpose();
						}
					}

								b0 -= (b_dist(0)+ b_col(0));
								b1 -= (b_dist(1)+ b_col(1));
								b2 -= (b_dist(2)+ b_col(2));
								b3 -= (b_dist(3)+ b_col(3));
								b4 -= (b_dist(4)+ b_col(4));
								b5 -= (b_dist(5)+ b_col(5));

								A00 += (A_col(0,0) + A_dist(0,0));
								A01 += (A_col(0,1) + A_dist(0,1));
								A02 += (A_col(0,2) + A_dist(0,2));
								A03 += (A_col(0,3) + A_dist(0,3));
								A04 += (A_col(0,4) + A_dist(0,4));
								A05 += (A_col(0,5) + A_dist(0,5));

								//						A10 += g(1)*g(0);
								A11 += (A_col(1,1) + A_dist(1,1));
								A12 += (A_col(1,2) + A_dist(1,2));
								A13 += (A_col(1,3) + A_dist(1,3));
								A14 += (A_col(1,4) + A_dist(1,4));
								A15 += (A_col(1,5) + A_dist(1,5));

								//						A20 += g(2)*g(0);
								//						A21 += g(2)*g(1);
								A22 += (A_col(2,2) + A_dist(2,2));
								A23 += (A_col(2,3) + A_dist(2,3));
								A24 += (A_col(2,4) + A_dist(2,4));
								A25 += (A_col(2,5) + A_dist(2,5));

								//						A30 += g(3)*g(0);
								//						A31 += g(3)*g(1);
								//						A32 += g(3)*g(2);
								A33 += (A_col(3,3) + A_dist(3,3));
								A34 += (A_col(3,4) + A_dist(3,4));
								A35 += (A_col(3,5) + A_dist(3,5));

								//						A40 += g(4)*g(0);
								//						A41 += g(4)*g(1);
								//						A42 += g(4)*g(2);
								//						A43 += g(4)*g(3);
								A44 += (A_col(4,4) + A_dist(4,4));
								A45 += (A_col(4,5) + A_dist(4,5));

								//						A50 += g(5)*g(0);
								//						A51 += g(5)*g(1);
								//						A52 += g(5)*g(2);
								//						A53 += g(5)*g(3);
								//						A54 += g(5)*g(4);
								A55 += (A_col(5,5) + A_dist(5,5));


							}


					}
				}


			A(0,0) = A00;
			A(0,1) = A01;
			A(0,2) = A02;
			A(0,3) = A03;
			A(0,4) = A04;
			A(0,5) = A05;

			A(1,0) = A01;
			A(1,1) = A11;
			A(1,2) = A12;
			A(1,3) = A13;
			A(1,4) = A14;
			A(1,5) = A15;

			A(2,0) = A02;
			A(2,1) = A12;
			A(2,2) = A22;
			A(2,3) = A23;
			A(2,4) = A24;
			A(2,5) = A25;

			A(3,0) = A03;
			A(3,1) = A13;
			A(3,2) = A23;
			A(3,3) = A33;
			A(3,4) = A34;
			A(3,5) = A35;

			A(4,0) = A04;
			A(4,1) = A14;
			A(4,2) = A24;
			A(4,3) = A34;
			A(4,4) = A44;
			A(4,5) = A45;

			A(5,0) = A05;
			A(5,1) = A15;
			A(5,2) = A25;
			A(5,3) = A35;
			A(5,4) = A45;
			A(5,5) = A55;

			b(0) = b0;
			b(1) = b1;
			b(2) = b2;
			b(3) = b3;
			b(4) = b4;
			b(5) = b5;
			itr++;
			//		std::cout<<A<<std::endl;
			//		getchar();
			Xi = A.ldlt().solve(A * Xi + b);

			//			energy_new = sumColorEnergy(Sophus::SE3::exp(Xi).matrix(), depthImg, colorImg);
			if ((fabs(Xi(0) - prev(0)) < epsilon_stop && fabs(Xi(1) - prev(1))
							< epsilon_stop && fabs(Xi(2) - prev(2)) < epsilon_stop && fabs(
									Xi(3) - prev(3)) < epsilon_stop && fabs(Xi(4) - prev(4))
							< epsilon_stop && fabs(Xi(5) - prev(5)) < epsilon_stop) || itr
					== MAX_ITR) {
				check = true;
			}
			//						if (energy_new >= energy_old || MAX_ITR == itr){
			//							check = true;
			//						}
			//		Sophus::Vector6d tmp = Xi - prev;
			//		if (tmp.norm() < epsilon_stop || itr == MAX_ITR){
			//			check = true;
			//		}
		}
		std::cout << "Iterations  " << itr << std::endl;
		return Xi;
	}

Sophus::Vector6d Track::SteepestDescent(const Sophus::Vector6d &Xi_0,
		const cv::Mat &depthImg, const cv::Mat &colorImg) {
	const float tau = 0.1;

	Sophus::Vector6d Xi = Xi_0;
	bool converge = false;
	int itr = 0;
	while (!converge) {
		Sophus::Vector6d tdx, tdy, tdz, rdx, rdy, rdz;
		tdx = tdy = tdz = rdx = rdy = rdz = Xi;
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;
		rdx(3) += epsilon;
		rdy(4) += epsilon;
		rdz(5) += epsilon;

		const float E0 = sumEnergy(depthImg, Sophus::SE3::exp(Xi).matrix());
		const float Edx = sumEnergy(depthImg, Sophus::SE3::exp(tdx).matrix());
		const float Edy = sumEnergy(depthImg, Sophus::SE3::exp(tdy).matrix());
		const float Edz = sumEnergy(depthImg, Sophus::SE3::exp(tdz).matrix());

		const float Erx = sumEnergy(depthImg, Sophus::SE3::exp(rdx).matrix());
		const float Ery = sumEnergy(depthImg, Sophus::SE3::exp(rdy).matrix());
		const float Erz = sumEnergy(depthImg, Sophus::SE3::exp(rdz).matrix());

		Sophus::Vector6d g = Sophus::Vector6d::Zero();
		g(0) = Edx - E0;
		g(1) = Edy - E0;
		g(2) = Edz - E0;
		g(3) = Erx - E0;
		g(4) = Ery - E0;
		g(5) = Erz - E0;

		Xi -= tau * g;

		itr++;

		if (itr == MAX_ITR) {
			converge = true;
		}

	}
	return Xi;
}
Eigen::MatrixXd Track::getDerivative(Eigen::Vector3d &w, Eigen::Vector3d &x)
{
	Eigen::Matrix3d S0, S1, S2;
	S0 << 0, 0, 0, 0, 0, -1, 0, 1, 0;
	S1 << 0, 0, 1, 0, 0, 0, -1, 0, 0;
	S2 << 0, -1, 0, 1, 0, 0, 0, 0, 0;
	// Calculate the norm of the vector w
	double wNorm = sqrt(sqr(w(0)) + sqr(w(1)) + sqr(w(2)));
	// Calculate the derivative with respect to t
	Eigen::Vector3d drdt1(1, 0, 0);
	Eigen::Vector3d drdt2(0, 1, 0);
	Eigen::Vector3d drdt3(0, 0, 1);
	// If wNorm > 0 we compute the derivatives for w, otherwise the Jacobian is [0, dr/dt]
	if (wNorm > 0) {
		Eigen::Matrix3d wHat;
		// Construct the skew symmetric matrix
		wHat << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
		//cout << " wHat \n" << wHat << endl;
		// Differentiate f(w) =  g1(w)*w_hat + g2(w)*w_hat^2
		double g1 = sin(wNorm) / wNorm;
		double g2 = (1 - cos(wNorm)) / sqr(wNorm);
		// The derivatives of g1 and g2 w.r.t. w0,w1,w2

		double dg1dw0 = (w(0) * cos(wNorm)) / sqr(wNorm)
			- (w(0) * sin(wNorm)) / (sqr(wNorm) * wNorm);
		double dg2dw0 = (w(0) * sin(wNorm)) / (sqr(wNorm) * wNorm)
			- (2 * w(0) * (1 - cos(wNorm))) / (sqr(sqr(wNorm)));


		Eigen::Matrix3d dfdw0 = dg1dw0 * wHat + g1 * S0 + dg2dw0 * wHat * wHat
			+ g2 * (S0 * wHat + wHat * S0);
		//cout << "dfdw0: " << dfdw0 << endl;
		double dg1dw1 = (w(1) * cos(wNorm)) / sqr(wNorm)
			- (w(1) * sin(wNorm)) / (sqr(wNorm) * wNorm);
		double dg2dw1 =
			(w(1) * sin(wNorm))
			/ (sqr(wNorm) * wNorm) - (2 * w(1)*(1 - cos(wNorm))) / sqr(sqr(wNorm));
		//printf("dg1dw1 %f\n dg2dw1 %f\n", dg1dw1, dg2dw1);
		Eigen::Matrix3d dfdw1 = dg1dw1 * wHat + g1 * S1 + dg2dw1 * wHat * wHat
			+ g2 * (S1 * wHat + wHat * S1);
		//cout << "dfdw1: \n" << dfdw1 << endl;
		double dg1dw2 = (w(2) * cos(wNorm)) / sqr(wNorm)
			- (w(2) * sin(wNorm)) / (sqr(wNorm) * wNorm);
		double dg2dw2 =
			(w(2) * sin(wNorm))
			/ (sqr(wNorm) * wNorm) - (2 * w(2)*(1 - cos(wNorm))) / sqr(sqr(wNorm));

		Eigen::Matrix3d dfdw2 = dg1dw2 * wHat + g1 * S2 + dg2dw2 * wHat * wHat
			+ g2 * (S2 * wHat + wHat * S2);

		// Compute part of the jacobian
		Eigen::Vector3d drdw0 = dfdw0 * x;
		Eigen::Vector3d drdw1 = dfdw1 * x;
		Eigen::Vector3d drdw2 = dfdw2 * x;

		Eigen::MatrixXd J(3, 6);
		J.block<3, 1>(0, 0) = drdw0;
		J.block<3, 1>(0, 1) = drdw1;
		J.block<3, 1>(0, 2) = drdw2;
		J.block<3, 1>(0, 3) = drdt1;
		J.block<3, 1>(0, 4) = drdt2;
		J.block<3, 1>(0, 5) = drdt3;
		return J;
	}
	else {
		Eigen::Vector3d drdw0 = S0 * x;
		Eigen::Vector3d drdw1 = S1 * x;
		Eigen::Vector3d drdw2 = S2 * x;
		Eigen::MatrixXd J(3, 6);
		J.block<3, 1>(0, 0) = drdw0;
		J.block<3, 1>(0, 1) = drdw1;
		J.block<3, 1>(0, 2) = drdw2;

		J.block<3, 1>(0, 3) = drdt1;
		J.block<3, 1>(0, 4) = drdt2;
		J.block<3, 1>(0, 5) = drdt3;
		return J;
	}
}

Eigen::VectorXd Track::getAnalyticGradient(Eigen::Vector3d &w, Eigen::Vector3d &t, Eigen::Vector3d &pLocal,
	const double V0, const double V0Dx, const double V0Dy, const double V0Dz, const double delta){

	// Compute outer derivative:
	float dx = (V0Dx - V0);
	float dy = (V0Dy - V0);
	float dz = (V0Dz - V0);

	//if (fabs(dx) > DIFFTHRES || fabs(dy) > DIFFTHRES || fabs(dz) > DIFFTHRES){
	//	dx = NAN;
	//	dy = NAN;
	//	dz = NAN;
	//}
	dx /= delta;
	dy /= delta;
	dz /= delta;

	Eigen::Vector3d outerDer(dx, dy, dz);
	Eigen::MatrixXd J = getDerivative(w, pLocal);
	Eigen::MatrixXd grad = (outerDer.transpose() * J);
	return grad.transpose();
}

Sophus::Vector6d Track::getAnalyticPose(const Sophus::Vector6d &Xi_0,
	const cv::Mat &depthImg) {
	Sophus::Vector6d Xi = Xi_0;
	Eigen::Matrix4d tmpCamera = Sophus::SE3::exp(Xi).matrix();
	Eigen::Vector3d t(tmpCamera(0, 3), tmpCamera(1, 3), tmpCamera(2, 3));
	Sophus::Affine3d tmp(tmpCamera);
	Sophus::SO3 P(tmp.rotation());
	Eigen::Vector3d w0 = Sophus::SO3::log(P);
	Xi(0) = w0(0);
	Xi(1) = w0(1);
	Xi(2) = w0(2);

	Xi(3) = t(0);
	Xi(4) = t(1);
	Xi(5) = t(2);

	bool check = false;
	int itr = 0;
	while (!check) {
		//Sophus::Vector6d tdx, tdy, tdz, rdx, rdy, rdz;
		Sophus::Vector6d prev = Xi;
		//	for (int itr = 0; itr < MAX_ITR; ++itr){

		Eigen::Vector3d t(Xi(3), Xi(4), Xi(5));
		Eigen::Vector3d w(Xi(0), Xi(1), Xi(2));
		//tdx = tdy = tdz = rdx = rdy = rdz = Xi;
		Eigen::Vector3d tdx = t;
		Eigen::Vector3d tdy = t;
		Eigen::Vector3d tdz = t;

		// This is for the numerical derivatives
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;


		const Eigen::Matrix3d R = Sophus::SO3::exp(w).matrix();
		Eigen::Matrix4d G0 = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 3; ++i){
			for (int j = 0; j < 3; ++j){
				G0(i, j) = R(i, j);
			}
		}
		G0(0, 3) = t(0);
		G0(1, 3) = t(1);
		G0(2, 3) = t(2);

		Eigen::Matrix4d Gdx = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d Gdy = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d Gdz = Eigen::Matrix4d::Identity();
		Gdx = Gdy = Gdz = G0;
		Gdx(0, 3) = tdx(0);
		Gdy(1, 3) = tdy(1);
		Gdz(2, 3) = tdz(2);

		Sophus::Matrix6d A = Sophus::Matrix6d::Zero();
		Sophus::Vector6d b = Sophus::Vector6d::Zero();
		int nbrofpoints = 0;
		for (int i = 0; i < depthImg.rows; ++i) {
			for (int j = 0; j < depthImg.cols; ++j) {
				const float z = depthImg.at<float>(i, j);
				if (z > 0) {
					const point local = grid->getLocal(j, i, z);
					const point g0 = grid->Multiply(G0, local);
					const float V0 = getEnergy(g0);
					if (V0 < (STOP2)) {
						//						float weight = getLSWeight(V0);
						float Vdx = getEnergy(grid->Multiply(Gdx, local));
						float Vdy = getEnergy(grid->Multiply(Gdy, local));
						float Vdz = getEnergy(grid->Multiply(Gdz, local));
						Eigen::Vector3d xLocal(local.x, local.y, local.z);
						Sophus::Vector6d  gradient = getAnalyticGradient(w, t, xLocal, V0, Vdx, Vdy, Vdz, epsilon);

						//						std::cout<<"V0: "<<V0<<std::endl;
						//						std::cout<<gradient<<std::endl;
						//						getchar();


						bool valid = checkGradient(gradient);
						if (valid) {
						
							b -= V0 * gradient;
							A += gradient * gradient.transpose();
						}
					}
				}
			}
		}
		//		std::cout<<nbrofpoints<<std::endl;
		//		getchar();
		//		std::cout<<A<<std::endl;
		//		getchar();
		//		A /= sqr(epsilon);
		//		b /= epsilon;
		itr++;
		//		std::cout<<A<<std::endl;
		//		getchar();
		Xi = A.ldlt().solve(A * Xi + b);

		if ((fabs(Xi(0) - prev(0)) < epsilon_stop && fabs(Xi(1) - prev(1))
			< epsilon_stop && fabs(Xi(2) - prev(2)) < epsilon_stop && fabs(
			Xi(3) - prev(3)) < epsilon_stop && fabs(Xi(4) - prev(4))
			< epsilon_stop && fabs(Xi(5) - prev(5)) < epsilon_stop) || itr
			== MAX_ITR) {
			check = true;
		}
	}
	std::cout << "Analytic Iterations  " << itr << std::endl;
	
	Eigen::Vector3d w(Xi(0), Xi(1), Xi(2));
	Eigen::Matrix3d R = Sophus::SO3::exp(w).matrix();
	Eigen::Matrix4d Camera = Eigen::Matrix4d::Identity();
	for (int i = 0; i < 3; ++i){
		for (int j = 0; j < 3; ++j){
			Camera(i, j) = R(i, j);
		}
	}
	Camera(0, 3) = Xi(3);
	Camera(1, 3) = Xi(4);
	Camera(2, 3) = Xi(5);
	Sophus::Affine3d tmpLast(Camera);
	Sophus::SE3 PL(tmpLast.rotation(), tmpLast.translation());

	return PL.log();

}

Sophus::Vector6d Track::GetAnalyticRGB_DPose(const Sophus::Vector6d &Xi_0,
	const cv::Mat &depthImg, const cv::Mat &colorImg) {
	Sophus::Vector6d Xi = Xi_0;

	// Transform SE3 to SO3 + Translation
	Eigen::Matrix4d tmpCamera = Sophus::SE3::exp(Xi).matrix();
	Eigen::Vector3d t(tmpCamera(0, 3), tmpCamera(1, 3), tmpCamera(2, 3));
	Sophus::Affine3d tmp(tmpCamera);
	Sophus::SO3 P(tmp.rotation());
	Eigen::Vector3d w0 = Sophus::SO3::log(P);
	Xi(0) = w0(0);
	Xi(1) = w0(1);
	Xi(2) = w0(2);

	Xi(3) = t(0);
	Xi(4) = t(1);
	Xi(5) = t(2);


	bool check = false;
	int itr = 0;
	float energy_new, energy_old;
	//	energy_new = sumColorEnergy(Sophus::SE3::exp(Xi).matrix(), depthImg,
	//			colorImg);

	while (!check) {
		energy_old = energy_new;

		Sophus::Vector6d prev = Xi;
		// Transform from SE3 to SO3 format

		Eigen::Vector3d t(Xi(3), Xi(4), Xi(5));
		Eigen::Vector3d w(Xi(0), Xi(1), Xi(2));
		//tdx = tdy = tdz = rdx = rdy = rdz = Xi;
		Eigen::Vector3d tdx = t;
		Eigen::Vector3d tdy = t;
		Eigen::Vector3d tdz = t;

		// This is for the numerical derivatives
		tdx(0) += epsilon;
		tdy(1) += epsilon;
		tdz(2) += epsilon;


		const Eigen::Matrix3d R = Sophus::SO3::exp(w).matrix();
		Eigen::Matrix4d G0 = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 3; ++i){
			for (int j = 0; j < 3; ++j){
				G0(i, j) = R(i, j);
			}
		}
		G0(0, 3) = t(0);
		G0(1, 3) = t(1);
		G0(2, 3) = t(2);

		Eigen::Matrix4d Gdx = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d Gdy = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d Gdz = Eigen::Matrix4d::Identity();
		Gdx = Gdy = Gdz = G0;
		Gdx(0, 3) = tdx(0);
		Gdy(1, 3) = tdy(1);
		Gdz(2, 3) = tdz(2);


		
		Sophus::Matrix6d A = Sophus::Matrix6d::Zero();
		Sophus::Vector6d b = Sophus::Vector6d::Zero();

		int i, j;
		float z, V0, V0_rgb;

		point local, global;
		pixel pix;
		Sophus::Vector6d g_dist, g_rgb;
		bool valid, validRGB;
		float A00, A01, A02, A03, A04, A05, A11, A12, A13, A14, A15, A22, A23,
			A24, A25, A33, A34, A35, A44, A45, A55, b0, b1, b2, b3, b4, b5;

		A00 = A01 = A02 = A03 = A04 = A05 = A11 = A12 = A13 = A14 = A15 = A22
			= A23 = A24 = A25 = A33 = A34 = A35 = A44 = A45 = A55 = b0 = b1
			= b2 = b3 = b4 = b5 = 0;



		point pdx, pdy, pdz, prx, pry, prz, p0;
		Sophus::Vector6d b_col, b_dist;
		Sophus::Matrix6d A_col, A_dist;

#pragma omp parallel for shared(depthImg, colorImg) private(V0_rgb,b_col,b_dist,A_col,A_dist,p0,pdx,pdy,pdz,prx,pry,prz,i,j,z,local,global,V0,g_dist,g_rgb,pix, valid,validRGB) reduction(+:A00,A01,A02,A03,A04,A05,A11,A12,A13,A14,A15,A22,A23,A24,A25,A33,A34,A35,A44,A45,A55) reduction(-:b0,b1,b2,b3,b4,b5)

		for (i = 0; i < depthImg.rows; i++) {
			for (j = 0; j < depthImg.cols; j++) {
				A_col = A_dist = Sophus::Matrix6d::Zero();
				b_col = b_dist = Sophus::Vector6d::Zero();

				z = depthImg.at<float>(i, j);
				if (z > 0) {
					pix = { i, j };
					local = grid->getLocal(j, i, z);
					point p0 = grid->Multiply(G0, local);

	
					V0 = getEnergy(p0);
					V0_rgb = getRGBError(p0, pix, colorImg);
					if (fabs(V0) < STOP2 && fabs(V0_rgb) < STOP2) {

						g_dist = Sophus::Vector6d::Zero();
						g_rgb = g_dist;
						Eigen::Vector3d xLocal(local.x, local.y, local.z);
						Eigen::Vector3d xGlobal(p0.x, p0.y, p0.z);
	
						getAnalyticGradientComb(w, t, xGlobal, xLocal, V0, V0_rgb, colorImg, pix, epsilon, g_dist, g_rgb);

						valid = checkGradient(g_dist);
						validRGB = checkGradient(g_rgb);
						if (valid && validRGB) {

							b_dist = (1-alpha)*V0*g_dist;
							A_dist = (1-alpha)*g_dist*g_dist.transpose();

							b_col = alpha*V0_rgb*g_rgb;
							A_col = alpha*g_rgb * g_rgb.transpose();
						}
					}



					b0 -= (b_dist(0) + b_col(0));
					b1 -= (b_dist(1) + b_col(1));
					b2 -= (b_dist(2) + b_col(2));
					b3 -= (b_dist(3) + b_col(3));
					b4 -= (b_dist(4) + b_col(4));
					b5 -= (b_dist(5) + b_col(5));

					A00 += (A_col(0, 0) + A_dist(0, 0));
					A01 += (A_col(0, 1) + A_dist(0, 1));
					A02 += (A_col(0, 2) + A_dist(0, 2));
					A03 += (A_col(0, 3) + A_dist(0, 3));
					A04 += (A_col(0, 4) + A_dist(0, 4));
					A05 += (A_col(0, 5) + A_dist(0, 5));

					//						A10 += g(1)*g(0);
					A11 += (A_col(1, 1) + A_dist(1, 1));
					A12 += (A_col(1, 2) + A_dist(1, 2));
					A13 += (A_col(1, 3) + A_dist(1, 3));
					A14 += (A_col(1, 4) + A_dist(1, 4));
					A15 += (A_col(1, 5) + A_dist(1, 5));

					//						A20 += g(2)*g(0);
					//						A21 += g(2)*g(1);
					A22 += (A_col(2, 2) + A_dist(2, 2));
					A23 += (A_col(2, 3) + A_dist(2, 3));
					A24 += (A_col(2, 4) + A_dist(2, 4));
					A25 += (A_col(2, 5) + A_dist(2, 5));

					//						A30 += g(3)*g(0);
					//						A31 += g(3)*g(1);
					//						A32 += g(3)*g(2);
					A33 += (A_col(3, 3) + A_dist(3, 3));
					A34 += (A_col(3, 4) + A_dist(3, 4));
					A35 += (A_col(3, 5) + A_dist(3, 5));

					//						A40 += g(4)*g(0);
					//						A41 += g(4)*g(1);
					//						A42 += g(4)*g(2);
					//						A43 += g(4)*g(3);
					A44 += (A_col(4, 4) + A_dist(4, 4));
					A45 += (A_col(4, 5) + A_dist(4, 5));

					//						A50 += g(5)*g(0);
					//						A51 += g(5)*g(1);
					//						A52 += g(5)*g(2);
					//						A53 += g(5)*g(3);
					//						A54 += g(5)*g(4);
					A55 += (A_col(5, 5) + A_dist(5, 5));


				}


			}
		}


		A(0, 0) = A00;
		A(0, 1) = A01;
		A(0, 2) = A02;
		A(0, 3) = A03;
		A(0, 4) = A04;
		A(0, 5) = A05;

		A(1, 0) = A01;
		A(1, 1) = A11;
		A(1, 2) = A12;
		A(1, 3) = A13;
		A(1, 4) = A14;
		A(1, 5) = A15;

		A(2, 0) = A02;
		A(2, 1) = A12;
		A(2, 2) = A22;
		A(2, 3) = A23;
		A(2, 4) = A24;
		A(2, 5) = A25;

		A(3, 0) = A03;
		A(3, 1) = A13;
		A(3, 2) = A23;
		A(3, 3) = A33;
		A(3, 4) = A34;
		A(3, 5) = A35;

		A(4, 0) = A04;
		A(4, 1) = A14;
		A(4, 2) = A24;
		A(4, 3) = A34;
		A(4, 4) = A44;
		A(4, 5) = A45;

		A(5, 0) = A05;
		A(5, 1) = A15;
		A(5, 2) = A25;
		A(5, 3) = A35;
		A(5, 4) = A45;
		A(5, 5) = A55;

		b(0) = b0;
		b(1) = b1;
		b(2) = b2;
		b(3) = b3;
		b(4) = b4;
		b(5) = b5;
		itr++;
		//		std::cout<<A<<std::endl;
		//		getchar();
		Xi = A.ldlt().solve(A * Xi + b);

		//			energy_new = sumColorEnergy(Sophus::SE3::exp(Xi).matrix(), depthImg, colorImg);
		if ((fabs(Xi(0) - prev(0)) < epsilon_stop && fabs(Xi(1) - prev(1))
			< epsilon_stop && fabs(Xi(2) - prev(2)) < epsilon_stop && fabs(
			Xi(3) - prev(3)) < epsilon_stop && fabs(Xi(4) - prev(4))
			< epsilon_stop && fabs(Xi(5) - prev(5)) < epsilon_stop) || itr
			== MAX_ITR) {
			check = true;
		}
		//						if (energy_new >= energy_old || MAX_ITR == itr){
		//							check = true;
		//						}
		//		Sophus::Vector6d tmp = Xi - prev;
		//		if (tmp.norm() < epsilon_stop || itr == MAX_ITR){
		//			check = true;
		//		}
	}
	std::cout << "Analytic Iterations  " << itr << " alpha: "<<alpha << std::endl;

	Eigen::Vector3d w(Xi(0), Xi(1), Xi(2));
	Eigen::Matrix3d R = Sophus::SO3::exp(w).matrix();
	Eigen::Matrix4d Camera = Eigen::Matrix4d::Identity();
	for (int i = 0; i < 3; ++i){
		for (int j = 0; j < 3; ++j){
			Camera(i, j) = R(i, j);
		}
	}
	Camera(0, 3) = Xi(3);
	Camera(1, 3) = Xi(4);
	Camera(2, 3) = Xi(5);
	Sophus::Affine3d tmpLast(Camera);
	Sophus::SE3 PL(tmpLast.rotation(), tmpLast.translation());

	return PL.log();
}

void Track::getAnalyticGradientComb(Eigen::Vector3d& w,
	Eigen::Vector3d& t, Eigen::Vector3d& xGlobal, Eigen::Vector3d& xLocal,
	float V0, float V0_RGB, const cv::Mat& colorImg, pixel& pix,
	float delta, Sophus::Vector6d &gradStruct, Sophus::Vector6d &gradRGB) {
	Eigen::Vector3f e1(delta, 0, 0);
	Eigen::Vector3f e2(0, delta, 0);
	Eigen::Vector3f e3(0, 0, delta);

	point e1_ = {delta + xGlobal(0),xGlobal(1),xGlobal(2)};
	point e2_ = { xGlobal(0), xGlobal(1) + delta, xGlobal(2) };
	point e3_ = { xGlobal(0), xGlobal(1), xGlobal(2) + delta};

	// Compute outer derivative structure
	float dx = (getEnergy(e1_) - V0) / delta;
	float dy = (getEnergy(e2_) - V0) / delta;
	float dz = (getEnergy(e3_)- V0) / delta;
	Eigen::Vector3d outerDer(dx, dy, dz);

	// Compute outer derivative RGB


	float dxRGB = (getRGBError(e1_, pix, colorImg) - V0_RGB) / delta;
	float dyRGB = (getRGBError(e2_, pix, colorImg) - V0_RGB) / delta;
	float dzRGB = (getRGBError(e3_, pix, colorImg) - V0_RGB) / delta;
	Eigen::Vector3d outerDerRGB(dxRGB, dyRGB, dzRGB);

	// Compute partiale derivative with respect w1
	Eigen::MatrixXd J = getDerivative(w, xLocal);
	Eigen::VectorXd tmpStruct = outerDer.transpose() * J;
	Eigen::VectorXd tmpRGB = outerDerRGB.transpose() * J;
	for (int i = 0; i < 6; i++){
		gradStruct(i) = tmpStruct(i);
		gradRGB(i) = tmpRGB(i);
	}
}