/*
 * Voxelgrid.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: erikbylow
 */

#include "Voxelgrid.h"
// Initializing the grid
static long int Memory;
int Neig = 1;

void setBoolandCdata(GridDisc *grid, const int cResolution) {
	for (int i = 0; i < cResolution; ++i) {
		for (int j = 0; j < cResolution; ++j) {
			for (int k = 0; k < cResolution; ++k) {
				grid[i + j * cResolution + k * cResolution * cResolution].isOccupied
						= false;
				//				memset(
				//						grid[i + j * cResolution + k * cResolution
				//								* cResolution].cdata, 0, 2*8 * sizeof(float));
			}
		}
	}
}

Voxelgrid::Voxelgrid(const float size, const int cResolution,
		const int fResolution, const float DELTAC) {
	const size_t csize = cResolution * cResolution * cResolution
			* sizeof(GridDisc);
	grid = (GridDisc*) malloc(csize);
	clength = size;
	if (cResolution > 1) {
		clength = size / (cResolution - 1);
	}
	flength = clength / (fResolution - 1);
	this->DELTAC = DELTAC;
	this->SIGMAC = -log(LOW) / sqr(EPSILON-DELTAC);
	Neig = int(ceil(DELTA/clength));
	std::cout << "flength: " << flength << std::endl;
	std::cout << "clength: " << clength << std::endl;
	std::cout<< "Neig: "<<Neig<<std::endl;
	setBoolandCdata(this->grid, cResolution);
	std::cout << "\nVoxelgrid created!\n" << std::endl;
	Memory += csize;
}

Voxelgrid::~Voxelgrid() {
	std::cout << "In destructor" << std::endl;
		for (int i = 0; i < cResolution; ++i) {
			for (int j = 0; j < cResolution; ++j) {
				for (int k = 0; k < cResolution; ++k) {
					if (grid[i + j * cResolution + k*cResolution*cResolution].isOccupied){
						free(grid[i + j*cResolution + k*cResolution*cResolution].fdata);
					}
				}
			}
		}
		free(grid);
}
// Return the 3D coordinate in the (local) camera frame
point Voxelgrid::getLocal(float x, float y, float z) {
	point tmp = { (x - cx) * z / fx, (y - cy) * z / fy, z };
	return tmp;
}
// Multiplying a 3D point with a rotation and translation matrix
point Voxelgrid::Multiply(const Eigen::Matrix4d &camera, const point &p) {
	point tmp = { camera(0, 0) * p.x + camera(0, 1) * p.y + camera(0, 2) * p.z
			+ camera(0, 3), camera(1, 0) * p.x + camera(1, 1) * p.y + camera(1,
			2) * p.z + camera(1, 3), camera(2, 0) * p.x + camera(2, 1) * p.y
			+ camera(2, 2) * p.z + camera(2, 3) };
	return tmp;
}
// Returns the index in the grid for a 3D-coordinate
Index Voxelgrid::getIndex(const point &p) {
	int i = floor((p.x + x_s) / (clength)); // x = -xs + i*step
	int j = floor((p.y + y_s) / (clength));
	int k = floor((p.z + z_s) / clength);
	Index tmp = { i, j, k };
	return tmp;
}

void checkInitialization(int INDEX, Data *grid) {
	std::cout << "Here" << std::endl;
	//	for (int i = 0; i < 2; ++i) {
	//		std::cout << i << std::endl;
	//		for (int j = 0; j < 2; ++j)
	//			for (int k = 0; k < 2; ++k) {
	//				std::cout << grid[i + j * 2 + k * 2
	//						* 2].distance << "  and w  " << grid[i + j
	//						* 2 + k * 2 * 2].distance
	//						<< " " << std::endl;
	//			}
	for (int i = 0; i < 8; ++i) {
		std::cout << grid[i].distance << "  and w  " << grid[i].weight
				<< std::endl;
	}
	std::cout << std::endl;
}

void Voxelgrid::split(int i, int j, int k, Index &index, size_t fsize) {

	if (index.i + i >= 0 && index.j + j >= 0 && index.k + k >= 0 && index.i + i
			< cResolution && index.j + j < cResolution && index.k + k
			< cResolution) {

		const int INDEX = (index.k + k) * cResolution * cResolution + (index.j
				+ j) * cResolution + (index.i + i);
		grid[INDEX].fdata = (Data*) malloc(fsize);
		memset(grid[INDEX].fdata, 0, fsize);
		grid[INDEX].isOccupied = true;
		Memory += fsize;
	}
}
// If a Voxel contains surface, it is divided into a finer grid
void Voxelgrid::splitGrid(const cv::Mat &img, const Eigen::Matrix4d &camera) {
	// We might want to allocate neighbouring grids as well.
	for (int i = 0; i < img.rows; ++i) {
		for (int j = 0; j < img.cols; ++j) {
			const float z = img.at<float> (i, j);
			if (z > 0) {
				point local = getLocal(j, i, z);
				point global = Multiply(camera, local);
				Index index = getIndex(global);
				if (index.i >= 0 && index.j >= 0 && index.k >= 0 && index.i
						< cResolution && index.j < cResolution && index.k
						< cResolution) {
						const size_t fsize = fResolution * fResolution
								* fResolution * sizeof(Data);
						for (int m = -Neig; m <= Neig; ++m) {
							for (int n = -Neig; n <= Neig; ++n) {
								for (int l = -Neig; l <= Neig; ++l) {
									const int INDEX = (index.k + l) * cResolution * cResolution
											+ (index.j + n) * cResolution + (index.i + m);
									if (!grid[INDEX].isOccupied){
									split(m, n, l, index, fsize);
									}
								}
							}
						}
						//						checkInitialization(INDEX, grid[INDEX].cdata);
						//						getchar();

				}
			}
		}
	}
	//	std::cout << "points.size(): " << points.size() << std::endl;
}
std::vector<point> Voxelgrid::getPoints() {
	return points;
}

struct GridDisc Voxelgrid::getDisc(int i, int j, int k) {
	return grid[i + cResolution * j + k * cResolution * cResolution];
}

float Voxelgrid::getclength() {
	return clength;
}
float Voxelgrid::getflength() {
	return flength;
}

pixel Voxelgrid::getPixel(const point &p) {
	const pixel tmp = { fx * p.x / p.z + cx, fy * p.y / p.z + cy };
	return tmp;
}

float getTDist(float d) {
	if (d < -DELTA) {
		return -DELTA;
	}
	if (d > DELTA2) {
		return DELTA2;
	}
	return d;
}

float getWeight(const float d) {
	if (d <= EPSILON) {
		return 1;
	}
	if (d < DELTA2) {
		//		return 0;
		return exp(-SIGMA * sqr(d - EPSILON));
	}
	return 0;
}

float Voxelgrid::getColorWeight(const float d) {
	if (d <= EPSILON) {
		return 1;
	}
	if (d <= DELTAC) {
		//		return 0;
		return exp(-SIGMAC * sqr(d - EPSILON));
	}
	return 0;
}
void Voxelgrid::updateFineGrid(const cv::Mat &depth,
		const Eigen::Matrix4d &camera, const int INDEX, const point &START,
		const cv::Mat &colorImg) {
	int i, j, k, fIndex;
	point global, point, local, p;
	float distcolweight, z, distance, weight, truncDist, D, W, cweight, Red,
			Green, Blue, CWeight, scale, red, green, blue;
	pixel pix;
	cv::Vec3b color;
#pragma omp parallel for shared(depth,colorImg, camera, START) private(i,j,k, fIndex, z, distance, weight, truncDist, D, W, cweight, Red, Green, Blue, CWeight, scale,red, green,blue,pix, color, global, local,p)
	for (k = 0; k < fResolution; ++k) {
		for (j = 0; j < fResolution; ++j) {
			for (i = 0; i < fResolution; ++i) {
				global = {START.x + i * flength, START.y + j
					* flength, START.z + k * flength};
				local = Multiply(camera, global);
				if (local.z > 0) {
					pix = getPixel(local);
					if (pix.y >= 0 && pix.y < depth.rows && pix.x >= 0 &&pix.x
							< depth.cols) {
						//
						z = depth.at<float> (pix.y, pix.x);
						if (z > 0 && z < ZMAX) {

							distance = local.z - z;

							weight = getWeight(distance);
							if (weight > 0) {
								truncDist = getTDist(distance);
								fIndex = i + j * fResolution + k
										* fResolution * fResolution;
								D =
										grid[INDEX].fdata[fIndex].distance;
								W =
										grid[INDEX].fdata[fIndex].weight;
								//								std::cout<<W<<std::endl;
								//								std::cout<<"TrunkDist: "<<truncDist<<std::endl;
								//								getchar();
								grid[INDEX].fdata[fIndex].distance = (D * W
										+ weight * truncDist) / (W + weight);
									grid[INDEX].fdata[fIndex].weight = W + weight;
								if (DO_COLOR == 1 ) {
									p = getLocal(pix.y, pix.x, z);
									distcolweight = getColorWeight(distance);
									cweight = distcolweight*fabs(z) / sqrt(
											sqr(p.x) + sqr(p.y) + sqr(p.z));
									if (cweight > 0 ) {
										Red = grid[INDEX].getRed(i,
												j, k);
										Green =
												grid[INDEX].getGreen(i, j, k);
										Blue = grid[INDEX].getBlue(
												i, j, k);
										CWeight =
												grid[INDEX].getCWeight(i, j, k);

										scale = 1.0/255;

										cv::Vec3b color = colorImg.at<
												cv::Vec3b> (pix.y, pix.x);

										red = color(2) * scale;
										green = color(1) * scale;
										blue = color(0) * scale;

										if (isnan(red) || isnan(green)
												|| isnan(blue)) {
											std::cout << "Not a number" << std::endl;
											continue;
										}

										grid[INDEX].fdata[fIndex].red = (Red*CWeight + red*cweight)/(CWeight + weight);
										grid[INDEX].fdata[fIndex].green = (Green*CWeight + green*cweight)/(CWeight + weight);
										grid[INDEX].fdata[fIndex].blue = (Blue*CWeight + blue*cweight)/(CWeight + weight);
//										if (CWeight < CWEIGHTMAX){
											grid[INDEX].fdata[fIndex].colorweight += cweight;
//										}

									}
								}

							}
						}
					}
				}
			}
			}
		}
	}

void Voxelgrid::updateCoarseGrid(const cv::Mat &depth,
		const Eigen::Matrix4d &camera, const int INDEX, const point &START) {
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			for (int k = 0; k < 2; ++k) {
				const point global = { START.x + i * clength, START.y + j
						* clength, START.z + k * clength };
				const point local = Multiply(camera, global);
				if (local.z > 0) {
					const pixel pixel = getPixel(local);
					if (pixel.y >= 0 && pixel.x >= 0 && pixel.y < depth.rows
							&& pixel.x < depth.cols) {
						float z = depth.at<float> (pixel.y, pixel.x);
						if (z > 0) {
							const float distance = local.z - z;
							const float weight = getWeight(distance);
							if (weight > 0) {
								const float truncDist = getTDist(distance);
								//								const float D = grid[INDEX].cdata[i + j * 2 + k
								//										* 4].distance;
								//								const float W = grid[INDEX].cdata[i + j * 2 + k
								//										* 4].weight;
								//								grid[INDEX].cdata[i + j * 2 + k * 4].distance
								//										= (D * W + truncDist * weight) / (W
								//												+ weight);
								//								grid[INDEX].cdata[i + j * 2 + k * 4].weight
								//										+= weight;
							}
						}
					}
				}
			}
		}
	}
}

void Voxelgrid::printSDF() {
	for (int i = 0; i < cResolution; ++i) {
		for (int j = 0; j < cResolution; ++j) {
			for (int k = 0; k < cResolution; ++k) {
				int index = i + cResolution * j + cResolution * cResolution * k;
				if (grid[index].isOccupied) {
					for (int l = 0; l < fResolution; ++l) {
						for (int m = 0; m < fResolution; ++m) {
							for (int n = 0; n < fResolution; n++) {
								std::cout
										<< grid[index].fdata[l + fResolution
												* m + fResolution * fResolution
												* m].distance << " ";
							}
							std::cout << std::endl;
						}
					}
					getchar();
				}

			}
		}
	}
}
void Voxelgrid::UpdateSDF(const cv::Mat &depth, const Eigen::Matrix4d &camera,
		const cv::Mat &color) {
	for (int k = 0; k < cResolution; ++k) {
		for (int j = 0; j < cResolution; ++j) {
			for (int i = 0; i < cResolution; ++i) {
				const point START = { -x_s + i * clength, -y_s + j * clength,
						-z_s + k * clength };
				const int INDEX = i + cResolution * j + cResolution
						* cResolution * k;
				if (grid[INDEX].isOccupied) {
					updateFineGrid(depth, camera, INDEX, START, color);
				} else {
					//					updateCoarseGrid(depth, camera, INDEX, START);
				}
			}
		}
	}
	//	printSDF();
}

long int Voxelgrid::getMemory() {
	return Memory;
}
