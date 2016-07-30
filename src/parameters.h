/*
 * parameters.h
 *
 *  Created on: Feb 3, 2013
 *      Author: erikbylow
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

const int TEDDY = 1376;
const int F1_DESK = 573;
const int HOUSEHOLD = 2486;
const int LARGE_NO_LOOP = 653;
const int WITH_LOOP = 1227;

const int RESOLUTION = 512;
#define sqr(a) ((a)*(a))

const float x_s = 65;
const float y_s = 65;
const float z_s = 65;

const float START_X = 0;
const float START_Y = 0;
const float START_Z = 0;

const int SET = 1;                                                                                                                                                                                                                                                                                                                                                                                                                                      ;

const double fx = 525/(float)SET;
const double fy = 525/(float)SET;
const double cx = 319.5/(float)SET;
const double cy = 239.5/(float)SET;

const double fx_inv = 1.0/fx;
const double fy_inv = 1.0/fy;

const float xstep = (2*x_s)/(RESOLUTION-1);
const float ystep = (2*y_s)/(RESOLUTION-1);
const float zstep = (2*z_s)/(RESOLUTION-1);


const float DELTA = 0.3;
const float DELTA2 = 0.30;
const float EPSILON = 0.025;
const float LOW = 0.01;
const float SIGMA = -log(LOW)/sqr(EPSILON-DELTA2);

//const float DELTAC = 0.1;

const int ISOVALUE = 0;
const int DO_COLOR = 1;

const int ROWS = 480;
const int COLS = 640;
const int RR = RESOLUTION*RESOLUTION;

//const int NBROFIMAGES = F1_DESK;

const float epsilon = 0.00001;
const float epsilon_stop = 0.0005;
const int MAX_ITR = 15;

const int STOP = 1000000;
const int STOP2 = STOP/2;


const float griddim = 2*x_s;
const int cResolution = floor(griddim/(DELTA));
const int fResolution = 20;

const float ZMAX = 6;

//const float alpha = 1;
//const float beta = 1;

#endif /* PARAMETERS_H_ */
