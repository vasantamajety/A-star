#ifndef UTILS_HPP
#define UTILS_HPP

#include "bits/stdc++.h"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#define MAPX 800
#define MAPY 800

#define VELOCITY_MAX 60

#define PRIORITY_OBSTACLE_NEAR 10
#define PRIORITY_MOVEMENT 5

#define GX 80
#define GY 80
#define Grid_Res 10

#define DX 240
#define DY 240

#define Theta 72
#define Theta_Res 5

#define BOT_L 34
#define BOT_W 20
#define BOT_M_ALPHA 30

#define PI 3.14159

#define COST_DIR_CHANGE_TO_FRONT 80
#define COST_DIR_CHANGE_TO_BACK 80
#define FRONT_STRAIGHT 5
#define FRONT_CURVED 7
#define BACK_STRAIGHT 5
#define BACK_CURVED 7

#define ANGLE_CHANGE_WEIGHT 20

#endif