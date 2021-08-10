//
// Created by Jason on 2021/8/8.
//

#ifndef UWB_G2O_MIRROR_H
#define UWB_G2O_MIRROR_H
#include "math.h"
#include "Eigen/Core"
#include "iostream"

Eigen::Vector2d mirror(double x,double y,double h,double theta){
    Eigen::Vector2d MpcLoc;
    MpcLoc(0) = x * cos(2*theta) + (y - h)* sin(2*theta);
    MpcLoc(1) = h + (h - y)* cos(2*theta) + x * sin(2*theta);
    return MpcLoc;
}

#endif //UWB_G2O_MIRROR_H
