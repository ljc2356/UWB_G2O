//
// Created by Jason on 2021/8/8.
//

#ifndef UWB_G2O_UWBTOOLS_H
#define UWB_G2O_UWBTOOLS_H

#include "math.h"
#include "Eigen/Core"
#include "iostream"
#include "fstream"

double wrapToPi(double angle);

Eigen::Vector2d mirror(double x,double y,double h,double theta);

void MatToCSV(Eigen::MatrixXd matResult, std::string  path);

#endif //UWB_G2O_UWBTOOLS_H
