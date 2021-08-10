//
// Created by Jason on 2021/8/7.
//

#ifndef UWB_G2O_WRAPTOPI_H
#define UWB_G2O_WRAPTOPI_H

#include <math.h>
#include "iostream"


double wrapToPi(double angle){
    double wrapedAngle;
    wrapedAngle = fmod((angle + M_PI),(2 * M_PI)) - M_PI;
    return wrapedAngle;
}



#endif //UWB_G2O_WRAPTOPI_H
