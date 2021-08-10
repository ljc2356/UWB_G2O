//
// Created by Jason on 2021/8/8.
//

#include "UWBtools.h"

double wrapToPi(double angle){
    double wrapedAngle;
    wrapedAngle = fmod((angle + M_PI),(2 * M_PI)) - M_PI;
    return wrapedAngle;
}

Eigen::Vector2d mirror(double x,double y,double h,double theta){
    Eigen::Vector2d MpcLoc;
    MpcLoc(0) = x * cos(2*theta) + (y - h)* sin(2*theta);
    MpcLoc(1) = h + (h - y)* cos(2*theta) + x * sin(2*theta);
    return MpcLoc;
}

void MatToCSV(Eigen::MatrixXd matResult, std::string path)
{
    int numDatas = matResult.rows();
    std::ofstream file(path);
    for(int i = 0;i<numDatas;i++)
    {
        file<<matResult(i,0)<<','<<matResult(i,1)<<'\n';
    }
    file.close();
}


