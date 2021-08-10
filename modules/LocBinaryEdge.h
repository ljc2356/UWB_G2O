//
// Created by Jason on 2021/8/7.
//

#ifndef UWB_G2O_LOCBINARYEDGE_H
#define UWB_G2O_LOCBINARYEDGE_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "g2o/core/base_binary_edge.h"
#include "LocVertex.h"
#include <math.h>

class LocBinaryEdge : public g2o::BaseBinaryEdge<2,Eigen::Vector2d ,LocVertex,LocVertex>{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError();
    virtual bool read(std::istream& in);
    virtual bool write(std:: ostream& out);

};


#endif //UWB_G2O_LOCBINARYEDGE_H
