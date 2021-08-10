//
// Created by Jason on 2021/8/8.
//

#ifndef UWB_G2O_LOCUNARYEDGE_H
#define UWB_G2O_LOCUNARYEDGE_H

#include "Eigen/Core"
#include "g2o/core/base_unary_edge.h"
#include "LocVertex.h"
#include "math.h"
#include "../tools/UWBtools.h"

class LocUnaryEdge: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,LocVertex>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError();
    virtual bool read(std::istream& in);
    virtual bool write(std:: ostream& out) const;
};


#endif //UWB_G2O_LOCUNARYEDGE_H
