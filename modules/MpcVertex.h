//
// Created by Jason on 2021/8/8.
//

#ifndef UWB_G2O_MPCVERTEX_H
#define UWB_G2O_MPCVERTEX_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "g2o/core/base_vertex.h"
#include "../tools/UWBtools.h"

class MpcVertex : public g2o::BaseVertex<2,Eigen::Vector2d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl();
    virtual void oplusImpl(const double * v);
    virtual bool read(std::istream& in);
    virtual bool write(std:: ostream& out) const;
};


#endif //UWB_G2O_MPCVERTEX_H
