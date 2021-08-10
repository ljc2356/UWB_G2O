//
// Created by Jason on 2021/8/8.
//

#ifndef UWB_G2O_MPCBINARYEDGE_H
#define UWB_G2O_MPCBINARYEDGE_H

#include "Eigen/Core"
#include "g2o/core/base_binary_edge.h"
#include "LocVertex.h"
#include "MpcVertex.h"
#include "../tools/UWBtools.h"
#include "math.h"


class MpcBinaryEdge : public g2o::BaseBinaryEdge<2,Eigen::Vector2d,LocVertex,MpcVertex>{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError();
    virtual bool read(std::istream& in);
    virtual bool write(std:: ostream& out) const;
};


#endif //UWB_G2O_MPCBINARYEDGE_H
