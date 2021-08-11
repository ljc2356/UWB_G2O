//
// Created by Jason on 2021/8/10.
//

#ifndef UWB_G2O_LOCLOSSVERTEX_H
#define UWB_G2O_LOCLOSSVERTEX_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "g2o/core/base_vertex.h"
#include "../tools/UWBtools.h"
#include <iostream>

class LocLossVertex : public g2o::BaseVertex<2,Eigen::Vector2d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl();

    void oplusImpl(const double * v);

    bool read(std::istream& in);

    bool write(std:: ostream& os) const;
};

#endif //UWB_G2O_LOCLOSSVERTEX_H
