//
// Created by Jason on 2021/8/7.
//

#include "LocVertex.h"



void LocVertex::setToOriginImpl() {
    _estimate << 0,0;
}

void LocVertex::oplusImpl(const double * v) {
    _estimate += Eigen::Vector2d(v);
}

bool LocVertex::read(std::istream &in) {
    return true;
}

bool LocVertex::write(std::ostream &os) const {
    return true;
}



