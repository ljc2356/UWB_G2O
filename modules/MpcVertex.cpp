//
// Created by Jason on 2021/8/8.
//

#include "MpcVertex.h"


void MpcVertex::setToOriginImpl() {
    _estimate << 0,0;
}

void MpcVertex::oplusImpl(const double * updata) {
    _estimate += Eigen::Vector2d(updata);
    _estimate(1) = wrapToPi(_estimate(1));
}

bool MpcVertex::read(std::istream &in) {
    return true;
}

bool MpcVertex::write(std::ostream &out) const{
    return true;
}

