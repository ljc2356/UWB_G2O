//
// Created by Jason on 2021/8/10.
//

#include "LocLossVertex.h"

void LocLossVertex::setToOriginImpl() {
    _estimate<<0,0;
}

void LocLossVertex::oplusImpl(const double *v) {
    _estimate += Eigen::Vector2d(v);
    _estimate(1) = wrapToPi(_estimate(1));
}

bool LocLossVertex::read(std::istream &in) {
    return true;
}

bool LocLossVertex::write(std::ostream &os) const {
    return true;
}
