//
// Created by Jason on 2021/8/10.
//

#include "LocMoveBinaryEdge.h"

void LocMoveBinaryEdge::computeError() {
    const LocVertex* LocLast = static_cast<const LocVertex*>(_vertices[0]);
    const LocVertex* LocNow = static_cast<const LocVertex*>(_vertices[1]);
    Eigen::Vector4d LocLastEst = LocLast->estimate();
    Eigen::Vector4d LocNowEst = LocNow->estimate();

    Eigen::MatrixXd A(4,4);
    A<< 1,0,1*0.079,0,
        0,1,0,1*0.079,
        0,0,1,0,
        0,0,0,1;

    Eigen::Vector4d LocNowPre = A * LocLastEst;
    _error(0) = LocNowPre(0) - LocNowEst(0);
    _error(1) = LocNowPre(1) - LocNowEst(1);
    _error(2) = LocNowPre(2) - LocNowEst(2);
    _error(3) = LocNowPre(3) - LocNowEst(3);

}

bool LocMoveBinaryEdge::read(std::istream &in) {
    return true;
}

bool LocMoveBinaryEdge::write(std::ostream &os) const {
    return true;
}
