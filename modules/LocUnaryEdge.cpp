//
// Created by Jason on 2021/8/8.
//

#include "LocUnaryEdge.h"



void LocUnaryEdge::computeError() {
    const LocVertex* Loc = static_cast<const LocVertex*>(_vertices[0]);
    Eigen::Vector2d LocXY = Loc->estimate();
    _error(0) = _measurement(0) - LocXY.norm();
    _error(1) = wrapToPi(1*(_measurement(1) - atan2(LocXY(1),LocXY(0))));
}

bool LocUnaryEdge::read(std::istream &in) {
    return true;
}

bool LocUnaryEdge::write(std::ostream &out) const{
    return true;
}

