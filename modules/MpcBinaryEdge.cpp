//
// Created by Jason on 2021/8/8.
//

#include "MpcBinaryEdge.h"


void MpcBinaryEdge::computeError() {

    const LocVertex* Loc = static_cast<const LocVertex*>(_vertices[0]);
    const MpcVertex* Mpc = static_cast<const MpcVertex*>(_vertices[1]);

    Eigen::Vector2d LocXY = Loc->estimate();
    Eigen::Vector2d MpcHT = Mpc->estimate();

    Eigen::Vector2d MpcLoc = mirror(LocXY(0),LocXY(1),MpcHT(0),MpcHT(1));
    _error(0) = 1*(_measurement(0) - MpcLoc.norm());
    _error(1) = wrapToPi(1*(_measurement(1) - atan2(MpcLoc(1),MpcLoc(0))));

}

bool MpcBinaryEdge::read(std::istream &in) {
    return true;
}

bool MpcBinaryEdge::write(std::ostream &out) const {
    return true;
}


