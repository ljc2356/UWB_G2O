//
// Created by Jason on 2021/8/7.
//

#include "LocBinaryEdge.h"
#include "../tools/wrapToPi.h"

void LocBinaryEdge::computeError() {

    const LocVertex* v_target = static_cast<const LocVertex*> ( _vertices[0]);
    const LocVertex* v_mpc = static_cast<const LocVertex*> ( _vertices[1]);
    const Eigen::Vector2d loc_target = v_target->estimate();
    const Eigen::Vector2d loc_mpc = v_mpc->estimate();
    Eigen::Vector2d temp_error;
    temp_error(0) = (loc_target - loc_mpc).norm();
    temp_error(1) = atan2();





}