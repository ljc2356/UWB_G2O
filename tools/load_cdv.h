//
// Created by Jason on 2021/8/7.
//

#ifndef UWB_G2O_LOAD_CDV_H
#define UWB_G2O_LOAD_CDV_H

#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Core"
#include <vector>
#include <fstream>

using namespace std;
using namespace Eigen;

template<typename T>
T load_csv(const std::string & path)
{
    std::ifstream in;
    in.open(path);
    std::string line;
    std::vector<double> values;
    int rows = 0;
    while (std::getline(in, line))
    {
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ','))
        {
            double val = std::stod(cell);
            values.push_back(val);
        }
        ++rows;
    }
    return Eigen::Map<const Eigen::Matrix<
        typename T::Scalar,
        T::RowsAtCompileTime,
        T::ColsAtCompileTime,
        RowMajor>>(values.data(), rows, values.size() / rows);
}


#endif //UWB_G2O_LOAD_CDV_H
