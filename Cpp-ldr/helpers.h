#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <utility>      // std::pair, std::make_pair
using namespace Eigen;

namespace Aux{

template<typename M>
void printLongMatrix(const M & table){
    std::cout << "First Line: " << std::endl;
    std::cout << table.row(0) << std::endl << ":" <<std::endl;
    std::cout << "Last Line: " << std::endl;
    std::cout << table.bottomRows(1) << std::endl;
}

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<typename M::Scalar> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

std::pair<bool, Vector3d> isValidTriangle(const Matrix3d& trngl_xyz, double tolerance=1e-6);

Array<bool, Dynamic, 1> areInTriangle(const Matrix<double, 3, Dynamic>& pts_xyz, const Matrix3d& trngl_xyz, const Vector3d& unit_normal);

} // namespace Aux

namespace Pointcloud{

} // namespace Pointcloud
