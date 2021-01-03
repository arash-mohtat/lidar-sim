#include "helpers.h"
#include <Eigen/Dense>
#include <utility>      // std::pair, std::make_pair
using namespace Eigen;

namespace Aux{

std::pair<bool, Vector3d> isValidTriangle(const Matrix3d& trngl_xyz, double tolerance){
    bool valid = false;
    Vector3d v1 = trngl_xyz.col(1) - trngl_xyz.col(0);
    Vector3d n = v1.cross(trngl_xyz.col(2) - trngl_xyz.col(0));
    double norm_n = n.norm();
    if (norm_n > tolerance) {
      valid = true;
      n = n/norm_n;
    }
    return std::make_pair(valid, n);
}

Array<bool, Dynamic, 1> areInTriangle(const Matrix<double, 3, Dynamic>& pts_xyz,
                                      const Matrix3d& trngl_xyz, const Vector3d& unit_normal){
    Vector3d v1 = trngl_xyz.col(1) - trngl_xyz.col(0); // vector from vertex A to B
    v1 = v1/v1.norm(); // shouldn't need double-checking (as a valid triangle)
    Vector3d v2 = unit_normal.cross(v1);
    Matrix<double, 3, 2> T;
    T << v1, v2; // This forms an orthonormal basis on the triangle plane which can be used for describing XY points in the plane
    MatrixXd pts_XY = T.transpose() * pts_xyz;
    Matrix<double,2,3> trngl_XY = T.transpose() * trngl_xyz;
    Array2d A = pts_XY.col(0).array();
    Array2d B = pts_XY.col(1).array() - A; // this is like v1 before normalization in XY coords (rather than xyz)
    Array2d C = pts_XY.col(2).array() - A;
    ArrayXXd P = pts_XY.array();
    P.colwise() -= A;
    double area = B(0)*C(1)-C(0)*B(1); // area of triangle (supposed to be nonzero as a valid triangle)

    // Barycentric coords of points P (row arrays)
    auto wA = (P.row(0)*(B(1)-C(1)) + P.row(1)*(C(0)-B(0)) + B(0)*C(1) - C(0)*B(1)) / area;
    auto wB = (P.row(0)*C(1) - P.row(1)*C(0)) / area;
    auto wC = (P.row(1)*B(0) - P.row(0)*B(1)) / area;

    return (wA>=0 && wA<=1 && wB>=0 && wB<=1 && wC>=0 && wC<=1).colwise().all().transpose();

}

} // namespace Aux

namespace Pointcloud{

} // namespace Pointcloud
