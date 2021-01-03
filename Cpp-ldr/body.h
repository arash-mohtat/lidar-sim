#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

class Body
{
public:
    Body(const std::string& tag, const std::string& points_file="", const std::string& vertex_connectivity_file="");
    ~Body() = default;
    void printSummary() const;
    void move(const Vector3d& delta_xyz);
    void rotate(char axis, double angle_deg);
    void append(const Body& other);
    Matrix<double, 3, Dynamic> getVertices(const Matrix4d& viewpoint=MatrixXd::Identity(4,4)) const;
    std::vector<Matrix3d> getTriangles(const Matrix4d& viewpoint=MatrixXd::Identity(4,4)) const;

    // any friend function or classes to add? Probably not!

protected:
    std::string m_tag;
    Matrix<double, 4, Dynamic> m_pts_hom; // 4-by-N matrix of homogenous points of all vertices
    Matrix<int, Dynamic, 3> m_vert_connect; // n-by-3 matrix defining connectivity of vertices
    Matrix4d m_transform; // 4-by-4 affine transformation matrix (ain't it better to use a special transform type?)
    //bool m_trnsfm_applied; // weather or not latest tranform is applied on points
};


