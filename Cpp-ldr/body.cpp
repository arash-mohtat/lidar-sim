#include "body.h"
#include "helpers.h"
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include <cctype>
#include <math.h>

using namespace Eigen;

Body::Body(const std::string & tag, const std::string & points_file,
           const std::string & vertex_connectivity_file) : m_tag(tag){
    // Initialize body points (vertices) and triangulat meshes (vertices connectivity matrix)
    // TODO: better error handling of load_csv in body construction
    // TODO: add relative path (rather than absolute)
    MatrixXd points_table;
    Matrix<int, Dynamic, Dynamic> vertices_table;

    if (!points_file.empty() && !vertex_connectivity_file.empty()) {
        points_table = Aux::load_csv<Matrix<double, Dynamic, Dynamic>>(points_file);
        vertices_table = Aux::load_csv<Matrix<int, Dynamic, Dynamic>>(vertex_connectivity_file);
    }
    else { //load a simple prism (not so easy to use a try-catch instead)
        std::cout << "Assigning a simple prism geometry."<< std::endl;
        Matrix<double, 4, 3> points_table_default;
        points_table_default << 0.0, 0.0, 0.0,
                                1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0;
        Matrix<int, 4, 3> vertices_table_default;
        vertices_table_default << 0, 1, 2,
                                  0, 1, 3,
                                  1, 2, 3,
                                  0, 2, 3;
        points_table = points_table_default;
        vertices_table = vertices_table_default;
        std::cout << "Successful assignment!" << std::endl;
    }

    m_pts_hom = MatrixXd::Constant(4, points_table.rows(), 1.0);
    m_pts_hom.block(0,0,3,points_table.rows()) = points_table.transpose();
    m_vert_connect = (vertices_table.array() - vertices_table.minCoeff()).matrix(); // enforce starting from 0
    // TODO: add sanity checks (assertions)

    // Initialize body transform
    m_transform.setIdentity(4,4);
    std::cout << "Successful contruction of body object ..." << std::endl << std::endl;
}

void Body::printSummary() const {
    std::cout << "The name of the object: " << m_tag << std::endl;
    std::cout << "The homogenous points of the body, in transposed form, are:"<< std::endl;
    Aux::printLongMatrix(m_pts_hom.transpose());
    std::cout << "Total number of points (vertices) is: " << m_pts_hom.cols() << std::endl << std::endl;
    std::cout << "The vertices connectivity matrix of the body is:"<< std::endl;
    Aux::printLongMatrix(m_vert_connect);
    std::cout << "Total number of triangular meshes is: " << m_vert_connect.rows() << std::endl << std::endl;
    std::cout << "The body's transform is:"<< std::endl;
    std::cout << m_transform << std::endl;
}

Matrix<double, 3, Dynamic> Body::getVertices(const Matrix4d& viewpoint) const {
    MatrixXd new_pts_hom = (viewpoint.inverse() * m_transform) * m_pts_hom;
    return new_pts_hom.block(0,0,3,m_pts_hom.cols()); // strip away the last row of 1's
}

std::vector<Matrix3d> Body::getTriangles(const Matrix4d& viewpoint) const {
    auto pts_xyz = getVertices(viewpoint); // 3-by-N matrix
    std::vector<Matrix3d> triangles;       // vector of n 3-by-3 matrices
    Matrix3d M;
    for  (int i=0; i < m_vert_connect.rows(); i++){ // TODO: range based for loop depending on Eigen version
        M.col(0) = pts_xyz.col(m_vert_connect(i,0));
        M.col(1) = pts_xyz.col(m_vert_connect(i,1));
        M.col(2) = pts_xyz.col(m_vert_connect(i,2));
        triangles.push_back(M);
    }
    return triangles;
}

void Body::move(const Vector3d& delta_xyz){
    m_transform.block(0,3,3,1) = m_transform.block(0,3,3,1) + delta_xyz;
}

void Body::rotate(char axis, double angle_deg){
    Matrix3d rotation = Matrix3d::Identity();
    double s = sin(angle_deg * M_PI / 180.0);
    double c = cos(angle_deg * M_PI / 180.0);
    switch (tolower(axis)){
        case 'x':
            rotation(1,1) = c; rotation(1,2) = -s;
            rotation(2,1) = s; rotation(2,2) = c;
            break;
        case 'y':
            rotation(0,0) = c; rotation(0,2) = -s;
            rotation(2,0) = s; rotation(2,2) = c;
            break;
        case 'z':
            rotation(0,0) = c; rotation(0,1) = -s;
            rotation(1,0) = s; rotation(1,1) = c;
            break;
        default:
          std::cout << "WARNING: Unrecognized Axis => transform not changed.";
    }
    m_transform.block(0,0,3,3) = rotation * m_transform.block(0,0,3,3); // pre-multiply
}







