#pragma once

#include <string>
#include <array>
#include <Eigen/Dense>
#include "body.h"
#include "helpers.h"


using namespace Eigen;

template<typename Scalar>
struct FieldOfView {
    Scalar horizontal;
    Scalar vertical;
};

struct PixelSpecs {
    int width;
    int height;
    int focal_length;
};

struct Grid {         // ArrayXXd rather than MatrixXd (since element-wise ops are needed here)
    ArrayXXd AZ;      // Azimuth Matrix
    ArrayXXd EL;      // Elevation matrix
    ArrayXXd u;       // horizontal image coords matrix
    ArrayXXd v;       // vertical image coords matrix
    ArrayXXd rho;     // spherical distance matrix, i.e. main lidar measurement
    ArrayXXd x;       // x-measurement matrix in lidar coords
    ArrayXXd y;       // y-measurement matrix in lidar coords
    ArrayXXd z;       // z-measurement matrix in lidar coords
};

class Lidar : public Body {

public:
    Lidar(const std::string& tag, double hFOV=120.0, int hRes=120, double vFOV=30.0, int vRes=16,
          double max_range=200.0, const std::string& points_file="", const std::string& vert_connect_file="");
    ~Lidar() = default;
    void printSummary(bool summarize_also_body=true) const;
    Grid scan(const Body& target, bool reset_measurements=true);

private:
    std::array<Matrix<double,3,Dynamic>, 2> transformCoords(const Body& target) const;
    void castRays(const Matrix3d& trngl_xyz, const Vector3d& unit_normal);
    double m_range;
    FieldOfView<double> m_FOV_deg;
    FieldOfView<int> m_FOV_resolution;
    PixelSpecs m_cam_specs; // camera-like pixel specs
    Grid m_grid; // measurement grid
};

