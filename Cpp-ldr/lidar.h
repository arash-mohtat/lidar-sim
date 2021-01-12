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
    void toXYZfile(const std::string& save_file);
};

struct SphrRect { // A rectangle in AZ-EL indices to use for Grid arrays -> .block(i,j,p,q)
    int i; // EL_top index
    int j; // AZ_left index
    int p; // EL_bottom_idx - EL_top_idx + 1
    int q; // AZ_right_idx - AZ_left_idx + 1
};

class Lidar : public Body {

public:
    Lidar(const std::string& tag, double hFOV=120.0, int hRes=120, double vFOV=30.0, int vRes=16,
          double max_range=200.0, const std::string& points_file="", const std::string& vert_connect_file="");
    ~Lidar() = default;
    void printSummary(bool summarize_also_body=true) const;
    Grid scan(const Body& target, bool reset_measurements=true);

private:
    void castRays(const Matrix3d& trngl_xyz, const Vector3d& unit_normal, const SphrRect& AZ_EL);
    SphrRect findAZ_EL_slice(const Matrix3d& trngl_xyz) const;
    static std::array<int, 2> findSlice(const double& grid0, const double& delta_grid, const int& length,
                                        const double& val_low, const double& val_high);
    double m_range;
    FieldOfView<double> m_FOV_deg;
    FieldOfView<int> m_FOV_resolution;
    PixelSpecs m_cam_specs; // camera-like pixel specs
    Grid m_grid; // measurement grid
};

