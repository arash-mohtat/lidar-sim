#include "lidar.h"
#include <vector>
#include <iostream>
#include <cctype>
#include <math.h>


using namespace Eigen;


Lidar::Lidar(const std::string& tag, double hFOV, int hRes,
             double vFOV, int vRes, double max_range, const std::string& points_file,
             const std::string& vert_connect_file): Body{tag, points_file, vert_connect_file}, m_range{max_range} {
    std::cout << "constructing a lidar object ..." << std::endl;

    // compute camera equivalent parameters
    double hFOV_rad = hFOV * M_PI / 180.0;
    double vFOV_rad = vFOV * M_PI / 180.0;
    int focal_length = std::max(1000, 1 + static_cast<int>(hRes/(2.0 * tan(hFOV_rad/2.0))));
    int width = static_cast<int>(2.0 * focal_length * tan(hFOV_rad/2.0));
    int height = static_cast<int>(2.0 * focal_length * tan(vFOV_rad/2.0) / cos(hFOV_rad/2.0));
    m_FOV_deg.horizontal = hFOV;
    m_FOV_deg.vertical = vFOV;
    m_FOV_resolution.horizontal = hRes;
    m_FOV_resolution.vertical = vRes;
    m_cam_specs.focal_length = focal_length;
    m_cam_specs.width = width;
    m_cam_specs.height = height;

    // discretize scanning angles, i.e. azimuth and elevation
    auto azimuth = RowVectorXd::LinSpaced(hRes, -hFOV_rad/2.0, hFOV_rad/2.0);
    auto elevation = VectorXd::LinSpaced(vRes, vFOV_rad/2.0, -vFOV_rad/2.0);

    // generate meshgrid in AZ-EL scanning rectangle
    m_grid.AZ = azimuth.replicate(vRes,1).array(); // ArrayXXd struct field
    m_grid.EL = elevation.replicate(1,hRes).array(); // ArrayXXd struct field

    // project scanning grid on image plane
    m_grid.u = width/2.0 + focal_length * tan(m_grid.AZ);
    m_grid.v = height/2.0 - focal_length * tan(m_grid.EL)/cos(m_grid.AZ);

    // initialize measurement grid
    m_grid.rho = ArrayXXd::Zero(vRes, hRes) + 2.0 * max_range; // anything above max_range is considered out-of-range
    m_grid.x = ArrayXXd::Zero(vRes, hRes);
    m_grid.y = ArrayXXd::Zero(vRes, hRes);
    m_grid.z = ArrayXXd::Zero(vRes, hRes);

    std::cout << "Lidar object constructed!" << std::endl;
}

void Lidar::printSummary(bool summarize_also_body) const {
    std::cout << "This lidar has a horizontal and vertical FOV (degrees) of: " << m_FOV_deg.horizontal << ", "
              << m_FOV_deg.vertical << std::endl;
    std::cout << "This lidar is similar to a camera with a focal length, width and height (pixels) of: "
              << m_cam_specs.focal_length << ", " << m_cam_specs.width << ", " << m_cam_specs.height << std::endl;
    if (summarize_also_body) {Body::printSummary();}
}

/* // not very useful actually!
std::array<Matrix<double,3,Dynamic>, 2> Lidar::transformCoords(const Body& target) const {
    MatrixXd points_cart = target.getVertices(m_transform);
    MatrixXd point_sphr = MatrixXd::Zero(3,points_xyz.cols()); // TODO: calc actual spherical coords
    std::cout << "transformed coordinates"<< std::endl;
    return {points_cart, points_sphr};
}
*/

Grid Lidar::scan(const Body& target, bool reset_measurements){
    if (reset_measurements){
        m_grid.rho = ArrayXXd::Zero(m_FOV_resolution.vertical, m_FOV_resolution.horizontal) + 2.0 * m_range;
    }
    //Matrix4d viewpoint = this->m_transform; // lidar's viewpoint (needs to be inverted)
    std::vector<Matrix3d> triangles_xyz = target.getTriangles(m_transform); // get body triangles from lidar's point-of-view
    for (auto trngl_xyz : triangles_xyz){
        std::pair<bool, Vector3d> result = Aux::isValidTriangle(trngl_xyz);
        if (!result.first) {continue;} // if tngl not valid
        castRays(trngl_xyz, result.second); // result.second = unit_normal
    }
    // prep x,y,z measurements via spherical transform when all rho's are updated
    m_grid.x = m_grid.rho * cos(m_grid.EL) * sin(m_grid.AZ);
    m_grid.y = m_grid.rho * cos(m_grid.EL) * cos(m_grid.AZ);
    m_grid.z = m_grid.rho * sin(m_grid.EL);
    return m_grid;
}

void Lidar::castRays(const Matrix3d& trngl_xyz, const Vector3d& unit_normal){
    //std::cout << "Casting rays on triangle below:" << std::endl << trngl_xyz << std::endl;

    // construct flattened (column arrays) sine and cosine of AZ and EL of laser rays
    Map<ArrayXd> AZ_flat(m_grid.AZ.data(), m_grid.AZ.size());
    Map<ArrayXd> EL_flat(m_grid.EL.data(), m_grid.EL.size());
    ArrayXd ca = cos(M_PI/2 - AZ_flat);
    ArrayXd sa = sin(M_PI/2 - AZ_flat);
    ArrayXd ce = cos(EL_flat);
    ArrayXd se = sin(EL_flat);

    // calculate the intersection of rays with the triangle plane
    double d = unit_normal.dot(trngl_xyz.col(0));
    ArrayXd rho = d / (unit_normal(0)*ce*ca + unit_normal(1)*ce*sa + unit_normal(2)*se);
    rho = (rho<0).select(2*m_range, rho); // set all negative rho's equal to 2*m_range
    ArrayXXd pts_xyz_cols(rho.rows(), 3); // initialize an n-by-3 array to stack xyz points
    pts_xyz_cols << rho*ce*ca, rho*ce*sa, rho*se;

    // select those intersections with the plane that are actually inside the triangular mesh
    //MatrixXd pts_xyz = pts_xyz_cols.transpose().matrix();
    //std::cout << "scanned pts_xyz on plane size: " << pts_xyz.rows() << ", " << pts_xyz.cols() <<std::endl;
    auto cond = Aux::areInTriangle(pts_xyz_cols.transpose().matrix(), trngl_xyz, unit_normal);
    rho = cond.select(rho, 2*m_range); // set those elements of rho that dont satisfy the being-in-tringl cond to out-of-range

    // back to rectangular grid and min-accumulate
    Map<ArrayXXd> rho_grid(rho.data(), m_FOV_resolution.vertical, m_FOV_resolution.horizontal);
    m_grid.rho = rho_grid.min(m_grid.rho);
}

