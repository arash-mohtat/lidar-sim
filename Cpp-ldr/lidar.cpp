#include "lidar.h"
#include <vector>
#include <array>
#include <iostream>
#include <cctype>
#include <math.h>
#include <fstream>


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

Grid Lidar::scan(const Body& target, bool reset_measurements){
    if (reset_measurements){
        m_grid.rho = ArrayXXd::Zero(m_FOV_resolution.vertical, m_FOV_resolution.horizontal) + 2.0 * m_range;
    }
    std::vector<Matrix3d> triangles_xyz = target.getTriangles(m_transform); // get body triangles from lidar's point-of-view

    // loop through triangles and cast relevant rays on
    int skipped_trngls_1 = 0; // skipped due to trngl invalidity
    int skipped_trngls_2 = 0; // skipped due to not being seen by rays
    //SphrRect AZ_EL{0, 0, m_FOV_resolution.vertical, m_FOV_resolution.horizontal}; // the entire grid
    for (const auto& trngl_xyz : triangles_xyz){
        std::pair<bool, Vector3d> result = Aux::isValidTriangle(trngl_xyz);
        if (!result.first) {skipped_trngls_1 +=1; continue;} // if tngl not valid
        SphrRect AZ_EL = findAZ_EL_slice(trngl_xyz); // TODO: needs debugging and can replace line above for
        if ((AZ_EL.p < 1) || (AZ_EL.q < 1)) {skipped_trngls_2 +=1; continue;}// if slice is empty, continue
        castRays(trngl_xyz, result.second, AZ_EL); // result.second = unit_normal
    }
    std::cout << "Skipped (invalidity) " << skipped_trngls_1 << " triangles out of " << triangles_xyz.size() << std::endl;
    std::cout << "Skipped (not seen by any ray) " << skipped_trngls_2 << " triangles out of " << triangles_xyz.size() << std::endl;

    // prep x,y,z measurements via spherical transform when all rho's are updated
    m_grid.x = m_grid.rho * cos(m_grid.EL) * sin(m_grid.AZ);
    m_grid.y = m_grid.rho * cos(m_grid.EL) * cos(m_grid.AZ);
    m_grid.z = m_grid.rho * sin(m_grid.EL);
    return m_grid;
}

std::array<int, 2> Lidar::findSlice(const double& grid0, const double& delta_grid, const int& length,
                                   const double& val_low, const double& val_high) {
    int idx_low = static_cast<int> (ceil((val_low - grid0)/delta_grid));
    int idx_high = static_cast<int> (floor((val_high - grid0)/delta_grid));
    if (idx_high < 0 || idx_low > (length-1)){
        return {0, 0}; // return an arbitrary empty slice
    }
    idx_low = std::clamp(idx_low, 0 , length-1);
    idx_high = std::clamp(idx_high, 0 , length-1);
    return {idx_low, idx_high - idx_low + 1}; // index low, index sequence length
}

SphrRect Lidar::findAZ_EL_slice(const Matrix3d& trngl_xyz) const {
    // trngl_xyz << xA, xB, xC,
    //              yA, yB, yC,
    //              zA, zB, zC; //with A, B, C vertices

    // transform triangle vertices to spherical coords
    Array3d rho_vertices = trngl_xyz.colwise().norm().array();
    Array3d AZ_vertices; //Eigen doesn't support atan2 and not worth converting to valarray
    for (int idx = 0; idx < 3; ++idx) {
        AZ_vertices(idx) = atan2(trngl_xyz(0, idx), trngl_xyz(1, idx));
    }
    Array3d EL_vertices = asin(trngl_xyz.row(2).transpose().array()/rho_vertices); //asin(z/rho)

    // find AZ and EL slices. Note: azimuths are columns growing from left to right, while elevations
    // are rows with values decreasing from top to bottom
    std::array<int, 2> jq = findSlice(m_grid.AZ(0,0), m_grid.AZ(0,1) - m_grid.AZ(0,0),
                                      m_FOV_resolution.horizontal,
                                      AZ_vertices.minCoeff(), AZ_vertices.maxCoeff()); // Left > Right
    std::array<int, 2> ip = findSlice(m_grid.EL(0,0), m_grid.EL(1,0) - m_grid.EL(0,0),
                                      m_FOV_resolution.vertical,
                                      EL_vertices.maxCoeff(), EL_vertices.minCoeff()); // Top < Bottom

    return SphrRect{ip[0], jq[0], ip[1], jq[1]}; // consistent with .block(i,j,p,q) in Eigen
}

void Lidar::castRays(const Matrix3d& trngl_xyz, const Vector3d& unit_normal,
                     const SphrRect& AZ_EL){
    //std::cout << "casting ray on this trngl:" << std::endl << trngl_xyz << std::endl;
    // construct flattened (column arrays) sine and cosine of AZ and EL of laser rays
    int block_size = AZ_EL.p * AZ_EL.q; // never should be zero or negative
    Map<ArrayXd> AZ_flat(m_grid.AZ.block(AZ_EL.i, AZ_EL.j, AZ_EL.p, AZ_EL.q).data(), block_size);
    Map<ArrayXd> EL_flat(m_grid.EL.block(AZ_EL.i, AZ_EL.j, AZ_EL.p, AZ_EL.q).data(), block_size);
    ArrayXd ca = cos(M_PI/2 - AZ_flat);
    ArrayXd sa = sin(M_PI/2 - AZ_flat);
    ArrayXd ce = cos(EL_flat);
    ArrayXd se = sin(EL_flat);

    // calculate the intersection of rays with the triangle plane
    double d = unit_normal.dot(trngl_xyz.col(0));
    ArrayXd rho = d / (unit_normal(0)*ce*ca + unit_normal(1)*ce*sa + unit_normal(2)*se);
    rho = (rho<0).select(2*m_range, rho); // to avoid backward ray intersection (when hFOV > 180)
    ArrayXXd pts_xyz_cols(rho.rows(), 3); // initialize an n-by-3 array to stack xyz points
    pts_xyz_cols << rho*ce*ca, rho*ce*sa, rho*se;

    // select those intersections with the plane that are actually inside the triangular mesh
    auto cond = Aux::areInTriangle(pts_xyz_cols.transpose().matrix(), trngl_xyz, unit_normal);
    rho = cond.select(rho, 2*m_range); // set those elements of rho that dont satisfy the being-in-tringl cond to out-of-range

    // back to rectangular grid and min-accumulate
    //std::cout << "Mapping back on grid:" << std::endl;
    Map<ArrayXXd> rho_grid(rho.data(), AZ_EL.p, AZ_EL.q);
    m_grid.rho.block(AZ_EL.i, AZ_EL.j, AZ_EL.p, AZ_EL.q)
            = rho_grid.min(m_grid.rho.block(AZ_EL.i, AZ_EL.j, AZ_EL.p, AZ_EL.q));
}

/*
void Grid::toXYZfile(const std::string& save_file) {
    // TODO: add option to discard inf (2*m_range) measurements
    // For now: setting out-of-range values to zero (instead of deleting them)
    std::ofstream file(save_file);
    if (file.is_open()) {
        Map<ArrayXd> x_flat(x.data(), x.size());
        Map<ArrayXd> y_flat(y.data(), y.size());
        Map<ArrayXd> z_flat(z.data(), z.size());
        ArrayXXd xyz(x.size(), 3);
        xyz << x_flat, y_flat, z_flat;
        file << xyz;
        std::cout << "saved to file ..." << std::endl;
      }
}
*/

void Grid::toXYZfile(const std::string& save_file, double rho_min, double rho_max) {
    // TODO: add option to discard inf (2*m_range) measurements
    // For now: setting out-of-range values to zero (instead of deleting them)
    std::ofstream file(save_file);
    if (rho_max == 0) {
        rho_max = rho.maxCoeff();
    }
    if (file.is_open()) {
        Map<ArrayXd> AZ_flat(AZ.data(), AZ.size());
        Map<ArrayXd> EL_flat(EL.data(), EL.size());
        Map<ArrayXd> rho_flat(rho.data(), rho.size());
        rho_flat = (rho_flat<rho_min || rho_flat>rho_max).select(0.0, rho_flat);       
        ArrayXXd xyz(rho_flat.size(), 3);
        xyz << rho_flat*cos(EL_flat)*sin(AZ_flat), rho_flat*cos(EL_flat)*cos(AZ_flat), rho_flat*sin(EL_flat);
        file << xyz;
        std::cout << "saved to file ..." << std::endl;
      }
}
