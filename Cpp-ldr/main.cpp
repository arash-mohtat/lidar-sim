#include <iostream>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include "lidar.h"
#include <chrono>
#include <filesystem>
namespace fs = std::filesystem;

#define timeNow() std::chrono::high_resolution_clock::now()

using namespace Eigen;

struct FilePaths{
    std::string pts_file;
    std::string mesh_file;
    std::string result_file; 
};

FilePaths loadFilePaths(){
    FilePaths paths;
#ifdef __CYGWIN__
    paths.pts_file = "/cygdrive/d/Personal/repos/lidar-sim/coords/hatchback_vertices_vehcoords.txt";
    paths.mesh_file = "/cygdrive/d/Personal/repos/lidar-sim/coords/hatchback_triangles_vehcoords.txt";
    paths.result_file = "/cygdrive/d/Personal/repos/lidar-sim/renderings/cpp_vehPtcld_windows.xyz";
#else
    paths.pts_file = "/media/arash/DATA/Personal/repos/lidar-sim/coords/hatchback_vertices_vehcoords.txt";
    paths.mesh_file = "/media/arash/DATA/Personal/repos/lidar-sim/coords/hatchback_triangles_vehcoords.txt";
    paths.result_file = "/media/arash/DATA/Personal/repos/lidar-sim/renderings/cpp_vehPtcld_linux.xyz";
#endif
    return paths;
}

void test_paths(){
    auto paths = loadFilePaths();
    std::cout << paths.pts_file << std::endl;
    fs::path p = fs::current_path();
    std::cout << "The current path " << p << '\n' << "root name " << p.root_name() << '\n'
              << "root directory " << p.root_directory() << std::endl;
    MatrixXd points_table = Aux::load_csv<MatrixXd>(paths.pts_file);
    Aux::printLongMatrix(points_table);
}

void test_Body(){
    auto paths = loadFilePaths();
    auto vehicle = Body("Hatchback", paths.pts_file, paths.mesh_file);
    //auto vehicle = Body("Hatchback");
    vehicle.printSummary();
    auto pts = vehicle.getVertices();
    std::cout << std::endl << "Vertices in viewpoint coordinates:" << std::endl;
    Aux::printLongMatrix(pts.transpose());
    auto triangles = vehicle.getTriangles();
    std::cout << "First triangle: "<< std::endl << triangles.at(0) << std::endl;
    std::cout << "3rd triangle: "<< std::endl << triangles.at(2) << std::endl;
    vehicle.move(Vector3d::Random(3,1));
    vehicle.rotate('z', 45.0);
    vehicle.printSummary();
}

void test_Lidar(){
    auto vehicle = Body("Hatchback");
    auto ldr = Lidar("VLP16", 120.0, 12, 30.0, 8);
    ldr.printSummary(false);
    auto scanned_grid = ldr.scan(vehicle);
    std::cout << "grid.rho = " << std::endl << scanned_grid.rho << std::endl;
}

void test_areInTriangle(){
    Matrix<double, 4, 3> pts_xyz_rows;
    pts_xyz_rows << 0.5, 1.0, 0.4,
                    0.1, 1.0, 1.5,
                    0.5, 1.0, 0.4,
                    0.6, 1.0, 0.8;
    Matrix3d trngl_xyz;
    trngl_xyz << 0.0, 1.0, 0.0,
                 1.0, 1.0, 0.0,
                 0.0, 1.0, 1.0;
    Vector3d unit_normal{0.0,-1.0,0.0};
    auto cond = Aux::areInTriangle(pts_xyz_rows.transpose(), trngl_xyz.transpose(), unit_normal);
    std::cout << "expected = 1, 0, 1, 0" << std::endl;
    std::cout << "cond = " << cond << std::endl;
}

void scanHatchback_withLidar(){

    // prep vehicle to be scanned
    auto paths = loadFilePaths();
    auto vehicle = Body("Hatchback", paths.pts_file, paths.mesh_file);
    vehicle.move(Vector3d{0.0, 6.4, 0.0});

    // prep lidar
    auto ldr = Lidar("VLP16", 120.0, 120, 30.0, 16);
    ldr.move(Vector3d{0.0, 0.0, 3.0});
    ldr.rotate('x', -15.0);
    ldr.printSummary(false);

    // make lidar scan vehicle
    auto t1 = timeNow();
    auto scanned_grid = ldr.scan(vehicle);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow() - t1).count();
    std::cout << "Scanning took this much time (sec): " << duration/1000.0 << std::endl;
    //std::cout << "grid.rho = " << std::endl << scanned_grid.rho << std::endl;

    // save results
    scanned_grid.toXYZfile(paths.result_file, 0.0, 200.0);
}


int main(){
    //test_paths();
    //test_Body();
    //test_Lidar();
    //test_areInTriangle();
    scanHatchback_withLidar();
    return 0;
}
