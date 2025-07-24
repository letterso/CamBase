#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <string_view>

#include "cam_radtan.h"
#include "cam_equi.h"

// Define a macro to check which implementation is active
#if defined(USE_OPENCV_IMPL) && defined(HAVE_OPENCV)
    #define ACTIVE_IMPL_NAME "(OpenCV)"
#else
    #define ACTIVE_IMPL_NAME "(Internal)"
#endif

// Helper to print results
void print_test_header(std::string_view title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '=') << std::endl;
}

// Correctness Test: Distort a point and then undistort it, and check the error.
void run_correctness_test(CamBase& camera, std::string_view model_name) {
    Eigen::Vector2d original_pt_norm(0.3, -0.2);
    Eigen::Vector2d distorted_pt = camera.distort_d(original_pt_norm);
    Eigen::Vector2d final_pt_norm = camera.undistort_d(distorted_pt);
    double error = (original_pt_norm - final_pt_norm).norm();

    std::cout << model_name << " " << ACTIVE_IMPL_NAME << " Correctness Test:\n";
    std::cout << "  Original Point:      (" << original_pt_norm.x() << ", " << original_pt_norm.y() << ")\n";
    std::cout << "  Distorted Pixel:     (" << distorted_pt.x() << ", " << distorted_pt.y() << ")\n";
    std::cout << "  Final Undistorted:   (" << final_pt_norm.x() << ", " << final_pt_norm.y() << ")\n";
    std::cout << "  Round-trip Error:    " << std::scientific << error << std::fixed << "\n";
    
    if (error > 1e-6) {
        std::cout << "  [WARNING] High error detected!\n";
    } else {
        std::cout << "  [SUCCESS] Low error.\n";
    }
}

// Speed Test: Distort and undistort a large number of points and measure time.
void run_speed_test(CamBase& camera, std::string_view model_name) {
    const int NUM_POINTS = 100000;
    std::vector<Eigen::Vector2d> points;
    points.reserve(NUM_POINTS);

    std::mt19937 gen(42);
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    for (int i = 0; i < NUM_POINTS; ++i) {
        points.emplace_back(dis(gen), dis(gen));
    }

    auto start_distort = std::chrono::high_resolution_clock::now();
    for (const auto& p : points) {
        volatile Eigen::Vector2d res = camera.distort_d(p);
    }
    auto end_distort = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> distort_ms = end_distort - start_distort;

    std::uniform_real_distribution<> dis_u(0, camera.w());
    std::uniform_real_distribution<> dis_v(0, camera.h());
    for (int i = 0; i < NUM_POINTS; ++i) {
        points[i] = Eigen::Vector2d(dis_u(gen), dis_v(gen));
    }

    auto start_undistort = std::chrono::high_resolution_clock::now();
    for (const auto& p : points) {
        volatile Eigen::Vector2d res = camera.undistort_d(p);
    }
    auto end_undistort = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> undistort_ms = end_undistort - start_undistort;

    std::cout << "\n" << model_name << " " << ACTIVE_IMPL_NAME << " Speed Test (" << NUM_POINTS << " points):\n";
    std::cout << "  distort() time:   " << distort_ms.count() << " ms\n";
    std::cout << "  undistort() time: " << undistort_ms.count() << " ms\n";
}


int main() {
    int width = 1280;
    int height = 720;

    {
        print_test_header("Radtan Model Test");
        std::vector<double> K_radtan = {458.6, 457.3, 639.5, 359.5};
        std::vector<double> D_radtan = {-0.283, 0.074, 0.0002, 0.00017};
        CamRadtan radtan_camera(width, height, K_radtan, D_radtan);
        run_correctness_test(radtan_camera, "Radtan");
        run_speed_test(radtan_camera, "Radtan");
    }

    {
        print_test_header("Equi (Fisheye) Model Test");
        std::vector<double> K_equi = {320.1, 319.9, 640.0, 360.0};
        std::vector<double> D_equi = {0.01, -0.005, 0.002, -0.001};
        CamEqui equi_camera(width, height, K_equi, D_equi);
        run_correctness_test(equi_camera, "Equi");
        run_speed_test(equi_camera, "Equi");
    }

    return 0;
}
