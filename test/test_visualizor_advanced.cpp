#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor_3d.h"
#include "log_report.h"
#include "slam_operations.h"
#include "math_kinematics.h"

#include "iostream"
#include "dirent.h"
#include "vector"
#include "cstring"

using namespace SLAM_VISUALIZOR;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test 3d visualizor of advanced items." RESET_COLOR);

    Visualizor3D::Clear();

    // Color of 3d gaussians.
    std::vector<RgbPixel> all_colors = {RgbColor::kHotPink, RgbColor::kGreen, RgbColor::kGold};
    // Create 3d gaussian.
    for (uint32_t i = 0; i < all_colors.size(); ++i) {
        Gaussian3D gaussian_3d;
        gaussian_3d.color() = all_colors[i];
        gaussian_3d.p_w() = Vec3(i * 0.3f, i * 0.4f, 2.5f + i * 0.3f);
        gaussian_3d.mid_opacity() = 1.0f;
        gaussian_3d.sigma_s() = Vec3(0.1f, 0.2f, 0.4f);
        gaussian_3d.sigma_q() = Quat::Identity();
        Visualizor3D::gaussians_3d().emplace_back(gaussian_3d);
    }

    Visualizor3D::strings().emplace_back(std::string("Show several 3d gaussians."));

    Visualizor3D::camera_view().p_wc = Vec3(6, 5, -20);
    Visualizor3D::camera_view().q_wc = Quat::Identity();

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh3DGaussians("Visualizor 3D", 10);
    }

    return 0;
}
