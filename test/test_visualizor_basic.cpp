#include "basic_type.h"
#include "datatype_image.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "visualizor_3d.h"

#include "cstring"
#include "dirent.h"
#include "iostream"
#include "vector"

using namespace slam_visualizor;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test 3d visualizor of basic items." RESET_COLOR);

    Visualizor3D::Clear();

    for (int32_t i = 0; i < 2; ++i) {
        for (int32_t j = 0; j < 2; ++j) {
            for (int32_t k = 0; k < 2; ++k) {
                const Vec3 point(i * 10, j * 10, k * 10);
                Visualizor3D::points().emplace_back(PointType {
                    .p_w = point,
                    .color = RgbColor::kCyan,
                    .radius = 2,
                });
            }
        }
    }

    Visualizor3D::dashed_lines().emplace_back(DashedLineType {
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(1, 0, 0),
        .dot_step = 2,
        .color = RgbColor::kRed,
    });
    Visualizor3D::dashed_lines().emplace_back(DashedLineType {
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(0, 1, 0),
        .dot_step = 10,
        .color = RgbColor::kGreen,
    });
    Visualizor3D::dashed_lines().emplace_back(DashedLineType {
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(0, 0, 1),
        .dot_step = 5,
        .color = RgbColor::kBlue,
    });

    Visualizor3D::poses().emplace_back(PoseType {
        .p_wb = Vec3::Ones(),
        .q_wb = Quat::Identity(),
        .scale = 10.0f,
    });

    Visualizor3D::ellipses().emplace_back(EllipseType {
        .p_w = Vec3::Zero(),
        .cov = Vec3(1, 4, 9).asDiagonal(),
        .color = RgbColor::kOrangeRed,
    });

    Visualizor3D::camera_poses().emplace_back(CameraPoseType {
        .p_wc = Vec3::Ones() * 2.0f,
        .q_wc = Quat::Identity(),
        .scale = 1.0f,
    });

    Visualizor3D::strings().emplace_back(std::string("I'm a string."));

    int32_t cnt = 0;
    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::strings().clear();
        Visualizor3D::strings().emplace_back(std::string("I'm a string ") + std::to_string(cnt++));
        Visualizor3D::strings().emplace_back(std::string("I'm a string too, ") + std::to_string(cnt++));
        Visualizor3D::Refresh("Visualizor 3D", 30);
    }

    return 0;
}
