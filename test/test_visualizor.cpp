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
    ReportInfo(YELLOW ">> Test 3d visualizor." RESET_COLOR);

    Visualizor3D::Clear();

    for (int32_t i = 0; i < 2; ++i) {
        for (int32_t j = 0; j < 2; ++j) {
            for (int32_t k = 0; k < 2; ++k) {
                const Vec3 point(i * 10, j * 10, k * 10);
                Visualizor3D::points().emplace_back(PointType{
                    .p_w = point,
                    .color = RgbPixel{.r = 0, .g = 255, .b = 255},
                    .radius = 2,
                });
            }
        }
    }

    Visualizor3D::lines().emplace_back(LineType{
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(1, 0, 0),
        .color = RgbPixel{.r = 255, .g = 0, .b = 0},
    });
    Visualizor3D::lines().emplace_back(LineType{
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(0, 1, 0),
        .color = RgbPixel{.r = 0, .g = 255, .b = 0},
    });
    Visualizor3D::lines().emplace_back(LineType{
        .p_w_i = Vec3::Zero(),
        .p_w_j = Vec3(0, 0, 1),
        .color = RgbPixel{.r = 0, .g = 0, .b = 255},
    });

    Visualizor3D::poses().emplace_back(PoseType{
        .p_wb = Vec3::Ones(),
        .q_wb = Quat::Identity(),
        .scale = 10.0f,
    });

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Visualizor 3D", 30);
    }

    return 0;
}
