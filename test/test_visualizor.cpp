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

    Visualizor3D visualizor;
    visualizor.Clear();

    for (int32_t i = 0; i < 5; ++i) {
        for (int32_t j = 0; j < 5; ++j) {
            for (int32_t k = 0; k < 5; ++k) {
                const Vec3 point(i, j, k);
                visualizor.points().emplace_back(PointType{
                    .p_w = point,
                    .color = RgbPixel{.r = 255, .g = 0, .b = 0},
                    .radius = 2,
                });
            }
        }
    }

    for (int32_t i = 0; i < 200; ++i) {
        visualizor.camera_view().p_wc += Vec3(0.05, 0.05, -0.01);
        visualizor.camera_view().q_wc *= Utility::ConvertAngleAxisToQuaternion(Vec3(0.001, -0.001, 0.001));
        visualizor.Show("Visualizor 3D", 10);
    }

    return 0;
}
