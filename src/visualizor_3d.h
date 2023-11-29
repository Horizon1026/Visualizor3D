#ifndef _SLAM_UTILITY_3D_VISUALIZOR_H_
#define _SLAM_UTILITY_3D_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

namespace SLAM_VISUALIZOR {

struct CameraView {
    Quat q_wc = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
    float fx = 200.0f;
    float fy = 200.0f;
    float cx = 400.0f;
    float cy = 400.0f;
};

struct PointType {
    Vec3 p_w = Vec3::Zero();
    RgbPixel color = RgbPixel{.r = 255, .g = 255, .b = 255};
    int32_t radius = 1;
};

struct LineType {
    Vec3 p_w_i = Vec3::Zero();
    Vec3 p_w_j = Vec3::Zero();
    RgbPixel color = RgbPixel{.r = 255, .g = 255, .b = 255};
};

/* Class Visualizor 3D Declaration. */
class Visualizor3D {

public:
    Visualizor3D() = default;
    ~Visualizor3D() = default;

    void Clear();

    void Show(const std::string &window_title);

    // Reference for member variables.
    CameraView &camera_view() { return camera_view_; }
    std::vector<PointType> &points() { return points_; }
    std::vector<LineType> &lines() { return lines_; }

private:
    CameraView camera_view_;

    std::vector<PointType> points_;
    std::vector<LineType> lines_;

};

}

#endif // end of _SLAM_UTILITY_3D_VISUALIZOR_H_
