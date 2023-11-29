#include "visualizor_3d.h"
#include "visualizor.h"

#include "math_kinematics.h"

#include "slam_memory.h"
#include "log_report.h"
#include "slam_operations.h"

namespace SLAM_VISUALIZOR {

void Visualizor3D::Clear() {
    points_.clear();
}

void Visualizor3D::Show(const std::string &window_title, const int32_t delay_ms) {
    // Create image to show.
    const int32_t image_rows = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t image_cols = static_cast<int32_t>(camera_view_.cx) * 2;
    const int32_t buf_size = image_rows * image_cols * 3;
    uint8_t *buf = (uint8_t *)SlamMemory::Malloc(buf_size * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);

    // Draw all points.
    for (const auto &point : points_) {
        const Vec3 p_c = camera_view_.q_wc.inverse() * (point.p_w - camera_view_.p_wc);
        CONTINUE_IF(p_c.z() < kZero);

        const Vec2 pixel_uv_float = Vec2(
            p_c.x() / p_c.z() * camera_view_.fx + camera_view_.cx,
            p_c.y() / p_c.z() * camera_view_.fy + camera_view_.cy);
        const Pixel pixel_uv = pixel_uv_float.cast<int32_t>();
        Visualizor::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), point.radius, point.color);
    }

    // Show image.
    Visualizor::ShowImage(window_title, show_image);
    Visualizor::WaitKey(delay_ms);
}

}
