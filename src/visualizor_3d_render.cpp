#include "visualizor_3d.h"
#include "image_painter.h"
#include "slam_memory.h"
#include "log_report.h"
#include "slam_operations.h"
#include "math_kinematics.h"

using namespace IMAGE_PAINTER;

namespace SLAM_VISUALIZOR {

namespace {
    constexpr float kMinValidViewDepth = 0.1f;
}

void Visualizor3D::Refresh(const std::string &window_title, const int32_t delay_ms) {
    // Create image to show.
    const int32_t image_rows = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t image_cols = static_cast<int32_t>(camera_view_.cx) * 2;
    const int32_t buf_size = image_rows * image_cols * 3;
    uint8_t *buf = (uint8_t *)SlamMemory::Malloc(buf_size * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);
    show_image.Clear();

    // Draw items in world frame.
    for (const auto &line : lines_) {
        Visualizor3D::RefreshLine(line, show_image);
    }
    for (const auto &point : points_) {
        Visualizor3D::RefreshPoint(point, show_image);
    }
    for (const auto &pose : poses_) {
        Visualizor3D::RefreshPose(pose, show_image);
    }
    for (const auto &ellipse : ellipses_) {
        Visualizor3D::RefreshEllipse(ellipse, show_image);
    }

    // Draw strings at the top-left of window.
    const int32_t font_size = 16;
    const std::string cam_view_q_str = std::string("[CameraView] q_wc[wxyz][") +
        std::to_string(camera_view_.q_wc.w()) + std::string(", ") +
        std::to_string(camera_view_.q_wc.x()) + std::string(", ") +
        std::to_string(camera_view_.q_wc.y()) + std::string(", ") +
        std::to_string(camera_view_.q_wc.z()) + std::string("].");
    const std::string cam_view_p_str = std::string("[CameraView] p_wc[xyz][") +
        std::to_string(camera_view_.p_wc.x()) + std::string(", ") +
        std::to_string(camera_view_.p_wc.y()) + std::string(", ") +
        std::to_string(camera_view_.p_wc.z()) + std::string("].");
    ImagePainter::DrawString(show_image, cam_view_p_str, font_size / 2, 0, RgbColor::kWhite, font_size);
    ImagePainter::DrawString(show_image, cam_view_q_str, font_size / 2, font_size, RgbColor::kWhite, font_size);
    for (uint32_t i = 0; i < strings_.size(); ++i) {
        ImagePainter::DrawString(show_image, strings_[i], font_size / 2, (i + 2) * font_size, RgbColor::kWhite, font_size);
    }

    // Show image.
    Visualizor3D::ShowImage(window_title, show_image);
    Visualizor3D::WaitKey(delay_ms);
}

void Visualizor3D::RefreshLine(const LineType &line, RgbImage &show_image) {
    Vec3 p_c_i = camera_view_.q_wc.inverse() * (line.p_w_i - camera_view_.p_wc);
    Vec3 p_c_j = camera_view_.q_wc.inverse() * (line.p_w_j - camera_view_.p_wc);
    RETURN_IF(p_c_i.z() < kMinValidViewDepth && p_c_j.z() < kMinValidViewDepth);

    // If one point of line is outside, cut this line to make the two points of new line all visilbe.
    if (p_c_i.z() < kMinValidViewDepth || p_c_j.z() < kMinValidViewDepth) {
        const float w = (p_c_i.z() - kMinValidViewDepth) / (p_c_i.z() - p_c_j.z());
        const Vec3 p_c_mid = Vec3(w * p_c_j.x() + (1.0f - w) * p_c_i.x(),
                                  w * p_c_j.y() + (1.0f - w) * p_c_i.y(),
                                  w * p_c_j.z() + (1.0f - w) * p_c_i.z());
        if (p_c_i.z() < kMinValidViewDepth) {
            p_c_i = p_c_mid;
        } else {
            p_c_j = p_c_mid;
        }
    }

    const Pixel pixel_uv_i = Visualizor3D::ConvertPointToImagePlane(p_c_i);
    const Pixel pixel_uv_j = Visualizor3D::ConvertPointToImagePlane(p_c_j);
    ImagePainter::DrawBressenhanLine(show_image, pixel_uv_i.x(), pixel_uv_i.y(), pixel_uv_j.x(), pixel_uv_j.y(), line.color);
}

void Visualizor3D::RefreshPoint(const PointType &point, RgbImage &show_image) {
    const Vec3 p_c = camera_view_.q_wc.inverse() * (point.p_w - camera_view_.p_wc);
    RETURN_IF(p_c.z() < kMinValidViewDepth);
    const Pixel pixel_uv = Visualizor3D::ConvertPointToImagePlane(p_c);
    ImagePainter::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), point.radius, point.color);
}

void Visualizor3D::RefreshPose(const PoseType &pose, RgbImage &show_image) {
    Visualizor3D::RefreshPoint(PointType{
        .p_w = pose.p_wb,
        .color = RgbColor::kWhite,
        .radius = 2,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(pose.scale, 0.0f, 0.0f),
        .color = RgbColor::kRed,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(0.0f, pose.scale, 0.0f),
        .color = RgbColor::kGreen,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(0.0f, 0.0f, pose.scale),
        .color = RgbColor::kBlue,
    }, show_image);
}

void Visualizor3D::RefreshEllipse(const EllipseType &ellipse, RgbImage &show_image) {
    // Transform gaussian ellipse into camera frame.
    const Vec3 p_c = camera_view_.q_wc.inverse() * (ellipse.p_w - camera_view_.p_wc);
    const Mat3 cov_c = camera_view_.q_wc.inverse() * ellipse.cov * camera_view_.q_wc;
    RETURN_IF(p_c.z() < kZero);

    // Compute focus of camera.
    const float focus = 0.5f * (camera_view_.fx + camera_view_.fy);

    // Transform 3d gaussian into 2d gaussian.
    const float inv_depth = 1.0f / p_c.z();
    const float inv_depth_2 = inv_depth * inv_depth;
    Mat2x3 jacobian_2d_3d = Mat2x3::Zero();
    if (!std::isnan(inv_depth)) {
        jacobian_2d_3d << inv_depth, 0, - p_c(0) * inv_depth_2,
                          0, inv_depth, - p_c(1) * inv_depth_2;
        jacobian_2d_3d = jacobian_2d_3d * focus;
    }
    const Mat2 pixel_cov = jacobian_2d_3d * cov_c * jacobian_2d_3d.transpose();
    const Vec2 pixel_uv = p_c.head<2>() * inv_depth * focus + Vec2(camera_view_.cx, camera_view_.cy);

    // Draw boundary of 2d gaussian ellipse.
    ImagePainter::DrawTrustRegionOfGaussian(show_image, pixel_uv, pixel_cov, ellipse.color);
}

void Visualizor3D::UpdateFocusViewDepth() {
    if (points_.empty()) {
        focus_view_depth_ = 1.0f;
        return;
    }

    std::vector<float> distances;
    if (Visualizor3D::key_x_pressed_) {
        distances.reserve(poses_.size());
        for (const auto &pose : poses_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (pose.p_wb - camera_view_.p_wc);
            if (p_c.z() > kZero) {
                distances.emplace_back(p_c.z());
            }
        }
    } else {
        distances.reserve(points_.size());
        for (const auto &point : points_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (point.p_w - camera_view_.p_wc);
            if (p_c.z() > kZero) {
                distances.emplace_back(p_c.z());
            }
        }
    }

    // Extract mid value.
    if (!distances.empty()) {
        focus_view_depth_ = distances[distances.size() >> 1];
    }
}

}
