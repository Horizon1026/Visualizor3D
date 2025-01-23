#include "visualizor_3d.h"
#include "image_painter.h"
#include "slam_memory.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "slam_basic_math.h"

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
    for (const auto &line : dashed_lines_) {
        Visualizor3D::RefreshDashedLine(line, show_image);
    }
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
    for (const auto &camera_pose : camera_poses_) {
        Visualizor3D::RefreshCameraPose(camera_pose, show_image);
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

void Visualizor3D::RefreshDashedLine(const DashedLineType &line, RgbImage &show_image) {
    ImagePainter::RenderDashedLineSegmentInCameraView(show_image, ImagePainter::CameraView{
        .fx = camera_view_.fx,
        .fy = camera_view_.fy,
        .cx = camera_view_.cx,
        .cy = camera_view_.cy,
        .p_wc = camera_view_.p_wc,
        .q_wc = camera_view_.q_wc,
    }, line.p_w_i, line.p_w_j, line.dot_step, line.color);
}

void Visualizor3D::RefreshLine(const LineType &line, RgbImage &show_image) {
    ImagePainter::RenderLineSegmentInCameraView(show_image, ImagePainter::CameraView{
        .fx = camera_view_.fx,
        .fy = camera_view_.fy,
        .cx = camera_view_.cx,
        .cy = camera_view_.cy,
        .p_wc = camera_view_.p_wc,
        .q_wc = camera_view_.q_wc,
    }, line.p_w_i, line.p_w_j, line.color);
}

void Visualizor3D::RefreshPoint(const PointType &point, RgbImage &show_image) {
    ImagePainter::RenderPointInCameraView(show_image, ImagePainter::CameraView{
        .fx = camera_view_.fx,
        .fy = camera_view_.fy,
        .cx = camera_view_.cx,
        .cy = camera_view_.cy,
        .p_wc = camera_view_.p_wc,
        .q_wc = camera_view_.q_wc,
    }, point.p_w, point.color, point.radius);
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
    ImagePainter::RenderEllipseInCameraView(show_image, ImagePainter::CameraView{
        .fx = camera_view_.fx,
        .fy = camera_view_.fy,
        .cx = camera_view_.cx,
        .cy = camera_view_.cy,
        .p_wc = camera_view_.p_wc,
        .q_wc = camera_view_.q_wc,
    }, ellipse.p_w, ellipse.cov, ellipse.color);
}

void Visualizor3D::RefreshCameraPose(const CameraPoseType &camera_pose, RgbImage &show_image) {
    Visualizor3D::RefreshPoint(PointType{
        .p_w = camera_pose.p_wc,
        .color = RgbColor::kWhite,
        .radius = 2,
    }, show_image);

    const float length_x = camera_pose.scale;
    const float length_y = camera_pose.scale * 0.7f;
    const float length_z = length_y;
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc,
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc,
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc,
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, - length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc,
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, - length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);

    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, length_y, length_z),
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, length_y, length_z),
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, - length_y, length_z),
        .color = RgbColor::kLightGreen,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc + camera_pose.q_wc * Vec3(- length_x, - length_y, length_z),
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, - length_y, length_z),
        .color = RgbColor::kOrangeRed,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, - length_y, length_z),
        .p_w_j = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, length_y, length_z),
        .color = RgbColor::kWhite,
    }, show_image);
}

}
