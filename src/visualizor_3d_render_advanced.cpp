#include "image_painter.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "visualizor_3d.h"

using namespace image_painter;

namespace slam_visualizor {

void Visualizor3D::Refresh3DGaussians(const std::string &window_title, const int32_t delay_ms) {
    // Create image to show.
    const int32_t image_rows = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t image_cols = static_cast<int32_t>(camera_view_.cx) * 2;
    const int32_t buf_size = image_rows * image_cols * 3;
    uint8_t *buf = (uint8_t *)SlamMemory::Malloc(buf_size * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);
    show_image.Clear();

    // Project 3d gaussian to 2d.
    guassians_2d_.clear();
    ellipses_.clear();
    std::vector<float> all_gaussian_depth;
    all_gaussian_depth.reserve(gaussians_3d_.size());
    for (const auto &guassian_3d: gaussians_3d_) {
        Gaussian2D gaussian_2d;
        guassian_3d.ProjectTo2D(camera_view_.p_wc, camera_view_.q_wc, gaussian_2d);
        guassians_2d_.emplace_back(gaussian_2d);
        all_gaussian_depth.emplace_back(gaussian_2d.depth());

        // Add ellipse as coutour of 3d guassians.
        ellipses_.emplace_back(EllipseType {
            .p_w = guassian_3d.p_w(),
            .cov = guassian_3d.sigma(),
            .color = guassian_3d.color(),
        });
    }

    // Sort gaussians by depth in ray space.
    std::vector<int32_t> indices;
    indices.reserve(gaussians_3d_.size());
    SlamOperation::ArgSort(all_gaussian_depth, indices);

    // Iterate each pixel of image to compute color.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - camera_view_.cx) / camera_view_.fx, (row - camera_view_.cy) / camera_view_.fy);

            float occluded_probability = 1.0f;
            Vec3 float_color = Vec3::Zero();
            for (const auto &index: indices) {
                const auto &gaussian_2d = guassians_2d_[index];
                const float powered_alpha = gaussian_2d.GetOpacityAt(uv, gaussian_2d.inv_sigma());
                CONTINUE_IF(powered_alpha < static_cast<float>(1.0f / 255.0f));

                const Vec3 rgb_color = Vec3(gaussian_2d.color().r, gaussian_2d.color().g, gaussian_2d.color().b);
                float_color += rgb_color * powered_alpha * occluded_probability;
                occluded_probability *= 1.0f - powered_alpha;
                BREAK_IF(occluded_probability < 1e-3f);
            }

            const RgbPixel background_color = show_image.GetPixelValueNoCheck(row, col);
            const RgbPixel pixel_color = RgbPixel {
                .r = static_cast<uint8_t>(std::min(255.0f, float_color.x()) * (1.0f - occluded_probability) +
                                          static_cast<float>(background_color.r) * occluded_probability),
                .g = static_cast<uint8_t>(std::min(255.0f, float_color.y()) * (1.0f - occluded_probability) +
                                          static_cast<float>(background_color.g) * occluded_probability),
                .b = static_cast<uint8_t>(std::min(255.0f, float_color.z()) * (1.0f - occluded_probability) +
                                          static_cast<float>(background_color.b) * occluded_probability),
            };
            show_image.SetPixelValueNoCheck(row, col, pixel_color);
        }
    }

    // Draw world frame.
    Visualizor3D::RefreshPose(
        PoseType {
            .p_wb = Vec3::Ones(),
            .q_wb = Quat::Identity(),
            .scale = 1.0f,
        },
        show_image);

    // Draw ellipses tobe coutour of 3d gaussians.
    for (const auto &ellipse: ellipses_) {
        Visualizor3D::RefreshEllipse(ellipse, show_image);
    }

    // Draw strings at the top-left of window.
    const int32_t font_size = 16;
    const std::string cam_view_q_str = std::string("[CameraView] q_wc[wxyz][") + std::to_string(camera_view_.q_wc.w()) + std::string(", ") +
                                       std::to_string(camera_view_.q_wc.x()) + std::string(", ") + std::to_string(camera_view_.q_wc.y()) + std::string(", ") +
                                       std::to_string(camera_view_.q_wc.z()) + std::string("].");
    const std::string cam_view_p_str = std::string("[CameraView] p_wc[xyz][") + std::to_string(camera_view_.p_wc.x()) + std::string(", ") +
                                       std::to_string(camera_view_.p_wc.y()) + std::string(", ") + std::to_string(camera_view_.p_wc.z()) + std::string("].");
    ImagePainter::DrawString(show_image, cam_view_p_str, font_size / 2, 0, RgbColor::kWhite, font_size);
    ImagePainter::DrawString(show_image, cam_view_q_str, font_size / 2, font_size, RgbColor::kWhite, font_size);
    for (uint32_t i = 0; i < strings_.size(); ++i) {
        ImagePainter::DrawString(show_image, strings_[i], font_size / 2, (i + 2) * font_size, RgbColor::kWhite, font_size);
    }

    // Show image.
    Visualizor3D::ShowImage(window_title, show_image);
    Visualizor3D::WaitKey(delay_ms);
}

}  // namespace slam_visualizor
