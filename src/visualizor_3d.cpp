#include "visualizor_3d.h"
#include "slam_memory.h"
#include "log_report.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_VISUALIZOR {

namespace {
    constexpr float kSpeedOfTranslation = 0.02f;
    constexpr float kSpeedOfRotation = 0.005f;
    constexpr float kSpeedOfScale = 0.1f;
}

std::map<std::string, VisualizorWindow3D> Visualizor3D::windows_;
bool Visualizor3D::some_key_pressed_ = false;
bool Visualizor3D::key_x_pressed_ = false;
bool Visualizor3D::mouse_left_pressed_ = false;
bool Visualizor3D::mouse_right_pressed_ = false;
float Visualizor3D::mouse_xpos_ = 0.0f;
float Visualizor3D::mouse_ypos_ = 0.0f;
Quat Visualizor3D::locked_camera_q_wc_ = Quat::Identity();
Vec3 Visualizor3D::locked_camera_p_wc_ = Vec3::Zero();
float Visualizor3D::focus_view_depth_ = 1.0f;
CameraView Visualizor3D::camera_view_;
std::vector<PointType> Visualizor3D::points_;
std::vector<LineType> Visualizor3D::lines_;
std::vector<PoseType> Visualizor3D::poses_;

Visualizor3D &Visualizor3D::GetInstance() {
    static Visualizor3D instance;
    return instance;
}

Visualizor3D::~Visualizor3D() {
    // Clear all windows and recovery resources.
    Visualizor3D::windows_.clear();
    glfwTerminate();
}

void Visualizor3D::ErrorCallback(int32_t error, const char *description) {
    ReportError("[Visualizor3D] Error detected :" << description);
}

void Visualizor3D::KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE: {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            }
            case GLFW_KEY_X: {
                Visualizor3D::key_x_pressed_ = true;
                break;
            }
            default: {
                Visualizor3D::some_key_pressed_ = true;
            }
        }
    } else {
        Visualizor3D::some_key_pressed_ = false;
        Visualizor3D::key_x_pressed_ = false;
    }
}

void Visualizor3D::ScrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
    Visualizor3D::UpdateFocusViewDepth();
    camera_view_.p_wc += camera_view_.q_wc * Vec3(0, 0, yoffset * focus_view_depth_ * kSpeedOfScale);
}

void Visualizor3D::MouseButtonCallback(GLFWwindow* window, int32_t button, int32_t action, int32_t mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            mouse_left_pressed_ = true;
            locked_camera_p_wc_ = camera_view_.p_wc;
            Visualizor3D::UpdateFocusViewDepth();
        } else if (action == GLFW_RELEASE) {
            mouse_left_pressed_ = false;
        }
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            mouse_right_pressed_ = true;
            locked_camera_p_wc_ = camera_view_.p_wc;
            locked_camera_q_wc_ = camera_view_.q_wc;
            Visualizor3D::UpdateFocusViewDepth();
        } else if (action == GLFW_RELEASE) {
            mouse_right_pressed_ = false;
        }
    }
}

void Visualizor3D::CursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    if (!mouse_left_pressed_ && !mouse_right_pressed_) {
        mouse_xpos_ = static_cast<float>(xpos);
        mouse_ypos_ = static_cast<float>(ypos);
    } else if (mouse_left_pressed_) {
        camera_view_.p_wc = locked_camera_p_wc_ - camera_view_.q_wc * Vec3(
            static_cast<float>(xpos) - mouse_xpos_, static_cast<float>(ypos) - mouse_ypos_, 0) * kSpeedOfTranslation;
    } else if (mouse_right_pressed_) {
        // Project camera view pose to frame o.
        const Vec3 p_wo = locked_camera_p_wc_ + locked_camera_q_wc_ * Vec3(0, 0, focus_view_depth_);
        const Quat q_wo = locked_camera_q_wc_;
        // T_oc = T_wo.inv * T_wc.
        // [R_oc  t_oc] = [R_wo.t  -R_wo.t * t_wo] * [R_wc  t_wc]
        //              = [R_wo.t * R_wc  R_wo.t * t_wc - R_wo.t * t_wo]
        const Vec3 p_oc = q_wo.inverse() * locked_camera_p_wc_ - q_wo.inverse() * p_wo;
        const Quat q_oc = q_wo.inverse() * locked_camera_q_wc_;

        // Compute delta rotation.
        const Vec3 angle_axis = Vec3(mouse_ypos_ - static_cast<float>(ypos),
                                     static_cast<float>(xpos) - mouse_xpos_,
                                     0) * kSpeedOfRotation;
        const Quat dq = Utility::ConvertAngleAxisToQuaternion(angle_axis);

        // Transform camera view base on frame o.
        const Vec3 new_p_oc = dq * p_oc;
        const Quat new_q_oc = (q_oc * dq).normalized();

        // Reproject camera view pose to frame w.
        // T_wc = T_wo * T_oc.
        // [R_wc  t_wc] = [R_wo  t_wo] * [R_oc  t_oc]
        //              = [R_wo * R_oc  R_wo * t_oc + t_wo]
        camera_view_.p_wc = q_wo * new_p_oc + p_wo;
        camera_view_.q_wc = q_wo * new_q_oc;
    }
}

VisualizorWindow3D *Visualizor3D::GetWindowPointer(const std::string &title, int32_t width, int32_t height) {
    auto item = Visualizor3D::windows_.find(title);
    if (item == Visualizor3D::windows_.end()) {
        // If window with selected title is not found, create a new window.
        auto iter = Visualizor3D::windows_.insert(std::make_pair(title, VisualizorWindow3D()));
        iter.first->second.glfw_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

        // If insert failed, clear it.
        if (iter.first->second.glfw_window == nullptr) {
            Visualizor3D::windows_.erase(title);
            return nullptr;
        }
        return &(iter.first->second);
    } else {
        // Return the exist window.
        return &(item->second);
    }
}

void Visualizor3D::Clear() {
    points_.clear();
    lines_.clear();
    poses_.clear();
}

void Visualizor3D::Refresh(const std::string &window_title, const int32_t delay_ms) {
    // Create image to show.
    const int32_t image_rows = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t image_cols = static_cast<int32_t>(camera_view_.cx) * 2;
    const int32_t buf_size = image_rows * image_cols * 3;
    uint8_t *buf = (uint8_t *)SlamMemory::Malloc(buf_size * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);
    show_image.Clear();

    for (const auto &line : lines_) {
        Visualizor3D::RefreshLine(line, show_image);
    }
    for (const auto &point : points_) {
        Visualizor3D::RefreshPoint(point, show_image);
    }
    for (const auto &pose : poses_) {
        Visualizor3D::RefreshPose(pose, show_image);
    }

    // Show image.
    Visualizor3D::ShowImage(window_title, show_image);
    Visualizor3D::WaitKey(delay_ms);
}

void Visualizor3D::RefreshLine(const LineType &line, RgbImage &show_image) {
    Vec3 p_c_i = camera_view_.q_wc.inverse() * (line.p_w_i - camera_view_.p_wc);
    Vec3 p_c_j = camera_view_.q_wc.inverse() * (line.p_w_j - camera_view_.p_wc);
    RETURN_IF(p_c_i.z() < kZero && p_c_j.z() < kZero);

    // If one point of line is outside, cut this line to make the two points of new line all visilbe.
    const float min_pz = 0.1f;
    if (p_c_i.z() < min_pz || p_c_j.z() < min_pz) {
        const float w = (p_c_i.z() - min_pz) / (p_c_i.z() - p_c_j.z());
        const Vec3 p_c_mid = Vec3(w * p_c_j.x() + (1.0f - w) * p_c_i.x(),
                                  w * p_c_j.y() + (1.0f - w) * p_c_i.y(),
                                  w * p_c_j.z() + (1.0f - w) * p_c_i.z());
        if (p_c_i.z() < min_pz) {
            p_c_i = p_c_mid;
        } else {
            p_c_j = p_c_mid;
        }
    }

    const Pixel pixel_uv_i = Visualizor3D::ConvertPointToImagePlane(p_c_i);
    const Pixel pixel_uv_j = Visualizor3D::ConvertPointToImagePlane(p_c_j);
    Visualizor3D::DrawBressenhanLine(show_image, pixel_uv_i.x(), pixel_uv_i.y(), pixel_uv_j.x(), pixel_uv_j.y(), line.color);
}

void Visualizor3D::RefreshPoint(const PointType &point, RgbImage &show_image) {
    const Vec3 p_c = camera_view_.q_wc.inverse() * (point.p_w - camera_view_.p_wc);
    RETURN_IF(p_c.z() < kZero);
    const Pixel pixel_uv = Visualizor3D::ConvertPointToImagePlane(p_c);
    Visualizor3D::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), point.radius, point.color);
}

void Visualizor3D::RefreshPose(const PoseType &pose, RgbImage &show_image) {
    Visualizor3D::RefreshPoint(PointType{
        .p_w = pose.p_wb,
        .color = RgbPixel{.r = 255, .g = 255, .b = 255},
        .radius = 2,
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(pose.scale, 0.0f, 0.0f),
        .color = RgbPixel{.r = 255, .g = 0, .b = 0},
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(0.0f, pose.scale, 0.0f),
        .color = RgbPixel{.r = 0, .g = 255, .b = 0},
    }, show_image);
    Visualizor3D::RefreshLine(LineType{
        .p_w_i = pose.p_wb,
        .p_w_j = pose.p_wb + pose.q_wb * Vec3(0.0f, 0.0f, pose.scale),
        .color = RgbPixel{.r = 0, .g = 150, .b = 255},
    }, show_image);
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
