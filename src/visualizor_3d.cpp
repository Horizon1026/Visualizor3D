#include "visualizor_3d.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"

namespace slam_visualizor {

namespace {
    constexpr float kSpeedOfTranslation = 1.0f;
    constexpr float kSpeedOfRotation = 0.005f;
    constexpr float kSpeedOfScale = 0.1f;
}  // namespace

// Basic parameters.
std::map<std::string, VisualizorWindow3D> Visualizor3D::windows_;
// Parameters for operations and camera view.
CameraView Visualizor3D::camera_view_;
bool Visualizor3D::some_key_pressed_ = false;
bool Visualizor3D::mouse_left_pressed_ = false;
bool Visualizor3D::mouse_right_pressed_ = false;
bool Visualizor3D::mouse_mid_pressed_ = false;
float Visualizor3D::mouse_xpos_ = 0.0f;
float Visualizor3D::mouse_ypos_ = 0.0f;
Quat Visualizor3D::locked_camera_q_wc_ = Quat::Identity();
Vec3 Visualizor3D::locked_camera_p_wc_ = Vec3::Zero();
float Visualizor3D::focus_view_depth_ = 1.0f;
// All storaged basic items.
std::vector<PointType> Visualizor3D::points_;
std::vector<LineType> Visualizor3D::lines_;
std::vector<DashedLineType> Visualizor3D::dashed_lines_;
std::vector<PoseType> Visualizor3D::poses_;
std::vector<EllipseType> Visualizor3D::ellipses_;
std::vector<CameraPoseType> Visualizor3D::camera_poses_;
std::vector<std::string> Visualizor3D::strings_;
// All storaged advanced items.
std::vector<Gaussian3D> Visualizor3D::gaussians_3d_;
std::vector<Gaussian2D> Visualizor3D::guassians_2d_;

Visualizor3D &Visualizor3D::GetInstance() {
    static Visualizor3D instance;
    return instance;
}

Visualizor3D::~Visualizor3D() {
    // Clear all windows and recovery resources.
    Visualizor3D::windows_.clear();
    glfwTerminate();
}

void Visualizor3D::ErrorCallback(int32_t error, const char *description) { ReportError("[Visualizor3D] Error detected :" << description); }

void Visualizor3D::KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE: {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            }
            default: {
                Visualizor3D::some_key_pressed_ = true;
            }
        }
    } else {
        Visualizor3D::some_key_pressed_ = false;
    }
}

void Visualizor3D::ScrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
    Visualizor3D::UpdateFocusViewDepth();
    camera_view_.p_wc += camera_view_.q_wc * Vec3(0, 0, yoffset * focus_view_depth_ * kSpeedOfScale);
}

void Visualizor3D::MouseButtonCallback(GLFWwindow *window, int32_t button, int32_t action, int32_t mods) {
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

    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            mouse_mid_pressed_ = true;
            locked_camera_p_wc_ = camera_view_.p_wc;
            locked_camera_q_wc_ = camera_view_.q_wc;
            Visualizor3D::UpdateFocusViewDepth();
        } else if (action == GLFW_RELEASE) {
            mouse_mid_pressed_ = false;
        }
    }
}

void Visualizor3D::CursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
    if (mouse_left_pressed_) {
        camera_view_.p_wc = locked_camera_p_wc_ - camera_view_.q_wc * Vec3(static_cast<float>(xpos) - mouse_xpos_, static_cast<float>(ypos) - mouse_ypos_, 0) /
                                                      camera_view_.fx * kSpeedOfTranslation * focus_view_depth_;
    } else if (mouse_right_pressed_) {
        // Project camera view pose to frame focus.
        // Frame focus should has the same x-axis as frame camera, and should has the same z-axis as frame world.
        const Vec3 x_axis_q_wf = locked_camera_q_wc_ * Vec3::UnitX();
        const Vec3 z_axis_q_wf = Vec3::UnitZ();
        Mat3 R_wf = Mat3::Zero();
        R_wf.col(0) = x_axis_q_wf;
        R_wf.col(2) = z_axis_q_wf;
        R_wf.col(1) = z_axis_q_wf.cross(x_axis_q_wf).normalized();
        const Quat q_wf = Quat(R_wf);
        const Vec3 p_wf = locked_camera_p_wc_ + locked_camera_q_wc_ * Vec3(0, 0, focus_view_depth_);
        // T_fc = T_wf.inv * T_wc.
        Vec3 p_fc = Vec3::Zero();
        Quat q_fc = Quat::Identity();
        Utility::ComputeTransformInverseTransform(p_wf, q_wf, locked_camera_p_wc_, locked_camera_q_wc_, p_fc, q_fc);
        // Compute delta rotation.
        const Vec3 angle_axis_z = Vec3(0, 0, mouse_xpos_ - static_cast<float>(xpos)) * kSpeedOfRotation;
        const Vec3 angle_axis_x = Vec3(mouse_ypos_ - static_cast<float>(ypos), 0, 0) * kSpeedOfRotation;
        // Rotate frame focus. Firstly rotate by axis z, then rotate by axis x.
        const Vec3 new_p_wf = p_wf;
        const Quat new_q_wf = q_wf * Utility::Exponent(angle_axis_z) * Utility::Exponent(angle_axis_x);
        // Recovery frame camera by fixed T_fc and rotated T_wf.
        camera_view_.q_wc = new_q_wf * q_fc;
        camera_view_.p_wc = new_p_wf - camera_view_.q_wc * Vec3(0, 0, focus_view_depth_);
    } else if (mouse_mid_pressed_) {
        // Force set camera view to be horizontal.
        Vec3 euler_rpy_q_wc = Utility::QuaternionToEuler(camera_view_.q_wc);
        euler_rpy_q_wc.x() = 0.0f;
        camera_view_.q_wc = Utility::EulerToQuaternion(euler_rpy_q_wc);
    } else {
        mouse_xpos_ = static_cast<float>(xpos);
        mouse_ypos_ = static_cast<float>(ypos);
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
        // Resize window size.
        glfwSetWindowSize(item->second.glfw_window, width, height);
        // Return the exist window.
        return &(item->second);
    }
}

void Visualizor3D::Clear() {
    Visualizor3D::points_.clear();
    Visualizor3D::lines_.clear();
    Visualizor3D::dashed_lines_.clear();
    Visualizor3D::poses_.clear();
    Visualizor3D::camera_poses_.clear();
    Visualizor3D::ellipses_.clear();
    Visualizor3D::strings_.clear();
}

void Visualizor3D::UpdateFocusViewDepth() {
    constexpr float kMinViewDepthForFocusViewDepth = 1.0f;
    constexpr float kMinSineAngleForFocusViewDepth = 0.03f;

    Vec3 current_focus_p_w = Vec3::Zero();
    int32_t number_of_focus_points_w = 0;
    if (number_of_focus_points_w == 0) {
        for (const auto &point: points_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (point.p_w - camera_view_.p_wc);
            CONTINUE_IF(p_c.z() < kMinViewDepthForFocusViewDepth);
            const float sine_angle = Vec3::UnitZ().cross(p_c.normalized()).norm();
            CONTINUE_IF(std::fabs(sine_angle) > kMinSineAngleForFocusViewDepth);
            current_focus_p_w += point.p_w;
            ++number_of_focus_points_w;
        }
    }
    if (number_of_focus_points_w == 0) {
        for (const auto &camera_pose: camera_poses_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (camera_pose.p_wc - camera_view_.p_wc);
            CONTINUE_IF(p_c.z() < kMinViewDepthForFocusViewDepth);
            const float sine_angle = Vec3::UnitZ().cross(p_c.normalized()).norm();
            CONTINUE_IF(std::fabs(sine_angle) > kMinSineAngleForFocusViewDepth);
            current_focus_p_w += camera_pose.p_wc;
            ++number_of_focus_points_w;
        }
    }
    if (number_of_focus_points_w == 0) {
        for (const auto &pose: poses_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (pose.p_wb - camera_view_.p_wc);
            CONTINUE_IF(p_c.z() < kMinViewDepthForFocusViewDepth);
            const float sine_angle = Vec3::UnitZ().cross(p_c.normalized()).norm();
            CONTINUE_IF(std::fabs(sine_angle) > kMinSineAngleForFocusViewDepth);
            current_focus_p_w += pose.p_wb;
            ++number_of_focus_points_w;
        }
    }
    if (number_of_focus_points_w == 0) {
        for (const auto &gaussian_3d: gaussians_3d_) {
            const Vec3 p_c = camera_view_.q_wc.inverse() * (gaussian_3d.p_w() - camera_view_.p_wc);
            CONTINUE_IF(p_c.z() < kMinViewDepthForFocusViewDepth);
            const float sine_angle = Vec3::UnitZ().cross(p_c.normalized()).norm();
            CONTINUE_IF(std::fabs(sine_angle) > kMinSineAngleForFocusViewDepth);
            current_focus_p_w += gaussian_3d.p_w();
            ++number_of_focus_points_w;
        }
    }

    // Compute focus view depth.
    if (number_of_focus_points_w > 0) {
        focus_view_depth_ = (current_focus_p_w / static_cast<float>(number_of_focus_points_w) - camera_view_.p_wc).norm();
    } else {
        focus_view_depth_ = std::fabs((camera_view_.q_wc.inverse() * camera_view_.p_wc).z());
    }
    focus_view_depth_ = std::max(1.0f, focus_view_depth_);
}

}  // namespace slam_visualizor
