#include "visualizor_3d.h"
#include "slam_memory.h"
#include "log_report.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_VISUALIZOR {

namespace {
    constexpr float kSpeedOfTranslation = 1.0f;
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
std::vector<EllipseType> Visualizor3D::ellipses_;
std::vector<CameraPoseType> Visualizor3D::camera_poses_;
std::vector<std::string> Visualizor3D::strings_;

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
            static_cast<float>(xpos) - mouse_xpos_, static_cast<float>(ypos) - mouse_ypos_, 0) /
            camera_view_.fx * kSpeedOfTranslation * focus_view_depth_;
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
        // Resize window size.
        glfwSetWindowSize(item->second.glfw_window, width, height);
        // Return the exist window.
        return &(item->second);
    }
}

void Visualizor3D::Clear() {
    points_.clear();
    lines_.clear();
    poses_.clear();
    camera_poses_.clear();
    ellipses_.clear();
    strings_.clear();
}

}
