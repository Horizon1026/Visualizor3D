#ifndef _SLAM_UTILITY_3D_VISUALIZOR_H_
#define _SLAM_UTILITY_3D_VISUALIZOR_H_

#include "basic_type.h"
#include "datatype_image.h"
#include "slam_log_reporter.h"

#include "3d_gaussian.h"

#include "glad.h"
#include "GLFW/glfw3.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "time.h"

namespace SLAM_VISUALIZOR {

using namespace SLAM_UTILITY;

struct CameraView {
    Quat q_wc = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
    float fx = 600.0f;
    float fy = 600.0f;
    float cx = 400.0f;
    float cy = 400.0f;
};

/* All basic items' type. */
struct PointType {
    Vec3 p_w = Vec3::Zero();
    RgbPixel color = RgbColor::kWhite;
    int32_t radius = 1;
};
struct LineType {
    Vec3 p_w_i = Vec3::Zero();
    Vec3 p_w_j = Vec3::Zero();
    RgbPixel color = RgbColor::kWhite;
};
struct PoseType {
    Vec3 p_wb = Vec3::Zero();
    Quat q_wb = Quat::Identity();
    float scale = 1.0f;
};
struct EllipseType {
    Vec3 p_w = Vec3::Zero();
    Mat3 cov = Mat3::Identity();
    RgbPixel color = RgbColor::kCyan;
};
struct CameraPoseType {
    Vec3 p_wc = Vec3::Zero();
    Quat q_wc = Quat::Identity();
    float scale = 1.0f;
};
struct VisualizorWindow3D {
    GLFWwindow *glfw_window = nullptr;
    GLuint texture_id = 0;
};

/* All advanced items' type. */

/* Class Visualizor3D 3D Declaration. */
class Visualizor3D {

public:
    virtual ~Visualizor3D();
    static Visualizor3D &GetInstance();

    // Render all basic items.
    static void Refresh(const std::string &window_title, const int32_t delay_ms = 0);
    // Render all 3d gaussians. Basic item 'strings' will also be rendered.
    static void Refresh3DGaussians(const std::string &window_title, const int32_t delay_ms = 0);

    // Support for interface.
    template <typename T>
    static bool ShowImage(const std::string &window_title, const T &image, bool resizable = false);
    static void WaitKey(int32_t delay_ms);
    static bool ShouldQuit();
    static void Clear();
    static void WindowList();

    // Reference for member variables.
    static std::map<std::string, VisualizorWindow3D> &windows() { return windows_; }
    static CameraView &camera_view() { return camera_view_; }
    static std::vector<PointType> &points() { return points_; }
    static std::vector<LineType> &lines() { return lines_; }
    static std::vector<PoseType> &poses() { return poses_; }
    static std::vector<EllipseType> &ellipses() { return ellipses_; }
    static std::vector<CameraPoseType> &camera_poses() { return camera_poses_; }
    static std::vector<std::string> &strings() { return strings_; }
    static std::vector<Gaussian3D> &gaussians_3d() { return gaussians_3d_; }

private:
	Visualizor3D() = default;

    // Support for rendering each basic type of items.
    static void RefreshLine(const LineType &line, RgbImage &show_image);
    static void RefreshPoint(const PointType &point, RgbImage &show_image);
    static void RefreshPose(const PoseType &pose, RgbImage &show_image);
    static void RefreshEllipse(const EllipseType &ellipse, RgbImage &show_image);
    static void RefreshCameraPose(const CameraPoseType &camera_pose, RgbImage &show_image);
    static Pixel ConvertPointToImagePlane(const Vec3 &p_c);

    // Callback functions.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);
    static void ScrollCallback(GLFWwindow *window, double xoffset, double yoffset);
    static void MouseButtonCallback(GLFWwindow* window, int32_t button, int32_t action, int32_t mods);
    static void CursorPosCallback(GLFWwindow* window, double xpos, double ypos);

    // Inner supports.
    static VisualizorWindow3D *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    template <typename T> static void PreprocessImage(const T &image, uint8_t *buff);
    template <typename T> static void CreateTextureByImage(const T &image, GLuint &texture_id);
    static void ShowTextureInCurrentWindow(GLuint texture_id);
    static void UpdateFocusViewDepth();

private:
    // Basic parameters.
    static std::map<std::string, VisualizorWindow3D> windows_;
    // Parameters for operations and camera view.
    static CameraView camera_view_;
    static bool some_key_pressed_;
    static bool mouse_left_pressed_;
    static bool mouse_right_pressed_;
    static float mouse_xpos_;
    static float mouse_ypos_;
    static Quat locked_camera_q_wc_;
    static Vec3 locked_camera_p_wc_;
    static float focus_view_depth_;
    // All storaged basic items.
    static std::vector<PointType> points_;
    static std::vector<LineType> lines_;
    static std::vector<PoseType> poses_;
    static std::vector<EllipseType> ellipses_;
    static std::vector<CameraPoseType> camera_poses_;
    static std::vector<std::string> strings_;
    // All storaged advanced items.
    static std::vector<Gaussian3D> gaussians_3d_;
    static std::vector<Gaussian2D> guassians_2d_;
};

}

#endif // end of _SLAM_UTILITY_3D_VISUALIZOR_H_
