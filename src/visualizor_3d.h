#ifndef _SLAM_UTILITY_3D_VISUALIZOR_H_
#define _SLAM_UTILITY_3D_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

#include "glad.h"
#include "GLFW/glfw3.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "time.h"

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

struct PoseType {
    Vec3 p_wb = Vec3::Zero();
    Quat q_wb = Quat::Identity();
    float scale = 1.0f;
};

struct VisualizorWindow3D {
    GLFWwindow *glfw_window = nullptr;
    GLuint texture_id = 0;
};

/* Class Visualizor3D 3D Declaration. */
class Visualizor3D {

public:
    virtual ~Visualizor3D();
    static Visualizor3D &GetInstance();

    // Support for interface.
    static void Refresh(const std::string &window_title, const int32_t delay_ms = 0);
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

private:
	Visualizor3D() = default;

    // Support for image draw.
    template <typename ImageType, typename PixelType>
    static void DrawSolidRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawHollowRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawBressenhanLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawNaiveLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawSolidCircle(ImageType &image, int32_t center_x, int32_t center_y, int32_t radius, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawHollowCircle(ImageType &image, int32_t center_x, int32_t center_y, int32_t radius, const PixelType &color);

    static void RefreshLine(const LineType &line, RgbImage &show_image);
    static void RefreshPoint(const PointType &point, RgbImage &show_image);
    static void RefreshPose(const PoseType &pose, RgbImage &show_image);

    // Support for convertion.
    template <typename Scalar>
    static uint8_t ConvertValueToUint8(Scalar value, Scalar max_value);
    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     GrayImage &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);
    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     RgbImage &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);
    static void ConvertUint8ToRgb(const uint8_t *gray,
                                  uint8_t *rgb,
                                  int32_t gray_size);
    static void ConvertRgbToUint8(const uint8_t *rgb,
                                  uint8_t *gray,
                                  int32_t gray_size);
    static void ConvertUint8ToRgbAndUpsideDown(const uint8_t *gray,
                                               uint8_t *rgb,
                                               int32_t gray_rows,
                                               int32_t gray_cols);
    static void ConvertRgbToBgrAndUpsideDown(const uint8_t *rgb,
                                             uint8_t *converted_rgb,
                                             int32_t rgb_rows,
                                             int32_t rgb_cols);
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
    // Member variables.
    static std::map<std::string, VisualizorWindow3D> windows_;
    static bool some_key_pressed_;
    static bool key_x_pressed_;
    static bool mouse_left_pressed_;
    static bool mouse_right_pressed_;
    static float mouse_xpos_;
    static float mouse_ypos_;
    static Quat locked_camera_q_wc_;
    static Vec3 locked_camera_p_wc_;
    static float focus_view_depth_;

    static CameraView camera_view_;
    static std::vector<PointType> points_;
    static std::vector<LineType> lines_;
    static std::vector<PoseType> poses_;
};

}

#endif // end of _SLAM_UTILITY_3D_VISUALIZOR_H_
