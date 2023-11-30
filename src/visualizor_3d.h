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

struct VisualizorWindow {
    GLFWwindow *glfw_window = nullptr;
    GLuint texture_id = 0;
};

/* Class Visualizor3D 3D Declaration. */
class Visualizor3D {

public:
    virtual ~Visualizor3D();
    static Visualizor3D &GetInstance();

    // Support for image show.
    static void Refresh(const std::string &window_title, const int32_t delay_ms = 0);
    template <typename T>
    static bool ShowImage(const std::string &window_title, const T &image, bool resizable = false);
    static void WaitKey(int32_t delay_ms);
    static void Clear();
    static void WindowList();

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

    // Reference for member variables.
    static std::map<std::string, VisualizorWindow> &windows() { return windows_; }
    static bool &some_key_pressed() { return some_key_pressed_; }
    static CameraView &camera_view() { return camera_view_; }
    static std::vector<PointType> &points() { return points_; }
    static std::vector<LineType> &lines() { return lines_; }

private:
	Visualizor3D() = default;

    // Callback function for image show.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);

    // Inner support for image show.
    static VisualizorWindow *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    template <typename T> static void PreprocessImage(const T &image, uint8_t *buff);
    template <typename T> static void CreateTextureByImage(const T &image, GLuint &texture_id);
    static void ShowTextureInCurrentWindow(GLuint texture_id);

private:
    // Member variables for image show.
    static std::map<std::string, VisualizorWindow> windows_;
    static bool some_key_pressed_;

    static CameraView camera_view_;
    static std::vector<PointType> points_;
    static std::vector<LineType> lines_;
};

}

#endif // end of _SLAM_UTILITY_3D_VISUALIZOR_H_
