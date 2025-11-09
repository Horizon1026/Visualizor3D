#include "image_painter.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "visualizor_3d.h"

#include "unistd.h"

using namespace IMAGE_PAINTER;

namespace SLAM_VISUALIZOR {

template bool Visualizor3D::ShowImage<GrayImage>(const std::string &window_title, const GrayImage &image, bool resizable);
template bool Visualizor3D::ShowImage<RgbImage>(const std::string &window_title, const RgbImage &image, bool resizable);
template <typename T>
bool Visualizor3D::ShowImage(const std::string &window_title, const T &image, bool resizable) {
    if (image.data() == nullptr || image.rows() < 1 || image.cols() < 1) {
        ReportError("[Visualizor3D] ShowImage() got an invalid image.");
        return false;
    }

    auto item = Visualizor3D::windows_.find(window_title);
    if (item == Visualizor3D::windows_.end()) {
        // New window. Create a new window and a new texture.
        glfwSetErrorCallback(Visualizor3D::ErrorCallback);

        if (!glfwInit()) {
            ReportError("[Visualizor3D] GLFW initialize failed.");
            return false;
        }

        if (resizable) {
            glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        } else {
            glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        }

        VisualizorWindow3D *window = GetWindowPointer(window_title, image.cols(), image.rows());
        glfwMakeContextCurrent(window->glfw_window);

        gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
        glfwSwapInterval(1);
        glfwShowWindow(window->glfw_window);

        glfwSetKeyCallback(window->glfw_window, Visualizor3D::KeyboardCallback);
        glfwSetScrollCallback(window->glfw_window, Visualizor3D::ScrollCallback);
        glfwSetMouseButtonCallback(window->glfw_window, Visualizor3D::MouseButtonCallback);
        glfwSetCursorPosCallback(window->glfw_window, Visualizor3D::CursorPosCallback);

        Visualizor3D::CreateTextureByImage<T>(image, window->texture_id);

    } else {
        // Exist window. Only change its texture.
        VisualizorWindow3D *window = GetWindowPointer(window_title, image.cols(), image.rows());
        glfwMakeContextCurrent(window->glfw_window);
        glfwSetWindowShouldClose(window->glfw_window, GLFW_FALSE);
        Visualizor3D::CreateTextureByImage<T>(image, window->texture_id);
    }

    return true;
}

void Visualizor3D::WaitKey(int32_t delay_ms) {
    // Display add window hidden in ShowImage().
    for (const auto &item: Visualizor3D::windows_) {
        const auto &glfw_window = item.second.glfw_window;
        if (!glfwWindowShouldClose(glfw_window)) {
            // If this window is already shown, do not focus on it.
            if (glfwGetWindowAttrib(glfw_window, GLFW_FOCUS_ON_SHOW) != GLFW_FALSE) {
                glfwSetWindowAttrib(glfw_window, GLFW_FOCUS_ON_SHOW, GLFW_FALSE);
            }
            glfwShowWindow(glfw_window);
        } else {
            glfwHideWindow(glfw_window);
        }
    }

    while (!Visualizor3D::windows_.empty()) {
        uint32_t closed_window_cnt = 0;
        for (const auto &item: Visualizor3D::windows_) {
            const auto &window = item.second;

            if (!glfwWindowShouldClose(window.glfw_window)) {
                glfwMakeContextCurrent(window.glfw_window);
                Visualizor3D::ShowTextureInCurrentWindow(window.texture_id);
                glfwSwapBuffers(window.glfw_window);
            } else {
                glfwHideWindow(window.glfw_window);
                ++closed_window_cnt;
            }

            glfwPollEvents();
        }

        if (delay_ms > 0) {
            usleep(delay_ms * 1000);
            break;
        }

        BREAK_IF(closed_window_cnt == Visualizor3D::windows_.size());
        BREAK_IF(Visualizor3D::some_key_pressed_);
    }

    // Resource recovery is in Destructor Function.
    Visualizor3D::some_key_pressed_ = false;
}

bool Visualizor3D::ShouldQuit() {
    for (const auto &item: Visualizor3D::windows_) {
        const auto &window = item.second;
        if (glfwWindowShouldClose(window.glfw_window)) {
            return true;
        }
    }
    return false;
}

template <>
void Visualizor3D::PreprocessImage<GrayImage>(const GrayImage &image, uint8_t *buff) {
    ImagePainter::ConvertUint8ToRgbAndUpsideDown(image.data(), buff, image.rows(), image.cols());
}

template <>
void Visualizor3D::PreprocessImage<RgbImage>(const RgbImage &image, uint8_t *buff) {
    ImagePainter::ConvertRgbToBgrAndUpsideDown(image.data(), buff, image.rows(), image.cols());
}

template void Visualizor3D::CreateTextureByImage<GrayImage>(const GrayImage &image, GLuint &texture_id);
template void Visualizor3D::CreateTextureByImage<RgbImage>(const RgbImage &image, GLuint &texture_id);
template <typename T>
void Visualizor3D::CreateTextureByImage(const T &image, GLuint &texture_id) {
    if (texture_id == 0) {
        glGenTextures(1, &texture_id);
    }
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    const int32_t size = image.rows() * image.cols();
    uint8_t *image_buff = (uint8_t *)SlamMemory::Malloc(size * 3 * sizeof(uint8_t));
    Visualizor3D::PreprocessImage<T>(image, image_buff);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols(), image.rows(), 0, GL_BGR, GL_UNSIGNED_BYTE, image_buff);

    SlamMemory::Free(image_buff);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void Visualizor3D::ShowTextureInCurrentWindow(GLuint texture_id) {
    int32_t width = 0;
    int32_t height = 0;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.f, 1.f, 0.f, 1.f, 0.f, 1.f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    glBegin(GL_QUADS);

    glTexCoord2f(0.f, 0.f);
    glVertex2f(0.f, 0.f);

    glTexCoord2f(1.f, 0.f);
    glVertex2f(1.f, 0.f);

    glTexCoord2f(1.f, 1.f);
    glVertex2f(1.f, 1.f);

    glTexCoord2f(0.f, 1.f);
    glVertex2f(0.f, 1.f);

    glEnd();
}

void Visualizor3D::WindowList() {
    ReportInfo("[Visualizor3D] All stored window.");
    for (auto &item: Visualizor3D::windows_) {
        ReportInfo(">> window title " << item.first);
        ReportInfo("   window ptr " << item.second.glfw_window);
        ReportInfo("   window texture " << item.second.texture_id);
        ReportInfo("   should close " << glfwWindowShouldClose(item.second.glfw_window));
    }
}

}  // namespace SLAM_VISUALIZOR
