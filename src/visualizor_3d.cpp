#include "visualizor_3d.h"
#include "slam_memory.h"

namespace SLAM_VISUALIZOR {

void Visualizor3D::Clear() {
    points_.clear();
}

void Visualizor3D::Show(const std::string &window_title) {
    const int32_t image_width = static_cast<int32_t>(camera_view_.cx) * 2;
    const int32_t image_height = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t buf_size = image_width * image_height * 3;
    uint8_t *buf = (uint8_t *)SlamMemory::Malloc(buf_size * sizeof(uint8_t));
    RgbImage image_matrix(buf, image_height, image_width, true);
}

}
