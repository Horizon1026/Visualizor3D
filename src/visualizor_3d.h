#ifndef _SLAM_UTILITY_3D_VISUALIZOR_H_
#define _SLAM_UTILITY_3D_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

namespace SLAM_VISUALIZOR {

struct PointType {
    Vec3 p_w = Vec3::Zero();
    RgbPixel color = RgbPixel{.r = 255, .g = 255, .b = 255};
    int32_t radius = 1;
};

/* Class Visualizor 3D Declaration. */
class Visualizor3D {

public:
    Visualizor3D() = default;
    ~Visualizor3D() = default;

    void Clear();

private:
    std::vector<PointType> points_;

};

}

#endif // end of _SLAM_UTILITY_3D_VISUALIZOR_H_
