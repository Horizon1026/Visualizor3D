#include "image_painter.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "visualizor_3d.h"

#include "algorithm"
#include "cmath"
#include "string"
#include "vector"

using namespace image_painter;

namespace slam_visualizor {

namespace {

    constexpr float kMinValidViewDepth = 0.1f;
    constexpr float kEllipseSigmaScale = 3.0f;
    constexpr float kDefaultFarDepth = 1000.0f;
    constexpr int32_t kFontSize = 16;

    /* All gpu resource handles shared by all windows. All windows share a single OpenGL
     * context, so programs, buffers and textures created here are visible everywhere.
     * The vao and the offscreen framebuffer (with its color/depth attachments) are kept
     * per window in VisualizorWindow3D, because vertex array objects and framebuffer
     * objects are NOT shared between contexts. */
    GLuint g_scene_program = 0;
    GLuint g_scene_vbo = 0;

    /* Text overlay gpu resources. */
    GLuint g_text_program = 0;
    GLuint g_text_vbo = 0;
    GLuint g_text_texture = 0;

    const char *kSceneVertexShader = "#version 150\n"
                                     "in vec3 a_pos;\n"
                                     "in vec3 a_color;\n"
                                     "in float a_radius;\n"
                                     "in float a_alpha;\n"
                                     "uniform mat3 u_rot_cw;\n"
                                     "uniform vec3 u_p_wc;\n"
                                     "uniform vec4 u_cam;\n"
                                     "uniform vec2 u_view;\n"
                                     "uniform vec2 u_depth;\n"
                                     "uniform float u_point_size;\n"
                                     "out vec3 v_color;\n"
                                     "out float v_alpha;\n"
                                     "void main() {\n"
                                     "    vec3 p_c = u_rot_cw * (a_pos - u_p_wc);\n"
                                     "    float z = p_c.z;\n"
                                     "    float depth_a = (u_depth.y + u_depth.x) / (u_depth.y - u_depth.x);\n"
                                     "    float depth_b = 2.0 * u_depth.x * u_depth.y / (u_depth.y - u_depth.x);\n"
                                     "    gl_Position.x = p_c.x * (2.0 * u_cam.x / u_view.x) + z * (2.0 * u_cam.z / u_view.x - 1.0);\n"
                                     "    gl_Position.y = -p_c.y * (2.0 * u_cam.y / u_view.y) + z * (1.0 - 2.0 * u_cam.w / u_view.y);\n"
                                     "    gl_Position.z = z * depth_a - depth_b;\n"
                                     "    gl_Position.w = z;\n"
                                     // A 1px point has no fragment whose gl_PointCoord is (0.5,0.5), so the round
                                     // point discard in the fragment shader would kill it. Clamp to a minimum of 2px.
                                     "    gl_PointSize = max(a_radius * u_point_size, 2.0);\n"
                                     "    v_color = a_color;\n"
                                     "    v_alpha = a_alpha;\n"
                                     "}\n";

    const char *kSceneFragmentShader = "#version 150\n"
                                       "in vec3 v_color;\n"
                                       "in float v_alpha;\n"
                                       "uniform int u_is_point;\n"
                                       "out vec4 frag_color;\n"
                                       "void main() {\n"
                                       "    if (u_is_point == 1) {\n"
                                       "        vec2 d = gl_PointCoord - vec2(0.5);\n"
                                       "        if (dot(d, d) > 0.25) discard;\n"
                                       "    }\n"
                                       "    frag_color = vec4(v_color, v_alpha);\n"
                                       "}\n";

    const char *kTextVertexShader = "#version 150\n"
                                    "in vec2 a_pos;\n"
                                    "in vec2 a_uv;\n"
                                    "out vec2 v_uv;\n"
                                    "void main() {\n"
                                    "    gl_Position = vec4(a_pos, 0.0, 1.0);\n"
                                    "    v_uv = a_uv;\n"
                                    "}\n";

    const char *kTextFragmentShader = "#version 150\n"
                                      "uniform sampler2D u_text;\n"
                                      "uniform vec3 u_text_color;\n"
                                      "in vec2 v_uv;\n"
                                      "out vec4 frag_color;\n"
                                      "void main() {\n"
                                      "    float alpha = texture(u_text, v_uv).r;\n"
                                      "    frag_color = vec4(u_text_color, alpha);\n"
                                      "}\n";

    GLuint CompileShader(GLenum type, const char *source) {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint compiled = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled == GL_FALSE) {
            char log[1024];
            GLsizei length = 0;
            glGetShaderInfoLog(shader, sizeof(log), &length, log);
            ReportError("[RefreshByGpu] Compile shader failed. " << log);
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

    GLuint CreateProgram(const char *vertex_src, const char *fragment_src, GLuint attrib0, const char *name0, GLuint attrib1, const char *name1,
                         GLuint attrib2 = 0, const char *name2 = nullptr, GLuint attrib3 = 0, const char *name3 = nullptr) {
        const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_src);
        const GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_src);
        if (vertex_shader == 0 || fragment_shader == 0) {
            return 0;
        }

        const GLuint program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);
        glBindAttribLocation(program, attrib0, name0);
        glBindAttribLocation(program, attrib1, name1);
        if (name2 != nullptr) {
            glBindAttribLocation(program, attrib2, name2);
        }
        if (name3 != nullptr) {
            glBindAttribLocation(program, attrib3, name3);
        }
        glLinkProgram(program);

        GLint linked = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (linked == GL_FALSE) {
            char log[1024];
            GLsizei length = 0;
            glGetProgramInfoLog(program, sizeof(log), &length, log);
            ReportError("[RefreshByGpu] Link program failed. " << log);
            glDeleteProgram(program);
            return 0;
        }

        glDetachShader(program, vertex_shader);
        glDetachShader(program, fragment_shader);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        return program;
    }

    bool EnsureSceneResources(VisualizorWindow3D &window) {
        if (g_scene_program == 0) {
            g_scene_program = CreateProgram(kSceneVertexShader, kSceneFragmentShader, 0, "a_pos", 1, "a_color", 2, "a_radius", 3, "a_alpha");
            if (g_scene_program == 0) {
                return false;
            }
            glGenBuffers(1, &g_scene_vbo);
        }

        // Vertex array objects are not shared between contexts, so create one vao for
        // this window in its own context, bound to the shared scene vbo.
        if (window.scene_vao == 0) {
            glGenVertexArrays(1, &window.scene_vao);
            glBindVertexArray(window.scene_vao);
            glBindBuffer(GL_ARRAY_BUFFER, g_scene_vbo);
            const GLsizei stride = 8 * static_cast<GLsizei>(sizeof(float));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(0));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(3 * sizeof(float)));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(6 * sizeof(float)));
            glEnableVertexAttribArray(3);
            glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(7 * sizeof(float)));
            glBindVertexArray(0);
        }
        return true;
    }

    bool EnsureFbo(VisualizorWindow3D &window, const int32_t width, const int32_t height) {
        if (window.fbo != 0 && window.fbo_width == width && window.fbo_height == height) {
            return true;
        }

        // Recovery old resources.
        if (window.fbo != 0) {
            glDeleteFramebuffers(1, &window.fbo);
            glDeleteTextures(1, &window.color_texture);
            glDeleteRenderbuffers(1, &window.depth_rbo);
        }
        window.fbo = 0;
        window.color_texture = 0;
        window.depth_rbo = 0;
        window.fbo_width = 0;
        window.fbo_height = 0;

        // Create color texture.
        glGenTextures(1, &window.color_texture);
        glBindTexture(GL_TEXTURE_2D, window.color_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // Create depth render buffer.
        glGenRenderbuffers(1, &window.depth_rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, window.depth_rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);

        // Attach color texture and depth render buffer to framebuffer.
        glGenFramebuffers(1, &window.fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, window.fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, window.color_texture, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, window.depth_rbo);
        const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE) {
            ReportError("[RefreshByGpu] Framebuffer is incomplete, status = " << status);
            return false;
        }

        window.fbo_width = width;
        window.fbo_height = height;
        return true;
    }

    bool EnsureTextResources(VisualizorWindow3D &window) {
        if (g_text_program == 0) {
            g_text_program = CreateProgram(kTextVertexShader, kTextFragmentShader, 0, "a_pos", 1, "a_uv");
            if (g_text_program == 0) {
                return false;
            }
            glGenBuffers(1, &g_text_vbo);

            glGenTextures(1, &g_text_texture);
            glBindTexture(GL_TEXTURE_2D, g_text_texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        // Create a per-window text vao in this window's own context.
        if (window.text_vao == 0) {
            glGenVertexArrays(1, &window.text_vao);
            glBindVertexArray(window.text_vao);
            glBindBuffer(GL_ARRAY_BUFFER, g_text_vbo);
            const GLsizei stride = 4 * static_cast<GLsizei>(sizeof(float));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(0));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(2 * sizeof(float)));
            glBindVertexArray(0);
        }
        return true;
    }

    /* One gpu draw unit: a single point, a line segment, or an ellipse boundary loop.
     * Records are sorted far-to-near before drawing, so alpha blending respects occlusion. */
    struct DrawRecord {
        int32_t primitive = GL_POINTS;  // GL_POINTS / GL_LINES.
        int32_t vertex_offset = 0;      // Vertex index into SceneVertexData::data.
        int32_t vertex_count = 0;       // 1 for a point, 2 for a line, 2*sample_cnt for an ellipse.
        float depth = 0.0f;             // View-space z of the item, used as the sort key (larger = farther).
    };

    /* Scene vertex data. Vertices are uploaded in collection order, while DrawRecords
     * keep the per-item draw ranges needed for far-to-near depth sorted alpha blending. */
    struct SceneVertexData {
        std::vector<float> data;
        std::vector<DrawRecord> records;
        float max_depth = 0.0f;  // Max distance of any vertex to camera, used to adapt far plane.
    };

    float ViewZ(const Vec3 &p_w, const CameraView &cam) {
        return (cam.q_wc.inverse() * (p_w - cam.p_wc)).z();
    }

    DrawRecord &PushRecord(SceneVertexData &scene, const int32_t primitive, const int32_t vertex_count, const float depth) {
        DrawRecord record;
        record.primitive = primitive;
        record.vertex_offset = static_cast<int32_t>(scene.data.size() / 8);
        record.vertex_count = vertex_count;
        record.depth = depth;
        scene.records.emplace_back(record);
        return scene.records.back();
    }

    void PushVertex(SceneVertexData &scene, const Vec3 &p_w, const RgbPixel &color, const float radius, const float alpha) {
        std::vector<float> &data = scene.data;
        data.emplace_back(p_w.x());
        data.emplace_back(p_w.y());
        data.emplace_back(p_w.z());
        data.emplace_back(static_cast<float>(color.r) / 255.0f);
        data.emplace_back(static_cast<float>(color.g) / 255.0f);
        data.emplace_back(static_cast<float>(color.b) / 255.0f);
        data.emplace_back(radius);
        data.emplace_back(alpha);

        scene.max_depth = std::max(scene.max_depth, (p_w - Visualizor3D::camera_view().p_wc).norm());
    }

    void AddLine(SceneVertexData &scene, const Vec3 &p_w_a, const Vec3 &p_w_b, const RgbPixel &color, const float alpha, const CameraView &cam) {
        PushVertex(scene, p_w_a, color, 0.0f, alpha);
        PushVertex(scene, p_w_b, color, 0.0f, alpha);
        PushRecord(scene, GL_LINES, 2, std::max(ViewZ(p_w_a, cam), ViewZ(p_w_b, cam)));
    }

    bool ClipSegmentAtNearPlane(Vec3 &p_c_a, Vec3 &p_c_b) {
        if (p_c_a.z() < kMinValidViewDepth && p_c_b.z() < kMinValidViewDepth) {
            return false;
        }
        if (p_c_a.z() < kMinValidViewDepth || p_c_b.z() < kMinValidViewDepth) {
            const float w = (p_c_a.z() - kMinValidViewDepth) / (p_c_a.z() - p_c_b.z());
            const Vec3 p_c_mid = w * p_c_b + (1.0f - w) * p_c_a;
            if (p_c_a.z() < kMinValidViewDepth) {
                p_c_a = p_c_mid;
            } else {
                p_c_b = p_c_mid;
            }
        }
        return true;
    }

    void AddDashedLine(SceneVertexData &scene, const DashedLineType &line, const CameraView &cam) {
        Vec3 p_c_a = cam.q_wc.inverse() * (line.p_w_i - cam.p_wc);
        Vec3 p_c_b = cam.q_wc.inverse() * (line.p_w_j - cam.p_wc);
        if (!ClipSegmentAtNearPlane(p_c_a, p_c_b)) {
            return;
        }

        // Estimate how many dots should be sampled by the projected pixel length.
        const Vec2 pixel_uv_a = Vec2(p_c_a.x() / p_c_a.z() * cam.fx + cam.cx, p_c_a.y() / p_c_a.z() * cam.fy + cam.cy);
        const Vec2 pixel_uv_b = Vec2(p_c_b.x() / p_c_b.z() * cam.fx + cam.cx, p_c_b.y() / p_c_b.z() * cam.fy + cam.cy);
        const float pixel_len = (pixel_uv_b - pixel_uv_a).norm();
        const int32_t sample_cnt = std::max(2, static_cast<int32_t>(pixel_len / std::max(1, line.dot_step)));

        const Vec3 p_w_a = cam.q_wc * p_c_a + cam.p_wc;
        const Vec3 p_w_b = cam.q_wc * p_c_b + cam.p_wc;
        for (int32_t i = 0; i < sample_cnt; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(sample_cnt - 1);
            const Vec3 p_w = (1.0f - t) * p_w_a + t * p_w_b;
            PushVertex(scene, p_w, line.color, 0.5f, line.alpha);
            PushRecord(scene, GL_POINTS, 1, ViewZ(p_w, cam));
        }
    }

    void AddEllipseBoundary(SceneVertexData &scene, const EllipseType &ellipse, const CameraView &cam) {
        // Transform gaussian ellipse into camera frame.
        const Vec3 p_c = cam.q_wc.inverse() * (ellipse.p_w - cam.p_wc);
        const Mat3 cov_c = cam.q_wc.inverse().toRotationMatrix() * ellipse.cov * cam.q_wc.toRotationMatrix();
        RETURN_IF(p_c.z() < kMinValidViewDepth);

        // Compute focus of camera and project 3d gaussian into 2d pixel gaussian.
        const float focus = 0.5f * (cam.fx + cam.fy);
        const float inv_depth = 1.0f / p_c.z();
        const float inv_depth_2 = inv_depth * inv_depth;
        Mat2x3 jacobian_2d_3d = Mat2x3::Zero();
        jacobian_2d_3d << inv_depth, 0, -p_c.x() * inv_depth_2, 0, inv_depth, -p_c.y() * inv_depth_2;
        jacobian_2d_3d = jacobian_2d_3d * focus;
        const Mat2 pixel_cov = jacobian_2d_3d * cov_c * jacobian_2d_3d.transpose();
        const Vec2 pixel_center = p_c.head<2>() * inv_depth * focus + Vec2(cam.cx, cam.cy);

        // Decompose 2d gaussian ellipse, which is the same as DrawTrustRegionOfGaussian.
        const Eigen::SelfAdjointEigenSolver<Mat2> saes(pixel_cov);
        const float a = std::sqrt(saes.eigenvalues()(1)) * 0.5f * kEllipseSigmaScale;
        const float b = std::sqrt(saes.eigenvalues()(0)) * 0.5f * kEllipseSigmaScale;
        const float cos_theta = saes.eigenvectors()(0, 0);
        const float sin_theta = saes.eigenvectors()(1, 0);
        RETURN_IF(std::isinf(a) || std::isnan(a) || std::isinf(b) || std::isnan(b));

        // Sample boundary of 2d gaussian ellipse, and back-project them into the depth plane of its center.
        const int32_t sample_cnt = std::max(16, std::min(256, static_cast<int32_t>(6.28f * std::max(a, b))));
        const float z0 = p_c.z();
        std::vector<Vec3> world_vertices;
        world_vertices.reserve(sample_cnt);
        for (int32_t i = 0; i < sample_cnt; ++i) {
            const float angle = 6.28f * static_cast<float>(i) / static_cast<float>(sample_cnt);
            const float cos_angle = std::cos(angle);
            const float sin_angle = std::sin(angle);
            const float pixel_x = pixel_center.x() + b * cos_angle * cos_theta - a * sin_angle * sin_theta;
            const float pixel_y = pixel_center.y() + b * cos_angle * sin_theta + a * sin_angle * cos_theta;
            const Vec3 p_c_vertex = Vec3((pixel_x - cam.cx) / cam.fx * z0, (pixel_y - cam.cy) / cam.fy * z0, z0);
            world_vertices.emplace_back(cam.q_wc * p_c_vertex + cam.p_wc);
        }

        // Draw the whole ellipse boundary as one closed line loop, sorted by its center depth.
        PushRecord(scene, GL_LINES, 2 * sample_cnt, p_c.z());
        for (int32_t i = 0; i < sample_cnt; ++i) {
            PushVertex(scene, world_vertices[i], ellipse.color, 0.0f, ellipse.alpha);
            PushVertex(scene, world_vertices[(i + 1) % sample_cnt], ellipse.color, 0.0f, ellipse.alpha);
        }
    }

    void AddPosePoint(SceneVertexData &scene, const PoseType &pose, const CameraView &cam) {
        PushVertex(scene, pose.p_wb, RgbColor::kWhite, 2.0f, pose.alpha);
        PushRecord(scene, GL_POINTS, 1, ViewZ(pose.p_wb, cam));
    }

    void AddPoseLines(SceneVertexData &scene, const PoseType &pose, const CameraView &cam) {
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(pose.scale, 0, 0), RgbColor::kRed, pose.alpha, cam);
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(0, pose.scale, 0), RgbColor::kGreen, pose.alpha, cam);
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(0, 0, pose.scale), RgbColor::kBlue, pose.alpha, cam);
    }

    void AddCameraPosePoint(SceneVertexData &scene, const CameraPoseType &camera_pose, const CameraView &cam) {
        PushVertex(scene, camera_pose.p_wc, RgbColor::kWhite, 2.0f, camera_pose.alpha);
        PushRecord(scene, GL_POINTS, 1, ViewZ(camera_pose.p_wc, cam));
    }

    void AddCameraPoseLines(SceneVertexData &scene, const CameraPoseType &camera_pose, const CameraView &cam) {
        const float length_x = camera_pose.scale;
        const float length_y = camera_pose.scale * 0.7f;
        const float length_z = length_y;
        const Vec3 p_fff = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, length_y, length_z);
        const Vec3 p_fbf = camera_pose.p_wc + camera_pose.q_wc * Vec3(-length_x, length_y, length_z);
        const Vec3 p_bbf = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, -length_y, length_z);
        const Vec3 p_bbb = camera_pose.p_wc + camera_pose.q_wc * Vec3(-length_x, -length_y, length_z);

        AddLine(scene, camera_pose.p_wc, p_fff, RgbColor::kWhite, camera_pose.alpha, cam);
        AddLine(scene, camera_pose.p_wc, p_fbf, RgbColor::kWhite, camera_pose.alpha, cam);
        AddLine(scene, camera_pose.p_wc, p_bbf, RgbColor::kWhite, camera_pose.alpha, cam);
        AddLine(scene, camera_pose.p_wc, p_bbb, RgbColor::kWhite, camera_pose.alpha, cam);
        AddLine(scene, p_fff, p_fbf, RgbColor::kWhite, camera_pose.alpha, cam);
        AddLine(scene, p_fbf, p_bbb, RgbColor::kLightGreen, camera_pose.alpha, cam);
        AddLine(scene, p_bbb, p_bbf, RgbColor::kOrangeRed, camera_pose.alpha, cam);
        AddLine(scene, p_bbf, p_fff, RgbColor::kWhite, camera_pose.alpha, cam);
    }

    void RenderSceneToFbo(VisualizorWindow3D &window, const int32_t width, const int32_t height) {
        const CameraView &cam = Visualizor3D::camera_view();

        // Collect every item as its own draw record. Records are sorted far-to-near
        // before drawing, so alpha blending respects occlusion between items.
        SceneVertexData scene;
        for (const auto &point: Visualizor3D::points()) {
            PushVertex(scene, point.p_w, point.color, static_cast<float>(point.radius), point.alpha);
            PushRecord(scene, GL_POINTS, 1, ViewZ(point.p_w, cam));
        }
        for (const auto &line: Visualizor3D::dashed_lines()) {
            AddDashedLine(scene, line, cam);
        }
        for (const auto &pose: Visualizor3D::poses()) {
            AddPosePoint(scene, pose, cam);
        }
        for (const auto &camera_pose: Visualizor3D::camera_poses()) {
            AddCameraPosePoint(scene, camera_pose, cam);
        }
        for (const auto &pose: Visualizor3D::poses()) {
            AddPoseLines(scene, pose, cam);
        }
        for (const auto &camera_pose: Visualizor3D::camera_poses()) {
            AddCameraPoseLines(scene, camera_pose, cam);
        }
        for (const auto &line: Visualizor3D::lines()) {
            AddLine(scene, line.p_w_i, line.p_w_j, line.color, line.alpha, cam);
        }
        for (const auto &ellipse: Visualizor3D::ellipses()) {
            AddEllipseBoundary(scene, ellipse, cam);
        }

        // Upload all vertices.
        glBindVertexArray(window.scene_vao);
        glBindBuffer(GL_ARRAY_BUFFER, g_scene_vbo);
        if (!scene.data.empty()) {
            glBufferData(GL_ARRAY_BUFFER, scene.data.size() * sizeof(float), scene.data.data(), GL_DYNAMIC_DRAW);
        }

        // Render into framebuffer with depth test. Blending stays enabled for every
        // item; with alpha == 1.0 the blend reduces to a plain overwrite, and items
        // with alpha < 1.0 blend with whatever is already behind them.
        glBindFramebuffer(GL_FRAMEBUFFER, window.fbo);
        glViewport(0, 0, width, height);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(g_scene_program);
        glUniformMatrix3fv(glGetUniformLocation(g_scene_program, "u_rot_cw"), 1, GL_FALSE, cam.q_wc.inverse().toRotationMatrix().data());
        glUniform3f(glGetUniformLocation(g_scene_program, "u_p_wc"), cam.p_wc.x(), cam.p_wc.y(), cam.p_wc.z());
        glUniform4f(glGetUniformLocation(g_scene_program, "u_cam"), cam.fx, cam.fy, cam.cx, cam.cy);
        glUniform2f(glGetUniformLocation(g_scene_program, "u_view"), static_cast<float>(width), static_cast<float>(height));
        const float far_depth = std::max(kDefaultFarDepth, scene.max_depth * 2.0f);
        glUniform2f(glGetUniformLocation(g_scene_program, "u_depth"), kMinValidViewDepth, far_depth);
        glUniform1f(glGetUniformLocation(g_scene_program, "u_point_size"), 2.0f);
        glEnable(GL_PROGRAM_POINT_SIZE);
        // The window context is a compatibility profile, where gl_PointCoord is only
        // well-defined when point sprites are enabled. Required for round points.
        glEnable(GL_POINT_SPRITE);

        // Far-to-near order lets a nearer item blend over the farther one already
        // written behind it, while the depth test still occludes items behind others.
        const GLint u_is_point_loc = glGetUniformLocation(g_scene_program, "u_is_point");
        std::sort(scene.records.begin(), scene.records.end(),
                  [](const DrawRecord &a, const DrawRecord &b) { return a.depth > b.depth; });
        for (const auto &record: scene.records) {
            glUniform1i(u_is_point_loc, record.primitive == GL_POINTS);
            glDrawArrays(record.primitive, record.vertex_offset, record.vertex_count);
        }

        // Draw text overlay on the top-left of the image, which is always in front of the scene.
        const int32_t font_size = kFontSize;
        std::vector<std::string> text_lines;
        text_lines.emplace_back(std::string("[CameraView] q_wc[wxyz][") + std::to_string(cam.q_wc.w()) + ", " + std::to_string(cam.q_wc.x()) + ", " +
                                std::to_string(cam.q_wc.y()) + ", " + std::to_string(cam.q_wc.z()) + "].");
        text_lines.emplace_back(std::string("[CameraView] p_wc[xyz][") + std::to_string(cam.p_wc.x()) + ", " + std::to_string(cam.p_wc.y()) + ", " +
                                std::to_string(cam.p_wc.z()) + "].");
        for (const auto &str: Visualizor3D::strings()) {
            text_lines.emplace_back(str);
        }

        // Rasterize text into a small gray mask with the same layout as the cpu pipeline.
        int32_t mask_width = 0;
        const int32_t mask_height = static_cast<int32_t>(text_lines.size()) * font_size + font_size;
        for (const auto &str: text_lines) {
            mask_width = std::max(mask_width, static_cast<int32_t>(str.size()) * (font_size >> 1));
        }
        mask_width += font_size;
        uint8_t *mask_buf = static_cast<uint8_t *>(SlamMemory::Malloc(mask_width * mask_height * sizeof(uint8_t)));
        GrayImage mask_image(mask_buf, mask_height, mask_width, true);
        mask_image.Clear();
        for (int32_t i = 0; i < static_cast<int32_t>(text_lines.size()); ++i) {
            ImagePainter::DrawString(mask_image, text_lines[i], font_size / 2, i * font_size, static_cast<uint8_t>(255), font_size);
        }

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, g_text_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, mask_width, mask_height, 0, GL_RED, GL_UNSIGNED_BYTE, mask_buf);

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glUseProgram(g_text_program);
        glUniform1i(glGetUniformLocation(g_text_program, "u_text"), 0);
        glUniform3f(glGetUniformLocation(g_text_program, "u_text_color"), 1.0f, 1.0f, 1.0f);

        glBindVertexArray(window.text_vao);
        glBindBuffer(GL_ARRAY_BUFFER, g_text_vbo);
        const float ndc_x0 = -1.0f;
        const float ndc_x1 = -1.0f + 2.0f * static_cast<float>(mask_width) / static_cast<float>(width);
        const float ndc_y_top = 1.0f;
        const float ndc_y_bottom = 1.0f - 2.0f * static_cast<float>(mask_height) / static_cast<float>(height);
        // Texture data row 0 is the top of the text, which should be rendered at the top of the window (v = 0).
        const float vertices[4 * 4] = {
            ndc_x0, ndc_y_bottom, 0.0f, 1.0f, ndc_x0, ndc_y_top, 0.0f, 0.0f, ndc_x1, ndc_y_bottom, 1.0f, 1.0f, ndc_x1, ndc_y_top, 1.0f, 0.0f,
        };
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        glDisable(GL_BLEND);
        // Unbind the shader program. The existing ShowTextureInCurrentWindow() display
        // path draws its full-screen quad through the fixed-function pipeline, and a
        // leftover bound program would intercept the quad and break the texture display.
        glUseProgram(0);
        glBindVertexArray(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

}  // namespace

void Visualizor3D::RefreshByGpu(const std::string &window_title, const int32_t delay_ms) {
    const int32_t image_rows = static_cast<int32_t>(camera_view_.cy) * 2;
    const int32_t image_cols = static_cast<int32_t>(camera_view_.cx) * 2;

    // Ensure window exists and the gpu context is current.
    VisualizorWindow3D *window = nullptr;
    auto item = windows_.find(window_title);
    if (item == windows_.end()) {
        glfwSetErrorCallback(Visualizor3D::ErrorCallback);
        if (!glfwInit()) {
            ReportError("[Visualizor3D] RefreshByGpu() glfw initialize failed.");
            return;
        }
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        window = GetWindowPointer(window_title, image_cols, image_rows);
        if (window == nullptr) {
            return;
        }
        glfwMakeContextCurrent(window->glfw_window);
        gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
        glfwSwapInterval(0);
        glfwShowWindow(window->glfw_window);

        glfwSetKeyCallback(window->glfw_window, Visualizor3D::KeyboardCallback);
        glfwSetScrollCallback(window->glfw_window, Visualizor3D::ScrollCallback);
        glfwSetMouseButtonCallback(window->glfw_window, Visualizor3D::MouseButtonCallback);
        glfwSetCursorPosCallback(window->glfw_window, Visualizor3D::CursorPosCallback);
    } else {
        window = GetWindowPointer(window_title, image_cols, image_rows);
        glfwMakeContextCurrent(window->glfw_window);
        glfwSetWindowShouldClose(window->glfw_window, GLFW_FALSE);
    }

    // Ensure all gpu resources.
    if (!EnsureSceneResources(*window)) {
        return;
    }
    if (!EnsureTextResources(*window)) {
        return;
    }
    if (!EnsureFbo(*window, image_cols, image_rows)) {
        return;
    }

    // Render the whole scene into this window's own offscreen framebuffer.
    RenderSceneToFbo(*window, image_cols, image_rows);

    // Display the color texture of this window's framebuffer.
    window->texture_id = window->color_texture;

    Visualizor3D::WaitKey(delay_ms);
}

}  // namespace slam_visualizor
