#include "visualizor_3d.h"

#include "image_painter.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "cstddef"
#include "initializer_list"
#include "limits"
#include "string"
#include "type_traits"
#include "vector"

using namespace image_painter;

namespace slam_visualizor {

namespace {

    constexpr float kMinValidViewDepth = 0.1f;
    constexpr float kEllipseSigmaScale = 3.0f;
    constexpr float kDefaultFarDepth = 1000.0f;
    // Reject corrupted/transient estimator output before it can make the adaptive
    // far plane infinite. This is deliberately far beyond normal SLAM map scales.
    constexpr float kMaxRenderableDistance = 1.0e6f;
    constexpr int32_t kFontSize = 16;

    struct GpuVertex {
        float position[3];
        float color[3];
        float radius;
        float alpha;
    };
    static_assert(std::is_standard_layout<GpuVertex>::value, "GpuVertex must have a stable OpenGL layout");
    static_assert(sizeof(GpuVertex) == 8 * sizeof(float), "GpuVertex must contain exactly eight packed floats");
    static_assert(offsetof(GpuVertex, color) == 3 * sizeof(float), "GpuVertex color offset mismatch");
    static_assert(offsetof(GpuVertex, radius) == 6 * sizeof(float), "GpuVertex radius offset mismatch");
    static_assert(offsetof(GpuVertex, alpha) == 7 * sizeof(float), "GpuVertex alpha offset mismatch");

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
                                     "    gl_PointSize = max(a_radius * 2.0, 2.0);\n"
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
                                       "    frag_color = vec4(v_color * v_alpha, v_alpha);\n"
                                       "}\n";

    const char *kOitCompositeVertexShader = "#version 150\n"
                                            "out vec2 v_uv;\n"
                                            "void main() {\n"
                                            "    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
                                            "    v_uv = p;\n"
                                            "    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);\n"
                                            "}\n";

    const char *kOitCompositeFragmentShader = "#version 150\n"
                                              "in vec2 v_uv;\n"
                                              "uniform sampler2D u_accum;\n"
                                              "out vec4 frag_color;\n"
                                              "void main() {\n"
                                              "    vec4 accum = texture(u_accum, v_uv);\n"
                                              "    vec3 color = accum.rgb / max(accum.a, 1e-5);\n"
                                              "    float alpha = 1.0 - exp(-accum.a);\n"
                                              "    frag_color = vec4(color, alpha);\n"
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

    GLuint CreateProgram(const char *vertex_src, const char *fragment_src, const std::initializer_list<const char *> attributes = {}) {
        const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_src);
        const GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_src);
        if (vertex_shader == 0 || fragment_shader == 0) {
            glDeleteShader(vertex_shader);
            glDeleteShader(fragment_shader);
            return 0;
        }

        const GLuint program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);
        GLuint location = 0;
        for (const char *attribute: attributes) {
            glBindAttribLocation(program, location++, attribute);
        }
        glLinkProgram(program);

        GLint linked = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        if (linked == GL_FALSE) {
            char log[1024];
            GLsizei length = 0;
            glGetProgramInfoLog(program, sizeof(log), &length, log);
            ReportError("[RefreshByGpu] Link program failed. " << log);
            glDeleteProgram(program);
            return 0;
        }

        return program;
    }

    void ConfigureTexture(const GLint min_filter, const GLint mag_filter) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    void DestroyRenderTargets(VisualizorWindow3D &window) {
        glDeleteFramebuffers(1, &window.fbo);
        glDeleteFramebuffers(1, &window.oit_fbo);
        glDeleteTextures(1, &window.color_texture);
        glDeleteTextures(1, &window.oit_accum_texture);
        glDeleteRenderbuffers(1, &window.depth_rbo);
        window.fbo = window.oit_fbo = 0;
        window.color_texture = window.oit_accum_texture = 0;
        window.depth_rbo = 0;
        window.fbo_width = window.fbo_height = 0;
    }

    bool EnsureSceneResources(VisualizorWindow3D &window) {
        if (window.scene_program == 0) {
            window.scene_program = CreateProgram(kSceneVertexShader, kSceneFragmentShader, {"a_pos", "a_color", "a_radius", "a_alpha"});
            if (window.scene_program == 0) {
                return false;
            }
            glGenBuffers(1, &window.scene_vbo);
        }
        if (window.oit_composite_program == 0) {
            window.oit_composite_program = CreateProgram(kOitCompositeVertexShader, kOitCompositeFragmentShader);
            if (window.oit_composite_program == 0) {
                return false;
            }
            glUseProgram(window.oit_composite_program);
            glUniform1i(glGetUniformLocation(window.oit_composite_program, "u_accum"), 0);
            glUseProgram(0);
        }

        // Create the VAO and bind it to this context's private streaming VBO.
        if (window.scene_vao == 0) {
            glGenVertexArrays(1, &window.scene_vao);
            glBindVertexArray(window.scene_vao);
            glBindBuffer(GL_ARRAY_BUFFER, window.scene_vbo);
            const GLsizei stride = static_cast<GLsizei>(sizeof(GpuVertex));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, position)));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, color)));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, radius)));
            glEnableVertexAttribArray(3);
            glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, alpha)));
            glBindVertexArray(0);
        }
        return true;
    }

    bool EnsureFbo(VisualizorWindow3D &window, const int32_t width, const int32_t height) {
        if (window.fbo != 0 && window.oit_fbo != 0 && window.fbo_width == width && window.fbo_height == height) {
            return true;
        }

        DestroyRenderTargets(window);

        // Create color texture.
        glGenTextures(1, &window.color_texture);
        glBindTexture(GL_TEXTURE_2D, window.color_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        ConfigureTexture(GL_LINEAR, GL_LINEAR);

        glGenTextures(1, &window.oit_accum_texture);
        glBindTexture(GL_TEXTURE_2D, window.oit_accum_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        ConfigureTexture(GL_NEAREST, GL_NEAREST);

        // Create depth render buffer.
        glGenRenderbuffers(1, &window.depth_rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, window.depth_rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);

        // Attach color texture and depth render buffer to framebuffer.
        glGenFramebuffers(1, &window.fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, window.fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, window.color_texture, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, window.depth_rbo);
        const GLenum color_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glGenFramebuffers(1, &window.oit_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, window.oit_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, window.oit_accum_texture, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, window.depth_rbo);
        const GLenum oit_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (color_status != GL_FRAMEBUFFER_COMPLETE || oit_status != GL_FRAMEBUFFER_COMPLETE) {
            ReportError("[RefreshByGpu] Framebuffer is incomplete, color = " << color_status << ", oit = " << oit_status);
            DestroyRenderTargets(window);
            return false;
        }

        window.fbo_width = width;
        window.fbo_height = height;
        return true;
    }

    bool EnsureTextResources(VisualizorWindow3D &window) {
        if (window.text_program == 0) {
            window.text_program = CreateProgram(kTextVertexShader, kTextFragmentShader, {"a_pos", "a_uv"});
            if (window.text_program == 0) {
                return false;
            }
            glGenBuffers(1, &window.text_vbo);

            glGenTextures(1, &window.text_texture);
            glBindTexture(GL_TEXTURE_2D, window.text_texture);
            ConfigureTexture(GL_LINEAR, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        // Create a per-window text vao in this window's own context.
        if (window.text_vao == 0) {
            glGenVertexArrays(1, &window.text_vao);
            glBindVertexArray(window.text_vao);
            glBindBuffer(GL_ARRAY_BUFFER, window.text_vbo);
            const GLsizei stride = 4 * static_cast<GLsizei>(sizeof(float));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(0));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void *>(2 * sizeof(float)));
            glBindVertexArray(0);
        }
        return true;
    }

    bool EnsureGpuResources(VisualizorWindow3D &window, const int32_t width, const int32_t height) {
        return EnsureSceneResources(window) && EnsureTextResources(window) && EnsureFbo(window, width, height);
    }

    struct SceneVertexData {
        std::vector<GpuVertex> opaque_points;
        std::vector<GpuVertex> opaque_lines;
        std::vector<GpuVertex> translucent_points;
        std::vector<GpuVertex> translucent_lines;
        float max_squared_distance = 0.0f;
    };

    void UploadBucket(const std::vector<GpuVertex> &bucket, size_t &vertex_offset) {
        if (!bucket.empty()) {
            glBufferSubData(GL_ARRAY_BUFFER, vertex_offset * sizeof(GpuVertex), bucket.size() * sizeof(GpuVertex), bucket.data());
        }
        vertex_offset += bucket.size();
    }

    void DrawPointAndLineRanges(const GLint is_point_uniform, const int32_t point_offset, const int32_t point_count, const int32_t line_offset,
                                const int32_t line_count) {
        if (point_count > 0) {
            glUniform1i(is_point_uniform, 1);
            glDrawArrays(GL_POINTS, point_offset, point_count);
        }
        if (line_count > 0) {
            glUniform1i(is_point_uniform, 0);
            glDrawArrays(GL_LINES, line_offset, line_count);
        }
    }

    std::vector<GpuVertex> &VertexBucket(SceneVertexData &scene, const bool is_point, const float alpha) {
        if (alpha >= 1.0f) {
            return is_point ? scene.opaque_points : scene.opaque_lines;
        }
        return is_point ? scene.translucent_points : scene.translucent_lines;
    }

    bool IsRenderablePosition(const Vec3 &p_w) {
        if (!p_w.allFinite()) {
            return false;
        }
        const float squared_distance = (p_w - Visualizor3D::camera_view().p_wc).squaredNorm();
        return std::isfinite(squared_distance) && squared_distance <= kMaxRenderableDistance * kMaxRenderableDistance;
    }

    bool IsRenderableStyle(const float alpha) { return std::isfinite(alpha) && alpha > 0.0f; }

    void PushVertex(SceneVertexData &scene, std::vector<GpuVertex> &data, const Vec3 &p_w, const RgbPixel &color, const float radius, const float alpha) {
        data.emplace_back(GpuVertex {
            .position = {p_w.x(), p_w.y(), p_w.z()},
            .color = {static_cast<float>(color.r) / 255.0f, static_cast<float>(color.g) / 255.0f, static_cast<float>(color.b) / 255.0f},
            .radius = radius,
            .alpha = alpha,
        });

        scene.max_squared_distance = std::max(scene.max_squared_distance, (p_w - Visualizor3D::camera_view().p_wc).squaredNorm());
    }

    void AddPoint(SceneVertexData &scene, const Vec3 &p_w, const RgbPixel &color, const float radius, const float alpha) {
        if (!IsRenderablePosition(p_w) || !IsRenderableStyle(alpha) || !std::isfinite(radius) || radius <= 0.0f) {
            return;
        }
        PushVertex(scene, VertexBucket(scene, true, alpha), p_w, color, radius, alpha);
    }

    void AddLine(SceneVertexData &scene, const Vec3 &p_w_a, const Vec3 &p_w_b, const RgbPixel &color, const float alpha) {
        // Validate the complete primitive before pushing either endpoint. Skipping only
        // one endpoint would shift every later GL_LINES pair and create long stray lines.
        if (!IsRenderablePosition(p_w_a) || !IsRenderablePosition(p_w_b) || !IsRenderableStyle(alpha)) {
            return;
        }
        auto &bucket = VertexBucket(scene, false, alpha);
        PushVertex(scene, bucket, p_w_a, color, 0.0f, alpha);
        PushVertex(scene, bucket, p_w_b, color, 0.0f, alpha);
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
        if (!IsRenderablePosition(line.p_w_i) || !IsRenderablePosition(line.p_w_j) || !IsRenderableStyle(line.alpha)) {
            return;
        }
        Vec3 p_c_a = cam.q_wc.inverse() * (line.p_w_i - cam.p_wc);
        Vec3 p_c_b = cam.q_wc.inverse() * (line.p_w_j - cam.p_wc);
        if (!p_c_a.allFinite() || !p_c_b.allFinite()) {
            return;
        }
        if (!ClipSegmentAtNearPlane(p_c_a, p_c_b)) {
            return;
        }

        // Estimate how many dots should be sampled by the projected pixel length.
        const Vec2 pixel_uv_a = Vec2(p_c_a.x() / p_c_a.z() * cam.fx + cam.cx, p_c_a.y() / p_c_a.z() * cam.fy + cam.cy);
        const Vec2 pixel_uv_b = Vec2(p_c_b.x() / p_c_b.z() * cam.fx + cam.cx, p_c_b.y() / p_c_b.z() * cam.fy + cam.cy);
        const float pixel_len = (pixel_uv_b - pixel_uv_a).norm();
        if (!std::isfinite(pixel_len)) {
            return;
        }
        const int32_t sample_cnt = std::max(2, std::min(100000, static_cast<int32_t>(pixel_len / std::max(1, line.dot_step))));

        const Vec3 p_w_a = cam.q_wc * p_c_a + cam.p_wc;
        const Vec3 p_w_b = cam.q_wc * p_c_b + cam.p_wc;
        for (int32_t i = 0; i < sample_cnt; ++i) {
            const float screen_t = static_cast<float>(i) / static_cast<float>(sample_cnt - 1);
            // Uniform interpolation in world/camera space is not uniform after a
            // perspective divide. For segments with a large depth difference it packs
            // many point sprites into a few pixels, making a dashed line look like a
            // solid ray. Apply perspective-correct interpolation so consecutive dots
            // remain approximately dot_step pixels apart on screen.
            const float inv_depth = (1.0f - screen_t) / p_c_a.z() + screen_t / p_c_b.z();
            const float t = (screen_t / p_c_b.z()) / inv_depth;
            const Vec3 p_w = (1.0f - t) * p_w_a + t * p_w_b;
            AddPoint(scene, p_w, line.color, 0.5f, line.alpha);
        }
    }

    void AddEllipseBoundary(SceneVertexData &scene, const EllipseType &ellipse, const CameraView &cam) {
        // Transform gaussian ellipse into camera frame.
        const Vec3 p_c = cam.q_wc.inverse() * (ellipse.p_w - cam.p_wc);
        const Mat3 cov_c = cam.q_wc.inverse().toRotationMatrix() * ellipse.cov * cam.q_wc.toRotationMatrix();
        RETURN_IF(!p_c.allFinite() || !cov_c.allFinite() || !IsRenderableStyle(ellipse.alpha) || p_c.z() < kMinValidViewDepth);

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

        for (int32_t i = 0; i < sample_cnt; ++i) {
            AddLine(scene, world_vertices[i], world_vertices[(i + 1) % sample_cnt], ellipse.color, ellipse.alpha);
        }
    }

    void AddPose(SceneVertexData &scene, const PoseType &pose) {
        AddPoint(scene, pose.p_wb, RgbColor::kWhite, 2.0f, pose.alpha);
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(pose.scale, 0, 0), RgbColor::kRed, pose.alpha);
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(0, pose.scale, 0), RgbColor::kGreen, pose.alpha);
        AddLine(scene, pose.p_wb, pose.p_wb + pose.q_wb * Vec3(0, 0, pose.scale), RgbColor::kBlue, pose.alpha);
    }

    void AddCameraPose(SceneVertexData &scene, const CameraPoseType &camera_pose) {
        AddPoint(scene, camera_pose.p_wc, RgbColor::kWhite, 2.0f, camera_pose.alpha);
        const float length_x = camera_pose.scale;
        const float length_y = camera_pose.scale * 0.7f;
        const float length_z = length_y;
        const Vec3 p_fff = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, length_y, length_z);
        const Vec3 p_fbf = camera_pose.p_wc + camera_pose.q_wc * Vec3(-length_x, length_y, length_z);
        const Vec3 p_bbf = camera_pose.p_wc + camera_pose.q_wc * Vec3(length_x, -length_y, length_z);
        const Vec3 p_bbb = camera_pose.p_wc + camera_pose.q_wc * Vec3(-length_x, -length_y, length_z);

        AddLine(scene, camera_pose.p_wc, p_fff, RgbColor::kWhite, camera_pose.alpha);
        AddLine(scene, camera_pose.p_wc, p_fbf, RgbColor::kWhite, camera_pose.alpha);
        AddLine(scene, camera_pose.p_wc, p_bbf, RgbColor::kWhite, camera_pose.alpha);
        AddLine(scene, camera_pose.p_wc, p_bbb, RgbColor::kWhite, camera_pose.alpha);
        AddLine(scene, p_fff, p_fbf, RgbColor::kWhite, camera_pose.alpha);
        AddLine(scene, p_fbf, p_bbb, RgbColor::kLightGreen, camera_pose.alpha);
        AddLine(scene, p_bbb, p_bbf, RgbColor::kOrangeRed, camera_pose.alpha);
        AddLine(scene, p_bbf, p_fff, RgbColor::kWhite, camera_pose.alpha);
    }

    void RenderSceneToFbo(VisualizorWindow3D &window, const int32_t width, const int32_t height) {
        const CameraView &cam = Visualizor3D::camera_view();

        SceneVertexData scene;
        const size_t point_hint =
            Visualizor3D::points().size() + Visualizor3D::dashed_lines().size() * 16 + Visualizor3D::poses().size() + Visualizor3D::camera_poses().size();
        const size_t line_hint = Visualizor3D::lines().size() * 2 + Visualizor3D::poses().size() * 6 + Visualizor3D::camera_poses().size() * 16 +
                                 Visualizor3D::ellipses().size() * 64;
        scene.opaque_points.reserve(point_hint);
        scene.translucent_points.reserve(point_hint);
        scene.opaque_lines.reserve(line_hint);
        scene.translucent_lines.reserve(line_hint);
        for (const auto &point: Visualizor3D::points()) {
            AddPoint(scene, point.p_w, point.color, static_cast<float>(point.radius), point.alpha);
        }
        for (const auto &line: Visualizor3D::dashed_lines()) {
            AddDashedLine(scene, line, cam);
        }
        for (const auto &pose: Visualizor3D::poses()) {
            AddPose(scene, pose);
        }
        for (const auto &camera_pose: Visualizor3D::camera_poses()) {
            AddCameraPose(scene, camera_pose);
        }
        for (const auto &line: Visualizor3D::lines()) {
            AddLine(scene, line.p_w_i, line.p_w_j, line.color, line.alpha);
        }
        for (const auto &ellipse: Visualizor3D::ellipses()) {
            AddEllipseBoundary(scene, ellipse, cam);
        }

        glBindVertexArray(window.scene_vao);
        glBindBuffer(GL_ARRAY_BUFFER, window.scene_vbo);
        const size_t opaque_point_count = scene.opaque_points.size();
        const size_t opaque_line_count = scene.opaque_lines.size();
        const size_t translucent_point_count = scene.translucent_points.size();
        const size_t translucent_line_count = scene.translucent_lines.size();
        const size_t total_vertex_count = opaque_point_count + opaque_line_count + translucent_point_count + translucent_line_count;
        if ((opaque_line_count & 1U) != 0U || (translucent_line_count & 1U) != 0U ||
            total_vertex_count > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
            ReportError("[RefreshByGpu] Invalid point/line vertex ranges: opaque points=" << opaque_point_count << ", opaque lines=" << opaque_line_count
                                                                                          << ", translucent points=" << translucent_point_count
                                                                                          << ", translucent lines=" << translucent_line_count);
            return;
        }
        if (total_vertex_count > 0) {
            // Orphan this window's private streaming VBO before uploading the four contiguous ranges.
            glBufferData(GL_ARRAY_BUFFER, total_vertex_count * sizeof(GpuVertex), nullptr, GL_STREAM_DRAW);
            size_t vertex_offset = 0;
            UploadBucket(scene.opaque_points, vertex_offset);
            UploadBucket(scene.opaque_lines, vertex_offset);
            UploadBucket(scene.translucent_points, vertex_offset);
            UploadBucket(scene.translucent_lines, vertex_offset);
        }

        // Draw opaque geometry first, then accumulate transparent geometry without sorting.
        glBindFramebuffer(GL_FRAMEBUFFER, window.fbo);
        glViewport(0, 0, width, height);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(window.scene_program);
        glUniformMatrix3fv(glGetUniformLocation(window.scene_program, "u_rot_cw"), 1, GL_FALSE, cam.q_wc.inverse().toRotationMatrix().data());
        glUniform3f(glGetUniformLocation(window.scene_program, "u_p_wc"), cam.p_wc.x(), cam.p_wc.y(), cam.p_wc.z());
        glUniform4f(glGetUniformLocation(window.scene_program, "u_cam"), cam.fx, cam.fy, cam.cx, cam.cy);
        glUniform2f(glGetUniformLocation(window.scene_program, "u_view"), static_cast<float>(width), static_cast<float>(height));
        const float far_depth = std::max(kDefaultFarDepth, std::sqrt(scene.max_squared_distance) * 2.0f);
        glUniform2f(glGetUniformLocation(window.scene_program, "u_depth"), kMinValidViewDepth, far_depth);
        glEnable(GL_PROGRAM_POINT_SIZE);
        // Unlike Binary_Data_Viewer's core profile, Visualizor3D keeps a compatibility
        // context for its fixed-function presentation pass. Point sprites must be
        // enabled in that profile before gl_PointCoord is valid in the fragment shader;
        // otherwise the circular discard can reject every point fragment.
        glEnable(GL_POINT_SPRITE);

        const GLint u_is_point_loc = glGetUniformLocation(window.scene_program, "u_is_point");
        const int32_t opaque_point_draw_count = static_cast<int32_t>(opaque_point_count);
        const int32_t opaque_line_draw_count = static_cast<int32_t>(opaque_line_count);
        const int32_t translucent_point_draw_count = static_cast<int32_t>(translucent_point_count);
        const int32_t translucent_line_draw_count = static_cast<int32_t>(translucent_line_count);
        DrawPointAndLineRanges(u_is_point_loc, 0, opaque_point_draw_count, opaque_point_draw_count, opaque_line_draw_count);

        if (translucent_point_draw_count > 0 || translucent_line_draw_count > 0) {
            const int32_t translucent_point_offset = opaque_point_draw_count + opaque_line_draw_count;
            const int32_t translucent_line_offset = translucent_point_offset + translucent_point_draw_count;
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBindFramebuffer(GL_FRAMEBUFFER, window.oit_fbo);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            glBlendFunc(GL_ONE, GL_ONE);
            DrawPointAndLineRanges(u_is_point_loc, translucent_point_offset, translucent_point_draw_count, translucent_line_offset,
                                   translucent_line_draw_count);

            glBindFramebuffer(GL_FRAMEBUFFER, window.fbo);
            glDisable(GL_DEPTH_TEST);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glUseProgram(window.oit_composite_program);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, window.oit_accum_texture);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            glDepthMask(GL_TRUE);
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
        glBindTexture(GL_TEXTURE_2D, window.text_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, mask_width, mask_height, 0, GL_RED, GL_UNSIGNED_BYTE, mask_buf);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glUseProgram(window.text_program);
        glUniform1i(glGetUniformLocation(window.text_program, "u_text"), 0);
        glUniform3f(glGetUniformLocation(window.text_program, "u_text_color"), 1.0f, 1.0f, 1.0f);

        glBindVertexArray(window.text_vao);
        glBindBuffer(GL_ARRAY_BUFFER, window.text_vbo);
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
        glDisable(GL_PROGRAM_POINT_SIZE);
        glDisable(GL_POINT_SPRITE);
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

    const bool is_new_window = windows_.find(window_title) == windows_.end();
    if (is_new_window) {
        glfwSetErrorCallback(Visualizor3D::ErrorCallback);
        if (!glfwInit()) {
            ReportError("[Visualizor3D] RefreshByGpu() glfw initialize failed.");
            return;
        }
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        // Match Binary_Data_Viewer's GL/GLSL generation. A compatibility profile is
        // required here because ShowTextureInCurrentWindow still uses a fixed-function
        // fullscreen quad after the programmable scene pass.
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    }

    VisualizorWindow3D *window = GetWindowPointer(window_title, image_cols, image_rows);
    if (window == nullptr) {
        return;
    }
    glfwMakeContextCurrent(window->glfw_window);
    if (is_new_window) {
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            ReportError("[Visualizor3D] RefreshByGpu() failed to load OpenGL functions.");
            return;
        }
        glfwSwapInterval(0);
        glfwShowWindow(window->glfw_window);

        glfwSetKeyCallback(window->glfw_window, Visualizor3D::KeyboardCallback);
        glfwSetScrollCallback(window->glfw_window, Visualizor3D::ScrollCallback);
        glfwSetMouseButtonCallback(window->glfw_window, Visualizor3D::MouseButtonCallback);
        glfwSetCursorPosCallback(window->glfw_window, Visualizor3D::CursorPosCallback);
    } else {
        glfwSetWindowShouldClose(window->glfw_window, GLFW_FALSE);
    }

    if (!EnsureGpuResources(*window, image_cols, image_rows)) {
        return;
    }

    // Render the whole scene into this window's own offscreen framebuffer.
    RenderSceneToFbo(*window, image_cols, image_rows);

    // Display the color texture of this window's framebuffer.
    window->texture_id = window->color_texture;

    Visualizor3D::WaitKey(delay_ms);
}

}  // namespace slam_visualizor
