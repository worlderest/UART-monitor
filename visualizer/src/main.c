#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "math3d.h"
#include "scene_assets.h"
#include "score_receiver.h"
#include "udp_receiver.h"
#include "udp_protocol.h"
#include "ui_theme.h"

#define SCREEN_WIDTH 1600
#define SCREEN_HEIGHT 900

#define DEFAULT_UDP_PORT 5005u
#define DEFAULT_SCORE_UDP_PORT 5006u
#define OSCILLOSCOPE_CAPACITY 2048
#define OSCILLOSCOPE_WINDOW_MS 2000ULL
#define SHOT_HISTORY_CAPACITY 256
#define SCORE_FLASH_HOLD_SECONDS 2.0
#define SCORE_FLASH_FADE_SECONDS 1.0
#define MIN_SCENE_TEXTURE_SIZE 64

#define UPPER_ARM_LEN 0.34f
#define FOREARM_LEN 0.28f
#define HAND_LEN 0.16f
#define HAND_BOARD_WIDTH (HAND_LEN * 0.60f)
#define HAND_BOARD_THICKNESS (HAND_LEN * 0.15f)
#define HAND_BOARD_LENGTH HAND_LEN
#define CAMERA_MIN_DISTANCE 0.65f
#define CAMERA_MAX_DISTANCE 6.00f
#define CAMERA_MIN_PITCH_DEG -80.0f
#define CAMERA_MAX_PITCH_DEG 80.0f
#define CAMERA_ORBIT_SENSITIVITY 0.0055f
#define CAMERA_ZOOM_STEP 0.16f

typedef struct PitchSample {
    uint64_t timestamp_ms;
    float pitch_deg;
    float pitch_dps;
} PitchSample;

typedef struct OrbitCameraState {
    Vector3 target;
    float distance;
    float yaw;
    float pitch;
    Vector3 default_target;
    float default_distance;
    float default_yaw;
    float default_pitch;
} OrbitCameraState;

typedef struct SceneCanvas {
    RenderTexture2D target;
    bool loaded;
    int width;
    int height;
} SceneCanvas;

typedef struct AppState {
    UmonUdpReceiver *udp_receiver;
    UmonScoreReceiver *score_receiver;
    unsigned short udp_port;
    unsigned short score_udp_port;
    bool has_frame;
    bool calibrated;
    bool has_score_event;
    bool model_preview_enabled;
    UmonUdpFrame latest_frame;
    UmonScoreEvent latest_score_event;
    double last_pose_packet_time;
    double last_score_packet_time;
    double score_flash_start_time;
    Quaternion reference_upper_arm;
    Quaternion reference_forearm;
    Quaternion reference_hand_palm;
    PitchSample samples[OSCILLOSCOPE_CAPACITY];
    int sample_count;
    int sample_head;
    uint64_t shot_timestamps[SHOT_HISTORY_CAPACITY];
    int shot_count;
    int shot_head;
    int session_shots;
    float session_score_sum;
    float session_best_score;
    float session_last_score;
} AppState;

typedef struct DashboardLayout {
    Rectangle header;
    Rectangle score_panel;
    Rectangle info_panel;
    Rectangle scene_panel;
    Rectangle scene_view;
    Rectangle waveform_panel;
    Rectangle waveform_graph;
} DashboardLayout;

static bool TryParsePort(const char *text, unsigned short *out_port);
static bool ParseCommandLine(int argc, char **argv, unsigned short *out_udp_port, unsigned short *out_score_udp_port);
static void PrintUsage(const char *program_name);
static bool IsValidUdpFrame(const UmonUdpFrame *frame);
static bool IsValidScoreEvent(const UmonScoreEvent *event);
static void AppendPitchSample(AppState *state, uint64_t timestamp_ms, float pitch_deg, float pitch_dps);
static void AppendShotMarker(AppState *state, uint64_t shot_timestamp_ms);
static void PollUdpFrames(AppState *state);
static void PollScoreEvents(AppState *state);
static int CollectRecentSamples(const AppState *state, PitchSample *out_samples, int capacity);
static int CollectRecentShotMarkers(const AppState *state, uint64_t *out_timestamps, int capacity, uint64_t cutoff_ms);
static bool ComputeArmRenderPose(const AppState *state, UmonArmRenderPose *out_pose);
static OrbitCameraState CreateOrbitCameraState(Vector3 position, Vector3 target);
static void ResetOrbitCamera(OrbitCameraState *state);
static void ApplyOrbitCamera(Camera3D *camera, const OrbitCameraState *state);
static void UpdateOrbitCamera(
    OrbitCameraState *state,
    Camera3D *camera,
    bool mouse_over_viewport,
    bool reset_requested
);
static bool EnsureSceneCanvas(SceneCanvas *canvas, int width, int height);
static void UnloadSceneCanvas(SceneCanvas *canvas);
static DashboardLayout ComputeDashboardLayout(int screen_width, int screen_height);
static void RenderSceneViewport(
    SceneCanvas *canvas,
    const UmonSceneAssets *assets,
    const UmonUiTheme *theme,
    Camera3D camera,
    const UmonArmRenderPose *pose,
    bool has_pose,
    bool draw_models
);
static void DrawFallbackArm(const UmonArmRenderPose *pose, float opacity);
static float ScoreFlashStrength(const AppState *state);
static void DrawHeaderBar(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    bool model_ready
);
static void DrawCompactStatBox(
    const UmonUiTheme *theme,
    Font font,
    Rectangle bounds,
    const char *label,
    const char *value,
    Color accent
);
static void DrawScorePanel(const AppState *state, const DashboardLayout *layout, const UmonUiTheme *theme, Font font);
static void DrawInfoPanel(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    bool model_ready
);
static void DrawScenePanel(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    const SceneCanvas *canvas,
    bool canvas_ready,
    bool model_ready
);
static void DrawWaveformPanel(const AppState *state, const DashboardLayout *layout, const UmonUiTheme *theme, Font font);

static Rectangle Inset(Rectangle bounds, float left, float top, float right, float bottom) {
    return (Rectangle){
        bounds.x + left,
        bounds.y + top,
        bounds.width - left - right,
        bounds.height - top - bottom,
    };
}

static bool TryParsePort(const char *text, unsigned short *out_port) {
    if (text == NULL || *text == '\0') {
        return false;
    }

    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == NULL || *end != '\0') {
        return false;
    }
    if (value < 1 || value > 65535) {
        return false;
    }

    *out_port = (unsigned short)value;
    return true;
}

static bool ParseCommandLine(int argc, char **argv, unsigned short *out_udp_port, unsigned short *out_score_udp_port) {
    *out_udp_port = DEFAULT_UDP_PORT;
    *out_score_udp_port = DEFAULT_SCORE_UDP_PORT;

    for (int index = 1; index < argc; ++index) {
        const char *argument = argv[index];
        if (strcmp(argument, "--udp-port") == 0) {
            if (index + 1 >= argc || !TryParsePort(argv[index + 1], out_udp_port)) {
                return false;
            }
            index += 1;
            continue;
        }
        if (strcmp(argument, "--score-udp-port") == 0) {
            if (index + 1 >= argc || !TryParsePort(argv[index + 1], out_score_udp_port)) {
                return false;
            }
            index += 1;
            continue;
        }
        return false;
    }

    return true;
}

static void PrintUsage(const char *program_name) {
    fprintf(stderr, "Usage: %s [--udp-port <1-65535>] [--score-udp-port <1-65535>]\n", program_name);
}

static bool IsValidUdpFrame(const UmonUdpFrame *frame) {
    return memcmp(frame->magic, UMON_UDP_MAGIC, 4) == 0 && frame->version == UMON_UDP_VERSION;
}

static bool IsValidScoreEvent(const UmonScoreEvent *event) {
    return memcmp(event->magic, UMON_SCORE_MAGIC, 4) == 0 && event->version == UMON_SCORE_VERSION;
}

static void AppendPitchSample(AppState *state, uint64_t timestamp_ms, float pitch_deg, float pitch_dps) {
    state->samples[state->sample_head].timestamp_ms = timestamp_ms;
    state->samples[state->sample_head].pitch_deg = pitch_deg;
    state->samples[state->sample_head].pitch_dps = pitch_dps;
    state->sample_head = (state->sample_head + 1) % OSCILLOSCOPE_CAPACITY;
    if (state->sample_count < OSCILLOSCOPE_CAPACITY) {
        state->sample_count += 1;
    }
}

static void AppendShotMarker(AppState *state, uint64_t shot_timestamp_ms) {
    state->shot_timestamps[state->shot_head] = shot_timestamp_ms;
    state->shot_head = (state->shot_head + 1) % SHOT_HISTORY_CAPACITY;
    if (state->shot_count < SHOT_HISTORY_CAPACITY) {
        state->shot_count += 1;
    }
}

static void PollUdpFrames(AppState *state) {
    for (;;) {
        UmonUdpFrame frame;
        int result = UmonUdpReceiverPoll(state->udp_receiver, &frame);
        if (result == 0) {
            break;
        }
        if (result < 0) {
            break;
        }
        if (!IsValidUdpFrame(&frame)) {
            continue;
        }

        state->latest_frame = frame;
        state->has_frame = true;
        state->last_pose_packet_time = GetTime();
        AppendPitchSample(state, frame.timestamp_ms, frame.hand_palm_pitch_deg, frame.hand_palm_pitch_dps);
    }
}

static void PollScoreEvents(AppState *state) {
    for (;;) {
        UmonScoreEvent event;
        int result = UmonScoreReceiverPoll(state->score_receiver, &event);
        if (result == 0) {
            break;
        }
        if (result < 0) {
            break;
        }
        if (!IsValidScoreEvent(&event)) {
            continue;
        }

        state->latest_score_event = event;
        state->has_score_event = true;
        state->last_score_packet_time = GetTime();
        state->score_flash_start_time = state->last_score_packet_time;
        state->session_shots += 1;
        state->session_last_score = event.total_score;
        state->session_score_sum += event.total_score;
        if (state->session_shots == 1 || event.total_score > state->session_best_score) {
            state->session_best_score = event.total_score;
        }
        AppendShotMarker(state, event.shot_timestamp_ms);
    }
}

static int CollectRecentSamples(const AppState *state, PitchSample *out_samples, int capacity) {
    if (state->sample_count == 0 || capacity <= 0) {
        return 0;
    }

    int oldest_index = (state->sample_head - state->sample_count + OSCILLOSCOPE_CAPACITY) % OSCILLOSCOPE_CAPACITY;
    uint64_t latest_timestamp = state->samples[(state->sample_head - 1 + OSCILLOSCOPE_CAPACITY) % OSCILLOSCOPE_CAPACITY].timestamp_ms;
    uint64_t cutoff = (latest_timestamp > OSCILLOSCOPE_WINDOW_MS) ? (latest_timestamp - OSCILLOSCOPE_WINDOW_MS) : 0;

    int out_count = 0;
    for (int index = 0; index < state->sample_count; ++index) {
        int sample_index = (oldest_index + index) % OSCILLOSCOPE_CAPACITY;
        PitchSample sample = state->samples[sample_index];
        if (sample.timestamp_ms < cutoff) {
            continue;
        }
        if (out_count >= capacity) {
            break;
        }
        out_samples[out_count++] = sample;
    }

    return out_count;
}

static int CollectRecentShotMarkers(const AppState *state, uint64_t *out_timestamps, int capacity, uint64_t cutoff_ms) {
    if (state->shot_count == 0 || capacity <= 0) {
        return 0;
    }

    int oldest_index = (state->shot_head - state->shot_count + SHOT_HISTORY_CAPACITY) % SHOT_HISTORY_CAPACITY;
    int out_count = 0;
    for (int index = 0; index < state->shot_count; ++index) {
        int marker_index = (oldest_index + index) % SHOT_HISTORY_CAPACITY;
        uint64_t shot_timestamp_ms = state->shot_timestamps[marker_index];
        if (shot_timestamp_ms < cutoff_ms) {
            continue;
        }
        if (out_count >= capacity) {
            break;
        }
        out_timestamps[out_count++] = shot_timestamp_ms;
    }

    return out_count;
}

static bool ComputeArmRenderPose(const AppState *state, UmonArmRenderPose *out_pose) {
    if (!state->has_frame || out_pose == NULL) {
        return false;
    }

    Quaternion upper_arm_q = UmonQuaternionFromWxyz(state->latest_frame.upper_arm);
    Quaternion forearm_q = UmonQuaternionFromWxyz(state->latest_frame.forearm);
    Quaternion hand_palm_q = UmonQuaternionFromWxyz(state->latest_frame.hand_palm);

    if (state->calibrated) {
        upper_arm_q = UmonRelativeQuaternion(upper_arm_q, state->reference_upper_arm);
        forearm_q = UmonRelativeQuaternion(forearm_q, state->reference_forearm);
        hand_palm_q = UmonRelativeQuaternion(hand_palm_q, state->reference_hand_palm);
    }

    out_pose->shoulder_joint = (Vector3){0.0f, 0.0f, 0.0f};
    out_pose->elbow_joint = Vector3Add(
        out_pose->shoulder_joint,
        UmonRotateVector((Vector3){0.0f, -UPPER_ARM_LEN, 0.0f}, upper_arm_q)
    );
    out_pose->wrist_joint = Vector3Add(
        out_pose->elbow_joint,
        UmonRotateVector((Vector3){0.0f, -FOREARM_LEN, 0.0f}, forearm_q)
    );
    out_pose->hand_end = Vector3Add(
        out_pose->wrist_joint,
        UmonRotateVector((Vector3){0.0f, -HAND_LEN, 0.0f}, hand_palm_q)
    );
    out_pose->upper_arm_orientation = upper_arm_q;
    out_pose->forearm_orientation = forearm_q;
    out_pose->hand_palm_orientation = hand_palm_q;
    out_pose->upper_arm_stale = state->latest_frame.upper_arm_stale != 0;
    out_pose->forearm_stale = state->latest_frame.forearm_stale != 0;
    out_pose->hand_palm_stale = state->latest_frame.hand_palm_stale != 0;
    return true;
}

static OrbitCameraState CreateOrbitCameraState(Vector3 position, Vector3 target) {
    OrbitCameraState state = {0};
    Vector3 offset = Vector3Subtract(position, target);
    float distance = Vector3Length(offset);
    if (distance < 0.001f) {
        distance = 0.001f;
        offset = (Vector3){0.0f, 0.0f, distance};
    }

    state.target = target;
    state.distance = distance;
    state.pitch = asinf(Clamp(offset.y / distance, -1.0f, 1.0f));
    state.yaw = atan2f(offset.x, offset.z);
    state.default_target = state.target;
    state.default_distance = state.distance;
    state.default_yaw = state.yaw;
    state.default_pitch = state.pitch;
    return state;
}

static void ResetOrbitCamera(OrbitCameraState *state) {
    state->target = state->default_target;
    state->distance = state->default_distance;
    state->yaw = state->default_yaw;
    state->pitch = state->default_pitch;
}

static void ApplyOrbitCamera(Camera3D *camera, const OrbitCameraState *state) {
    float cos_pitch = cosf(state->pitch);
    Vector3 offset = {
        sinf(state->yaw) * cos_pitch * state->distance,
        sinf(state->pitch) * state->distance,
        cosf(state->yaw) * cos_pitch * state->distance,
    };

    camera->target = state->target;
    camera->position = Vector3Add(state->target, offset);
    camera->up = (Vector3){0.0f, 1.0f, 0.0f};
}

static void UpdateOrbitCamera(
    OrbitCameraState *state,
    Camera3D *camera,
    bool mouse_over_viewport,
    bool reset_requested
) {
    if (reset_requested) {
        ResetOrbitCamera(state);
    }

    if (mouse_over_viewport && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 mouse_delta = GetMouseDelta();
        state->yaw -= mouse_delta.x * CAMERA_ORBIT_SENSITIVITY;
        state->pitch += mouse_delta.y * CAMERA_ORBIT_SENSITIVITY;
    }

    if (mouse_over_viewport) {
        float wheel_move = GetMouseWheelMove();
        if (wheel_move != 0.0f) {
            state->distance -= wheel_move * CAMERA_ZOOM_STEP;
        }
    }

    state->pitch = Clamp(state->pitch, CAMERA_MIN_PITCH_DEG * DEG2RAD, CAMERA_MAX_PITCH_DEG * DEG2RAD);
    state->distance = Clamp(state->distance, CAMERA_MIN_DISTANCE, CAMERA_MAX_DISTANCE);
    ApplyOrbitCamera(camera, state);
}

static bool EnsureSceneCanvas(SceneCanvas *canvas, int width, int height) {
    width = width < MIN_SCENE_TEXTURE_SIZE ? MIN_SCENE_TEXTURE_SIZE : width;
    height = height < MIN_SCENE_TEXTURE_SIZE ? MIN_SCENE_TEXTURE_SIZE : height;

    if (canvas->loaded && canvas->width == width && canvas->height == height) {
        return true;
    }

    if (canvas->loaded) {
        UnloadRenderTexture(canvas->target);
        canvas->loaded = false;
    }

    canvas->target = LoadRenderTexture(width, height);
    canvas->loaded = canvas->target.id != 0;
    canvas->width = width;
    canvas->height = height;
    return canvas->loaded;
}

static void UnloadSceneCanvas(SceneCanvas *canvas) {
    if (!canvas->loaded) {
        return;
    }

    UnloadRenderTexture(canvas->target);
    canvas->loaded = false;
    canvas->width = 0;
    canvas->height = 0;
}

static DashboardLayout ComputeDashboardLayout(int screen_width, int screen_height) {
    float margin = 20.0f;
    float gap = 16.0f;
    float header_height = 72.0f;
    float bottom_height = Clamp(screen_height * 0.30f, 220.0f, 300.0f);
    float left_width = Clamp(screen_width * 0.26f, 300.0f, 360.0f);

    float content_top = margin + header_height + gap;
    float content_height = screen_height - margin - bottom_height - gap - content_top;
    float score_height = Clamp(content_height * 0.46f, 176.0f, 228.0f);
    float info_height = content_height - score_height - gap;

    DashboardLayout layout = {0};
    layout.header = (Rectangle){margin, margin, screen_width - margin * 2.0f, header_height};
    layout.score_panel = (Rectangle){margin, content_top, left_width, score_height};
    layout.info_panel = (Rectangle){margin, layout.score_panel.y + layout.score_panel.height + gap, left_width, info_height};
    layout.scene_panel = (Rectangle){
        layout.score_panel.x + left_width + gap,
        content_top,
        screen_width - margin - (layout.score_panel.x + left_width + gap),
        content_height,
    };
    layout.scene_view = Inset(layout.scene_panel, 14.0f, 44.0f, 14.0f, 16.0f);
    layout.waveform_panel = (Rectangle){margin, screen_height - margin - bottom_height, screen_width - margin * 2.0f, bottom_height};
    layout.waveform_graph = Inset(layout.waveform_panel, 18.0f, 56.0f, 18.0f, 20.0f);
    return layout;
}

static void DrawFallbackArm(const UmonArmRenderPose *pose, float opacity) {
    Color joint_color = Fade((Color){241, 245, 250, 255}, opacity);
    Color live_bone_color = Fade((Color){88, 200, 255, 255}, opacity);
    Color stale_joint_color = Fade((Color){232, 91, 70, 255}, opacity);
    Color stale_bone_color = Fade((Color){214, 100, 92, 255}, opacity);
    Color hand_color = pose->hand_palm_stale ? stale_joint_color : Fade((Color){255, 211, 107, 255}, opacity);

    DrawCylinderEx(
        pose->shoulder_joint,
        pose->elbow_joint,
        0.028f,
        0.024f,
        14,
        (pose->upper_arm_stale || pose->forearm_stale) ? stale_bone_color : live_bone_color
    );
    DrawCylinderEx(
        pose->elbow_joint,
        pose->wrist_joint,
        0.024f,
        0.020f,
        14,
        (pose->forearm_stale || pose->hand_palm_stale) ? stale_bone_color : live_bone_color
    );

    Vector3 hand_axis = Vector3Subtract(pose->hand_end, pose->wrist_joint);
    Vector3 hand_center = Vector3Add(pose->wrist_joint, Vector3Scale(hand_axis, 0.5f));
    Matrix hand_rotation = QuaternionToMatrix(pose->hand_palm_orientation);
    rlPushMatrix();
        rlTranslatef(hand_center.x, hand_center.y, hand_center.z);
        rlMultMatrixf(MatrixToFloat(hand_rotation));
        DrawCubeV(
            (Vector3){0.0f, 0.0f, 0.0f},
            (Vector3){HAND_BOARD_WIDTH, HAND_BOARD_LENGTH, HAND_BOARD_THICKNESS},
            hand_color
        );
        DrawCubeWiresV(
            (Vector3){0.0f, 0.0f, 0.0f},
            (Vector3){HAND_BOARD_WIDTH, HAND_BOARD_LENGTH, HAND_BOARD_THICKNESS},
            Fade((Color){250, 246, 220, 255}, opacity)
        );
    rlPopMatrix();

    DrawSphere(pose->shoulder_joint, 0.036f, pose->upper_arm_stale ? stale_joint_color : joint_color);
    DrawSphere(pose->elbow_joint, 0.032f, pose->forearm_stale ? stale_joint_color : joint_color);
    DrawSphere(pose->wrist_joint, 0.028f, pose->hand_palm_stale ? stale_joint_color : joint_color);
    DrawSphere(pose->hand_end, 0.022f, hand_color);
}

static void RenderSceneViewport(
    SceneCanvas *canvas,
    const UmonSceneAssets *assets,
    const UmonUiTheme *theme,
    Camera3D camera,
    const UmonArmRenderPose *pose,
    bool has_pose,
    bool draw_models
) {
    BeginTextureMode(canvas->target);
    ClearBackground((Color){8, 12, 20, 255});
    DrawCircleGradient(
        (Vector2){canvas->width * 0.76f, canvas->height * 0.18f},
        canvas->height * 0.34f,
        Fade(theme->accent_cyan, 0.10f),
        BLANK
    );

    BeginMode3D(camera);
    DrawPlane((Vector3){0.0f, -0.92f, 0.0f}, (Vector2){3.2f, 3.2f}, Fade(theme->accent_slate, 0.42f));
    DrawCubeWiresV(
        (Vector3){0.0f, -0.42f, 0.0f},
        (Vector3){1.10f, 0.90f, 0.72f},
        Fade(theme->panel_border, 0.14f)
    );

    if (has_pose) {
        if (draw_models) {
            DrawFallbackArm(pose, 0.18f);
            UmonSceneAssetsDrawArm(assets, pose);
        } else {
            DrawFallbackArm(pose, 1.0f);
        }
    }

    EndMode3D();
    EndTextureMode();
}

static float ScoreFlashStrength(const AppState *state) {
    if (!state->has_score_event) {
        return 0.0f;
    }

    double elapsed = GetTime() - state->score_flash_start_time;
    double total = SCORE_FLASH_HOLD_SECONDS + SCORE_FLASH_FADE_SECONDS;
    if (elapsed >= total) {
        return 0.0f;
    }
    if (elapsed <= SCORE_FLASH_HOLD_SECONDS) {
        return 1.0f;
    }
    return 1.0f - (float)((elapsed - SCORE_FLASH_HOLD_SECONDS) / SCORE_FLASH_FADE_SECONDS);
}

static void DrawHeaderBar(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    bool model_ready
) {
    bool pose_live = state->has_frame && (GetTime() - state->last_pose_packet_time) < 0.70;
    bool score_live = state->has_score_event && (GetTime() - state->last_score_packet_time) < 5.0;

    UmonUiDrawPanelAlt(theme, layout->header, font, "STATUS");
    UmonUiDrawText(
        font,
        "UPPER MONITOR",
        (Vector2){layout->header.x + 18.0f, layout->header.y + 18.0f},
        22.0f,
        theme->text_primary
    );
    UmonUiDrawText(
        font,
        "F11 fullscreen  |  LMB orbit  |  Wheel zoom  |  V reset  |  M model preview",
        (Vector2){layout->header.x + 18.0f, layout->header.y + 42.0f},
        12.0f,
        theme->text_muted
    );

    float chip_width = 102.0f;
    float chip_gap = 10.0f;
    Rectangle chip = {
        layout->header.x + layout->header.width - (chip_width * 4.0f + chip_gap * 3.0f) - 18.0f,
        layout->header.y + 18.0f,
        chip_width,
        24.0f,
    };
    UmonUiDrawChip(theme, chip, font, pose_live ? "POSE LIVE" : "POSE WAIT", pose_live ? theme->accent_green : theme->accent_red);
    chip.x += chip_width + chip_gap;
    UmonUiDrawChip(theme, chip, font, score_live ? "SCORE LIVE" : "SCORE WAIT", score_live ? theme->accent_amber : theme->accent_slate);
    chip.x += chip_width + chip_gap;
    UmonUiDrawChip(
        theme,
        chip,
        font,
        model_ready ? (state->model_preview_enabled ? "MODEL ON" : "MODEL READY") : "MODEL OFF",
        model_ready ? (state->model_preview_enabled ? theme->accent_green : theme->accent_cyan) : theme->accent_red
    );
    chip.x += chip_width + chip_gap;
    UmonUiDrawChip(theme, chip, font, TextFormat("SHOTS %02d", state->session_shots), theme->accent_amber);

    UmonUiDrawTextRight(
        font,
        TextFormat("Pose %u  |  Score %u", state->udp_port, state->score_udp_port),
        (Vector2){layout->header.x + layout->header.width - 20.0f, layout->header.y + layout->header.height - 20.0f},
        12.0f,
        theme->text_muted
    );
}

static void DrawCompactStatBox(
    const UmonUiTheme *theme,
    Font font,
    Rectangle bounds,
    const char *label,
    const char *value,
    Color accent
) {
    DrawRectangleRounded(bounds, 0.18f, 10, Fade(theme->accent_slate, 0.22f));
    DrawRectangleRoundedLinesEx(bounds, 0.18f, 10, 1.0f, Fade(accent, 0.22f));
    UmonUiDrawText(font, label, (Vector2){bounds.x + 10.0f, bounds.y + 8.0f}, 11.0f, theme->text_muted);
    UmonUiDrawText(font, value, (Vector2){bounds.x + 10.0f, bounds.y + 22.0f}, 22.0f, accent);
}

static void DrawScorePanel(const AppState *state, const DashboardLayout *layout, const UmonUiTheme *theme, Font font) {
    UmonUiDrawPanelAlt(theme, layout->score_panel, font, "SHOT SCORE");

    float flash = ScoreFlashStrength(state);
    if (flash > 0.0f) {
        DrawRectangleRoundedLinesEx(
            layout->score_panel,
            0.12f,
            12,
            2.0f,
            Fade(theme->accent_amber, 0.35f + flash * 0.45f)
        );
    }

    float average_score = state->session_shots > 0 ? state->session_score_sum / (float)state->session_shots : 0.0f;
    const char *score_text = state->has_score_event ? TextFormat("%05.1f", state->latest_score_event.total_score) : "--.-";
    UmonUiDrawText(
        font,
        score_text,
        (Vector2){layout->score_panel.x + 18.0f, layout->score_panel.y + 50.0f},
        56.0f,
        theme->accent_amber
    );
    UmonUiDrawText(
        font,
        "/100",
        (Vector2){layout->score_panel.x + 194.0f, layout->score_panel.y + 86.0f},
        16.0f,
        theme->text_muted
    );

    UmonUiDrawProgressBar(
        theme,
        (Rectangle){
            layout->score_panel.x + 18.0f,
            layout->score_panel.y + 118.0f,
            layout->score_panel.width - 36.0f,
            12.0f,
        },
        state->has_score_event ? state->latest_score_event.total_score / 100.0f : 0.0f,
        theme->accent_amber
    );

    Rectangle stat_area = Inset(layout->score_panel, 18.0f, 146.0f, 18.0f, 18.0f);
    float gap = 10.0f;
    float box_width = (stat_area.width - gap * 2.0f) / 3.0f;
    DrawCompactStatBox(
        theme,
        font,
        (Rectangle){stat_area.x, stat_area.y, box_width, stat_area.height},
        "AVG",
        state->session_shots > 0 ? TextFormat("%04.1f", average_score) : "--.-",
        theme->accent_cyan
    );
    DrawCompactStatBox(
        theme,
        font,
        (Rectangle){stat_area.x + box_width + gap, stat_area.y, box_width, stat_area.height},
        "BEST",
        state->session_shots > 0 ? TextFormat("%04.1f", state->session_best_score) : "--.-",
        theme->accent_green
    );
    DrawCompactStatBox(
        theme,
        font,
        (Rectangle){stat_area.x + (box_width + gap) * 2.0f, stat_area.y, box_width, stat_area.height},
        "LAST",
        state->session_shots > 0 ? TextFormat("%04.1f", state->session_last_score) : "--.-",
        theme->text_primary
    );
}

static void DrawInfoPanel(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    bool model_ready
) {
    UmonUiDrawPanel(theme, layout->info_panel, font, "KEY METRICS");

    Rectangle inner = Inset(layout->info_panel, 16.0f, 46.0f, 16.0f, 16.0f);
    float row_height = 28.0f;
    float gap = 7.0f;

    if (!state->has_frame) {
        UmonUiDrawText(font, "Waiting for pose stream...", (Vector2){inner.x, inner.y + 2.0f}, 15.0f, theme->accent_amber);
        UmonUiDrawText(font, model_ready ? "Model assets ready, press M to preview" : "Model assets unavailable", (Vector2){inner.x, inner.y + 28.0f}, 12.0f, model_ready ? theme->accent_green : theme->accent_red);
        return;
    }

    Rectangle row = {inner.x, inner.y, inner.width, row_height};
    UmonUiDrawMetricRow(theme, font, row, "Sequence", TextFormat("%u  |  sample %u/4", state->latest_frame.seq_cnt, state->latest_frame.sample_idx), theme->text_primary);
    row.y += row_height + gap;
    UmonUiDrawMetricRow(theme, font, row, "Pitch", TextFormat("%.2f deg", state->latest_frame.hand_palm_pitch_deg), theme->accent_cyan);
    row.y += row_height + gap;
    UmonUiDrawMetricRow(theme, font, row, "dPitch", TextFormat("%.2f deg/s", state->latest_frame.hand_palm_pitch_dps), theme->accent_cyan);
    row.y += row_height + gap;
    UmonUiDrawMetricRow(
        theme,
        font,
        row,
        "Stale",
        TextFormat(
            "UA:%s  FA:%s  HP:%s",
            state->latest_frame.upper_arm_stale ? "Y" : "N",
            state->latest_frame.forearm_stale ? "Y" : "N",
            state->latest_frame.hand_palm_stale ? "Y" : "N"
        ),
        (state->latest_frame.upper_arm_stale || state->latest_frame.forearm_stale || state->latest_frame.hand_palm_stale)
            ? theme->accent_red
            : theme->accent_green
    );
    row.y += row_height + gap;
    UmonUiDrawMetricRow(theme, font, row, "Calibration", state->calibrated ? "ACTIVE" : "OFF", state->calibrated ? theme->accent_cyan : theme->text_secondary);
    row.y += row_height + gap;
    UmonUiDrawMetricRow(
        theme,
        font,
        row,
        "Render",
        model_ready ? (state->model_preview_enabled ? "MODEL PREVIEW" : "SKELETON") : "SKELETON",
        model_ready ? (state->model_preview_enabled ? theme->accent_green : theme->accent_cyan) : theme->accent_amber
    );

    if (state->has_score_event) {
        UmonUiDrawText(
            font,
            TextFormat(
                "Jump %.2f   Recovery %.0f ms",
                state->latest_score_event.max_muzzle_jump_deg,
                state->latest_score_event.recovery_time_ms
            ),
            (Vector2){inner.x, layout->info_panel.y + layout->info_panel.height - 26.0f},
            12.0f,
            theme->text_muted
        );
    }
}

static void DrawScenePanel(
    const AppState *state,
    const DashboardLayout *layout,
    const UmonUiTheme *theme,
    Font font,
    const SceneCanvas *canvas,
    bool canvas_ready,
    bool model_ready
) {
    UmonUiDrawPanelAlt(theme, layout->scene_panel, font, "3D VIEW");
    UmonUiDrawChip(
        theme,
        (Rectangle){layout->scene_panel.x + layout->scene_panel.width - 160.0f, layout->scene_panel.y + 12.0f, 136.0f, 22.0f},
        font,
        model_ready ? (state->model_preview_enabled ? "MODEL ON" : "MODEL READY") : "MODEL OFF",
        model_ready ? (state->model_preview_enabled ? theme->accent_green : theme->accent_cyan) : theme->accent_red
    );

    DrawRectangleRounded(layout->scene_view, 0.04f, 8, Fade((Color){4, 8, 14, 255}, 0.92f));
    DrawRectangleRoundedLinesEx(layout->scene_view, 0.04f, 8, 1.0f, Fade(theme->panel_border, 0.55f));

    if (canvas_ready) {
        DrawTexturePro(
            canvas->target.texture,
            (Rectangle){0.0f, 0.0f, (float)canvas->width, -(float)canvas->height},
            layout->scene_view,
            (Vector2){0.0f, 0.0f},
            0.0f,
            WHITE
        );
    }

    if (!state->has_frame) {
        const char *waiting = "Waiting for pose stream...";
        Vector2 size = MeasureTextEx(font, waiting, 20.0f, 20.0f * 0.03f);
        UmonUiDrawText(
            font,
            waiting,
            (Vector2){
                layout->scene_view.x + (layout->scene_view.width - size.x) * 0.5f,
                layout->scene_view.y + (layout->scene_view.height - size.y) * 0.5f,
            },
            20.0f,
            theme->accent_amber
        );
    }

    UmonUiDrawText(
        font,
        "LMB orbit  |  Wheel zoom  |  V reset  |  M preview models",
        (Vector2){layout->scene_view.x + 12.0f, layout->scene_view.y + layout->scene_view.height - 22.0f},
        12.0f,
        theme->text_muted
    );
}

static void DrawWaveformPanel(const AppState *state, const DashboardLayout *layout, const UmonUiTheme *theme, Font font) {
    UmonUiDrawPanel(theme, layout->waveform_panel, font, "HAND_PALM PITCH");

    PitchSample samples[OSCILLOSCOPE_CAPACITY];
    int sample_count = CollectRecentSamples(state, samples, OSCILLOSCOPE_CAPACITY);
    if (sample_count > 0) {
        UmonUiDrawChip(
            theme,
            (Rectangle){layout->waveform_panel.x + layout->waveform_panel.width - 240.0f, layout->waveform_panel.y + 12.0f, 108.0f, 22.0f},
            font,
            TextFormat("PITCH %.2f", samples[sample_count - 1].pitch_deg),
            theme->accent_cyan
        );
        UmonUiDrawChip(
            theme,
            (Rectangle){layout->waveform_panel.x + layout->waveform_panel.width - 122.0f, layout->waveform_panel.y + 12.0f, 98.0f, 22.0f},
            font,
            TextFormat("DPS %.0f", samples[sample_count - 1].pitch_dps),
            theme->accent_amber
        );
    }

    DrawRectangleRounded(layout->waveform_graph, 0.03f, 8, Fade((Color){5, 9, 16, 255}, 0.92f));
    DrawRectangleRoundedLinesEx(layout->waveform_graph, 0.03f, 8, 1.0f, Fade(theme->panel_border, 0.50f));

    if (sample_count < 2) {
        UmonUiDrawText(
            font,
            "Waiting for waveform data...",
            (Vector2){layout->waveform_graph.x + 12.0f, layout->waveform_graph.y + 14.0f},
            15.0f,
            theme->text_muted
        );
        return;
    }

    uint64_t latest_timestamp = samples[sample_count - 1].timestamp_ms;
    uint64_t window_start = (latest_timestamp > OSCILLOSCOPE_WINDOW_MS) ? (latest_timestamp - OSCILLOSCOPE_WINDOW_MS) : 0;

    float min_pitch = samples[0].pitch_deg;
    float max_pitch = samples[0].pitch_deg;
    for (int index = 1; index < sample_count; ++index) {
        min_pitch = fminf(min_pitch, samples[index].pitch_deg);
        max_pitch = fmaxf(max_pitch, samples[index].pitch_deg);
    }

    float center = (min_pitch + max_pitch) * 0.5f;
    float span = max_pitch - min_pitch;
    if (span < 20.0f) {
        min_pitch = center - 10.0f;
        max_pitch = center + 10.0f;
    }
    span = max_pitch - min_pitch;
    min_pitch -= span * 0.1f;
    max_pitch += span * 0.1f;

    float range = max_pitch - min_pitch;
    if (range < 0.001f) {
        range = 0.001f;
    }

    for (int grid = 0; grid <= 4; ++grid) {
        float y = layout->waveform_graph.y + (layout->waveform_graph.height / 4.0f) * grid;
        DrawLine(
            (int)layout->waveform_graph.x,
            (int)y,
            (int)(layout->waveform_graph.x + layout->waveform_graph.width),
            (int)y,
            Fade(theme->accent_slate, 0.26f)
        );
    }

    if (min_pitch <= 0.0f && max_pitch >= 0.0f) {
        float zero_ratio = (0.0f - min_pitch) / range;
        float zero_y = layout->waveform_graph.y + layout->waveform_graph.height - zero_ratio * layout->waveform_graph.height;
        DrawLine(
            (int)layout->waveform_graph.x,
            (int)zero_y,
            (int)(layout->waveform_graph.x + layout->waveform_graph.width),
            (int)zero_y,
            Fade(theme->text_secondary, 0.35f)
        );
    }

    uint64_t shot_markers[SHOT_HISTORY_CAPACITY];
    int marker_count = CollectRecentShotMarkers(state, shot_markers, SHOT_HISTORY_CAPACITY, window_start);
    for (int marker_index = 0; marker_index < marker_count; ++marker_index) {
        float x_ratio = (float)(shot_markers[marker_index] - window_start) / (float)OSCILLOSCOPE_WINDOW_MS;
        float x = layout->waveform_graph.x + x_ratio * layout->waveform_graph.width;
        DrawLine(
            (int)x,
            (int)layout->waveform_graph.y,
            (int)x,
            (int)(layout->waveform_graph.y + layout->waveform_graph.height),
            Fade(theme->accent_amber, 0.55f)
        );
    }

    Vector2 points[OSCILLOSCOPE_CAPACITY];
    for (int index = 0; index < sample_count; ++index) {
        float x_ratio = (float)(samples[index].timestamp_ms - window_start) / (float)OSCILLOSCOPE_WINDOW_MS;
        float y_ratio = (samples[index].pitch_deg - min_pitch) / range;
        points[index].x = layout->waveform_graph.x + x_ratio * layout->waveform_graph.width;
        points[index].y = layout->waveform_graph.y + layout->waveform_graph.height - y_ratio * layout->waveform_graph.height;
    }

    for (int index = 1; index < sample_count; ++index) {
        DrawLineBezier(points[index - 1], points[index], 2.4f, theme->accent_cyan);
    }

    UmonUiDrawText(
        font,
        TextFormat("Range %.1f .. %.1f deg  |  Window 2.0 s", min_pitch, max_pitch),
        (Vector2){layout->waveform_graph.x + 10.0f, layout->waveform_graph.y + 10.0f},
        12.0f,
        theme->text_muted
    );
}

int main(int argc, char **argv) {
    unsigned short udp_port = DEFAULT_UDP_PORT;
    unsigned short score_udp_port = DEFAULT_SCORE_UDP_PORT;
    if (!ParseCommandLine(argc, argv, &udp_port, &score_udp_port)) {
        PrintUsage(argv[0]);
        return 1;
    }

    AppState state = {0};
    state.udp_port = udp_port;
    state.score_udp_port = score_udp_port;
    state.model_preview_enabled = false;

    state.udp_receiver = UmonUdpReceiverCreate(udp_port);
    if (state.udp_receiver == NULL) {
        fprintf(stderr, "Failed to create UDP socket on port %u\n", udp_port);
        return 1;
    }
    state.score_receiver = UmonScoreReceiverCreate(score_udp_port);
    if (state.score_receiver == NULL) {
        fprintf(stderr, "Failed to create score UDP socket on port %u\n", score_udp_port);
        UmonUdpReceiverDestroy(state.udp_receiver);
        return 1;
    }

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Upper Monitor Visualizer");
    if (!IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE)) {
        ToggleBorderlessWindowed();
    }
    SetTargetFPS(60);

    UmonSceneAssets assets = {0};
    UmonSceneAssetsLoad(&assets, UPPER_ARM_LEN, FOREARM_LEN, HAND_LEN);
    Font ui_font = UmonSceneAssetsGetFont(&assets);
    const UmonUiTheme *theme = UmonUiThemeDefault();

    Camera3D camera = {0};
    camera.position = (Vector3){1.7f, 0.55f, 2.15f};
    camera.target = (Vector3){0.0f, -0.55f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 42.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    OrbitCameraState orbit_camera = CreateOrbitCameraState(camera.position, camera.target);
    ApplyOrbitCamera(&camera, &orbit_camera);

    SceneCanvas scene_canvas = {0};

    while (!WindowShouldClose()) {
        PollUdpFrames(&state);
        PollScoreEvents(&state);

        bool model_ready = UmonSceneAssetsHasModelLayer(&assets);

        if (IsKeyPressed(KEY_F11)) {
            ToggleBorderlessWindowed();
        }
        if (IsKeyPressed(KEY_M) && model_ready) {
            state.model_preview_enabled = !state.model_preview_enabled;
        }
        if (IsKeyPressed(KEY_C) && state.has_frame) {
            state.reference_upper_arm = UmonQuaternionFromWxyz(state.latest_frame.upper_arm);
            state.reference_forearm = UmonQuaternionFromWxyz(state.latest_frame.forearm);
            state.reference_hand_palm = UmonQuaternionFromWxyz(state.latest_frame.hand_palm);
            state.calibrated = true;
        }
        if (IsKeyPressed(KEY_R)) {
            state.calibrated = false;
        }

        int screen_width = GetScreenWidth();
        int screen_height = GetScreenHeight();
        DashboardLayout layout = ComputeDashboardLayout(screen_width, screen_height);

        Vector2 mouse_position = GetMousePosition();
        bool mouse_over_3d_view = CheckCollisionPointRec(mouse_position, layout.scene_view);
        UpdateOrbitCamera(&orbit_camera, &camera, mouse_over_3d_view, IsKeyPressed(KEY_V));

        UmonArmRenderPose pose = {0};
        bool has_pose = ComputeArmRenderPose(&state, &pose);
        bool canvas_ready = EnsureSceneCanvas(&scene_canvas, (int)layout.scene_view.width, (int)layout.scene_view.height);
        if (canvas_ready) {
            RenderSceneViewport(
                &scene_canvas,
                &assets,
                theme,
                camera,
                &pose,
                has_pose,
                model_ready && state.model_preview_enabled
            );
        }

        BeginDrawing();
        UmonUiDrawBackground(theme, screen_width, screen_height);

        DrawHeaderBar(&state, &layout, theme, ui_font, model_ready);
        DrawScorePanel(&state, &layout, theme, ui_font);
        DrawInfoPanel(&state, &layout, theme, ui_font, model_ready);
        DrawScenePanel(&state, &layout, theme, ui_font, &scene_canvas, canvas_ready, model_ready);
        DrawWaveformPanel(&state, &layout, theme, ui_font);

        EndDrawing();
    }

    UnloadSceneCanvas(&scene_canvas);
    UmonSceneAssetsUnload(&assets);
    CloseWindow();
    UmonScoreReceiverDestroy(state.score_receiver);
    UmonUdpReceiverDestroy(state.udp_receiver);
    return 0;
}
