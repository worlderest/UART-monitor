#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "math3d.h"
#include "udp_receiver.h"
#include "udp_protocol.h"

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 800

#define UDP_PORT 5005
#define OSCILLOSCOPE_CAPACITY 2048
#define OSCILLOSCOPE_WINDOW_MS 2000ULL

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

typedef struct AppState {
    UmonUdpReceiver *udp_receiver;
    bool has_frame;
    bool calibrated;
    UmonUdpFrame latest_frame;
    Quaternion reference_upper_arm;
    Quaternion reference_forearm;
    Quaternion reference_hand_palm;
    PitchSample samples[OSCILLOSCOPE_CAPACITY];
    int sample_count;
    int sample_head;
} AppState;

static bool IsValidUdpFrame(const UmonUdpFrame *frame);
static void AppendPitchSample(AppState *state, uint64_t timestamp_ms, float pitch_deg, float pitch_dps);
static void PollUdpFrames(AppState *state);
static void DrawArmSkeleton(const AppState *state);
static void DrawStatusText(const AppState *state);
static void DrawOscilloscope(const AppState *state, Rectangle bounds);
static int CollectRecentSamples(const AppState *state, PitchSample *out_samples, int capacity);
static OrbitCameraState CreateOrbitCameraState(Vector3 position, Vector3 target);
static void ResetOrbitCamera(OrbitCameraState *state);
static void ApplyOrbitCamera(Camera3D *camera, const OrbitCameraState *state);
static void UpdateOrbitCamera(
    OrbitCameraState *state,
    Camera3D *camera,
    bool mouse_over_viewport,
    bool reset_requested
);

static bool IsValidUdpFrame(const UmonUdpFrame *frame) {
    return memcmp(frame->magic, UMON_UDP_MAGIC, 4) == 0 && frame->version == UMON_UDP_VERSION;
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
        AppendPitchSample(state, frame.timestamp_ms, frame.hand_palm_pitch_deg, frame.hand_palm_pitch_dps);
    }
}

static void DrawArmSkeleton(const AppState *state) {
    if (!state->has_frame) {
        return;
    }

    Quaternion upper_arm_q = UmonQuaternionFromWxyz(state->latest_frame.upper_arm);
    Quaternion forearm_q = UmonQuaternionFromWxyz(state->latest_frame.forearm);
    Quaternion hand_palm_q = UmonQuaternionFromWxyz(state->latest_frame.hand_palm);

    if (state->calibrated) {
        upper_arm_q = UmonRelativeQuaternion(upper_arm_q, state->reference_upper_arm);
        forearm_q = UmonRelativeQuaternion(forearm_q, state->reference_forearm);
        hand_palm_q = UmonRelativeQuaternion(hand_palm_q, state->reference_hand_palm);
    }

    Vector3 shoulder_joint = {0.0f, 0.0f, 0.0f};
    Vector3 upper_arm = {0.0f, -UPPER_ARM_LEN, 0.0f};
    Vector3 forearm = {0.0f, -FOREARM_LEN, 0.0f};
    Vector3 hand_palm_axis = {0.0f, -HAND_BOARD_LENGTH, 0.0f};

    Vector3 elbow_joint = Vector3Add(shoulder_joint, UmonRotateVector(upper_arm, upper_arm_q));
    Vector3 wrist_joint = Vector3Add(elbow_joint, UmonRotateVector(forearm, forearm_q));
    Vector3 hand_axis_world = UmonRotateVector(hand_palm_axis, hand_palm_q);
    Vector3 hand_center = Vector3Add(wrist_joint, Vector3Scale(hand_axis_world, 0.5f));
    Vector3 hand_end = Vector3Add(wrist_joint, hand_axis_world);

    bool upper_arm_stale = state->latest_frame.upper_arm_stale != 0;
    bool forearm_stale = state->latest_frame.forearm_stale != 0;
    bool hand_palm_stale = state->latest_frame.hand_palm_stale != 0;

    Color joint_color = (Color){245, 246, 248, 255};
    Color live_bone_color = (Color){120, 198, 255, 255};
    Color stale_joint_color = (Color){232, 91, 70, 255};
    Color stale_bone_color = (Color){214, 100, 92, 180};
    Color hand_color = hand_palm_stale ? stale_joint_color : (Color){255, 211, 107, 255};

    DrawCylinderEx(
        shoulder_joint,
        elbow_joint,
        0.035f,
        0.03f,
        16,
        (upper_arm_stale || forearm_stale) ? stale_bone_color : live_bone_color
    );
    DrawCylinderEx(
        elbow_joint,
        wrist_joint,
        0.03f,
        0.025f,
        16,
        (forearm_stale || hand_palm_stale) ? stale_bone_color : live_bone_color
    );
    Matrix hand_rotation = QuaternionToMatrix(hand_palm_q);
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
            (Color){255, 245, 190, 255}
        );
    rlPopMatrix();

    DrawSphere(shoulder_joint, 0.045f, upper_arm_stale ? stale_joint_color : joint_color);
    DrawSphere(elbow_joint, 0.04f, forearm_stale ? stale_joint_color : joint_color);
    DrawSphere(wrist_joint, 0.035f, hand_palm_stale ? stale_joint_color : joint_color);
    DrawSphere(hand_end, 0.028f, hand_color);
}

static void DrawStatusText(const AppState *state) {
    int left = 20;
    int top = 16;

    DrawText("Real sensor mode: C calibrates. STM32 gun-test mode: use R to clear calibration.", left, top, 22, (Color){245, 246, 248, 255});
    DrawText(
        TextFormat("UDP %d | LMB orbit | Wheel zoom | V reset view | C calibrate | R reset calibration | ESC exit", UDP_PORT),
        left,
        top + 30,
        18,
        (Color){176, 186, 204, 255}
    );

    if (!state->has_frame) {
        DrawText("Waiting for Python UDP data...", left, top + 58, 18, (Color){255, 196, 111, 255});
        return;
    }

    DrawText(
        TextFormat(
            "seq=%u sample=%u | stale UA:%u FA:%u HP:%u | hand palm pitch %.2f deg | dPitch %.2f deg/s",
            state->latest_frame.seq_cnt,
            state->latest_frame.sample_idx,
            state->latest_frame.upper_arm_stale,
            state->latest_frame.forearm_stale,
            state->latest_frame.hand_palm_stale,
            state->latest_frame.hand_palm_pitch_deg,
            state->latest_frame.hand_palm_pitch_dps
        ),
        left,
        top + 58,
        18,
        (Color){176, 186, 204, 255}
    );
}

static int CollectRecentSamples(const AppState *state, PitchSample *out_samples, int capacity) {
    if (state->sample_count == 0 || capacity <= 0) {
        return 0;
    }

    int oldest_index = (state->sample_head - state->sample_count + OSCILLOSCOPE_CAPACITY) % OSCILLOSCOPE_CAPACITY;
    uint64_t latest_timestamp = state->samples[(state->sample_head - 1 + OSCILLOSCOPE_CAPACITY) % OSCILLOSCOPE_CAPACITY].timestamp_ms;
    uint64_t cutoff = (latest_timestamp > OSCILLOSCOPE_WINDOW_MS) ? (latest_timestamp - OSCILLOSCOPE_WINDOW_MS) : 0;

    int out_count = 0;
    for (int i = 0; i < state->sample_count; ++i) {
        int index = (oldest_index + i) % OSCILLOSCOPE_CAPACITY;
        PitchSample sample = state->samples[index];
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

static OrbitCameraState CreateOrbitCameraState(Vector3 position, Vector3 target) {
    OrbitCameraState state = {0};
    Vector3 offset = Vector3Subtract(position, target);
    float distance = Vector3Length(offset);
    if (distance < 0.001f) {
        distance = 0.001f;
        offset = (Vector3){0.0f, 0.0f, distance};
    }

    float pitch = asinf(Clamp(offset.y / distance, -1.0f, 1.0f));
    float yaw = atan2f(offset.x, offset.z);

    state.target = target;
    state.distance = distance;
    state.yaw = yaw;
    state.pitch = pitch;
    state.default_target = target;
    state.default_distance = distance;
    state.default_yaw = yaw;
    state.default_pitch = pitch;
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

static void DrawOscilloscope(const AppState *state, Rectangle bounds) {
    Color panel_color = (Color){15, 20, 29, 245};
    Color border_color = (Color){66, 82, 112, 255};
    Color trace_color = (Color){86, 203, 249, 255};
    Color zero_line_color = (Color){89, 111, 143, 120};
    Color text_color = (Color){232, 237, 243, 255};
    Color axis_color = (Color){150, 164, 183, 255};

    DrawRectangleRec(bounds, panel_color);
    DrawRectangleLinesEx(bounds, 1.0f, border_color);
    DrawText("Hand Palm Pitch Oscilloscope", (int)bounds.x + 16, (int)bounds.y + 10, 22, text_color);

    PitchSample samples[OSCILLOSCOPE_CAPACITY];
    int sample_count = CollectRecentSamples(state, samples, OSCILLOSCOPE_CAPACITY);
    if (sample_count < 2) {
        DrawText("Waiting for waveform data...", (int)bounds.x + 16, (int)bounds.y + 44, 18, axis_color);
        return;
    }

    uint64_t latest_timestamp = samples[sample_count - 1].timestamp_ms;
    uint64_t window_start = (latest_timestamp > OSCILLOSCOPE_WINDOW_MS) ? (latest_timestamp - OSCILLOSCOPE_WINDOW_MS) : 0;

    float min_pitch = samples[0].pitch_deg;
    float max_pitch = samples[0].pitch_deg;
    for (int i = 1; i < sample_count; ++i) {
        if (samples[i].pitch_deg < min_pitch) {
            min_pitch = samples[i].pitch_deg;
        }
        if (samples[i].pitch_deg > max_pitch) {
            max_pitch = samples[i].pitch_deg;
        }
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

    float graph_left = bounds.x + 14.0f;
    float graph_right = bounds.x + bounds.width - 14.0f;
    float graph_top = bounds.y + 46.0f;
    float graph_bottom = bounds.y + bounds.height - 18.0f;
    float graph_width = graph_right - graph_left;
    float graph_height = graph_bottom - graph_top;
    float range = max_pitch - min_pitch;
    if (range < 0.001f) {
        range = 0.001f;
    }

    if (min_pitch <= 0.0f && max_pitch >= 0.0f) {
        float zero_ratio = (0.0f - min_pitch) / range;
        float zero_y = graph_bottom - zero_ratio * graph_height;
        DrawLine((int)graph_left, (int)zero_y, (int)graph_right, (int)zero_y, zero_line_color);
    }

    Vector2 points[OSCILLOSCOPE_CAPACITY];
    for (int i = 0; i < sample_count; ++i) {
        float x_ratio = (float)(samples[i].timestamp_ms - window_start) / (float)OSCILLOSCOPE_WINDOW_MS;
        float y_ratio = (samples[i].pitch_deg - min_pitch) / range;
        points[i].x = graph_left + x_ratio * graph_width;
        points[i].y = graph_bottom - y_ratio * graph_height;
    }

    DrawLineStrip(points, sample_count, trace_color);
    DrawText(
        TextFormat(
            "Pitch %.2f deg | dPitch %.2f deg/s | Window 2.0 s",
            samples[sample_count - 1].pitch_deg,
            samples[sample_count - 1].pitch_dps
        ),
        (int)bounds.x + 16,
        (int)bounds.y + 78,
        18,
        axis_color
    );
}

int main(void) {
    AppState state = {0};
    state.udp_receiver = UmonUdpReceiverCreate(UDP_PORT);
    if (state.udp_receiver == NULL) {
        fprintf(stderr, "Failed to create UDP socket on port %d\n", UDP_PORT);
        return 1;
    }

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Upper Monitor Visualizer");
    SetTargetFPS(60);

    Camera3D camera = {0};
    camera.position = (Vector3){1.7f, 0.55f, 2.15f};
    camera.target = (Vector3){0.0f, -0.55f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 42.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    OrbitCameraState orbit_camera = CreateOrbitCameraState(camera.position, camera.target);
    ApplyOrbitCamera(&camera, &orbit_camera);

    while (!WindowShouldClose()) {
        PollUdpFrames(&state);

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
        int top_height = (screen_height * 2) / 3;
        Vector2 mouse_position = GetMousePosition();
        bool mouse_over_3d_view = mouse_position.x >= 0.0f &&
                                  mouse_position.x < (float)screen_width &&
                                  mouse_position.y >= 0.0f &&
                                  mouse_position.y < (float)top_height;
        UpdateOrbitCamera(&orbit_camera, &camera, mouse_over_3d_view, IsKeyPressed(KEY_V));

        Rectangle oscilloscope_bounds = {
            0.0f,
            (float)top_height,
            (float)screen_width,
            (float)(screen_height - top_height),
        };

        BeginDrawing();
        ClearBackground((Color){6, 10, 18, 255});

        BeginScissorMode(0, 0, screen_width, top_height);
        BeginMode3D(camera);
        DrawGrid(12, 0.2f);
        DrawArmSkeleton(&state);
        EndMode3D();
        EndScissorMode();

        DrawRectangle(0, top_height - 2, screen_width, 2, (Color){66, 82, 112, 255});
        DrawOscilloscope(&state, oscilloscope_bounds);
        DrawStatusText(&state);

        EndDrawing();
    }

    CloseWindow();
    UmonUdpReceiverDestroy(state.udp_receiver);
    return 0;
}
