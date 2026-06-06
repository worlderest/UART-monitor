#ifndef UMON_SCENE_ASSETS_H
#define UMON_SCENE_ASSETS_H

#include <stdbool.h>

#include "raylib.h"

#define UMON_RESOURCE_PATH_CAPACITY 512

typedef struct UmonArmRenderPose {
    Vector3 shoulder_joint;
    Vector3 elbow_joint;
    Vector3 wrist_joint;
    Vector3 hand_end;
    Quaternion upper_arm_orientation;
    Quaternion forearm_orientation;
    Quaternion hand_palm_orientation;
    bool upper_arm_stale;
    bool forearm_stale;
    bool hand_palm_stale;
} UmonArmRenderPose;

typedef struct UmonModelAsset {
    bool loaded;
    Model model;
    Matrix local_transform;
} UmonModelAsset;

typedef struct UmonSceneAssets {
    char resource_root[UMON_RESOURCE_PATH_CAPACITY];
    bool font_loaded;
    Font ui_font;
    bool albedo_loaded;
    Texture2D albedo;
    bool normal_loaded;
    Texture2D normal;
    bool metalness_loaded;
    Texture2D metalness;
    UmonModelAsset upper_arm;
    UmonModelAsset forearm;
    UmonModelAsset hand_palm;
} UmonSceneAssets;

bool UmonSceneAssetsLoad(UmonSceneAssets *assets, float upper_arm_len, float forearm_len, float hand_len);
void UmonSceneAssetsUnload(UmonSceneAssets *assets);
Font UmonSceneAssetsGetFont(const UmonSceneAssets *assets);
bool UmonSceneAssetsHasModelLayer(const UmonSceneAssets *assets);
void UmonSceneAssetsDrawArm(const UmonSceneAssets *assets, const UmonArmRenderPose *pose);

#endif
