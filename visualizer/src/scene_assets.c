#include "scene_assets.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raymath.h"

typedef enum AxisId {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
} AxisId;

typedef struct ModelAlignmentConfig {
    AxisId length_axis;
    bool proximal_at_max;
} ModelAlignmentConfig;

static bool CopyString(char *destination, size_t capacity, const char *source) {
    if (destination == NULL || capacity == 0 || source == NULL) {
        return false;
    }

    int written = snprintf(destination, capacity, "%s", source);
    return written >= 0 && (size_t)written < capacity;
}

static bool JoinPath(char *destination, size_t capacity, const char *base, const char *leaf) {
    if (destination == NULL || capacity == 0 || base == NULL || leaf == NULL) {
        return false;
    }

    size_t base_length = strlen(base);
    bool needs_separator = base_length > 0 && base[base_length - 1] != '/' && base[base_length - 1] != '\\';
    int written = snprintf(destination, capacity, "%s%s%s", base, needs_separator ? "/" : "", leaf);
    return written >= 0 && (size_t)written < capacity;
}

static bool ResolveResourceRoot(char *destination, size_t capacity) {
    const char *application_directory = GetApplicationDirectory();
    if (application_directory == NULL || *application_directory == '\0') {
        return false;
    }

    char candidate[UMON_RESOURCE_PATH_CAPACITY];
    if (JoinPath(candidate, sizeof(candidate), application_directory, "resources") && DirectoryExists(candidate)) {
        return CopyString(destination, capacity, candidate);
    }
    if (JoinPath(candidate, sizeof(candidate), application_directory, "../resources") && DirectoryExists(candidate)) {
        return CopyString(destination, capacity, candidate);
    }

    return false;
}

static void LoadOptionalTexture(Texture2D *texture, bool *loaded, const char *path) {
    if (texture == NULL || loaded == NULL) {
        return;
    }

    *loaded = false;
    if (!FileExists(path)) {
        return;
    }

    *texture = LoadTexture(path);
    *loaded = texture->id != 0;
}

static Vector3 AxisVector(AxisId axis) {
    switch (axis) {
        case AXIS_X:
            return (Vector3){1.0f, 0.0f, 0.0f};
        case AXIS_Y:
            return (Vector3){0.0f, 1.0f, 0.0f};
        default:
            return (Vector3){0.0f, 0.0f, 1.0f};
    }
}

static Matrix BuildModelLocalTransform(Model model, float target_length, ModelAlignmentConfig config) {
    BoundingBox bounds = GetModelBoundingBox(model);
    Vector3 size = Vector3Subtract(bounds.max, bounds.min);

    Vector3 translation = {0.0f, 0.0f, 0.0f};
    Vector3 axis = AxisVector(config.length_axis);
    float major_extent = 0.0f;

    switch (config.length_axis) {
        case AXIS_X:
            major_extent = size.x;
            translation = (Vector3){
                -(config.proximal_at_max ? bounds.max.x : bounds.min.x),
                -0.5f * (bounds.min.y + bounds.max.y),
                -0.5f * (bounds.min.z + bounds.max.z),
            };
            break;
        case AXIS_Y:
            major_extent = size.y;
            translation = (Vector3){
                -0.5f * (bounds.min.x + bounds.max.x),
                -(config.proximal_at_max ? bounds.max.y : bounds.min.y),
                -0.5f * (bounds.min.z + bounds.max.z),
            };
            break;
        case AXIS_Z:
        default:
            major_extent = size.z;
            translation = (Vector3){
                -0.5f * (bounds.min.x + bounds.max.x),
                -0.5f * (bounds.min.y + bounds.max.y),
                -(config.proximal_at_max ? bounds.max.z : bounds.min.z),
            };
            break;
    }

    float safe_extent = fmaxf(major_extent, 0.001f);
    float scale = target_length / safe_extent;

    Matrix translate_matrix = MatrixTranslate(translation.x, translation.y, translation.z);
    Matrix rotate_matrix = QuaternionToMatrix(QuaternionFromVector3ToVector3(axis, (Vector3){0.0f, -1.0f, 0.0f}));
    Matrix scale_matrix = MatrixScale(scale, scale, scale);

    return MatrixMultiply(scale_matrix, MatrixMultiply(rotate_matrix, translate_matrix));
}

static void ApplyCommonTextures(UmonSceneAssets *assets, Model *model) {
    if (assets == NULL || model == NULL) {
        return;
    }

    for (int material_index = 0; material_index < model->materialCount; ++material_index) {
        Material *material = &model->materials[material_index];
        if (assets->albedo_loaded) {
            SetMaterialTexture(material, MATERIAL_MAP_ALBEDO, assets->albedo);
        }
        if (assets->normal_loaded) {
            SetMaterialTexture(material, MATERIAL_MAP_NORMAL, assets->normal);
        }
        if (assets->metalness_loaded) {
            SetMaterialTexture(material, MATERIAL_MAP_METALNESS, assets->metalness);
        }
    }
}

static void UnloadModelAsset(UmonModelAsset *asset) {
    if (asset == NULL || !asset->loaded) {
        return;
    }

    UnloadModel(asset->model);
    memset(asset, 0, sizeof(*asset));
}

static bool LoadModelAsset(
    UmonSceneAssets *assets,
    UmonModelAsset *asset,
    const char *relative_path,
    float target_length,
    ModelAlignmentConfig config
) {
    if (assets == NULL || asset == NULL || relative_path == NULL || assets->resource_root[0] == '\0') {
        return false;
    }

    char full_path[UMON_RESOURCE_PATH_CAPACITY];
    if (!JoinPath(full_path, sizeof(full_path), assets->resource_root, relative_path) || !FileExists(full_path)) {
        return false;
    }

    Model model = LoadModel(full_path);
    if (model.meshCount <= 0 || model.meshes == NULL || model.materials == NULL) {
        if (model.meshes != NULL || model.materials != NULL) {
            UnloadModel(model);
        }
        return false;
    }

    ApplyCommonTextures(assets, &model);
    asset->loaded = true;
    asset->model = model;
    asset->local_transform = BuildModelLocalTransform(model, target_length, config);
    return true;
}

static void DrawModelAsset(
    const UmonModelAsset *asset,
    Vector3 joint_origin,
    Quaternion orientation,
    Color tint
) {
    if (asset == NULL || !asset->loaded) {
        return;
    }

    Matrix world_transform = MatrixMultiply(
        MatrixTranslate(joint_origin.x, joint_origin.y, joint_origin.z),
        MatrixMultiply(QuaternionToMatrix(QuaternionNormalize(orientation)), asset->local_transform)
    );

    for (int mesh_index = 0; mesh_index < asset->model.meshCount; ++mesh_index) {
        int material_index = asset->model.meshMaterial[mesh_index];
        if (material_index < 0 || material_index >= asset->model.materialCount) {
            continue;
        }

        Material material = asset->model.materials[material_index];
        material.maps[MATERIAL_MAP_ALBEDO].color = tint;
        DrawMesh(asset->model.meshes[mesh_index], material, world_transform);
    }
}

bool UmonSceneAssetsLoad(UmonSceneAssets *assets, float upper_arm_len, float forearm_len, float hand_len) {
    if (assets == NULL) {
        return false;
    }

    memset(assets, 0, sizeof(*assets));
    if (!ResolveResourceRoot(assets->resource_root, sizeof(assets->resource_root))) {
        return false;
    }

    char path[UMON_RESOURCE_PATH_CAPACITY];

    if (JoinPath(path, sizeof(path), assets->resource_root, "fonts/ShareTechMono-Regular.ttf") && FileExists(path)) {
        assets->ui_font = LoadFontEx(path, 64, NULL, 0);
        assets->font_loaded = assets->ui_font.texture.id != 0;
    }

    if (JoinPath(path, sizeof(path), assets->resource_root, "textures/armLow_DefaultMaterial_AlbedoTransparency.png")) {
        LoadOptionalTexture(&assets->albedo, &assets->albedo_loaded, path);
    }
    if (JoinPath(path, sizeof(path), assets->resource_root, "textures/armLow_DefaultMaterial_Normal.png")) {
        LoadOptionalTexture(&assets->normal, &assets->normal_loaded, path);
    }
    if (JoinPath(path, sizeof(path), assets->resource_root, "textures/armLow_DefaultMaterial_MetallicSmoothness.png")) {
        LoadOptionalTexture(&assets->metalness, &assets->metalness_loaded, path);
    }

    LoadModelAsset(
        assets,
        &assets->upper_arm,
        "3D_model/upper_arm.obj",
        upper_arm_len,
        (ModelAlignmentConfig){.length_axis = AXIS_Z, .proximal_at_max = false}
    );
    LoadModelAsset(
        assets,
        &assets->forearm,
        "3D_model/forearm1.obj",
        forearm_len,
        (ModelAlignmentConfig){.length_axis = AXIS_Z, .proximal_at_max = false}
    );
    LoadModelAsset(
        assets,
        &assets->hand_palm,
        "3D_model/hand_palm.obj",
        hand_len,
        (ModelAlignmentConfig){.length_axis = AXIS_Z, .proximal_at_max = false}
    );

    return true;
}

void UmonSceneAssetsUnload(UmonSceneAssets *assets) {
    if (assets == NULL) {
        return;
    }

    UnloadModelAsset(&assets->upper_arm);
    UnloadModelAsset(&assets->forearm);
    UnloadModelAsset(&assets->hand_palm);

    if (assets->albedo_loaded) {
        UnloadTexture(assets->albedo);
        assets->albedo_loaded = false;
    }
    if (assets->normal_loaded) {
        UnloadTexture(assets->normal);
        assets->normal_loaded = false;
    }
    if (assets->metalness_loaded) {
        UnloadTexture(assets->metalness);
        assets->metalness_loaded = false;
    }
    if (assets->font_loaded) {
        UnloadFont(assets->ui_font);
        assets->font_loaded = false;
    }

    memset(assets->resource_root, 0, sizeof(assets->resource_root));
}

Font UmonSceneAssetsGetFont(const UmonSceneAssets *assets) {
    if (assets != NULL && assets->font_loaded) {
        return assets->ui_font;
    }
    return GetFontDefault();
}

bool UmonSceneAssetsHasModelLayer(const UmonSceneAssets *assets) {
    if (assets == NULL) {
        return false;
    }

    return assets->upper_arm.loaded && assets->forearm.loaded && assets->hand_palm.loaded;
}

void UmonSceneAssetsDrawArm(const UmonSceneAssets *assets, const UmonArmRenderPose *pose) {
    if (assets == NULL || pose == NULL) {
        return;
    }

    Color live_tint = WHITE;
    Color stale_tint = (Color){255, 168, 168, 210};
    Color hand_live_tint = (Color){255, 245, 220, 255};

    DrawModelAsset(
        &assets->upper_arm,
        pose->shoulder_joint,
        pose->upper_arm_orientation,
        pose->upper_arm_stale ? stale_tint : live_tint
    );
    DrawModelAsset(
        &assets->forearm,
        pose->elbow_joint,
        pose->forearm_orientation,
        pose->forearm_stale ? stale_tint : live_tint
    );
    DrawModelAsset(
        &assets->hand_palm,
        pose->wrist_joint,
        pose->hand_palm_orientation,
        pose->hand_palm_stale ? stale_tint : hand_live_tint
    );
}
