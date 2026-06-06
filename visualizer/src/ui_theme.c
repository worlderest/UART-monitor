#include "ui_theme.h"

#include "raymath.h"

static float UiSpacing(float font_size) {
    return font_size * 0.03f;
}

static Rectangle Inset(Rectangle bounds, float inset) {
    return (Rectangle){
        bounds.x + inset,
        bounds.y + inset,
        bounds.width - inset * 2.0f,
        bounds.height - inset * 2.0f,
    };
}

const UmonUiTheme *UmonUiThemeDefault(void) {
    static const UmonUiTheme theme = {
        .bg_top = {5, 11, 18, 255},
        .bg_bottom = {2, 6, 12, 255},
        .panel_fill = {10, 18, 30, 220},
        .panel_fill_alt = {14, 24, 39, 232},
        .panel_border = {74, 99, 132, 255},
        .panel_shadow = {0, 0, 0, 110},
        .text_primary = {233, 240, 249, 255},
        .text_secondary = {174, 192, 214, 255},
        .text_muted = {117, 138, 164, 255},
        .accent_cyan = {73, 209, 255, 255},
        .accent_amber = {255, 198, 92, 255},
        .accent_red = {238, 90, 78, 255},
        .accent_green = {85, 224, 154, 255},
        .accent_slate = {54, 73, 99, 255},
    };

    return &theme;
}

void UmonUiDrawBackground(const UmonUiTheme *theme, int screen_width, int screen_height) {
    DrawRectangleGradientV(0, 0, screen_width, screen_height, theme->bg_top, theme->bg_bottom);

    DrawCircleGradient(
        (Vector2){screen_width * 0.82f, screen_height * 0.18f},
        screen_height * 0.30f,
        Fade(theme->accent_cyan, 0.10f),
        BLANK
    );
    DrawCircleGradient(
        (Vector2){screen_width * 0.18f, screen_height * 0.78f},
        screen_height * 0.34f,
        Fade(theme->accent_amber, 0.06f),
        BLANK
    );

    for (int row = 0; row < 14; ++row) {
        float y = 80.0f + row * 64.0f;
        DrawLine(
            0,
            (int)y,
            screen_width,
            (int)y,
            Fade(theme->accent_slate, 0.18f)
        );
    }
}

static void DrawPanelBase(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *title, Color fill) {
    Rectangle shadow = bounds;
    shadow.x += 8.0f;
    shadow.y += 8.0f;
    DrawRectangleRounded(shadow, 0.12f, 12, Fade(theme->panel_shadow, 0.70f));
    DrawRectangleRounded(bounds, 0.12f, 12, fill);
    DrawRectangleRoundedLinesEx(bounds, 0.12f, 12, 1.2f, theme->panel_border);

    Rectangle title_bar = {
        bounds.x + 12.0f,
        bounds.y + 12.0f,
        bounds.width - 24.0f,
        26.0f,
    };
    DrawRectangleRounded(title_bar, 0.45f, 10, Fade(theme->accent_slate, 0.32f));
    DrawRectangleRec(
        (Rectangle){title_bar.x + 8.0f, title_bar.y + 6.0f, 18.0f, 2.0f},
        theme->accent_cyan
    );
    DrawTextEx(
        font,
        title,
        (Vector2){title_bar.x + 34.0f, title_bar.y + 2.0f},
        18.0f,
        UiSpacing(18.0f),
        theme->text_secondary
    );
}

void UmonUiDrawPanel(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *title) {
    DrawPanelBase(theme, bounds, font, title, theme->panel_fill);
}

void UmonUiDrawPanelAlt(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *title) {
    DrawPanelBase(theme, bounds, font, title, theme->panel_fill_alt);
}

void UmonUiDrawText(Font font, const char *text, Vector2 position, float font_size, Color color) {
    DrawTextEx(font, text, position, font_size, UiSpacing(font_size), color);
}

void UmonUiDrawTextRight(Font font, const char *text, Vector2 anchor, float font_size, Color color) {
    Vector2 size = MeasureTextEx(font, text, font_size, UiSpacing(font_size));
    DrawTextEx(
        font,
        text,
        (Vector2){anchor.x - size.x, anchor.y},
        font_size,
        UiSpacing(font_size),
        color
    );
}

void UmonUiDrawMetricRow(
    const UmonUiTheme *theme,
    Font font,
    Rectangle bounds,
    const char *label,
    const char *value,
    Color value_color
) {
    Rectangle inner = Inset(bounds, 10.0f);
    DrawRectangleRounded(bounds, 0.18f, 10, Fade(theme->accent_slate, 0.24f));
    DrawTextEx(
        font,
        label,
        (Vector2){inner.x, inner.y + 2.0f},
        16.0f,
        UiSpacing(16.0f),
        theme->text_muted
    );
    UmonUiDrawTextRight(
        font,
        value,
        (Vector2){inner.x + inner.width, inner.y + 2.0f},
        16.0f,
        value_color
    );
}

void UmonUiDrawChip(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *text, Color accent) {
    DrawRectangleRounded(bounds, 0.42f, 10, Fade(accent, 0.16f));
    DrawRectangleRoundedLinesEx(bounds, 0.42f, 10, 1.0f, Fade(accent, 0.85f));

    Vector2 size = MeasureTextEx(font, text, 15.0f, UiSpacing(15.0f));
    DrawTextEx(
        font,
        text,
        (Vector2){
            bounds.x + (bounds.width - size.x) * 0.5f,
            bounds.y + (bounds.height - size.y) * 0.5f,
        },
        15.0f,
        UiSpacing(15.0f),
        accent
    );
}

void UmonUiDrawProgressBar(const UmonUiTheme *theme, Rectangle bounds, float ratio, Color fill_color) {
    ratio = Clamp(ratio, 0.0f, 1.0f);
    DrawRectangleRounded(bounds, 0.45f, 12, Fade(theme->accent_slate, 0.32f));

    Rectangle fill = bounds;
    fill.width *= ratio;
    if (fill.width > 0.0f) {
        DrawRectangleRounded(fill, 0.45f, 12, fill_color);
    }

    DrawRectangleRoundedLinesEx(bounds, 0.45f, 12, 1.0f, Fade(theme->panel_border, 0.9f));
}
