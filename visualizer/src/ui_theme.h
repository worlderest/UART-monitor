#ifndef UMON_UI_THEME_H
#define UMON_UI_THEME_H

#include "raylib.h"

typedef struct UmonUiTheme {
    Color bg_top;
    Color bg_bottom;
    Color panel_fill;
    Color panel_fill_alt;
    Color panel_border;
    Color panel_shadow;
    Color text_primary;
    Color text_secondary;
    Color text_muted;
    Color accent_cyan;
    Color accent_amber;
    Color accent_red;
    Color accent_green;
    Color accent_slate;
} UmonUiTheme;

const UmonUiTheme *UmonUiThemeDefault(void);
void UmonUiDrawBackground(const UmonUiTheme *theme, int screen_width, int screen_height);
void UmonUiDrawPanel(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *title);
void UmonUiDrawPanelAlt(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *title);
void UmonUiDrawText(Font font, const char *text, Vector2 position, float font_size, Color color);
void UmonUiDrawTextRight(Font font, const char *text, Vector2 anchor, float font_size, Color color);
void UmonUiDrawMetricRow(
    const UmonUiTheme *theme,
    Font font,
    Rectangle bounds,
    const char *label,
    const char *value,
    Color value_color
);
void UmonUiDrawChip(const UmonUiTheme *theme, Rectangle bounds, Font font, const char *text, Color accent);
void UmonUiDrawProgressBar(const UmonUiTheme *theme, Rectangle bounds, float ratio, Color fill_color);

#endif
