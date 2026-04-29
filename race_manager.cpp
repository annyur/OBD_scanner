#include "race_manager.h"
#include <Arduino.h>
#include <math.h>
#include <SensorQMI8658.hpp>

extern SensorQMI8658 qmi;

static lv_ui *s_ui = NULL;
static bool   s_active = false;
static uint32_t s_last_g_ms = 0;

/* G 值校准零偏 */
static float s_g_offset_x = 0.0f;
static float s_g_offset_y = 0.0f;

/* race_label_G_point 移动参数（父容器 = race_cont_G） */
#define METER_CX        233     /* 73 + 320/2 */
#define METER_CY        233
#define METER_RADIUS    145     /* 320/2 - 15 margin */
#define POINT_SIZE      20
#define POINT_HALF      10

/* ================================================================
 *  G-force 计算与 label_G_point 位置更新
 * ================================================================ */
static void update_g_force(uint32_t now_ms)
{
    if (!s_ui || !s_ui->race_label_G_point) return;

    /* 50Hz */
    if (now_ms - s_last_g_ms < 20) return;
    s_last_g_ms = now_ms;

    if (!qmi.getDataReady()) return;

    float ax, ay, az;
    qmi.getAccelerometer(ax, ay, az);

    float ax_c = ax - s_g_offset_x;
    float ay_c = ay - s_g_offset_y;

    const float G = 9.80665f;
    float g_right =  ax_c / G;
    float g_front = -ay_c / G;

    g_right = constrain(g_right, -2.0f, 2.0f);
    g_front = constrain(g_front, -2.0f, 2.0f);

    /* 映射到像素偏移（2G = METER_RADIUS） */
    float dx = (g_right / 2.0f) * METER_RADIUS;
    float dy = -(g_front / 2.0f) * METER_RADIUS;

    /* 限制在圆形范围内 */
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > METER_RADIUS && dist > 0.0f) {
        float s = METER_RADIUS / dist;
        dx *= s;
        dy *= s;
    }

    /* label_G_point 位置（race_cont_G 坐标系，中心 - 半尺寸） */
    int16_t px = METER_CX + (int16_t)dx - POINT_HALF;
    int16_t py = METER_CY + (int16_t)dy - POINT_HALF;

    lv_obj_set_pos(s_ui->race_label_G_point, px, py);

    /* 强制标记整个 race 页面重绘 — 绕过 LVGL 局部刷新在重叠透明对象时的计算错误 */
    lv_obj_invalidate(s_ui->race);

    /* ---- 四个方向 G 值（只在变化时更新，减少重绘） ---- */
    char buf[8];
    static char s_buf_top[8]   = "";
    static char s_buf_down[8]  = "";
    static char s_buf_right[8] = "";
    static char s_buf_left[8]  = "";

    snprintf(buf, sizeof(buf), "%.2f", g_front < 0 ? -g_front : 0.0f);
    if (strcmp(buf, s_buf_top) != 0) {
        strcpy(s_buf_top, buf);
        if (s_ui->race_label_G_top) lv_label_set_text(s_ui->race_label_G_top, buf);
    }

    snprintf(buf, sizeof(buf), "%.2f", g_front > 0 ?  g_front : 0.0f);
    if (strcmp(buf, s_buf_down) != 0) {
        strcpy(s_buf_down, buf);
        if (s_ui->race_label_G_down) lv_label_set_text(s_ui->race_label_G_down, buf);
    }

    snprintf(buf, sizeof(buf), "%.2f", g_right > 0 ?  g_right : 0.0f);
    if (strcmp(buf, s_buf_right) != 0) {
        strcpy(s_buf_right, buf);
        if (s_ui->race_label_G_right) lv_label_set_text(s_ui->race_label_G_right, buf);
    }

    snprintf(buf, sizeof(buf), "%.2f", g_right < 0 ? -g_right : 0.0f);
    if (strcmp(buf, s_buf_left) != 0) {
        strcpy(s_buf_left, buf);
        if (s_ui->race_label_left) lv_label_set_text(s_ui->race_label_left, buf);
    }
}

/* ================================================================
 *  G 值校准
 * ================================================================ */
static void calibrate_g(void)
{
    if (!qmi.getDataReady()) return;
    float ax, ay, az;
    qmi.getAccelerometer(ax, ay, az);
    s_g_offset_x = ax;
    s_g_offset_y = ay;
}

/* ================================================================
 *  外部接口
 * ================================================================ */
void race_manager_init(lv_ui *ui)
{
    s_ui = ui;
    calibrate_g();
}

/* 去掉 label 背景/边框/阴影 */
static void clear_label_bg(lv_obj_t *obj)
{
    if (!obj) return;
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
}

void race_manager_enter(void)
{
    s_active = true;
    calibrate_g();

    /* 关掉图片圆角裁切 — GUI Guider 默认开启，频繁刷新时导致重绘花屏 */
    if (s_ui->race_img_3) {
        lv_obj_set_style_clip_corner(s_ui->race_img_3, false, LV_PART_MAIN);
    }

    /* 确保 point 在最上层 */
    if (s_ui->race_label_G_point) {
        lv_obj_move_foreground(s_ui->race_label_G_point);
    }
}

void race_manager_exit(void)
{
    s_active = false;
}

bool race_manager_is_active(void)
{
    return s_active;
}

void race_manager_update(uint32_t now_ms)
{
    if (!s_active || !s_ui) return;
    update_g_force(now_ms);
}
