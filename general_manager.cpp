#include "general_manager.h"
#include <Arduino.h>
#include <SensorQMI8658.hpp>
#include <SensorPCF85063.hpp>

/* 测试模式：1=模拟数据驱动(未连接OBD时测试用)，0=真实OBD数据 */
#define GENERAL_TEST_MODE  0

/* 外部全局硬件 */
extern SensorQMI8658 qmi;
extern SensorPCF85063 rtc;

static lv_ui *s_ui = NULL;
static bool   s_active = false;

/* 计时 */
static uint32_t s_last_time_ms = 0;
static uint32_t s_last_temp_ms = 0;
static uint32_t s_last_arc_ms  = 0;
static uint32_t s_last_avg_ms  = 0;   /* 平均label更新计时 */
static uint32_t s_last_odometer_ms = 0; /* 里程积分计时 */

/* ================================================================
 *  能量数据（瞬时功率）
 * ================================================================ */
static float s_slider_power = 0.0f;
static float s_arc_power    = 0.0f;

/* 平均功率 / 平均油耗 累加器 */
static float  s_power_sum   = 0.0f;
static long   s_power_count = 0;
static long   s_fuel_sum    = 0;   /* 油耗累加（单位：0.1L/100km） */
static long   s_fuel_count  = 0;
static float  s_avg_power   = 0.0f;
static float  s_avg_fuel    = 0.0f;

/* 车辆总里程 / 行程里程 */
static float s_odometer = 0.0f;
static float s_trip_distance = 0.0f; /* 本次行程小计里程 */

/* ================================================================
 *  OBD 数据缓存（由 bluetooth_manager 写入）
 * ================================================================ */
static int s_obd_rpm   = 0;
static int s_obd_speed = 0;
static int s_obd_oil   = 0;   /* 油温/冷却液温度 */
static bool s_obd_oil_valid = false; /* 是否收到过真实油温 */

/* 更新 slider + label_energy_number */
static void update_slider_display(void)
{
    if (!s_ui) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", s_slider_power);

    if (s_ui->general_label_energy_number) {
        lv_label_set_text(s_ui->general_label_energy_number, buf);
    }

    if (s_ui->general_slider_energy) {
        int center = 50;
        int mapped = (int)(s_slider_power + center);

        if (s_slider_power >= 0) {
            lv_bar_set_start_value(s_ui->general_slider_energy, center, LV_ANIM_OFF);
            lv_bar_set_value(s_ui->general_slider_energy, mapped, LV_ANIM_OFF);
        } else {
            lv_bar_set_start_value(s_ui->general_slider_energy, mapped, LV_ANIM_OFF);
            lv_bar_set_value(s_ui->general_slider_energy, center, LV_ANIM_OFF);
        }

        lv_color_t color;
        if (s_slider_power < 0) {
            color = lv_color_hex(0x00FF00);
        } else if (s_slider_power < 50.0f) {
            color = lv_color_hex(0xFF6500);
        } else {
            color = lv_color_hex(0xFF0000);
        }
        lv_obj_set_style_bg_color(s_ui->general_slider_energy, color, LV_PART_INDICATOR);
    }

    /* 累加平均功率 */
    s_power_sum += s_slider_power;
    s_power_count++;
}

/* 更新 arc + label_energy_number_2 */
static void update_arc_display(void)
{
    if (!s_ui) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", s_arc_power);

    if (s_ui->general_label_energy_number_2) {
        lv_label_set_text(s_ui->general_label_energy_number_2, buf);
    }

    if (s_ui->general_arc_energy) {
        lv_arc_set_value(s_ui->general_arc_energy, (int)s_arc_power);
    }
}

/* 更新平均功率 / 平均油耗 / 里程（每 1 秒） */
static void update_avg_labels(void)
{
    if (!s_ui) return;

    char buf[16];

    /* 平均功率 */
    if (s_power_count > 0) {
        s_avg_power = s_power_sum / (float)s_power_count;
    }
    snprintf(buf, sizeof(buf), "%.1f", s_avg_power);
    if (s_ui->general_label_2) {
        lv_label_set_text(s_ui->general_label_2, buf);
    }

    /* 平均油耗 */
    if (s_fuel_count > 0) {
        s_avg_fuel = (float)s_fuel_sum / (float)s_fuel_count / 10.0f; /* 0.1L/100km → L/100km */
    }
    snprintf(buf, sizeof(buf), "%.1f", s_avg_fuel);
    if (s_ui->general_label_3) {
        lv_label_set_text(s_ui->general_label_3, buf);
    }
}

/* 基于速度积分更新小计里程 */
static void update_odometer(void)
{
    if (!s_ui) return;

    uint32_t now_ms = millis();
    if (s_last_odometer_ms > 0 && s_obd_speed > 0) {
        float dt = (now_ms - s_last_odometer_ms) / 1000.0f / 3600.0f; /* 小时 */
        if (dt > 0.0f && dt < 1.0f) {
            s_trip_distance += s_obd_speed * dt; /* km = km/h * h */
        }
    }
    s_last_odometer_ms = now_ms;

    /* 显示：优先用外部设置的总里程 + 本次行程 */
    float display_km = s_odometer + s_trip_distance;
    if (s_ui->general_label_1) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", display_km);
        lv_label_set_text(s_ui->general_label_1, buf);
    }
}

/* 外部接口：功率 */
void general_manager_set_energy(float power)
{
    s_slider_power = constrain(power, -50.0f, 50.0f);
    s_arc_power    = s_slider_power;
    update_slider_display();
    update_arc_display();
}

/* 外部接口：里程（单位 km，保留 1 位小数） */
void general_manager_set_odometer(float km)
{
    s_odometer = km;
    update_odometer(); /* 立即刷新显示 */
}

/* 外部接口：油耗（单位：0.1L/100km，例如 65 = 6.5L/100km） */
void general_manager_set_fuel(int fuel_x10)
{
    s_fuel_sum += fuel_x10;
    s_fuel_count++;
}

/* 外部接口：油温/冷却液温度 */
void general_manager_set_oil(int oil_temp)
{
    s_obd_oil = constrain(oil_temp, -40, 200);
    s_obd_oil_valid = true;
}

/* ================================================================
 *  OBD 数据接口（由 bluetooth_manager / OBD 模块调用）
 * ================================================================ */
void general_manager_set_obd_data(int rpm, int speed)
{
    s_obd_rpm   = constrain(rpm,   0, 8000);
    s_obd_speed = constrain(speed, 0, 240);
}

/* ================================================================
 *  时间 / 温度更新
 * ================================================================ */
static void update_time(void)
{
    if (!s_ui || !s_ui->general_label_time) return;

    struct tm timeinfo;
    rtc.getDateTime(&timeinfo);
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    lv_label_set_text(s_ui->general_label_time, buf);
}

static void update_temp(void)
{
    if (!s_ui || !s_ui->general_label_temp) return;

    float temp = 0.0f;
    bool ok = qmi.getDataReady() && (temp = qmi.getTemperature_C(), true);
    char buf[8];
    snprintf(buf, sizeof(buf), ok ? "%.1f" : "--.-", temp);
    lv_label_set_text(s_ui->general_label_temp, buf);
}

/* ================================================================
 *  OBD 数据驱动 Arc / Label（RPM / Speed / Oil）
 * ================================================================ */
static void update_obd_display(uint32_t now_ms)
{
    if (!s_ui) return;

    /* 每 100ms 刷新一次 */
    if (now_ms - s_last_arc_ms < 100) return;
    s_last_arc_ms = now_ms;

#if GENERAL_TEST_MODE
    /* 测试模式：模拟数据自动生成，验证显示效果 */
    static int test_rpm   = 0;
    static int test_speed = 0;
    test_rpm   = (test_rpm   + 80) % 8000;
    test_speed = (test_speed + 2) % 240;
    s_obd_rpm   = test_rpm;
    s_obd_speed = test_speed;

    /* 测试模式：oil 模拟波动 */
    s_obd_oil = 75 + (int)(15.0f * sinf((float)now_ms / 2000.0f));
    s_obd_oil_valid = true;
#else
    /* 真实模式：s_obd_rpm / s_obd_speed 由外部 OBD 模块写入 */
    /* oil 在没有外部数据时保持原值，不再自行波动 */
    (void)now_ms;  /* 避免编译警告 */
#endif

    /* RPM */
    if (s_ui->general_arc_rpm) {
        lv_arc_set_value(s_ui->general_arc_rpm, s_obd_rpm);
    }
    if (s_ui->general_label_rpm_number) {
        if (s_obd_rpm == 0) {
            /* 转速为 0 时隐藏 label */
            lv_label_set_text(s_ui->general_label_rpm_number, "");
        } else {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", s_obd_rpm);
            lv_label_set_text(s_ui->general_label_rpm_number, buf);
        }
    }

    /* Speed */
    if (s_ui->general_arc_speed) {
        lv_arc_set_value(s_ui->general_arc_speed, s_obd_speed);
    }
    if (s_ui->general_label_speed_number) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", s_obd_speed);
        lv_label_set_text(s_ui->general_label_speed_number, buf);
    }

    /* Oil */
    if (s_ui->general_arc_oil) {
        lv_arc_set_value(s_ui->general_arc_oil, s_obd_oil);
    }
    if (s_ui->general_label_oil_number) {
        char buf[8];
        if (s_obd_oil_valid || GENERAL_TEST_MODE) {
            snprintf(buf, sizeof(buf), "%d", s_obd_oil);
        } else {
            snprintf(buf, sizeof(buf), "--"); /* 无真实油温数据时显示 -- */
        }
        lv_label_set_text(s_ui->general_label_oil_number, buf);
    }

    /* 强制标记整个 general 页面重绘 — 绕过 LVGL 局部刷新在重叠透明对象时的计算错误 */
    lv_obj_invalidate(s_ui->general);
}

/* ================================================================
 *  外部接口
 * ================================================================ */
void general_manager_init(lv_ui *ui)
{
    s_ui = ui;
    if (s_ui->general_slider_energy) {
        /* 范围 0~100，50 对应实际值 0（中心点） */
        lv_slider_set_range(s_ui->general_slider_energy, 0, 100);
    }
    /* 初始化时刷新所有显示 */
    update_slider_display();
    update_arc_display();
    update_avg_labels();
    update_odometer();
}

void general_manager_enter(void)
{
    s_active = true;
    /* 进入时立即刷新一次 */
    update_time();
    update_temp();
    update_slider_display();
    update_arc_display();
    update_avg_labels();
    update_odometer();

    /* 把数字标签移到最上层，防止 arc 重绘时覆盖 */
    if (s_ui->general_label_rpm_number)   lv_obj_move_foreground(s_ui->general_label_rpm_number);
    if (s_ui->general_label_speed_number) lv_obj_move_foreground(s_ui->general_label_speed_number);
    if (s_ui->general_label_oil_number)  lv_obj_move_foreground(s_ui->general_label_oil_number);
    if (s_ui->general_label_energy_number) lv_obj_move_foreground(s_ui->general_label_energy_number);
    if (s_ui->general_label_energy_number_2) lv_obj_move_foreground(s_ui->general_label_energy_number_2);
}

void general_manager_exit(void)
{
    s_active = false;
}

bool general_manager_is_active(void)
{
    return s_active;
}

void general_manager_update(uint32_t now_ms)
{
    if (!s_active || !s_ui) return;

    /* 时间: 1秒 */
    if (now_ms - s_last_time_ms >= 1000) {
        s_last_time_ms = now_ms;
        update_time();
    }

    /* 温度: 3秒 */
    if (now_ms - s_last_temp_ms >= 3000) {
        s_last_temp_ms = now_ms;
        update_temp();
    }

    /* OBD 数据驱动 RPM / Speed / Oil */
    update_obd_display(now_ms);

    /* 平均功率 / 油耗 / 里程: 1秒 */
    if (now_ms - s_last_avg_ms >= 1000) {
        s_last_avg_ms = now_ms;
        update_avg_labels();
        update_odometer();
    }

#if GENERAL_TEST_MODE
    /* 测试模式：energy 模拟波动（-50 ~ 50） */
    static int test_energy = 0;
    static int test_energy_dir = 1;
    if ((now_ms / 50) % 2 == 0) {
        test_energy += test_energy_dir * 2;
        if (test_energy >= 45) test_energy_dir = -1;
        if (test_energy <= -45) test_energy_dir = 1;
        s_slider_power = constrain(test_energy, -50, 50);
        s_arc_power    = s_slider_power;
        update_slider_display();
        update_arc_display();
    }
#endif
}
