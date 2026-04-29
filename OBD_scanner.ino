/*
 * hybridhud.ino — 基于仓库无旋转版本 + 蓝牙模块
 * 无 sw_rotate, 无 lv_disp_set_rotation, USB朝左自然方向
 */

#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Preferences.h>

#include "src/gui_guider.h"
#include "src/custom.h"
#include "setting_manager.h"
#include "bluetooth_manager.h"
#include "general_manager.h"
#include "race_manager.h"
#include "TouchDrvCSTXXX.hpp"
#include <SensorPCF85063.hpp>
#include <SensorQMI8658.hpp>

// ---- Pin definitions ----
#define LCD_SDIO0    4
#define LCD_SDIO1    5
#define LCD_SDIO2    6
#define LCD_SDIO3    7
#define LCD_SCLK    38
#define LCD_CS      12
#define LCD_RESET   39
#define LCD_WIDTH  466
#define LCD_HEIGHT 466

#define IIC_SDA     15
#define IIC_SCL     14
#define TP_INT      11
#define TP_RESET    40

// ---- Gesture thresholds ----
#define SETTING_OPEN_Y_THRESHOLD     80
#define SETTING_CLOSE_Y_THRESHOLD   420

// ---- Animation & timing ----
#define ANIM_DURATION_MS            20
#define SWITCH_LOCK_TIME            20
#define TIME_UPDATE_INTERVAL_MS     1000
#define TEMP_UPDATE_INTERVAL_MS     3000

// ---- Global hardware objects ----
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_GFX *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);

TouchDrvCST92xx touch;
SensorPCF85063 rtc;
SensorQMI8658 qmi;
lv_ui guider_ui;

// ---- LVGL display buffer ----
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;

// ---- Internal screen IDs (for state tracking) ----
typedef enum {
    SCREEN_GENERAL = 0,
    SCREEN_RACE,
    SCREEN_SETTING,
    SCREEN_BLUETOOTH
} screen_id_t;

static screen_id_t current_screen = SCREEN_GENERAL;
static screen_id_t prev_main_screen = SCREEN_GENERAL;
static bool is_switching = false;
static uint32_t switch_unlock_ms = 0;

// ---- Periodic update timing (bluetooth only) ----
// general_manager 和 race_manager 各自维护自己的计时器

// ---- Touch state (updated by my_touchpad_read) ----
static volatile int16_t g_touch_x = 0, g_touch_y = 0;
static volatile uint8_t g_touch_pressed = 0;

// ---- Gesture state ----
typedef enum {
    GESTURE_IDLE = 0,
    GESTURE_ACTIVE
} gesture_state_t;
static gesture_state_t gs_state = GESTURE_IDLE;
static int16_t gs_sx, gs_sy, gs_ex, gs_ey;

// ---- NVS preferences ----
static Preferences prefs;

// ============================================================
// Display flush callback
// ============================================================
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
    lv_disp_flush_ready(disp);
}

// ============================================================
// Touchpad read callback (raw physical coordinates)
// ============================================================
static void my_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    int16_t tx[5], ty[5];
    uint8_t n = touch.getPoint(tx, ty, touch.getSupportTouchPoint());
    if (n > 0) {
        g_touch_x = tx[0];
        g_touch_y = ty[0];
        g_touch_pressed = 1;
        data->point.x = tx[0];
        data->point.y = ty[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        g_touch_pressed = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ============================================================
// Screen switching
// ============================================================
static void app_switch_screen(app_screen_t target, bool animate)
{
    if (is_switching) return;

    /* Theme 按钮：在 general 和 race 之间切换 */
    if (target == APP_SCREEN_TOGGLE) {
        target = (prev_main_screen == SCREEN_RACE) ? APP_SCREEN_GENERAL : APP_SCREEN_RACE;
    }

    /* 离开蓝牙页面时清理 */
    if (current_screen == SCREEN_BLUETOOTH) {
        bluetooth_manager_exit();
    }

    lv_obj_t *target_obj = NULL;
    switch (target) {
        case APP_SCREEN_GENERAL:   target_obj = guider_ui.general;   break;
        case APP_SCREEN_RACE:      target_obj = guider_ui.race;      break;
        case APP_SCREEN_SETTING:   target_obj = guider_ui.setting;   break;
        case APP_SCREEN_BLUETOOTH: target_obj = guider_ui.bluetooth; break;
    }
    if (!target_obj || target_obj == lv_scr_act()) return;

    if (animate && target == APP_SCREEN_SETTING) {
        lv_scr_load_anim(target_obj, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, ANIM_DURATION_MS, 0, false);
    } else {
        lv_scr_load(target_obj);
    }

    is_switching = true;
    switch_unlock_ms = millis() + ANIM_DURATION_MS + SWITCH_LOCK_TIME;

    if (target == APP_SCREEN_GENERAL) {
        current_screen = SCREEN_GENERAL;
        prev_main_screen = SCREEN_GENERAL;
    } else if (target == APP_SCREEN_RACE) {
        current_screen = SCREEN_RACE;
        prev_main_screen = SCREEN_RACE;
    } else if (target == APP_SCREEN_BLUETOOTH) {
        current_screen = SCREEN_BLUETOOTH;
    } else {
        current_screen = SCREEN_SETTING;
    }

    /* 子页面生命周期 */
    if (target == APP_SCREEN_BLUETOOTH) {
        bluetooth_manager_enter();
    } else if (target == APP_SCREEN_GENERAL) {
        general_manager_enter();
        race_manager_exit();
    } else if (target == APP_SCREEN_RACE) {
        race_manager_enter();
        general_manager_exit();
    }

    if (target != APP_SCREEN_SETTING && target != APP_SCREEN_BLUETOOTH) {
        prefs.begin("hybridhud", false);
        prefs.putUChar("last_screen", (uint8_t)current_screen);
        prefs.end();
    }
}

// ============================================================
// Gesture processing (pure physical coordinates)
// ============================================================
static void process_gesture(int16_t dx, int16_t dy, int16_t start_y)
{
    if (is_switching) return;
    if (current_screen == SCREEN_BLUETOOTH) return;  /* 蓝牙页面禁用滑动手势 */

    const int16_t THRESH = 30;
    if (abs((int)dx) < THRESH && abs((int)dy) < THRESH) return;
    if (abs((int)dy) <= abs((int)dx)) return;

    if (dy > 0) {
        // Swipe down: open setting from top edge
        if (current_screen != SCREEN_SETTING && start_y < SETTING_OPEN_Y_THRESHOLD) {
            prev_main_screen = current_screen;
            app_switch_screen(APP_SCREEN_SETTING, true);
        }
    } else {
        // Swipe up: close setting from bottom edge
        if (current_screen == SCREEN_SETTING && start_y > SETTING_CLOSE_Y_THRESHOLD) {
            app_switch_screen(
                (prev_main_screen == SCREEN_RACE) ? APP_SCREEN_RACE : APP_SCREEN_GENERAL, false);
        }
    }
}

// ============================================================
// Data update: 已移至 general_manager / race_manager
// ============================================================

// ============================================================
// Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("[HybridHUD] Booting...");

    // Touch controller reset
    pinMode(TP_RESET, OUTPUT);
    digitalWrite(TP_RESET, LOW);
    delay(30);
    digitalWrite(TP_RESET, HIGH);
    delay(50);

    // I2C bus
    Wire.begin(IIC_SDA, IIC_SCL);

    // Touch init
    touch.setPins(TP_RESET, TP_INT);
    touch.begin(Wire, 0x5A, IIC_SDA, IIC_SCL);
    touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
    touch.setMirrorXY(true, true);

    // RTC init
    rtc.begin(Wire, IIC_SDA, IIC_SCL);

    // QMI8658 IMU init
    if (!qmi.begin(Wire, 0x6B, IIC_SDA, IIC_SCL)) {
        Serial.println("[WARN] QMI8658 init failed");
    } else {
        qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                                SensorQMI8658::ACC_ODR_1000Hz,
                                SensorQMI8658::LPF_MODE_0);
        qmi.enableAccelerometer();
    }

    // Display init
    Serial.println("[DBG] gfx begin...");
    gfx->begin();

    // LVGL init
    Serial.println("[DBG] lv_init...");
    lv_init();

    uint32_t buf_size = LCD_WIDTH * 40;  /* 40行缓冲区，稳定运行方案 */
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    buf2 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!buf1 || !buf2) {
        Serial.println("[FATAL] DMA buffer allocation failed");
        while (1) { delay(100); }
    }
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_size);

    // Display driver — 无旋转
    Serial.println("[DBG] Register display driver...");
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Input device (touch) driver
    Serial.println("[DBG] Register touch driver...");
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Init UI screen delete flags
    Serial.println("[DBG] init_scr_del_flag...");
    init_scr_del_flag(&guider_ui);

    // Build all screens
    Serial.println("[DBG] setup_scr_general...");
    setup_scr_general(&guider_ui);
    Serial.println("[DBG] setup_scr_race...");
    setup_scr_race(&guider_ui);
    Serial.println("[DBG] setup_scr_setting...");
    setup_scr_setting(&guider_ui);
    Serial.println("[DBG] setup_scr_bluetooth...");
    setup_scr_bluetooth(&guider_ui);
    Serial.println("[DBG] All screens built OK");

    // Disable scrolling on main screens
    lv_obj_clear_flag(guider_ui.general, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_clear_flag(guider_ui.race,    LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_CHAIN);
    Serial.println("[DBG] Scroll flags cleared");

    // Init managers
    Serial.println("[DBG] Init setting_manager...");
    setting_manager_init(&guider_ui);
    setting_manager_set_switch_cb(app_switch_screen);

    Serial.println("[DBG] Init bluetooth_manager...");
    bluetooth_manager_init(&guider_ui);
    bluetooth_manager_set_switch_cb(app_switch_screen);

    Serial.println("[DBG] Init general_manager...");
    general_manager_init(&guider_ui);

    Serial.println("[DBG] Init race_manager...");
    race_manager_init(&guider_ui);

    // Restore last main screen from NVS
    Serial.println("[DBG] Load NVS...");
    prefs.begin("hybridhud", true);
    uint8_t last = prefs.getUChar("last_screen", SCREEN_GENERAL);
    prefs.end();

    Serial.println("[DBG] Load initial screen...");
    if (last == SCREEN_RACE) {
        current_screen = SCREEN_RACE;
        prev_main_screen = SCREEN_RACE;
        lv_scr_load(guider_ui.race);
        race_manager_enter();
    } else {
        current_screen = SCREEN_GENERAL;
        prev_main_screen = SCREEN_GENERAL;
        lv_scr_load(guider_ui.general);
        general_manager_enter();
    }

    Serial.println("[HybridHUD] Ready");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
    uint32_t now = millis();

    // Clear switch lock after animation completes
    if (is_switching && now >= switch_unlock_ms) {
        is_switching = false;
    }

    // LVGL heartbeat
    lv_tick_inc(5);
    lv_timer_handler();

    // 根据当前页面选择性调用 manager update（提升效率）
    if (current_screen == SCREEN_GENERAL) {
        general_manager_update(now);
    } else if (current_screen == SCREEN_RACE) {
        race_manager_update(now);
    }

    // Bluetooth scan result refresh
    bluetooth_manager_update();

    // Gesture detection using global touch coordinates
    if (g_touch_pressed) {
        if (gs_state == GESTURE_IDLE) {
            gs_sx = g_touch_x; gs_sy = g_touch_y;
            gs_ex = g_touch_x; gs_ey = g_touch_y;
            gs_state = GESTURE_ACTIVE;
        } else {
            gs_ex = g_touch_x; gs_ey = g_touch_y;
        }
    } else if (gs_state == GESTURE_ACTIVE) {
        process_gesture(gs_ex - gs_sx, gs_ey - gs_sy, gs_sy);
        gs_state = GESTURE_IDLE;
    }

    delay(5);
}
