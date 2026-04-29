/*
 * OBD_scanner.ino — 精简版：仅保留 Bluetooth 扫描界面
 * 删除: general、race、setting 界面及对应 manager
 * 保留: bluetooth_manager (扫描核心) + 单屏 UI
 */

#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Preferences.h>

#include "src/gui_guider.h"
#include "src/custom.h"
#include "bluetooth_manager.h"
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

// ---- Animation & timing ----
#define ANIM_DURATION_MS            20
#define SWITCH_LOCK_TIME            20

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

// ---- Touch state (updated by my_touchpad_read) ----
static volatile int16_t g_touch_x = 0, g_touch_y = 0;
static volatile uint8_t g_touch_pressed = 0;

// ---- Internal screen IDs ----
typedef enum {
    SCREEN_BLUETOOTH = 0
} screen_id_t;

static screen_id_t current_screen = SCREEN_BLUETOOTH;
static bool is_switching = false;
static uint32_t switch_unlock_ms = 0;

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
        data->point.y = tx[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        g_touch_pressed = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ============================================================
// Screen switching (单屏模式，仅 bluetooth)
// ============================================================
static void app_switch_screen(app_screen_t target, bool animate)
{
    (void)animate; // 单屏不需要动画方向
    if (is_switching) return;
    if (target != APP_SCREEN_BLUETOOTH) return; // 只支持 bluetooth
    if (lv_scr_act() == guider_ui.bluetooth) return;

    bluetooth_manager_exit();
    lv_scr_load(guider_ui.bluetooth);
    bluetooth_manager_enter();
    is_switching = true;
    switch_unlock_ms = millis() + ANIM_DURATION_MS + SWITCH_LOCK_TIME;
    current_screen = SCREEN_BLUETOOTH;
}

// ============================================================
// Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("[OBDScanner] Booting...");

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

    uint32_t buf_size = LCD_WIDTH * 40;
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    buf2 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!buf1 || !buf2) {
        Serial.println("[FATAL] DMA buffer allocation failed");
        while (1) { delay(100); }
    }
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_size);

    // Display driver
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

    // Build only bluetooth screen
    Serial.println("[DBG] setup_scr_bluetooth...");
    setup_scr_bluetooth(&guider_ui);
    Serial.println("[DBG] All screens built OK");

    // Init bluetooth_manager
    Serial.println("[DBG] Init bluetooth_manager...");
    bluetooth_manager_init(&guider_ui);
    bluetooth_manager_set_switch_cb(app_switch_screen);

    // Load bluetooth screen directly
    Serial.println("[DBG] Load initial screen...");
    current_screen = SCREEN_BLUETOOTH;
    lv_scr_load(guider_ui.bluetooth);
    bluetooth_manager_enter();

    Serial.println("[OBDScanner] Ready");
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

    // 蓝牙扫描核心
    bluetooth_manager_update();

    delay(5);
}