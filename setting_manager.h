#ifndef SETTING_MANAGER_H
#define SETTING_MANAGER_H

#include "lvgl.h"
#include "src/gui_guider.h"

typedef enum {
    APP_SCREEN_GENERAL = 0,
    APP_SCREEN_RACE,
    APP_SCREEN_SETTING,
    APP_SCREEN_BLUETOOTH,
    APP_SCREEN_TOGGLE
} app_screen_t;

typedef void (*app_switch_cb_t)(app_screen_t target, bool animate);

void setting_manager_init(lv_ui *ui);
void setting_manager_set_switch_cb(app_switch_cb_t cb);

#endif
