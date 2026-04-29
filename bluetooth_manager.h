#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include "lvgl.h"
#include "src/gui_guider.h"
#include "setting_manager.h"

void bluetooth_manager_init(lv_ui *ui);
void bluetooth_manager_set_switch_cb(app_switch_cb_t cb);
void bluetooth_manager_enter(void);
void bluetooth_manager_exit(void);
void bluetooth_manager_update(void);

#endif
