/* bluetooth_manager.h — OBD Scanner 蓝牙/OBD 核心接口 */
#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include "src/gui_guider.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 初始化 ---- */
void bluetooth_manager_init(lv_ui* ui);
void bluetooth_manager_set_switch_cb(app_switch_cb_t cb);
void bluetooth_manager_enter(void);
void bluetooth_manager_exit(void);
void bluetooth_manager_update(void);

/* ---- 状态 ---- */
bool bluetooth_is_connected(void);
const char* bluetooth_connected_name(void);
const char* bluetooth_connected_addr(void);

/* ---- 缓存数据 (供 UI 显示) ---- */
int     obd_rpm(void);
int     obd_speed(void);
int     obd_coolant(void);
int     obd_oil_temp(void);
float   obd_epa_soc(void);
int     obd_mil_on(void);
int     obd_dtc_count(void);
float   obd_batt_v(void);
int     obd_dist(void);
int     obd_map(void);
int     obd_iat(void);
int     obd_throttle(void);
float   obd_abs_load(void);
int     obd_torque_pct(void);
int     obd_torque_ref(void);
float   obd_odometer(void);

/* BMS */
float   bms_max_cell_v(void);
float   bms_min_cell_v(void);
int     bms_max_temp(void);
int     bms_min_temp(void);
int     bms_max_pos(void);
int     bms_min_pos(void);
int     bms_energy_wh(void);
float   bms_total_v(void);

/* SGCM */
int     sgcm_rpm(void);
float   sgcm_current_a(void);
float   sgcm_current_b(void);
int     sgcm_rated(void);

#ifdef __cplusplus
}
#endif

#endif