/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_bluetooth(lv_ui *ui)
{
    //Write codes bluetooth
    ui->bluetooth = lv_obj_create(NULL);
    lv_obj_set_size(ui->bluetooth, 466, 466);
    lv_obj_set_scrollbar_mode(ui->bluetooth, LV_SCROLLBAR_MODE_OFF);

    //Write style for bluetooth, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->bluetooth, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes bluetooth_bt_label_title
    ui->bluetooth_bt_label_title = lv_label_create(ui->bluetooth);
    lv_label_set_text(ui->bluetooth_bt_label_title, "Bluetooth");
    lv_label_set_long_mode(ui->bluetooth_bt_label_title, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->bluetooth_bt_label_title, 133, 39);
    lv_obj_set_size(ui->bluetooth_bt_label_title, 200, 38);

    //Write style for bluetooth_bt_label_title, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->bluetooth_bt_label_title, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->bluetooth_bt_label_title, &lv_font_montserratMedium_36, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->bluetooth_bt_label_title, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->bluetooth_bt_label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->bluetooth_bt_label_title, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes bluetooth_btn_back
    ui->bluetooth_btn_back = lv_btn_create(ui->bluetooth);
    ui->bluetooth_btn_back_label = lv_label_create(ui->bluetooth_btn_back);
    lv_label_set_text(ui->bluetooth_btn_back_label, "Back");
    lv_label_set_long_mode(ui->bluetooth_btn_back_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->bluetooth_btn_back_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->bluetooth_btn_back, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->bluetooth_btn_back_label, LV_PCT(100));
    lv_obj_set_pos(ui->bluetooth_btn_back, 183, 379);
    lv_obj_set_size(ui->bluetooth_btn_back, 100, 50);

    //Write style for bluetooth_btn_back, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_btn_back, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->bluetooth_btn_back, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth_btn_back, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->bluetooth_btn_back, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_btn_back, 50, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->bluetooth_btn_back, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->bluetooth_btn_back, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->bluetooth_btn_back, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->bluetooth_btn_back, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->bluetooth_btn_back, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes bluetooth_bt_sw_enable
    ui->bluetooth_bt_sw_enable = lv_switch_create(ui->bluetooth);
    lv_obj_set_pos(ui->bluetooth_bt_sw_enable, 122, 86);
    lv_obj_set_size(ui->bluetooth_bt_sw_enable, 100, 50);

    //Write style for bluetooth_bt_sw_enable, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_bt_sw_enable, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->bluetooth_bt_sw_enable, lv_color_hex(0xe6e2e6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth_bt_sw_enable, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->bluetooth_bt_sw_enable, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_bt_sw_enable, 50, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->bluetooth_bt_sw_enable, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for bluetooth_bt_sw_enable, Part: LV_PART_INDICATOR, State: LV_STATE_CHECKED.
    lv_obj_set_style_bg_opa(ui->bluetooth_bt_sw_enable, 255, LV_PART_INDICATOR|LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(ui->bluetooth_bt_sw_enable, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_CHECKED);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth_bt_sw_enable, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_CHECKED);
    lv_obj_set_style_border_width(ui->bluetooth_bt_sw_enable, 0, LV_PART_INDICATOR|LV_STATE_CHECKED);

    //Write style for bluetooth_bt_sw_enable, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_bt_sw_enable, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->bluetooth_bt_sw_enable, lv_color_hex(0xffffff), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth_bt_sw_enable, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->bluetooth_bt_sw_enable, 0, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_bt_sw_enable, 50, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes bluetooth_bt_list_devices
    ui->bluetooth_bt_list_devices = lv_list_create(ui->bluetooth);
    ui->bluetooth_bt_list_devices_item0 = lv_list_add_btn(ui->bluetooth_bt_list_devices, LV_SYMBOL_BLUETOOTH, "list");
    lv_obj_set_pos(ui->bluetooth_bt_list_devices, 75, 148);
    lv_obj_set_size(ui->bluetooth_bt_list_devices, 329, 210);
    lv_obj_set_scrollbar_mode(ui->bluetooth_bt_list_devices, LV_SCROLLBAR_MODE_OFF);

    //Write style state: LV_STATE_DEFAULT for &style_bluetooth_bt_list_devices_main_main_default
    static lv_style_t style_bluetooth_bt_list_devices_main_main_default;
    ui_init_style(&style_bluetooth_bt_list_devices_main_main_default);

    lv_style_set_pad_top(&style_bluetooth_bt_list_devices_main_main_default, 5);
    lv_style_set_pad_left(&style_bluetooth_bt_list_devices_main_main_default, 5);
    lv_style_set_pad_right(&style_bluetooth_bt_list_devices_main_main_default, 5);
    lv_style_set_pad_bottom(&style_bluetooth_bt_list_devices_main_main_default, 5);
    lv_style_set_bg_opa(&style_bluetooth_bt_list_devices_main_main_default, 255);
    lv_style_set_bg_color(&style_bluetooth_bt_list_devices_main_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_bluetooth_bt_list_devices_main_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_bluetooth_bt_list_devices_main_main_default, 1);
    lv_style_set_border_opa(&style_bluetooth_bt_list_devices_main_main_default, 255);
    lv_style_set_border_color(&style_bluetooth_bt_list_devices_main_main_default, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_bluetooth_bt_list_devices_main_main_default, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_bluetooth_bt_list_devices_main_main_default, 3);
    lv_style_set_shadow_width(&style_bluetooth_bt_list_devices_main_main_default, 0);
    lv_obj_add_style(ui->bluetooth_bt_list_devices, &style_bluetooth_bt_list_devices_main_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_bluetooth_bt_list_devices_main_scrollbar_default
    static lv_style_t style_bluetooth_bt_list_devices_main_scrollbar_default;
    ui_init_style(&style_bluetooth_bt_list_devices_main_scrollbar_default);

    lv_style_set_radius(&style_bluetooth_bt_list_devices_main_scrollbar_default, 3);
    lv_style_set_bg_opa(&style_bluetooth_bt_list_devices_main_scrollbar_default, 255);
    lv_style_set_bg_color(&style_bluetooth_bt_list_devices_main_scrollbar_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_bluetooth_bt_list_devices_main_scrollbar_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(ui->bluetooth_bt_list_devices, &style_bluetooth_bt_list_devices_main_scrollbar_default, LV_PART_SCROLLBAR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_bluetooth_bt_list_devices_extra_btns_main_default
    static lv_style_t style_bluetooth_bt_list_devices_extra_btns_main_default;
    ui_init_style(&style_bluetooth_bt_list_devices_extra_btns_main_default);

    lv_style_set_pad_top(&style_bluetooth_bt_list_devices_extra_btns_main_default, 5);
    lv_style_set_pad_left(&style_bluetooth_bt_list_devices_extra_btns_main_default, 5);
    lv_style_set_pad_right(&style_bluetooth_bt_list_devices_extra_btns_main_default, 5);
    lv_style_set_pad_bottom(&style_bluetooth_bt_list_devices_extra_btns_main_default, 5);
    lv_style_set_border_width(&style_bluetooth_bt_list_devices_extra_btns_main_default, 0);
    lv_style_set_text_color(&style_bluetooth_bt_list_devices_extra_btns_main_default, lv_color_hex(0x0D3055));
    lv_style_set_text_font(&style_bluetooth_bt_list_devices_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_bluetooth_bt_list_devices_extra_btns_main_default, 255);
    lv_style_set_radius(&style_bluetooth_bt_list_devices_extra_btns_main_default, 3);
    lv_style_set_bg_opa(&style_bluetooth_bt_list_devices_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_bluetooth_bt_list_devices_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_bluetooth_bt_list_devices_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(ui->bluetooth_bt_list_devices_item0, &style_bluetooth_bt_list_devices_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_bluetooth_bt_list_devices_extra_texts_main_default
    static lv_style_t style_bluetooth_bt_list_devices_extra_texts_main_default;
    ui_init_style(&style_bluetooth_bt_list_devices_extra_texts_main_default);

    lv_style_set_pad_top(&style_bluetooth_bt_list_devices_extra_texts_main_default, 5);
    lv_style_set_pad_left(&style_bluetooth_bt_list_devices_extra_texts_main_default, 5);
    lv_style_set_pad_right(&style_bluetooth_bt_list_devices_extra_texts_main_default, 5);
    lv_style_set_pad_bottom(&style_bluetooth_bt_list_devices_extra_texts_main_default, 5);
    lv_style_set_border_width(&style_bluetooth_bt_list_devices_extra_texts_main_default, 0);
    lv_style_set_text_color(&style_bluetooth_bt_list_devices_extra_texts_main_default, lv_color_hex(0x000000));
    lv_style_set_text_font(&style_bluetooth_bt_list_devices_extra_texts_main_default, &lv_font_montserratMedium_36);
    lv_style_set_text_opa(&style_bluetooth_bt_list_devices_extra_texts_main_default, 255);
    lv_style_set_radius(&style_bluetooth_bt_list_devices_extra_texts_main_default, 3);
    lv_style_set_transform_width(&style_bluetooth_bt_list_devices_extra_texts_main_default, 0);
    lv_style_set_bg_opa(&style_bluetooth_bt_list_devices_extra_texts_main_default, 255);
    lv_style_set_bg_color(&style_bluetooth_bt_list_devices_extra_texts_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_bluetooth_bt_list_devices_extra_texts_main_default, LV_GRAD_DIR_NONE);

    //Write codes bluetooth_bt_btn_scan
    ui->bluetooth_bt_btn_scan = lv_btn_create(ui->bluetooth);
    ui->bluetooth_bt_btn_scan_label = lv_label_create(ui->bluetooth_bt_btn_scan);
    lv_label_set_text(ui->bluetooth_bt_btn_scan_label, "Scan");
    lv_label_set_long_mode(ui->bluetooth_bt_btn_scan_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->bluetooth_bt_btn_scan_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->bluetooth_bt_btn_scan, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->bluetooth_bt_btn_scan_label, LV_PCT(100));
    lv_obj_set_pos(ui->bluetooth_bt_btn_scan, 252, 86);
    lv_obj_set_size(ui->bluetooth_bt_btn_scan, 100, 50);

    //Write style for bluetooth_bt_btn_scan, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_bt_btn_scan, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->bluetooth_bt_btn_scan, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->bluetooth_bt_btn_scan, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->bluetooth_bt_btn_scan, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_bt_btn_scan, 30, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->bluetooth_bt_btn_scan, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->bluetooth_bt_btn_scan, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->bluetooth_bt_btn_scan, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->bluetooth_bt_btn_scan, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->bluetooth_bt_btn_scan, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes bluetooth_arc_1
    ui->bluetooth_arc_1 = lv_arc_create(ui->bluetooth);
    lv_arc_set_mode(ui->bluetooth_arc_1, LV_ARC_MODE_NORMAL);
    lv_arc_set_range(ui->bluetooth_arc_1, 0, 100);
    lv_arc_set_bg_angles(ui->bluetooth_arc_1, 0, 360);
    lv_arc_set_value(ui->bluetooth_arc_1, 0);
    lv_arc_set_rotation(ui->bluetooth_arc_1, 90);
    lv_obj_set_style_arc_rounded(ui->bluetooth_arc_1, 0,  LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(ui->bluetooth_arc_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_pos(ui->bluetooth_arc_1, 0, 0);
    lv_obj_set_size(ui->bluetooth_arc_1, 460, 460);

    //Write style for bluetooth_arc_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->bluetooth_arc_1, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->bluetooth_arc_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for bluetooth_arc_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_arc_width(ui->bluetooth_arc_1, 12, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui->bluetooth_arc_1, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(ui->bluetooth_arc_1, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for bluetooth_arc_1, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->bluetooth_arc_1, 0, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui->bluetooth_arc_1, 0, LV_PART_KNOB|LV_STATE_DEFAULT);

    //The custom code of bluetooth.


    //Update current screen layout.
    lv_obj_update_layout(ui->bluetooth);

}
