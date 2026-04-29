/*  bluetooth_manager.cpp — v3.2 里程+扩展扫描版
 *  修复: mode_pid构造错误导致所有标准PID unhandled
 *  新增: 010B MAP, 010F IAT, 0111 Throttle, 0143 AbsLoad, 0162/0163 Torque
 *  新增: 01A6 Odometer 总里程 (4字节, 替代0167——0167返回值与实际里程不符)
 *  新增: SGCM(7E7) DID扫描 — 0808转速/0809相电流A/0812额定功率/080A相电流B
 *  实验: HPCM(7E0) IGBT温度 + BECM(7E4) 电池电流 + BMS(F210-F217)
 *  移除: 0110 MAF(NO DATA), HYB(7B1无效), 015C/015D(NO DATA)
 */
#include "bluetooth_manager.h"
#include "general_manager.h"
#include <Arduino.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define BT_MAX_DEVICES  12
#define BT_NAME_LEN     32
#define BT_ADDR_LEN     18
#define ICAR_SERVICE_UUID   "000018f0-0000-1000-8000-00805f9b34fb"
#define ICAR_READ_CHAR_UUID "00002af0-0000-1000-8000-00805f9b34fb"
#define ICAR_WRITE_CHAR_UUID "00002af1-0000-1000-8000-00805f9b34fb"

typedef struct { char name[BT_NAME_LEN]; char addr[BT_ADDR_LEN]; esp_ble_addr_type_t addr_type; } bt_device_t;
static bt_device_t s_devices[BT_MAX_DEVICES];
static int  s_device_count = 0;
static volatile bool s_scan_done = false, s_scanning = false;
static lv_ui *s_ui = NULL;
static app_switch_cb_t s_switch_cb = NULL;
static BLEScan *pBLEScan = NULL;
static Preferences bt_prefs;
static BLEClient *s_client = NULL;
static bool s_is_connected = false, s_ble_enabled = false;
static char s_conn_addr[BT_ADDR_LEN] = "", s_conn_name[BT_NAME_LEN] = "";
static int s_connecting_idx = -1, s_target_idx = -1, s_state = 0;
static bool s_auto_conn_after_scan = false;
static char s_auto_conn_name[BT_NAME_LEN] = "";
static BLERemoteCharacteristic *s_write_char = NULL, *s_read_char = NULL;
static char s_obd_rx_buf[512] = "";
static bool s_obd_rx_ready = false;
static int s_init_step = 0;
static bool s_init_done = false;
static uint32_t s_init_last_ms = 0;

/* ---- PID轮询状态 ---- */
#define PID_COUNT 15
static const char* s_pid_cmds[PID_COUNT] = {
    "010C",     // 0: RPM
    "010D",     // 1: Speed
    "0105",     // 2: Coolant Temp
    "013C",     // 3: Oil Temp
    "015B",     // 4: EPA Hybrid Battery SOC
    "0101",     // 5: MIL / DTC count
    "0142",     // 6: Battery Voltage
    "0131",     // 7: Distance (since codes cleared)
    "010B",     // 8: MAP 进气歧管压力
    "010F",     // 9: IAT 进气温度
    "0111",     // 10: Throttle 节气门位置
    "0143",     // 11: Absolute Load
    "0162",     // 12: Actual Engine Torque %
    "0163",     // 13: Engine Reference Torque
    "01A6",     // 14: Odometer 总里程 (4字节: (A×2^24+B×2^16+C×2^8+D)/10 km)
};
static const char* s_pid_names[PID_COUNT] = {
    "RPM", "Speed", "Coolant", "OilTemp", "EPA_SOC", "MIL", "BattV", "Dist",
    "MAP", "IAT", "Throttle", "AbsLoad", "Torque%", "RefTorque", "Odometer"
};
static uint32_t s_last_pid_ms = 0;
static int s_pid_idx = 0;
static bool s_pid_sent = false;

/* 缓存数据 */
static int  s_cached_rpm = 0, s_cached_speed = 0, s_cached_coolant = -40;
static int  s_cached_oil = -40;
static float s_cached_epa_soc = 0;
static int  s_cached_mil_on = 0, s_cached_dtc_count = 0;
static float s_cached_batt_v = 0;
static int  s_cached_dist = 0;       // km
static int  s_cached_map = 0;        // kPa
static int  s_cached_iat = -40;      // °C
// static float s_cached_maf = 0;    // MAF 0110 返回 NO DATA, 已移除
static int  s_cached_throttle = 0;   // %
static float s_cached_abs_load = 0;  // %
static int  s_cached_torque_pct = 0; // % (实际扭矩百分比, 原始值A-125)
static int  s_cached_torque_ref = 0; // Nm
static float s_cached_odometer = 0;  // km (0167 总里程, 3字节公式)

/* ---- SGCM 数据 (7E7 启动/发电控制模块) ---- */
static int  s_cached_sgcm_rpm = 0;      // DID 0808
static float s_cached_sgcm_current = 0; // DID 0809 (A, 待标定)
static int  s_cached_sgcm_rated = 0;    // DID 0812
static float s_cached_sgcm_volt = 0;    // DID 080A 相电流B (复用变量名, 0.01A/bit)

/* ---- 电源/能耗数据 ---- */
static float s_cached_power_kw = 0;   // 电机功率估算

/* ================================================================
 *  ECU DID 扫描配置
 * ================================================================ */
typedef void (*did_parser_t)(uint16_t did, const uint8_t* d, int len);
struct EcuConfig {
    const char* label;
    uint16_t    tx_addr, rx_addr;
    const uint16_t* dids;
    int did_count;
    did_parser_t parser;
    bool needs_session;  // true = 需要先发送10 03进入扩展诊断会话
};
/* ---- 生产用 DID 列表 ---- */
static const uint16_t s_bms_dids[] = {0xF250,0xF251,0xF252,0xF253,0xF254,0xF255,0xF229,0xF228};
static const uint16_t s_sgcm_dids[] = {0x0808,0x0809,0x0812,0x080A};

/* ---- 实验扫描 DID 列表 (需要车上验证) ---- */
/* HPCM(混动动力总成): 7E0→7E8, 扫描F100-F103找IGBT/电机温度 */
static const uint16_t s_hpcm_scan_dids[] = {0xF100,0xF101,0xF102,0xF103};
/* BECM(电池能量控制): 7E4→7EC, 扫描F220-F223找电池电流 */
static const uint16_t s_becm_scan_dids[] = {0xF220,0xF221,0xF222,0xF223};
/* 最终扫描: BMS F210-F217 (最后可能包含电流的区间) */
static const uint16_t s_bms_scan_dids[] = {0xF210,0xF211,0xF212,0xF213,0xF214,0xF215,0xF216,0xF217};

static void parse_bms_did(uint16_t did,const uint8_t*d,int len);
static void parse_sgcm_did(uint16_t did,const uint8_t*d,int len);
static void parse_hpcm_scan_did(uint16_t did,const uint8_t*d,int len);  // 实验: HPCM IGBT温度
static void parse_becm_scan_did(uint16_t did,const uint8_t*d,int len);  // 实验: BECM电池电流
static void parse_bms_scan_did(uint16_t did,const uint8_t*d,int len);   // 实验: BMS F210-F217

/* 
 * ECU扫描列表 —— 【全开扫描模式】
 * 生产使用时只保留前2个(BMS+SGCM)，注释掉后面的扫描项
 */
static const EcuConfig s_ecu_list[] = {
    {"BMS",      0x7A1, 0x7A9, s_bms_dids, 8, parse_bms_did, false},
    {"SGCM",     0x7E7, 0x7EF, s_sgcm_dids, 4, parse_sgcm_did, false},
    {"HPCM",     0x7E0, 0x7E8, s_hpcm_scan_dids, 4, parse_hpcm_scan_did, false},   // 扫描: IGBT温度
    {"BECM",     0x7E4, 0x7EC, s_becm_scan_dids, 4, parse_becm_scan_did, false},   // 扫描: 电池电流
    {"BMS_SCAN", 0x7A1, 0x7A9, s_bms_scan_dids, 8, parse_bms_scan_did, false},      // 扫描: F210-F217
};
#define ECU_COUNT (sizeof(s_ecu_list)/sizeof(s_ecu_list[0]))

static bool s_in_burst = false;
static int s_burst_ecu = 0, s_burst_did = 0, s_burst_phase = 0;
static uint32_t s_burst_cmd_ms = 0, s_burst_last_ms = 0;

/* ================================================================ */
static void refresh_device_list(void);
static void start_scan(void);
static void do_connect(void);
static void do_auto_reconnect(void);
static void do_disconnect(bool full_cleanup);

static void save_ble_state(void){bt_prefs.begin("bt_prefs",false);bt_prefs.putBool("enabled",s_ble_enabled);bt_prefs.end();}
static void load_ble_state(void){bt_prefs.begin("bt_prefs",true);s_ble_enabled=bt_prefs.getBool("enabled",true);bt_prefs.end();}
static void save_last_device(const char*a,const char*n,uint8_t t){bt_prefs.begin("bt_prefs",false);bt_prefs.putString("last_addr",a);bt_prefs.putString("last_name",n);bt_prefs.putUChar("last_type",t);bt_prefs.end();}
static bool load_last_device(char*a,char*n,uint8_t*t){bt_prefs.begin("bt_prefs",true);String A=bt_prefs.getString("last_addr",""),N=bt_prefs.getString("last_name","");uint8_t T=bt_prefs.getUChar("last_type",BLE_ADDR_TYPE_PUBLIC);bt_prefs.end();if(A.length()==0)return false;strncpy(a,A.c_str(),BT_ADDR_LEN-1);a[BT_ADDR_LEN-1]=0;strncpy(n,N.c_str(),BT_NAME_LEN-1);n[BT_NAME_LEN-1]=0;*t=T;return true;}

class BTClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*){s_is_connected=true;s_connecting_idx=-1;Serial.println("[BT] onConnect");}
    void onDisconnect(BLEClient*){s_is_connected=false;s_conn_addr[0]=0;s_conn_name[0]=0;s_write_char=NULL;s_read_char=NULL;Serial.println("[BT] onDisconnect");}
};
static void on_notify(BLERemoteCharacteristic*,uint8_t*p,size_t l,bool){
    if(!l)return;size_t c=strlen(s_obd_rx_buf);size_t a=sizeof(s_obd_rx_buf)-1-c;
    size_t n=l<a?l:a;if(n){memcpy(s_obd_rx_buf+c,p,n);s_obd_rx_buf[c+n]=0;}s_obd_rx_ready=true;
}

static bool discover_services(void){
    if(!s_client||!s_client->isConnected())return false;
    BLERemoteService*svc=s_client->getService(ICAR_SERVICE_UUID);
    if(!svc){Serial.println("[BT-ELM] Svc missing");return false;}
    s_write_char=svc->getCharacteristic(ICAR_WRITE_CHAR_UUID);
    s_read_char=svc->getCharacteristic(ICAR_READ_CHAR_UUID);
    if(!s_write_char||!s_read_char){Serial.println("[BT-ELM] Char missing");return false;}
    if(s_read_char->canNotify())s_read_char->registerForNotify(on_notify);
    s_obd_rx_buf[0]=0;s_obd_rx_ready=false;return true;
}

/* ================================================================
 *  工具函数: 从十六进制字符串提取字节 (不跨帧)
 * ================================================================ */
static int hex_bytes_from_str(const char* hex_str, uint8_t* out, int max_out)
{
    int len = strlen(hex_str);
    int count = 0;
    for (int i = 0; i < len - 1 && count < max_out; i += 2) {
        char h = hex_str[i], l = hex_str[i+1];
        if (!isxdigit(h) || !isxdigit(l)) break;
        char buf[3] = {h, l, '\0'};
        out[count++] = (uint8_t)strtol(buf, NULL, 16);
    }
    return count;
}

/* 从CAN帧中提取数据字节 (跳过3字符CAN ID)
 * 帧格式: "7E804410C0000" 或 "7E8 04 41 0C 00 00"
 * 返回数据部分的起始指针和长度
 */
static bool extract_frame_data(const char* frame, uint8_t* data_out, int* data_len_out, int max_len)
{
    if (!frame || strlen(frame) < 6) return false;
    
    // 复制帧以便处理
    char buf[64];
    strncpy(buf, frame, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    
    // 移除所有空格
    char compact[64];
    int ci = 0;
    for (int i = 0; buf[i] && ci < 60; i++) {
        if (buf[i] != ' ') compact[ci++] = buf[i];
    }
    compact[ci] = '\0';
    
    int clen = strlen(compact);
    if (clen < 6) return false;
    
    // 前3字符是CAN ID, 第4字符开始是数据
    const char* data_hex = compact + 3;
    int data_hex_len = clen - 3;
    
    // 某些适配器输出4字符CAN ID (如 "7E80"), 需要检测
    if (data_hex_len > 0 && isxdigit(compact[3])) {
        // 正常3字符ID格式: 7E8 + 数据
        // 但数据部分可能以DLC开头(1字节)或直接是数据
        // 尝试直接全部转换
        int n = hex_bytes_from_str(data_hex, data_out, max_len);
        if (n >= 2) {
            *data_len_out = n;
            return true;
        }
    }
    return false;
}

/* ================================================================
 *  PID 解析 — 帧分割方式 (修复多ECU响应横跳)
 * ================================================================ */
static bool parse_pid_response(const char* rx, int pid_idx)
{
    if (!rx || !rx[0]) return false;
    
    const char* pid_str = s_pid_cmds[pid_idx];
    uint8_t mode = 0x41;
    if (pid_str[0] == '0' && pid_str[1] == '2') mode = 0x42;
    else if (pid_str[0] == '0' && pid_str[1] == '3') mode = 0x43;
    
    // 构建要搜索的模式+PID字符串 (如 "410C")
    char mode_pid[8];
    snprintf(mode_pid, sizeof(mode_pid), "%02X%c%c", mode, pid_str[2], pid_str[3]);
    
    // 按空格分割多ECU响应
    char buf[512];
    strncpy(buf, rx, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    
    char* tok = strtok(buf, " \r\n>");
    while (tok) {
        if (strlen(tok) < 8) { tok = strtok(NULL, " \r\n>"); continue; }
        
        // 在该帧中搜索 mode+pid
        char* mp = strstr(tok, mode_pid);
        if (!mp) { tok = strtok(NULL, " \r\n>"); continue; }
        
        // 找到匹配的帧, 提取数据字节
        // 帧格式: CANID + [DLC] + mode_pid + data
        // 例如: 7E804410C0000  -> CANID=7E8, DLC=04, mode_pid=410C, data=0000
        // mp 指向 mode_pid 的开头, 数据在 mp+4 之后
        
        uint8_t bytes[8] = {0};
        int bc = hex_bytes_from_str(mp + 4, bytes, 8);
        
        bool found = false;
        switch (pid_idx) {
            case 0: // 010C RPM: (A*256+B)/4
                if (bc >= 2) {
                    s_cached_rpm = (((uint16_t)bytes[0] << 8) | bytes[1]) / 4;
                    found = true;
                    Serial.printf("[OBD] 010C RPM=%d\n", s_cached_rpm);
                }
                break;
            case 1: // 010D Speed: A km/h
                if (bc >= 1) {
                    s_cached_speed = bytes[0];
                    found = true;
                    Serial.printf("[OBD] 010D Speed=%dkm/h\n", s_cached_speed);
                }
                break;
            case 2: // 0105 Coolant: A-40
                if (bc >= 1) {
                    s_cached_coolant = (int)bytes[0] - 40;
                    found = true;
                    Serial.printf("[OBD] 0105 Coolant=%dC\n", s_cached_coolant);
                }
                break;
            case 3: // 013C Oil Temp: A-40
                if (bc >= 1) {
                    s_cached_oil = (int)bytes[0] - 40;
                    found = true;
                    Serial.printf("[OBD] 013C OilTemp=%dC\n", s_cached_oil);
                }
                break;
            case 4: { // 015B EPA SOC: A*100/255
                if (bc >= 1) {
                    s_cached_epa_soc = bytes[0] * 100.0f / 255.0f;
                    found = true;
                    Serial.printf("[OBD] 015B EPA_SOC=%.1f%%\n", s_cached_epa_soc);
                }
                break;
            }
            case 5: { // 0101 MIL Status
                if (bc >= 2) {
                    s_cached_mil_on = (bytes[0] >> 7) & 1;
                    s_cached_dtc_count = bytes[0] & 0x7F;
                    found = true;
                    Serial.printf("[OBD] 0101 MIL=%d DTCs=%d\n", s_cached_mil_on, s_cached_dtc_count);
                }
                break;
            }
            case 6: { // 0142 Battery Voltage: (A*256+B)*0.001
                if (bc >= 2) {
                    s_cached_batt_v = ((uint16_t)bytes[0] << 8 | bytes[1]) * 0.001f;
                    found = true;
                    Serial.printf("[OBD] 0142 BattV=%.2fV\n", s_cached_batt_v);
                }
                break;
            }
            case 7: { // 0131 Distance: A*256+B km
                if (bc >= 2) {
                    s_cached_dist = ((uint16_t)bytes[0] << 8) | bytes[1];
                    found = true;
                    Serial.printf("[OBD] 0131 Dist=%dkm\n", s_cached_dist);
                }
                break;
            }
            case 8: { // 010B MAP: A kPa
                if (bc >= 1) {
                    s_cached_map = bytes[0];
                    found = true;
                    Serial.printf("[OBD] 010B MAP=%dkPa\n", s_cached_map);
                }
                break;
            }
            case 9: { // 010F IAT: A-40
                if (bc >= 1) {
                    s_cached_iat = (int)bytes[0] - 40;
                    found = true;
                    Serial.printf("[OBD] 010F IAT=%dC\n", s_cached_iat);
                }
                break;
            }
            case 10: { // 0111 Throttle: A*100/255 %
                if (bc >= 1) {
                    s_cached_throttle = (int)(bytes[0] * 100.0f / 255.0f);
                    found = true;
                    Serial.printf("[OBD] 0111 Throttle=%d%%\n", s_cached_throttle);
                }
                break;
            }
            case 11: { // 0143 Absolute Load: (A*256+B)*100/255 %
                if (bc >= 2) {
                    s_cached_abs_load = (((uint16_t)bytes[0] << 8) | bytes[1]) * 100.0f / 255.0f;
                    found = true;
                    Serial.printf("[OBD] 0143 AbsLoad=%.1f%%\n", s_cached_abs_load);
                }
                break;
            }
            case 12: { // 0162 Actual Torque: A-125 %
                if (bc >= 1) {
                    s_cached_torque_pct = (int)bytes[0] - 125;
                    found = true;
                    Serial.printf("[OBD] 0162 TorquePct=%d%%\n", s_cached_torque_pct);
                }
                break;
            }
            case 13: { // 0163 Reference Torque: (A*256+B) Nm
                if (bc >= 2) {
                    s_cached_torque_ref = ((uint16_t)bytes[0] << 8) | bytes[1];
                    found = true;
                    Serial.printf("[OBD] 0163 RefTorque=%dNm\n", s_cached_torque_ref);
                }
                break;
            }
            case 14: { // 01A6 Odometer: (A×2^24+B×2^16+C×2^8+D)/10 km (4字节)
                if (bc >= 4) {
                    uint32_t raw = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8) | bytes[3];
                    s_cached_odometer = raw / 10.0f;
                    found = true;
                    Serial.printf("[OBD] 01A6 Odometer=%.1fkm\n", s_cached_odometer);
                }
                break;
            }
        }
        
        if (found) return true;
        // 继续搜索下一个帧(多个ECU可能都回复了同一PID)
        tok = strtok(NULL, " \r\n>");
    }
    return false;
}

/* ================================================================
 *  SGCM DID 解析（7E7 启动/发电控制模块）
 * ================================================================ */
static void parse_sgcm_did(uint16_t did, const uint8_t* d, int len){
    switch(did){
        case 0x0808: // 推断: 发电机/启电机转速
            if(len>=2){
                s_cached_sgcm_rpm = ((uint16_t)d[0]<<8)|d[1];
                Serial.printf("[SGCM] 0808 转速=%d RPM\n", s_cached_sgcm_rpm);
            }
            break;
        case 0x0809: // 推断: 相电流 (量纲待验证)
            if(len>=2){
                uint16_t raw = ((uint16_t)d[0]<<8)|d[1];
                s_cached_sgcm_current = raw * 0.01f;
                Serial.printf("[SGCM] 0809 电流=%.2f A (raw=0x%04X)\n", s_cached_sgcm_current, raw);
            }
            break;
        case 0x0812: // 推断: 额定/最大转速上限 (固定值10000)
            if(len>=2){
                s_cached_sgcm_rated = ((uint16_t)d[0]<<8)|d[1];
                Serial.printf("[SGCM] 0812 额定值=%d\n", s_cached_sgcm_rated);
            }
            break;
        case 0x080A: // 推断: 另一路相电流 (与0809构成三相电流中的两路)
            if(len>=2){
                uint16_t raw = ((uint16_t)d[0]<<8)|d[1];
                s_cached_sgcm_volt = raw * 0.01f;  // 复用缓存变量名, 实际含义是电流B
                Serial.printf("[SGCM] 080A 电流B=%.2f A (raw=0x%04X)\n", s_cached_sgcm_volt, raw);
            }
            break;
        default:{
            Serial.printf("[SGCM] 0x%04X raw=0x",did);
            for(int i=0;i<len;i++)Serial.printf("%02X",d[i]);
            Serial.println();
        }
    }
}

/* ================================================================
 *  实验扫描 DID 解析器 —— 只打印原始值, 等人工标定含义
 * ================================================================ */
/* ---- HPCM扫描: 找IGBT/电机温度 ----
 * GPEU在7E6上失败, IGBT温度可能集成在HPCM(7E8)中
 * 正常IGBT温度: 冷车≈环境温度, 行驶50-80C, 高负荷90-110C
 */
static void parse_hpcm_scan_did(uint16_t did, const uint8_t* d, int len){
    Serial.printf("[HPCM-SCAN] DID=0x%04X len=%d raw=0x", did, len);
    for(int i=0;i<len;i++) Serial.printf("%02X",d[i]);
    Serial.println();
    if(len==1){
        int t_raw=(int)d[0];
        Serial.printf("  -> [1字节] 原始=%d  温度(-40)=%dC  温度(-273)=%dC\n",
                       t_raw, t_raw-40, t_raw-273);
    }else if(len==2){
        uint16_t u=((uint16_t)d[0]<<8)|d[1];
        int16_t s=(int16_t)u;
        Serial.printf("  -> [2字节] 无符号=%u  有符号=%d  温度(-40)=%dC\n", u, s, s-40);
    }
}

/* ---- BECM扫描: 找Battery Total Current ----
 * BECM=电池能量控制模块, 7E4→7EC
 * 可能包含直接的电池直流电流数据
 */
static void parse_becm_scan_did(uint16_t did, const uint8_t* d, int len){
    Serial.printf("[BECM-SCAN] DID=0x%04X len=%d raw=0x", did, len);
    for(int i=0;i<len;i++) Serial.printf("%02X",d[i]);
    Serial.println();
    if(len==2){
        uint16_t u=((uint16_t)d[0]<<8)|d[1];
        int16_t s=(int16_t)u;
        Serial.printf("  -> [2字节] 有符号=%d  电流(1A)=%dA  电流(0.1A)=%.1fA  电流(0.01A)=%.2fA  温度(-40)=%dC\n",
                       s, s, s*0.1f, s*0.01f, s-40);
    }else if(len==1){
        int8_t sb=(int8_t)d[0];
        Serial.printf("  -> [1字节] 有符号=%d  电流(1A)=%dA\n", sb, sb);
    }
}
/* ---- BMS扫描: 找Battery Total Current ----
 * CarScanner显示待机电流约1A, 对应功耗0.3kW (373V×1A≈373W)
 * 可能量纲: 有符号16bit × 0.01A 或 × 0.1A 或 × 1A
 */
static void parse_bms_scan_did(uint16_t did, const uint8_t* d, int len){
    Serial.printf("[BMS-SCAN] DID=0x%04X len=%d raw=0x", did, len);
    for(int i=0;i<len;i++) Serial.printf("%02X",d[i]);
    Serial.println();
    if(len==2){
        uint16_t u=((uint16_t)d[0]<<8)|d[1];
        int16_t s=(int16_t)u;
        Serial.printf("  -> [2字节] 有符号=%d  电流(1A)=%dA  电流(0.1A)=%.1fA  电流(0.01A)=%.2fA\n",
                       s, s, s*0.1f, s*0.01f);
    }else if(len==1){
        int8_t sb=(int8_t)d[0];
        Serial.printf("  -> [1字节] 无符号=%d  有符号=%d  电流(1A)=%dA\n", d[0], sb, sb);
    }
}

/* ================================================================
 *  BMS DID 解析（Car Scanner验证版）
 * ================================================================ */
static void parse_bms_did(uint16_t did,const uint8_t*d,int len){
    switch(did){
        case 0xF228:if(len>=2){float v=(((uint16_t)d[0]<<8)|d[1])*0.1f;Serial.printf("[BMS] F228 总电压=%.1fV\n",v);}break;
        case 0xF250:if(len>=2){float v=(((uint16_t)d[0]<<8)|d[1])*0.001f;Serial.printf("[BMS] F250 最高单体=%.3fV\n",v);}break;
        case 0xF251:if(len>=2){float v=(((uint16_t)d[0]<<8)|d[1])*0.001f;Serial.printf("[BMS] F251 最低单体=%.3fV\n",v);}break;
        case 0xF252:if(len>=1){int t=(int)d[0]-40;Serial.printf("[BMS] F252 最高温度=%dC\n",t);}break;
        case 0xF253:if(len>=1){int t=(int)d[0]-40;Serial.printf("[BMS] F253 最低温度=%dC\n",t);}break;
        case 0xF254:if(len>=1){Serial.printf("[BMS] F254 最高位置=%d\n",d[0]);}break;
        case 0xF255:if(len>=1){Serial.printf("[BMS] F255 最低位置=%d\n",d[0]);}break;
        case 0xF229:if(len>=2){uint16_t w=((uint16_t)d[0]<<8)|d[1];Serial.printf("[BMS] F229 剩余能量=%dWh\n",w);}break;
        default:{
            Serial.printf("[BMS] 0x%04X raw=0x",did);
            for(int i=0;i<len;i++)Serial.printf("%02X",d[i]);
            Serial.println();
        }
    }
}

static void parse_did_frames(const char* rx, uint16_t expect_did, uint16_t expect_rx, did_parser_t parser)
{
    char buf[512]; strncpy(buf, rx, sizeof(buf)-1); buf[sizeof(buf)-1] = '\0';
    char* tok = strtok(buf, " \r\n>");
    while (tok) {
        int len = strlen(tok);
        if (len < 8) { tok = strtok(NULL, " \r\n>"); continue; }
        char idstr[4] = {tok[0], tok[1], tok[2], '\0'};
        uint16_t cid = (uint16_t)strtol(idstr, NULL, 16);
        if (cid != expect_rx) { tok = strtok(NULL, " \r\n>"); continue; }
        const char* pay = tok + 3;
        int pl = strlen(pay);
        if (pl < 4 || pl % 2 != 0) { tok = strtok(NULL, " \r\n>"); continue; }
        uint8_t bytes[8]; int dlc = pl / 2;
        for (int i = 0; i < dlc; i++) { char b[3] = {pay[i*2], pay[i*2+1], '\0'}; bytes[i] = (uint8_t)strtol(b, NULL, 16); }
        if (dlc < 4) { tok = strtok(NULL, " \r\n>"); continue; }
        uint8_t pci = bytes[0];
        int data_len = pci - 3;
        if (data_len <= 0) { tok = strtok(NULL, " \r\n>"); continue; }
        if (bytes[1] == 0x62) {
            uint16_t rdid = ((uint16_t)bytes[2] << 8) | bytes[3];
            if (rdid == expect_did) { parser(rdid, &bytes[4], data_len); return; }
        }
        tok = strtok(NULL, " \r\n>");
    }
}

/* ================================================================
 *  ELM327 初始化
 * ================================================================ */
static void elm_tx(const char*cmd){
    if(!s_write_char)return;char b[32];snprintf(b,sizeof(b),"%s\r",cmd);
    s_write_char->writeValue(b);s_obd_rx_ready=false;s_obd_rx_buf[0]=0;s_init_last_ms=millis();
}
static bool elm_w(int t){return s_obd_rx_ready||(int)(millis()-s_init_last_ms)>t;}
static bool elm_ok(){return strstr(s_obd_rx_buf,"OK")!=nullptr;}
static bool elm_h(const char*p){return strstr(s_obd_rx_buf,p)!=nullptr;}
static void lrx(const char*tag,int s){
    if(!s_obd_rx_ready||!s_obd_rx_buf[0])return;char t[256];strncpy(t,s_obd_rx_buf,sizeof(t)-1);t[sizeof(t)-1]=0;
    for(int i=0;t[i];i++)if(t[i]=='\r'||t[i]=='\n')t[i]=' ';
    Serial.printf("%s RX(%02d): [%s]\n",tag,s,t);
}

static void elm_init_poll(void){
    if(s_init_done||!s_write_char||!s_is_connected)return;
    switch(s_init_step){
        case 0:elm_tx("ATZ");s_init_step=1;break;
        case 1:if(elm_w(2000)){lrx("[ELM]",1);if(elm_h("ELM327")||elm_h(">")){elm_tx("ATE0");s_init_step=2;}else{s_init_step=0;elm_tx("ATZ");}}break;
        case 2:if(elm_w(800)){lrx("[ELM]",2);elm_tx("ATH1");s_init_step=3;}break;
        case 3:if(elm_w(800)){lrx("[ELM]",3);elm_tx("ATSP0");s_init_step=4;}break;
        case 4:if(elm_w(800)){lrx("[ELM]",4);elm_tx("ATE0");s_init_step=5;}break;
        case 5:if(elm_w(800)){lrx("[ELM]",5);elm_tx("ATH1");s_init_step=6;}break;
        case 6:if(elm_w(800)){lrx("[ELM]",6);elm_tx("ATM0");s_init_step=7;}break;
        case 7:if(elm_w(800)){lrx("[ELM]",7);elm_tx("ATS0");s_init_step=8;}break;
        case 8:if(elm_w(800)){lrx("[ELM]",8);elm_tx("ATAT1");s_init_step=9;}break;
        case 9:if(elm_w(800)){lrx("[ELM]",9);elm_tx("ATAL");s_init_step=10;}break;
        case 10:if(elm_w(800)){lrx("[ELM]",10);elm_tx("ATST64");s_init_step=11;}break;
        case 11:if(elm_w(800)){lrx("[ELM]",11);elm_tx("0100");s_init_step=12;}break;
        case 12:if(!elm_w(3000))return;lrx("[ELM]",12);if(elm_h("SEARCHING")&&!elm_h("7E")){s_init_last_ms=millis();s_obd_rx_ready=false;return;}s_init_last_ms=millis();s_init_step=13;break;
        case 13:if((int)(millis()-s_init_last_ms)<800)return;Serial.println("[ELM] Init complete");s_init_done=true;s_burst_last_ms=millis();break;
        default:s_init_done=true;break;
    }
}

/* ================================================================
 *  BMS DID Burst（保留）
 * ================================================================ */
static void did_burst_poll(uint32_t now){
    if(!s_in_burst||!s_write_char)return;
    const EcuConfig*ecu=&s_ecu_list[s_burst_ecu];
    switch(s_burst_phase){
        case 0:{ // ATSH 设置目标地址
            if(!s_pid_sent){s_obd_rx_buf[0]=0;s_obd_rx_ready=false;
                char c[16];snprintf(c,sizeof(c),"ATSH%03X\r",ecu->tx_addr);
                s_write_char->writeValue(c);s_pid_sent=true;s_burst_cmd_ms=now;
                Serial.printf("[BURST] === %s ATSH%03X ===\n",ecu->label,ecu->tx_addr);
            }
            if(s_obd_rx_ready||(int)(now-s_burst_cmd_ms)>800){
                lrx("[ECU]",s_burst_ecu);
                // 如果需要会话模式，先发送10 03
                if(ecu->needs_session){
                    s_burst_phase=10;  // 跳到会话初始化阶段
                }else{
                    s_burst_phase=1;s_burst_did=0;
                }
                s_pid_sent=false;
            }
            break;
        }
        case 10:{ // 扩展诊断会话初始化 (10 03)
            if(!s_pid_sent){s_obd_rx_buf[0]=0;s_obd_rx_ready=false;
                s_write_char->writeValue("1003\r");
                s_pid_sent=true;s_burst_cmd_ms=now;
                Serial.printf("[BURST] %s 10 03 (扩展会话)\n",ecu->label);
            }
            if(s_obd_rx_ready||(int)(now-s_burst_cmd_ms)>1200){
                lrx("[ECU]",s_burst_ecu);
                if(s_obd_rx_ready){
                    if(strstr(s_obd_rx_buf,"50")){
                        Serial.printf("[BURST] %s 会话建立成功\n",ecu->label);
                    }else{
                        Serial.printf("[BURST] %s 会话建立失败(继续尝试)\n",ecu->label);
                    }
                }
                s_burst_phase=1;s_burst_did=0;s_pid_sent=false;
            }
            break;
        }
        case 1:{ // 读DID
            if(s_burst_did<ecu->did_count){
                if(!s_pid_sent){s_obd_rx_buf[0]=0;s_obd_rx_ready=false;
                    char c[16];snprintf(c,sizeof(c),"22%04X\r",ecu->dids[s_burst_did]);
                    s_write_char->writeValue(c);s_pid_sent=true;s_burst_cmd_ms=now;
                    Serial.printf("[ECU] %s DID:%04X\n",ecu->label,ecu->dids[s_burst_did]);
                }
                if(s_obd_rx_ready||(int)(now-s_burst_cmd_ms)>1200){
                    lrx("[ECU]",s_burst_did);
                    if(s_obd_rx_ready)parse_did_frames(s_obd_rx_buf,ecu->dids[s_burst_did],ecu->rx_addr,ecu->parser);
                    s_burst_did++;s_pid_sent=false;
                }
            }else{s_burst_ecu++;s_pid_sent=false;if(s_burst_ecu<(int)ECU_COUNT)s_burst_phase=0;else s_burst_phase=2;}
            break;
        }
        case 2:{ // ATSH7DF 恢复
            if(!s_pid_sent){s_obd_rx_buf[0]=0;s_obd_rx_ready=false;s_write_char->writeValue("ATSH7DF\r");s_pid_sent=true;s_burst_cmd_ms=now;}
            if(s_obd_rx_ready||(int)(now-s_burst_cmd_ms)>800){lrx("[OBD]",99);s_in_burst=false;s_burst_last_ms=now;s_pid_sent=false;s_burst_ecu=0;Serial.println("[BURST] === end ===");}
            break;
        }
    }
}

/* ================================================================
 *  主轮询 — 8个PID + BMS burst
 * ================================================================ */
static void obd_poll(uint32_t now){
    if(!s_is_connected||!s_write_char||!s_init_done)return;

    if(!s_in_burst&&!s_pid_sent&&(int)(now-s_burst_last_ms)>5000){
        s_in_burst=true;s_burst_phase=0;s_burst_ecu=0;s_burst_did=0;s_pid_sent=false;
        Serial.println("[BURST] === BMS start ===");
    }
    if(s_in_burst){did_burst_poll(now);return;}

    if(!s_pid_sent){
        if((int)(now-s_last_pid_ms)<300)return;
        s_last_pid_ms=now;s_obd_rx_buf[0]=0;s_obd_rx_ready=false;
        char cmd[8];snprintf(cmd,sizeof(cmd),"%s\r",s_pid_cmds[s_pid_idx]);
        s_write_char->writeValue(cmd);
        Serial.printf("[OBD] TX:%s (%s)\n",s_pid_cmds[s_pid_idx],s_pid_names[s_pid_idx]);
        s_pid_sent=true;return;
    }
    if(!s_obd_rx_ready&&(int)(now-s_last_pid_ms)<1000)return;

    if(s_obd_rx_ready&&s_obd_rx_buf[0]){
        if(parse_pid_response(s_obd_rx_buf,s_pid_idx)){
            // PID解析成功，更新UI
            general_manager_set_obd_data(s_cached_rpm, s_cached_speed);
            // Oil 显示真实机油温度(013C)，如果机油温度有效则优先用它
            if (s_cached_oil >= -40) {
                general_manager_set_oil(s_cached_oil);
            } else if (s_cached_coolant >= -40) {
                // 退而求其次用冷却液温度
                general_manager_set_oil(s_cached_coolant);
            }
        }else{
            char t[256];strncpy(t,s_obd_rx_buf,sizeof(t)-1);t[sizeof(t)-1]=0;
            for(int i=0;t[i];i++)if(t[i]=='\r'||t[i]=='\n')t[i]=' ';
            if(strlen(t)>3)Serial.printf("[OBD] unhandled:[%s]\n",t);
        }
    }
    s_pid_sent=false;
    s_pid_idx=(s_pid_idx+1)%PID_COUNT;
}

/* ================================================================
 *  BLE 连接/扫描/UI
 * ================================================================ */
static void do_disconnect(bool full){
    s_write_char=NULL;s_read_char=NULL;s_init_done=false;s_init_step=0;
    s_in_burst=false;s_burst_phase=0;s_pid_sent=false;
    if(s_client){if(s_client->isConnected())s_client->disconnect();if(full){delete s_client;s_client=NULL;}}
    s_is_connected=false;s_conn_addr[0]=0;s_conn_name[0]=0;s_connecting_idx=-1;
}
static void do_connect(void){
    if(s_target_idx<0||s_target_idx>=s_device_count){s_state=0;return;}
    const char*a=s_devices[s_target_idx].addr,*n=s_devices[s_target_idx].name;
    esp_ble_addr_type_t t=s_devices[s_target_idx].addr_type;
    do_disconnect(true);s_client=BLEDevice::createClient();s_client->setClientCallbacks(new BTClientCB());
    if(s_client->connect(BLEAddress(a),t)){
        strncpy(s_conn_addr,a,BT_ADDR_LEN-1);s_conn_addr[BT_ADDR_LEN-1]=0;
        strncpy(s_conn_name,n,BT_NAME_LEN-1);s_conn_name[BT_NAME_LEN-1]=0;
        save_last_device(a,n,(uint8_t)t);discover_services();
    }else{if(s_client){delete s_client;s_client=NULL;}}
    s_state=0;refresh_device_list();
}
static void do_auto_reconnect(void){
    char a[BT_ADDR_LEN],n[BT_NAME_LEN];uint8_t t=BLE_ADDR_TYPE_PUBLIC;
    if(!load_last_device(a,n,&t)){s_state=3;return;}
    do_disconnect(true);delay(100);s_client=BLEDevice::createClient();s_client->setClientCallbacks(new BTClientCB());
    if(s_client->connect(BLEAddress(a),(esp_ble_addr_type_t)t)){
        strncpy(s_conn_addr,a,BT_ADDR_LEN-1);s_conn_addr[BT_ADDR_LEN-1]=0;
        strncpy(s_conn_name,n,BT_NAME_LEN-1);s_conn_name[BT_NAME_LEN-1]=0;
        s_is_connected=true;discover_services();s_state=0;
    }else{if(s_client){delete s_client;s_client=NULL;}strncpy(s_auto_conn_name,n,BT_NAME_LEN-1);s_auto_conn_name[BT_NAME_LEN-1]=0;s_auto_conn_after_scan=true;s_state=3;}
    refresh_device_list();
}
class BTScanCB : public BLEAdvertisedDeviceCallbacks{
    void onResult(BLEAdvertisedDevice d)override{
        if(s_device_count>=BT_MAX_DEVICES)return;
        bt_device_t*dev=&s_devices[s_device_count];
        if(d.haveName()){strncpy(dev->name,d.getName().c_str(),BT_NAME_LEN-1);dev->name[BT_NAME_LEN-1]=0;}
        else{String A=d.getAddress().toString();snprintf(dev->name,BT_NAME_LEN,"[%s]",A.substring(0,8).c_str());}
        strncpy(dev->addr,d.getAddress().toString().c_str(),BT_ADDR_LEN-1);dev->addr[BT_ADDR_LEN-1]=0;
        dev->addr_type=d.getAddressType();s_device_count++;
    }
};
static void on_scan_done(BLEScanResults){s_scanning=false;s_scan_done=true;}
static void start_scan(void){
    if(!s_ui||!s_ble_enabled)return;
    if(s_scanning&&pBLEScan){pBLEScan->stop();s_scanning=false;delay(50);}
    s_device_count=0;s_scan_done=false;lv_obj_clean(s_ui->bluetooth_bt_list_devices);
    lv_obj_t*b=lv_list_add_btn(s_ui->bluetooth_bt_list_devices,LV_SYMBOL_BLUETOOTH,"Scanning...");lv_obj_clear_flag(b,LV_OBJ_FLAG_CLICKABLE);
    pBLEScan=BLEDevice::getScan();pBLEScan->setAdvertisedDeviceCallbacks(new BTScanCB(),true);
    pBLEScan->setActiveScan(true);pBLEScan->setInterval(100);pBLEScan->setWindow(99);
    pBLEScan->start(5,on_scan_done,false);s_scanning=true;s_state=0;
}
static void refresh_device_list(void){
    if(!s_ui||!s_ui->bluetooth_bt_list_devices)return;
    lv_obj_clean(s_ui->bluetooth_bt_list_devices);
    if(s_device_count==0){lv_obj_t*b=lv_list_add_btn(s_ui->bluetooth_bt_list_devices,LV_SYMBOL_BLUETOOTH,"No devices");lv_obj_clear_flag(b,LV_OBJ_FLAG_CLICKABLE);return;}
    for(int i=0;i<s_device_count;i++){
        char buf[64];
        if(s_connecting_idx==i)snprintf(buf,sizeof(buf),"... %s",s_devices[i].name);
        else if(s_is_connected&&strcmp(s_conn_addr,s_devices[i].addr)==0)snprintf(buf,sizeof(buf),"\xE2\x97\x8F %s",s_devices[i].name);
        else snprintf(buf,sizeof(buf),"  %s",s_devices[i].name);
        lv_obj_t*b=lv_list_add_btn(s_ui->bluetooth_bt_list_devices,LV_SYMBOL_BLUETOOTH,buf);
        lv_obj_add_event_cb(b,[](lv_event_t*e){
            int idx=(int)(intptr_t)lv_event_get_user_data(e);
            if(idx<0||idx>=s_device_count)return;
            if(s_is_connected&&strcmp(s_conn_addr,s_devices[idx].addr)==0){do_disconnect(true);refresh_device_list();return;}
            if(s_scanning&&pBLEScan){pBLEScan->stop();s_scanning=false;delay(50);}
            s_connecting_idx=idx;s_target_idx=idx;s_state=4;refresh_device_list();
        },LV_EVENT_CLICKED,(void*)(intptr_t)i);
        if(s_is_connected&&strcmp(s_conn_addr,s_devices[i].addr)==0)lv_obj_set_style_text_color(b,lv_color_hex(0x00e676),LV_PART_MAIN);
    }
}

static void on_back(lv_event_t*){if(s_switch_cb)s_switch_cb(APP_SCREEN_SETTING,false);}
static void on_sw(lv_event_t*e){lv_obj_t*sw=lv_event_get_target(e);bool on=lv_obj_has_state(sw,LV_STATE_CHECKED);if(on==s_ble_enabled)return;if(on){s_ble_enabled=true;save_ble_state();s_state=1;}else{if(s_scanning&&pBLEScan){pBLEScan->stop();s_scanning=false;}do_disconnect(true);s_ble_enabled=false;s_state=0;save_ble_state();}}
static void on_scan_btn(lv_event_t*){if(!s_ble_enabled){s_ble_enabled=true;save_ble_state();s_state=1;return;}s_state=3;}

void bluetooth_manager_init(lv_ui*ui){
    s_ui=ui;load_ble_state();
    if(ui->bluetooth_btn_back)lv_obj_add_event_cb(ui->bluetooth_btn_back,on_back,LV_EVENT_CLICKED,NULL);
    if(ui->bluetooth_bt_sw_enable){lv_obj_add_event_cb(ui->bluetooth_bt_sw_enable,on_sw,LV_EVENT_VALUE_CHANGED,NULL);
        if(s_ble_enabled)lv_obj_add_state(ui->bluetooth_bt_sw_enable,LV_STATE_CHECKED);else lv_obj_clear_state(ui->bluetooth_bt_sw_enable,LV_STATE_CHECKED);
    }
    if(ui->bluetooth_bt_btn_scan)lv_obj_add_event_cb(ui->bluetooth_bt_btn_scan,on_scan_btn,LV_EVENT_CLICKED,NULL);
    if(s_ble_enabled){s_ble_enabled=false;s_state=1;}
}
void bluetooth_manager_set_switch_cb(app_switch_cb_t cb){s_switch_cb=cb;}
void bluetooth_manager_enter(void){
    if(!s_ui||!s_ui->bluetooth_bt_list_devices)return;lv_obj_clean(s_ui->bluetooth_bt_list_devices);
    if(s_ui->bluetooth_bt_sw_enable){if(s_ble_enabled)lv_obj_add_state(s_ui->bluetooth_bt_sw_enable,LV_STATE_CHECKED);else lv_obj_clear_state(s_ui->bluetooth_bt_sw_enable,LV_STATE_CHECKED);}
    char a[BT_ADDR_LEN],n[BT_NAME_LEN];uint8_t t=BLE_ADDR_TYPE_PUBLIC;
    if(s_ble_enabled&&load_last_device(a,n,&t)){
        char b[64];if(s_is_connected&&strcmp(s_conn_addr,a)==0)snprintf(b,sizeof(b),"\xE2\x97\x8F %s",n);else snprintf(b,sizeof(b),"Last: %s",n);
        lv_obj_t*btn=lv_list_add_btn(s_ui->bluetooth_bt_list_devices,LV_SYMBOL_BLUETOOTH,b);lv_obj_clear_flag(btn,LV_OBJ_FLAG_CLICKABLE);
        if(s_is_connected)lv_obj_set_style_text_color(btn,lv_color_hex(0x00e676),LV_PART_MAIN);
    }else{lv_obj_t*btn=lv_list_add_btn(s_ui->bluetooth_bt_list_devices,LV_SYMBOL_BLUETOOTH,s_ble_enabled?"Press Scan":"Enable BT first");lv_obj_clear_flag(btn,LV_OBJ_FLAG_CLICKABLE);}
}
void bluetooth_manager_exit(void){if(s_scanning&&pBLEScan){pBLEScan->stop();s_scanning=false;}}
void bluetooth_manager_update(void){
    if(s_scan_done){s_scan_done=false;refresh_device_list();if(s_auto_conn_after_scan){s_auto_conn_after_scan=false;for(int i=0;i<s_device_count;i++){if(strcmp(s_devices[i].name,s_auto_conn_name)==0){s_connecting_idx=i;s_target_idx=i;s_state=4;refresh_device_list();break;}}s_auto_conn_name[0]=0;}}
    switch(s_state){case 1:BLEDevice::init("HybridHUD");s_ble_enabled=true;s_state=2;break;case 2:do_auto_reconnect();break;case 3:start_scan();break;case 4:do_connect();break;default:break;}
    if(s_write_char&&s_is_connected){if(!s_init_done)elm_init_poll();else obd_poll(millis());}
}
bool bluetooth_is_connected(void){return s_is_connected;}
const char* bluetooth_connected_name(void){return s_conn_name[0]?s_conn_name:nullptr;}
const char* bluetooth_connected_addr(void){return s_conn_addr[0]?s_conn_addr:nullptr;}