/* ecu_data.h — OBD Scanner 已验证数据汇总
 * 车辆: Mazda EZ-6 / 深蓝 SL03 增程版
 * 适配器: ELM327 v2.3 (蓝牙)
 * 开发板: ESP32-S3-Touch-AMOLED-1.75
 */

#ifndef ECU_DATA_H
#define ECU_DATA_H

/* ================================================================
 *  一、标准 OBD PID (已通过 HPCM 7E8 验证)
 * ================================================================ */

#define PID_RPM              "010C"   // (A*256+B)/4        [0-16383 RPM]
#define PID_SPEED            "010D"   // A                  [0-255 km/h]
#define PID_COOLANT          "0105"   // A-40               [-40-215 C]
#define PID_OIL_TEMP         "013C"   // A-40               [-40-215 C]
#define PID_EPA_SOC          "015B"   // A*100/255          [0-100%]
#define PID_MIL              "0101"   // A:bit7=MIL,bit0-6=DTC count
#define PID_BATT_V           "0142"   // (A*256+B)*0.001    [0-65.535 V]
#define PID_DIST             "0131"   // A*256+B            [0-65535 km]
#define PID_MAP              "010B"   // A                  [0-255 kPa]
#define PID_IAT              "010F"   // A-40               [-40-215 C]
#define PID_THROTTLE         "0111"   // A*100/255          [0-100%]
#define PID_ABS_LOAD         "0143"   // (A*256+B)*100/255  [0-25700%]
#define PID_TORQUE_PCT       "0162"   // A-125              [-125-130%]
#define PID_TORQUE_REF       "0163"   // A*256+B            [0-65535 Nm]
#define PID_ODOMETER         "01A6"   // (A*2^24+B*2^16+C*2^8+D)/10 [km]  <-- 总里程(正确)

/* 以下 PID 返回 NO DATA (车辆不支持):
 * 015C Runtime, 015D FuelLevel, 0110 MAF
 * 0167 Odometer 返回错误值(21177km vs 实际8203km)
 */

/* ================================================================
 *  二、BMS DID (7A1 -> 7A9, ATSH7A1)
 * ================================================================ */

#define BMS_DID_MAX_CELL_V   0xF250   // (A*256+B)*0.001    [V]   e.g. 3.279V
#define BMS_DID_MIN_CELL_V   0xF251   // (A*256+B)*0.001    [V]   e.g. 3.276V
#define BMS_DID_MAX_TEMP     0xF252   // A-40               [C]   e.g. 18C
#define BMS_DID_MIN_TEMP     0xF253   // A-40               [C]   e.g. 17C
#define BMS_DID_MAX_POS      0xF254   // A                  [-]   e.g. 82
#define BMS_DID_MIN_POS      0xF255   // A                  [-]   e.g. 16
#define BMS_DID_ENERGY       0xF229   // A*256+B            [Wh]  e.g. 6009Wh
#define BMS_DID_TOTAL_V      0xF228   // (A*256+B)*0.1      [V]   e.g. 373.6V

/* BMS 扩展 DID 扫描结果:
 * F22A-F22F, F240-F243: 数值太小(0-39), 不是电流
 * F230-F239: 电流限制参数(345A/323A/112A/704A等), 不是实时电流
 */

/* ================================================================
 *  三、SGCM DID (7E7 -> 7EF, ATSH7E7)
 *  SGCM = 启动/发电控制模块 (Start/Generator Control Module)
 * ================================================================ */

#define SGCM_DID_RPM         0x0808   // A*256+B            [RPM] e.g. 1373 RPM
#define SGCM_DID_CURRENT_A   0x0809   // (A*256+B)*0.01      [A]   e.g. 10.23A (相电流A)
#define SGCM_DID_RATED       0x0812   // A*256+B            [-]   e.g. 10000 (额定转速上限)
#define SGCM_DID_CURRENT_B   0x080A   // (A*256+B)*0.01      [A]   e.g. 10.20A (相电流B)

/* 瞬时功率估算:
 *   电池功率(kW) = BMS_F228(V) * SGCM_0809(A) * 0.1 / 1000
 *   待机: 373.6V * 10.23A * 0.1 / 1000 = 0.38 kW (与实际 0.3kWh 吻合)
 *   0.1 = 交流相电流到直流母线电流的经验调制比
 */

/* ================================================================
 *  四、实验扫描记录 (未找到有效数据)
 * ================================================================ */

/* DMCM (7E3 -> 7EB): 全部返回 7F2231 (requestSequenceError)
 *   即使发送 10 03 扩展会话也失败 (7F1011)
 *   结论: DMCM 需要更复杂的 UDS 认证流程
 *
 * GPEU (7E6 -> 7EE): 全部返回 7F2231
 *   结论: 地址错误或需要会话初始化
 *
 * HPCM (7E0 -> 7E8) F100-F103: 全部返回 7F2231
 *   结论: HPCM 不支持 DID 读取
 *
 * BECM (7E4 -> 7EC) F220-F223: 全部返回 7F2211
 *   结论: BECM 不支持这些 DID
 */

/* ================================================================
 *  五、OBD 层面缺失的数据 & 替代方案
 * ================================================================ */

/* 缺失 1: 电机/IGBT 温度
 * 状态: 未找到直接来源
 * 替代: 使用 BMS_F252 电池温度(18C) 或 0105 冷却液温度(19C)
 */

/* 缺失 2: 电池总电流 (直流)
 * 状态: BMS 无直接电流 DID
 * 替代: SGCM_0809(10.23A) * 0.1 = ~1.02A (待机, 吻合)
 *       或 功率 = F228(V) * SGCM_0809(A) / 1000 = 电机侧功率
 */

/* 缺失 3: 燃油量 / 剩余油量里程
 * 状态: 015D 返回 NO DATA, 无其他燃油 PID
 * 替代: 通过 EPA_SOC / 平均能耗 估算
 */

/* 缺失 4: 当前时间
 * 状态: OBD 无时间 PID
 * 替代: 开发板 LC76G GPS 模块获取 UTC 时间
 */

/* 缺失 5: G-Force
 * 状态: OBD 无惯性传感器数据
 * 替代: 开发板 QMI8658 IMU 加速度计
 */

#endif
