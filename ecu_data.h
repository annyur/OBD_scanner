#ifndef ECU_DATA_H
#define ECU_DATA_H

/* ================================================================
 *  Vehicle ECU Configuration Data (Privacy-Scrubbed)
 *  Protocol: ISO 15765-4 CAN (11-bit ID, 500 kbaud)
 * ================================================================ */

#define ECU_PROTOCOL_NAME       "ISO 15765-4 CAN"
#define ECU_CAN_ID_TYPE         11
#define ECU_CAN_BAUDRATE        500000
#define ECU_CAN_ID_FORMAT       0

/* OBD-II 监测计数器（排放诊断计数，非隐私） */
#define OBDCOND_COUNT           7168
#define IGNCNTR_COUNT           18690
#define CATS1_COMP_COUNT        4608
#define CATS1_COND_COUNT        12544
#define CATS2_COMP_COUNT        18688
#define CATS2_COND_COUNT        0
#define O2S1_COMP_COUNT         0
#define O2S1_COND_COUNT         6656
#define O2S2_COMP_COUNT         18688
#define O2S2_COND_COUNT         0
#define EGRCOMP_COUNT           0
#define EGRCOND_COUNT           11520
#define AIRCOMP_COUNT           18688
#define AIRCOND_COUNT           0
#define EVAPCOMP_COUNT          0
#define EVAPCOND_COUNT          7936
#define SO2SCOMP1_COUNT         4608
#define SO2SCOND1_COUNT         15616
#define SO2SCOMP2_COUNT         18688
#define SO2SCOND2_COUNT         0
#define AFRCOMP1_COUNT          0
#define AFRCOND1_COUNT          0
#define AFRCOMP2_COUNT          0
#define AFRCOND2_COUNT          0
#define PFCCOMP1_COUNT          0
#define PFC1_COND_COUNT         0
#define PFCCOMP2_COUNT          0
#define PFC2_COND_COUNT         0

/* 燃油点火循环计数器 */
#define IGNITION_CYCLE_COUNT    256

/* ================================================================
 *  ECU List (CAN ID + 功能名称)
 * ================================================================ */
typedef struct {
    const char *can_id;
    const char *name;
} ecu_entry_t;

static const ecu_entry_t ECU_HPCM = { "7E8", "HPCM-HybridPwrCtrl" };
static const ecu_entry_t ECU_DMCM = { "7EB", "DMCM-DriveMotorCtrl" };
static const ecu_entry_t ECU_BECM = { "7EC", "BECM-B+EnergyCtrl" };
static const ecu_entry_t ECU_BSCM = { "7ED", "BSCM-BrakeSystem" };
static const ecu_entry_t ECU_VTMC = { "7EE", "VTMC-ThermalMgmt" };
static const ecu_entry_t ECU_SGCM = { "7EF", "SGCM-Start/GenCtrl" };

static const ecu_entry_t * const ECU_LIST[] = {
    &ECU_HPCM, &ECU_DMCM, &ECU_BECM, &ECU_BSCM, &ECU_VTMC, &ECU_SGCM, NULL
};

#define ECU_COUNT  6

/* ================================================================
 *  ECU 查找辅助函数
 * ================================================================ */
static inline const ecu_entry_t* get_ecu_by_index(int idx)
{
    if (idx < 0 || idx >= ECU_COUNT) return NULL;
    return ECU_LIST[idx];
}

static inline const ecu_entry_t* get_ecu_by_can_id(const char *can_id)
{
    for (int i = 0; i < ECU_COUNT; i++) {
        if (strcmp(ECU_LIST[i]->can_id, can_id) == 0) {
            return ECU_LIST[i];
        }
    }
    return NULL;
}

#endif
