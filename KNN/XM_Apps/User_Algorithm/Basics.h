#ifndef BASICS_H
#define BASICS_H

#include "xm_api.h"
#include "mti-630.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* --- 공통 enum / struct --- */
typedef enum {
    Record_OFF,
    Record_ON,
} RecordingState_t;

typedef enum {
    Enable_OFF,
    Enable_ON,
} Enable_t;

typedef enum {
    SmartAssist_OFF,
    SmartAssist_ON,
} SmartAssist_t;

typedef enum {
    NONE = 0,
    RSOS = 1,
    RSTS = 2,
    LSOS = 3,
    LSTS = 4,
} GaitMode_t;

typedef struct __attribute__((packed)) {
    uint16_t sof;
    uint16_t len;

    uint32_t loopCnt;
    uint8_t  h10Mode;
    uint8_t  h10AssistLevel;
    uint8_t  SmartAssist;

    float leftHipAngle,  rightHipAngle;
    float leftThighAngle, rightThighAngle;

    float leftHipTorque,  rightHipTorque;
    float leftHipMotorAngle, rightHipMotorAngle;

    float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
    float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
    float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
    float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;

    int32_t is_moving;
    int32_t hc_count;

    int32_t R_count_upeak;
    int32_t L_count_upeak;
    int32_t R_count_dpeak;
    int32_t L_count_dpeak;

    uint8_t tau_max_setting;

    uint8_t s_gait_mode;
    float   s_g_knn_conf;

    // KNN normalization / cycle period (debug/analysis)
    float T_swing_ms;
    float T_swing_SOS_ms;
    float T_swing_STS_ms;
    // Most recent swing times whose KNN confidence was exactly 1.0 (used for normalization).
    float T_swing_SOS_ms_conf1;
    float T_swing_STS_ms_conf1;
    float TswingRecording_ms;
    float s_vel_HC;
    float s_T_HC_s;
    float s_norm_vel_HC;
    float s_norm_T_HC;
    float s_scaling_X;
    float s_scaling_Y;

    // Adaptive parameters (debug/analysis)
    float s_t_gap_R_ms;
    float s_t_gap_L_ms;
    float s_hc_deg_thresh;
    float s_thres_up;
    float s_thres_down;

    uint16_t crc;
} SavingData_t;

// Wire-format safety checks:
// - 'crc' must be the last field.
// - 'len' is transmitted as sizeof(SavingData_t) (see FillAndSendSavingData).
// NOTE: Do NOT hard-code a specific size here unless your PC parser expects it.
#ifndef PACKING_ASSERTS_ENABLED
#define PACKING_ASSERTS_ENABLED
#endif

#ifdef PACKING_ASSERTS_ENABLED
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
    _Static_assert(offsetof(SavingData_t, crc) + sizeof(((SavingData_t*)0)->crc) == sizeof(SavingData_t),
                   "SavingData_t layout error: 'crc' must be the last field");
    #else
    typedef char SavingData_t_crc_must_be_last[
        ((offsetof(SavingData_t, crc) + sizeof(((SavingData_t*)0)->crc)) == sizeof(SavingData_t)) ? 1 : -1
    ];
    #endif
#endif

/* --- 공유 전역 변수 extern --- */
extern uint32_t      s_dataSaveLoopCnt;
extern SavingData_t  SavingData;
extern XmLogicLevel_t sync_signal;
extern XmLogicLevel_t sync_signal_pre;

extern XmBtnEvent_t XM_button1;
extern XmBtnEvent_t XM_button2;
extern XmBtnEvent_t XM_button3;

extern RecordingState_t Recording;
extern SmartAssist_t   SmartAssist;

extern bool xsensIMUenableRes;

// From KNN.c (debug/mode recognition)
extern volatile int s_is_moving;
extern volatile int hc_count;
extern volatile int R_count_upeak;
extern volatile int L_count_upeak;
extern volatile int R_count_dpeak;
extern volatile int L_count_dpeak;

// From KNN.c (button-controlled tau_max)
extern volatile int tau_max_setting;

// From KNN.c (KNN classification outputs)
extern volatile GaitMode_t s_gait_mode;
extern volatile float s_g_knn_conf;

// From KNN.c (KNN normalization/cycle debug)
extern volatile float T_swing_ms;
extern volatile float T_swing_SOS_ms;
extern volatile float T_swing_STS_ms;
extern volatile float T_swing_SOS_ms_conf1;
extern volatile float T_swing_STS_ms_conf1;
extern volatile float TswingRecording_ms;
extern volatile float s_vel_HC_dbg;
extern volatile float s_T_HC_s_dbg;
extern volatile float s_dbg_norm_vel_HC;
extern volatile float s_dbg_norm_T_HC;
extern volatile float s_dbg_scaling_X;
extern volatile float s_dbg_scaling_Y;

// From KNN.c (adaptive parameters; expose via volatile mirrors for telemetry)
extern volatile float s_dbg_t_gap_R_ms;
extern volatile float s_dbg_t_gap_L_ms;
extern volatile float s_dbg_hc_deg_thresh;
extern volatile float s_dbg_thres_up;
extern volatile float s_dbg_thres_down;

#define TRANSMIT_SOF  (0xAA55u)

/* --- 함수 프로토타입 --- */
void     UpdateXsensImuEnable(void);
void     FillAndSendSavingData(void);
uint16_t CalcCrc16(const uint8_t* data, uint32_t length);
void     UpdateRecordingState(void);
void     UpdateSmartAssistState(void);

#endif /* BASICS_H */
