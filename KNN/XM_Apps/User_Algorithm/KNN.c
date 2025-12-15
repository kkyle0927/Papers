/**
 ******************************************************************************
 * @file    user_app.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 18, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api.h"      // 통합 API 헤더
#include "mti-630.h"
#include "Basics.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// KNN dataset
#include "KNN_ref_dataset_v1.h"

// Uses the same f-vector decoding approach as user_app.c (slot-based IIR)
#define F_VECTOR_NUM_MODES   10
#define F_VECTOR_BUFF_SIZE   10

// Control loop timebase (User_Loop is 2ms / 500Hz)
#define CTRL_DT_MS           2.0f
#define CTRL_FS_HZ           (1000.0f / CTRL_DT_MS)

static inline uint16_t MsToTicks_U16(float ms)
{
    if (ms <= 0.0f) return 0u;
    // round to nearest tick
    const float ticks_f = (ms / CTRL_DT_MS) + 0.5f;
    if (ticks_f >= 65535.0f) return 65535u;
    return (uint16_t)ticks_f;
}

static inline uint32_t MsToTicks_U32(float ms)
{
    if (ms <= 0.0f) return 0u;
    const float ticks_f = (ms / CTRL_DT_MS) + 0.5f;
    if (ticks_f >= 4294967295.0f) return 4294967295u;
    return (uint32_t)ticks_f;
}

typedef struct {
    float a0, a1, a2;
    float b1, b2;
    float gain_inv;
} FVecModeParam_t;

static const FVecModeParam_t kFVecModeParam[F_VECTOR_NUM_MODES] =
{
    { 0.006728316f, 0.013456632f, 0.006728316f, 1.980099502f, -0.980198510f, 1.0f },
    { 0.003380923f, 0.006761846f, 0.003380923f, 1.990024938f, -0.990049813f, 1.0f },
    { 0.002257699f, 0.004515399f, 0.002257699f, 1.993344426f, -0.993355500f, 1.0f },
    { 0.001356427f, 0.002712852f, 0.001356427f, 1.996003996f, -0.996007988f, 1.0f },
    { 0.000904852f, 0.001809705f, 0.000904852f, 1.997335110f, -0.997336885f, 1.0f },
    { 0.000678891f, 0.001357783f, 0.000678891f, 1.998001000f, -0.998001999f, 1.0f },
    { 0.000339643f, 0.000679288f, 0.000339643f, 1.999000250f, -0.999000500f, 1.0f },
    { 0.000226762f, 0.000453524f, 0.000226762f, 1.999333444f, -0.999333556f, 1.0f },
    { 0.000170083f, 0.000340167f, 0.000170083f, 1.999500062f, -0.999500125f, 1.0f },
    { 0.000135891f, 0.000271783f, 0.000135891f, 1.999600040f, -0.999600080f, 1.0f },
};

typedef struct {
    uint8_t  mode_idx;
    float    tau_max;
    uint16_t delay;
    uint32_t t_end;
    uint32_t time_stamp;
    float u, u_old1, u_old2;
    float tau, tau_old1, tau_old2;
    uint8_t is_full;
} FVecSlot_t;

static FVecSlot_t s_fvec_buf_LH[F_VECTOR_BUFF_SIZE];
static FVecSlot_t s_fvec_buf_RH[F_VECTOR_BUFF_SIZE];

// Manuscript f-vector (single): {mode_idx, tau_max, delay_ms, t_end_ms}
// Applies identically to RH and LH.
static const float kManuscriptFVec[4] = { 0.0f, 3.0f, 0.0f, 400.0f };


/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PUBLIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// --- Task State Machine Handle ---
static XmTsmHandle_t s_userHandle;

// 이 친구들은 이제 static 빼고 “정의”만 담당
uint32_t      s_dataSaveLoopCnt = 0;
SavingData_t  SavingData;
XmLogicLevel_t sync_signal      = XM_LOW;
XmLogicLevel_t sync_signal_pre  = XM_LOW;

XmBtnEvent_t XM_button1;
XmBtnEvent_t XM_button2;
XmBtnEvent_t XM_button3;

RecordingState_t Recording = Record_OFF;
SmartAssist_t    SmartAssist = SmartAssist_OFF;

bool xsensIMUenableRes = false;

// ---- USER_DEFINED_CTRL (ported) state ----
static float assist_level = 1.5f;

static bool Rflag_assist = false;
static bool Lflag_assist = false;
static bool Rstop_assist = false;
static bool Lstop_assist = false;

static uint32_t s_loop_count = 0;
static float s_Rdeg[3] = {0.0f, 0.0f, 0.0f};
static float s_Ldeg[3] = {0.0f, 0.0f, 0.0f};

// Filters (ported; timebase adjusted for CTRL_FS_HZ)
static const float s_fs = CTRL_FS_HZ;
static const float s_fc_theta = 10.0f;
static const float s_alpha_theta = 0.0591174f;
static const float s_fc_omega = 25.0f;
static const float s_alpha_omega = 0.1357552f;
static float s_y_prev_R = 0.0f;
static float s_y_prev_L = 0.0f;
static float s_w_prev_R = 0.0f;
static float s_w_prev_L = 0.0f;

static bool s_R_time_upcond = false;
static bool s_L_time_upcond = false;
static bool s_HC_time_upcond = false;
static bool s_HC_Rswing4upcond = false;
static bool s_HC_Lswing4upcond = false;

static int s_R_count_upeak = 0;
static int s_R_count_dpeak = 0;
static int s_L_count_upeak = 0;
static int s_L_count_dpeak = 0;
static int s_HC_count = 0;

// Timing parameters are in absolute time (ms), not ticks.
static float s_t_gap_ms = 400.0f;
static float s_thres_up = 10.0f;
static float s_thres_down = 10.0f;
static float s_R_lowpeak_val = 10.0f;
static float s_L_lowpeak_val = 10.0f;
static float s_R_uppeak_val = 50.0f;
static float s_L_uppeak_val = 50.0f;
static float s_var = 0.0f;

static float s_R_time_afterup_ms = 0.0f;
static float s_L_time_afterup_ms = 0.0f;
static float s_HC_time_after_ms = 0.0f;
static float s_swing_period_ms = 400.0f;

static float s_vel_HC = 0.0f;
static float s_R_swing_time_ms = 0.0f;
static float s_L_swing_time_ms = 0.0f;
static float s_T_HC_ms = 0.0f;
static float s_norm_vel_HC = 0.0f;
static float s_norm_T_HC = 0.0f;

static float s_scaling_X = 105.6426f;
static float s_scaling_Y = 835.8209f;

#ifndef KNN_REF_COUNT
#  define KNN_REF_COUNT    REF_COUNT
#endif
#ifndef KNN_REF_SPLIT
#  define KNN_REF_SPLIT    REF_TYPE1_COUNT
#endif
#ifndef KNN_K
#  define KNN_K            9
#endif

static int s_R_g_knn_label = 0;
static int s_L_g_knn_label = 0;
static float s_g_knn_conf = 0.0f;


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void Disconnected_Entry(void);
static void Disconnected_Loop(void);

static void Standby_Entry(void);
static void Standby_Loop(void);

static void Active_Entry(void);
static void Active_Loop(void);
static void Active_Exit(void);

// Gait recognition helpers (used in Active_Loop)
typedef struct {
    float Rdeg_filtered;
    float Ldeg_filtered;
    float wR_f;
    float wL_f;
    int   is_moving;
} GaitFeatures_t;

typedef struct {
    int  is_moving;
    bool r_lower_peak_event;
    bool l_lower_peak_event;
    bool r_uprise_after_lower;
    bool l_uprise_after_lower;
    bool r_stop_assist;
    bool l_stop_assist;
} GaitRecognitionResult_t;

static void GaitModeRecognition_Update(float rightThighDeg, float leftThighDeg,
                                       GaitFeatures_t* feat_out);
static void GaitModeRecognition_DetectEvents(const GaitFeatures_t* feat,
                                             GaitRecognitionResult_t* res_out);
static void adaptive_assist(const GaitRecognitionResult_t* rec,
                            bool* trig_R, bool* trig_L,
                            float* out_tau_R, float* out_tau_L);

static void FVecDecoder_InitSingle(FVecSlot_t buf[]);
static void FVecDecoder_Init(void);
static void FVecDecoder_TriggerSingle(FVecSlot_t buf[], const float f_vec[4]);
static float FVecDecoder_StepSingle(FVecSlot_t buf[]);
static void TriggerManuscriptFVecSingle(FVecSlot_t buf[], const float fvec[4]);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* =================================================================
 * [필수 구현 1] 초기화 함수 예시 (User_Setup)
 * - 전원 인가 후 딱 한 번 실행됩니다.
 * - TSM 생성, 변수 초기화, LED/Button 초기 설정 등을 수행합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Setup(void)
{
    s_userHandle = XM_TSM_Create(XM_STATE_OFF);

    // OFF 상태 등록
    XmStateConfig_t off_conf = {
        .id = XM_STATE_OFF,
        .on_entry = Disconnected_Entry,
        .on_loop  = Disconnected_Loop
    };
    XM_TSM_AddState(s_userHandle, &off_conf);

    // STANDBY 상태 등록
    XmStateConfig_t sb_conf = {
        .id = XM_STATE_STANDBY,
        .on_entry = Standby_Entry,
        .on_loop  = Standby_Loop
    };
    XM_TSM_AddState(s_userHandle, &sb_conf);

    // ACTIVE 상태 등록
    XmStateConfig_t act_conf = {
        .id = XM_STATE_ACTIVE,
        .on_entry = Active_Entry,
        .on_loop  = Active_Loop,
        .on_exit  = Active_Exit
    };
    XM_TSM_AddState(s_userHandle, &act_conf);

    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);

    // SYNC 초기값
    sync_signal = XM_LOW;
	sync_signal_pre = XM_LOW;

    FVecDecoder_Init();
}


/* =================================================================
 * [필수 구현 2] 반복 루프 함수 예시 (User_Loop)
 * - 2ms(500Hz) 주기로 계속 호출됩니다.
 * - 제어 알고리즘, TSM 실행 로직을 여기에 작성합니다.
 * - 내부 IPO(Input-Process-Output)모델이 적용되어 있습니다.
 * - xm_api_data.h를 통해 RxData에 대해 확인할 수 있습니다.
 * - XM_SetControlMode함수를 Setup에서 실시간 제어 모드 / 모니터링 모드를 선택해야 합니다. (기본 모니터링 모드)
 * - 모니터링 모드에서는 H10으로 제어 명령을 전송하지 않습니다. 
 * - 실시간 제어 모드에서는 H10으로 Torque input을 2ms 주기로 전송합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Loop(void)
{
	// CM 연결 상태를 최우선으로 확인하여, 연결이 끊겼을 경우 OFF 상태로 강제 전환합니다.
    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_OFF);
    }
    // TSM 실행 (현재 상태에 맞는 Loop 함수가 실행됨)
    XM_TSM_Run(s_userHandle);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* --- 상태별 동작 함수 정의 예시 --- */
static void Disconnected_Entry(void)
{
}

static void Disconnected_Loop(void)
{
    if (XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_STANDBY);
    }
}

static void Standby_Entry(void)
{
	// XSENS IMU 부팅 시간 확보 (전원 인가 후 최소 150ms 필요)
    HAL_Delay(150);
    XM_SetControlMode(XM_CTRL_MONITOR);
    UpdateXsensImuEnable();
}

static void Standby_Loop(void)
{
	UpdateRecordingState();
    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
    }
    UpdateXsensImuEnable();

}

static void Active_Entry(void)
{
	XM_SetControlMode(XM_CTRL_TORQUE);
    FVecDecoder_Init();
}

static void Active_Loop(void)
{
	UpdateRecordingState();
    
    // --- USER_DEFINED_CTRL: trigger manuscript-defined f-vector and decode/actuate in real-time ---
    bool trig_R = false, trig_L = false;
    float ignored_R = 0.0f, ignored_L = 0.0f;
    GaitFeatures_t feat;
    GaitRecognitionResult_t rec;
    GaitModeRecognition_Update(XM.status.h10.leftThighAngle, XM.status.h10.leftThighAngle, &feat);
    GaitModeRecognition_DetectEvents(&feat, &rec);
    adaptive_assist(&rec, &trig_R, &trig_L, &ignored_R, &ignored_L);

    if (trig_R && !Rstop_assist) {
        TriggerManuscriptFVecSingle(s_fvec_buf_RH, kManuscriptFVec);
    }
    if (trig_L && !Lstop_assist) {
        TriggerManuscriptFVecSingle(s_fvec_buf_LH, kManuscriptFVec);
    }

    float tau_R = FVecDecoder_StepSingle(s_fvec_buf_RH);
    float tau_L = FVecDecoder_StepSingle(s_fvec_buf_LH);
    XM_SetAssistTorqueRH(tau_R);
    XM_SetAssistTorqueLH(tau_L);
}

static void Active_Exit(void)
{

}



static inline float KNN_LPF(float x, float* y_prev, float alpha)
{
    *y_prev = (1.0f - alpha) * (*y_prev) + alpha * x;
    return *y_prev;
}

static int knn_2label_majority_ud(const float (*xy)[2], int n, int split,
                                  float x, float y, int k, float* conf_out)
{
    if (n <= 0) { if (conf_out) *conf_out = 0.0f; return 0; }

    int kk = k;
    if (kk < 1) kk = 1;
    else if (kk > KNN_K) kk = KNN_K;

    float   best_d2[KNN_K];
    uint8_t best_lab[KNN_K];
    int used = 0;

    for (int i = 0; i < kk; ++i) { best_d2[i] = 1e30f; best_lab[i] = 0; }

    for (int i = 0; i < n; ++i) {
        float dx = x - xy[i][0];
        float dy = y - xy[i][1];
        float d2 = dx*dx + dy*dy;
        uint8_t lab = (i < split) ? 1u : 2u;

        if (used < kk) {
            int j = used++;
            best_d2[j] = d2; best_lab[j] = lab;
            while (j > 0 && best_d2[j] < best_d2[j-1]) {
                float tmpd = best_d2[j-1]; best_d2[j-1] = best_d2[j]; best_d2[j] = tmpd;
                uint8_t tmpl = best_lab[j-1]; best_lab[j-1] = best_lab[j]; best_lab[j] = tmpl;
                --j;
            }
        } else if (d2 < best_d2[kk-1]) {
            int j = kk-1;
            best_d2[j] = d2; best_lab[j] = lab;
            while (j > 0 && best_d2[j] < best_d2[j-1]) {
                float tmpd = best_d2[j-1]; best_d2[j-1] = best_d2[j]; best_d2[j] = tmpd;
                uint8_t tmpl = best_lab[j-1]; best_lab[j-1] = best_lab[j]; best_lab[j] = tmpl;
                --j;
            }
        }
    }

    int c1 = 0, c2 = 0;
    for (int i = 0; i < used; ++i) {
        if (best_lab[i] == 1u) ++c1; else ++c2;
    }

    int label = (c1 >= c2) ? 1 : 2;
    if (conf_out) {
        int maxc = (c1 >= c2) ? c1 : c2;
        *conf_out = (used > 0) ? ((float)maxc / (float)used) : 0.0f;
    }
    return label;
}

static int update_standing_moving_flag_w_ud(float wR_metric, float wL_metric)
{
    const float V_ON_THRESH   = 40.0f;
    const float V_OFF_THRESH  = 15.0f;
    const int   ON_HOLD_TICKS = 10;
    const int   OFF_HOLD_TICKS= 50;

    typedef enum { GAIT_STANDING = 0, GAIT_MOVING = 1 } gait_state_t;
    static gait_state_t s_state = GAIT_STANDING;
    static int s_on_cnt = 0, s_off_cnt = 0;

    float v_metric = (wR_metric > wL_metric) ? wR_metric : wL_metric;
    int evidence_move  = (v_metric >= V_ON_THRESH);
    int evidence_stand = (v_metric <= V_OFF_THRESH);

    if (s_state == GAIT_STANDING) {
        if (evidence_move) {
            if (++s_on_cnt >= ON_HOLD_TICKS) {
                s_state = GAIT_MOVING; s_on_cnt = 0; s_off_cnt = 0;
            }
        } else {
            s_on_cnt = 0;
        }
    } else {
        if (evidence_stand) {
            if (++s_off_cnt >= OFF_HOLD_TICKS) {
                s_state = GAIT_STANDING; s_on_cnt = 0; s_off_cnt = 0;
            }
        } else {
            s_off_cnt = 0;
        }
    }

    return (s_state == GAIT_MOVING) ? 1 : 0;
}


static void GaitModeRecognition_Update(float rightThighDeg, float leftThighDeg,
                                      GaitFeatures_t* feat_out)
{
    if (feat_out) {
        memset(feat_out, 0, sizeof(*feat_out));
    }

    float Rdeg_filtered = KNN_LPF(rightThighDeg, &s_y_prev_R, s_alpha_theta);
    float Ldeg_filtered = KNN_LPF(leftThighDeg, &s_y_prev_L, s_alpha_theta);

    s_Rdeg[0] = s_Rdeg[1];
    s_Ldeg[0] = s_Ldeg[1];
    s_Rdeg[1] = s_Rdeg[2];
    s_Ldeg[1] = s_Ldeg[2];
    s_Rdeg[2] = Rdeg_filtered;
    s_Ldeg[2] = Ldeg_filtered;

    float wR = (s_Rdeg[2] - s_Rdeg[1]) * s_fs;
    float wL = (s_Ldeg[2] - s_Ldeg[1]) * s_fs;

    float wR_abs = fabsf(wR);
    float wL_abs = fabsf(wL);
    float wR_f = KNN_LPF(wR_abs, &s_w_prev_R, s_alpha_omega);
    float wL_f = KNN_LPF(wL_abs, &s_w_prev_L, s_alpha_omega);

    int is_moving = update_standing_moving_flag_w_ud(wR_f, wL_f);

    if (feat_out) {
        feat_out->Rdeg_filtered = Rdeg_filtered;
        feat_out->Ldeg_filtered = Ldeg_filtered;
        feat_out->wR_f = wR_f;
        feat_out->wL_f = wL_f;
        feat_out->is_moving = is_moving;
    }
}

static void GaitModeRecognition_DetectEvents(const GaitFeatures_t* feat,
                                             GaitRecognitionResult_t* res_out)
{
    static int s_prev_count_dpeak_R = 0;
    static int s_prev_count_dpeak_L = 0;
    static bool s_prev_Rflag_assist = false;
    static bool s_prev_Lflag_assist = false;

    if (res_out) {
        memset(res_out, 0, sizeof(*res_out));
    }

    s_loop_count++;
    s_R_time_afterup_ms += CTRL_DT_MS;
    s_L_time_afterup_ms += CTRL_DT_MS;
    s_HC_time_after_ms += CTRL_DT_MS;
    s_R_swing_time_ms += CTRL_DT_MS;
    s_L_swing_time_ms += CTRL_DT_MS;

    const int is_moving = (feat != NULL) ? feat->is_moving : 0;
    if (res_out) res_out->is_moving = is_moving;

    if (s_R_time_afterup_ms >= s_t_gap_ms) s_R_time_upcond = true;
    if (s_L_time_afterup_ms >= s_t_gap_ms) s_L_time_upcond = true;
    if (s_HC_time_after_ms >= s_t_gap_ms) s_HC_time_upcond = true;

    if (s_HC_time_upcond && (s_Rdeg[2] - s_Ldeg[2]) * (s_Rdeg[1] - s_Ldeg[1]) <= 0.0f &&
        s_Rdeg[2] > s_thres_down && s_Ldeg[2] > s_thres_down) {
        s_HC_time_upcond = false;
        s_HC_time_after_ms = 0.0f;
        s_HC_count++;

        if (s_Rdeg[2] - s_Rdeg[1] >= s_Ldeg[2] - s_Ldeg[1] && s_L_uppeak_val > 40.0f) {
            s_HC_Rswing4upcond = true;
            s_vel_HC = (s_Rdeg[2] - s_Rdeg[1]) / (CTRL_DT_MS * 0.001f);
            s_T_HC_ms = s_R_swing_time_ms;

            const float swing_period_s = s_swing_period_ms * 0.001f;
            s_norm_vel_HC = (s_vel_HC * swing_period_s) / s_scaling_X;
            s_norm_T_HC = ((s_T_HC_ms * 0.001f) / swing_period_s) / s_scaling_Y;

            s_R_g_knn_label = knn_2label_majority_ud(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT,
                                                    s_norm_vel_HC, s_norm_T_HC, KNN_K, &s_g_knn_conf);
            if (s_R_g_knn_label == 1) {
                Rstop_assist = true;
            }
        } else if (s_Ldeg[2] - s_Ldeg[1] >= s_Rdeg[2] - s_Rdeg[1] && s_R_uppeak_val > 40.0f) {
            s_HC_Lswing4upcond = true;
            s_vel_HC = (s_Ldeg[2] - s_Ldeg[1]) / (CTRL_DT_MS * 0.001f);
            s_T_HC_ms = s_L_swing_time_ms;

            const float swing_period_s = s_swing_period_ms * 0.001f;
            s_norm_vel_HC = (s_vel_HC * swing_period_s) / s_scaling_X;
            s_norm_T_HC = ((s_T_HC_ms * 0.001f) / swing_period_s) / s_scaling_Y;

            s_L_g_knn_label = knn_2label_majority_ud(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT,
                                                    s_norm_vel_HC, s_norm_T_HC, KNN_K, &s_g_knn_conf);
            if (s_L_g_knn_label == 1) {
                Lstop_assist = true;
            }
        }
    }

    // Upper peak detection
    if ((s_Rdeg[2] - s_Rdeg[1]) * (s_Rdeg[1] - s_Rdeg[0]) < s_var &&
        (s_Rdeg[2] - s_Rdeg[1]) < 0.0f &&
        s_Rdeg[2] >= s_thres_up && s_R_time_upcond) {
        s_R_uppeak_val = s_Rdeg[2];
        s_R_time_upcond = false;
        s_R_count_upeak++;
        s_swing_period_ms = s_R_swing_time_ms;
        s_R_time_afterup_ms = 0.0f;
        s_HC_Rswing4upcond = false;
    }

    if ((s_Ldeg[2] - s_Ldeg[1]) * (s_Ldeg[1] - s_Ldeg[0]) < s_var &&
        (s_Ldeg[2] - s_Ldeg[1]) < 0.0f &&
        s_Ldeg[2] >= s_thres_up && s_L_time_upcond) {
        s_L_uppeak_val = s_Ldeg[2];
        s_L_time_upcond = false;
        s_L_count_upeak++;
        s_swing_period_ms = s_L_swing_time_ms;
        s_L_time_afterup_ms = 0.0f;
        s_HC_Lswing4upcond = false;
    }

    // Lower peak detection
    if (s_Rdeg[2] > s_R_lowpeak_val + 10.0f) {
        s_R_lowpeak_val = s_thres_down;
    }
    if ((s_Rdeg[2]-s_Rdeg[1])*(s_Rdeg[1]-s_Rdeg[0]) < s_var &&
        (s_Rdeg[2] - s_Rdeg[1]) >= 0.0f &&
        s_Rdeg[2] <= s_R_lowpeak_val + 3.0f) {
        s_R_count_dpeak++;
        s_R_lowpeak_val = s_Rdeg[2];
        s_R_swing_time_ms = 0.0f;
        if (is_moving) {
            s_R_g_knn_label = 0;
            Rstop_assist = false;
            Rflag_assist = true;
        }
    }

    if (s_Ldeg[2] > s_L_lowpeak_val + 10.0f) {
        s_L_lowpeak_val = s_thres_down;
    }
    if ((s_Ldeg[2]-s_Ldeg[1])*(s_Ldeg[1]-s_Ldeg[0]) < s_var &&
        (s_Ldeg[2] - s_Ldeg[1]) >= 0.0f &&
        s_Ldeg[2] <= s_L_lowpeak_val + 3.0f) {
        s_L_count_dpeak++;
        s_L_lowpeak_val = s_Ldeg[2];
        s_L_swing_time_ms = 0.0f;
        if (is_moving) {
            s_L_g_knn_label = 0;
            Lstop_assist = false;
            Lflag_assist = true;
        }
    }

    if (res_out) {
        res_out->r_lower_peak_event = (s_R_count_dpeak != s_prev_count_dpeak_R);
        res_out->l_lower_peak_event = (s_L_count_dpeak != s_prev_count_dpeak_L);

        // Rising-after-lower should only fire once when the flag was armed and then the rise condition is met.
        const bool r_rise_now = (Rflag_assist && (s_Rdeg[2] > s_R_lowpeak_val + 5.0f));
        const bool l_rise_now = (Lflag_assist && (s_Ldeg[2] > s_L_lowpeak_val + 5.0f));
        res_out->r_uprise_after_lower = (r_rise_now && s_prev_Rflag_assist);
        res_out->l_uprise_after_lower = (l_rise_now && s_prev_Lflag_assist);

        res_out->r_stop_assist = Rstop_assist;
        res_out->l_stop_assist = Lstop_assist;
    }

    s_prev_count_dpeak_R = s_R_count_dpeak;
    s_prev_count_dpeak_L = s_L_count_dpeak;
    s_prev_Rflag_assist = Rflag_assist;
    s_prev_Lflag_assist = Lflag_assist;
}

static void adaptive_assist(const GaitRecognitionResult_t* rec,
                            bool* trig_R, bool* trig_L,
                            float* out_tau_R, float* out_tau_L)
{
    if (trig_R) *trig_R = false;
    if (trig_L) *trig_L = false;
    if (out_tau_R) *out_tau_R = 0.0f;
    if (out_tau_L) *out_tau_L = 0.0f;

    const bool r_stop = (rec != NULL) ? rec->r_stop_assist : false;
    const bool l_stop = (rec != NULL) ? rec->l_stop_assist : false;

    if (rec != NULL) {
        if (rec->r_uprise_after_lower) {
            if (trig_R) *trig_R = true;
            Rflag_assist = false;
        }
        if (rec->l_uprise_after_lower) {
            if (trig_L) *trig_L = true;
            Lflag_assist = false;
        }
    }

    if (out_tau_R) *out_tau_R = r_stop ? 0.0f : assist_level;
    if (out_tau_L) *out_tau_L = l_stop ? 0.0f : assist_level;
}

/* --- Current thigh angle values --- */
static float currentLeftThighAngle = 0.0f;
static float currentRightThighAngle = 0.0f;



/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTIONS
 * - 상세 구현은 파일 끝으로 이동
 *------------------------------------------------------------
 */

static void FVecDecoder_InitSingle(FVecSlot_t buf[])
{
    for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
        memset(&buf[i], 0, sizeof(FVecSlot_t));
    }
}

static void FVecDecoder_Init(void)
{
    FVecDecoder_InitSingle(s_fvec_buf_LH);
    FVecDecoder_InitSingle(s_fvec_buf_RH);
}

static void FVecDecoder_TriggerSingle(FVecSlot_t buf[], const float f_vec[4])
{
    for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
        if (!buf[i].is_full) {
            FVecSlot_t *s = &buf[i];
            s->mode_idx   = (uint8_t) f_vec[0];
            s->tau_max    = f_vec[1];
            s->delay      = (uint16_t) f_vec[2];
            s->t_end      = (uint32_t) f_vec[3];
            s->time_stamp = 0;
            s->u          = 0.0f;
            s->u_old1     = 0.0f;
            s->u_old2     = 0.0f;
            s->tau        = 0.0f;
            s->tau_old1   = 0.0f;
            s->tau_old2   = 0.0f;
            s->is_full    = 1;
            break;
        }
    }
}

static float FVecDecoder_StepSingle(FVecSlot_t buf[])
{
    float y_sum = 0.0f;

    for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
        FVecSlot_t *s = &buf[i];
        if (!s->is_full) continue;

        if (s->time_stamp == s->delay) {
            s->u = s->tau_max;
        }

        uint8_t idx = s->mode_idx;
        if (idx >= F_VECTOR_NUM_MODES) {
            s->is_full = 0;
            continue;
        }
        const FVecModeParam_t *p = &kFVecModeParam[idx];

        float tau_raw =
              p->b1 * s->tau_old1
            + p->b2 * s->tau_old2
            + p->a0 * s->u
            + p->a1 * s->u_old1
            + p->a2 * s->u_old2;

        float tau = tau_raw * p->gain_inv;

        s->tau_old2 = s->tau_old1;
        s->tau_old1 = tau;

        s->u_old2   = s->u_old1;
        s->u_old1   = s->u;
        s->u        = 0.0f;

        s->tau = tau;
        y_sum += tau;
        s->time_stamp++;

        if (s->time_stamp >= (uint16_t)(s->t_end + s->delay)) {
            memset(s, 0, sizeof(FVecSlot_t));
            s->is_full = 0;
        }
    }

    return y_sum;
}

static void TriggerManuscriptFVecSingle(FVecSlot_t buf[], const float fvec[4])
{
    // fvec format: {mode_idx, tau_max, delay_ms, t_end_ms}
    // Deterministic +1 control tick offset so the overlay starts on the next control tick.
    const float delay_ms = fvec[2] + CTRL_DT_MS;

    float e[4];
    e[0] = fvec[0];
    e[1] = fvec[1];
    e[2] = (float)MsToTicks_U16(delay_ms);
    e[3] = (float)MsToTicks_U32(fvec[3]);
    FVecDecoder_TriggerSingle(buf, e);
}
