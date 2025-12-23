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
#include "KNN_ref_dataset_251223.h"

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

// Assist torque amplitude (tau_max) controlled by XM_BTN_1 click.
// Cycles: 0 -> 1 -> 2 -> ... -> 7 -> 0 -> ...
// Exposed (non-static) for Live Expressions / CDC.
volatile int tau_max_setting = 0;


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

static RecordingState_t s_prevRecordingState = Record_OFF;

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
static const float s_fc_theta = 20.0f;
static const float s_alpha_theta = 0.2008486f;
static const float s_fc_omega = 25.0f;
static const float s_alpha_omega = 0.1357552f;
static float s_y_prev_R = 0.0f;
static float s_y_prev_L = 0.0f;
static float s_w_prev_R = 0.0f;
static float s_w_prev_L = 0.0f;

static bool s_R_time_upcond = false;
static bool s_L_time_upcond = false;
static bool s_HC_time_upcond_R = false;
static bool s_HC_time_upcond_L = false;
static bool s_HC_Rswing4upcond = false;
static bool s_HC_Lswing4upcond = false;
static bool s_transition_trigger_R = false;
static bool s_transition_trigger_L = false;

// Peak detection counters (watch via Live Expressions / send via CDC)
volatile int R_count_upeak = 0;
volatile int R_count_dpeak = 0;
volatile int L_count_upeak = 0;
volatile int L_count_dpeak = 0;
// HC detection counter (increments on each HC event)
volatile int hc_count = 0;

// Timing parameters are in absolute time (ms), not ticks.
// Adaptive per-leg gate time (ms). Initialized to 300 ms.
static float s_t_gap_R_ms = 300.0f;
static float s_t_gap_L_ms = 300.0f;
// Upper-peak angle threshold (deg). Adaptive from last STS-classified HC:
// threshold = 0.8 * (swing-leg angle at that HC). Initial value requested: 10 deg.
static float s_thres_up = 10.0f;
// Lower-peak baseline threshold (deg). Adaptive from last STS-classified HC:
// threshold = 0.8 * (swing-leg angle at that HC). Initial value requested: 10 deg.
static float s_thres_down = 10.0f;
static float s_R_lowpeak_val = 10.0f;
static float s_L_lowpeak_val = 10.0f;
static float s_var = 0.0f;

static float s_R_stance_angle_gate = 20.0f;
static float s_L_stance_angle_gate = 20.0f;
static float s_R_prev_stance_angle_at_dpeak = 0.0f;
static float s_L_prev_stance_angle_at_dpeak = 0.0f;
static bool  s_R_stance_angle_sampled = false;
static bool  s_L_stance_angle_sampled = false;
volatile float s_dbg_R_stance_angle_gate = 20.0f;
volatile float s_dbg_L_stance_angle_gate = 20.0f;
volatile float s_dbg_R_stance_angle_sample = 0.0f;
volatile float s_dbg_L_stance_angle_sample = 0.0f;

static float s_R_time_afterup_ms = 0.0f;
static float s_L_time_afterup_ms = 0.0f;
static float s_HC_time_after_R_ms = 0.0f;
static float s_HC_time_after_L_ms = 0.0f;
// Normalization swing period (ms): uses mean of latest SOS/STS swing times.
static float s_swing_period_ms = 300.0f;

// Adaptive HC gate based on swing-leg thigh angle magnitude (deg).
// Updated when an HC is classified as STS: threshold = 0.6 * (swing-leg angle at that HC).
// Initial value requested: 10 deg.
static float s_hc_deg_thresh = 10.0f;

// Track swing time separately by classification category (no left/right split).
static float s_T_swing_SOS_ms = 198.0f;
static float s_T_swing_STS_ms = 245.0f;
static bool s_last_HC_class_is_valid = false;
static bool s_last_HC_is_STS = false;

// Debug-visible normalization swing period (ms). Mirrors s_swing_period_ms for Live Expressions.
volatile float T_swing_ms = 300.0f;

// Debug-visible latest swing times by class (ms).
volatile float T_swing_SOS_ms = 198.0f;
volatile float T_swing_STS_ms = 245.0f;
volatile float TswingRecording_ms = 0.0f;

static float s_vel_HC = 0.0f;
static float s_R_swing_time_ms = 0.0f;
static float s_L_swing_time_ms = 0.0f;
static float s_T_HC_ms = 0.0f;
static float s_norm_vel_HC = 0.0f;
static float s_norm_T_HC = 0.0f;

// Debug-visible raw HC feature values.
volatile float s_vel_HC_dbg = 0.0f;
volatile float s_T_HC_s_dbg = 0.0f;

static float s_scaling_X = 92.15f;
static float s_scaling_Y = 0.91f;

// CDC/Live-expression mirrors (s_* are static above; these are global and volatile).
volatile float s_dbg_norm_vel_HC = 0.0f;
volatile float s_dbg_norm_T_HC = 0.0f;
volatile float s_dbg_scaling_X = 92.15f;
volatile float s_dbg_scaling_Y = 0.91f;

// CDC/Live-expression mirrors for adaptive parameters
volatile float s_dbg_t_gap_R_ms = 300.0f;
volatile float s_dbg_t_gap_L_ms = 300.0f;
volatile float s_dbg_hc_deg_thresh = 10.0f;
volatile float s_dbg_thres_up = 10.0f;
volatile float s_dbg_thres_down = 10.0f;

volatile float s_g_knn_conf = 0.0f;

#ifndef KNN_REF_COUNT
#  define KNN_REF_COUNT    REF_COUNT
#endif
#ifndef KNN_REF_SPLIT
#  define KNN_REF_SPLIT    REF_TYPE1_COUNT
#endif
#ifndef KNN_K
#  define KNN_K            23
#endif

// KNN classification outputs (watch via Live Expressions / send via CDC)
// Exposed as a single combined gait mode.
volatile GaitMode_t s_gait_mode = NONE;

typedef enum {
    GAIT_LATCH_NONE = 0,
    GAIT_LATCH_R = 1,
    GAIT_LATCH_L = 2,
} GaitLatchLeg_t;

static GaitLatchLeg_t s_gait_mode_latch_leg = GAIT_LATCH_NONE;

// Debug-visible trigger flags (use in Live Expressions)
volatile bool trig_R = false;
volatile bool trig_L = false;

// Debug-visible moving/standing snapshot (use in Live Expressions)
volatile int s_is_moving = 0;

// Debug-visible raw/processed signals to diagnose why is_moving stays 0
volatile float s_dbg_wR_f = 0.0f;
volatile float s_dbg_wL_f = 0.0f;

// Debug-visible computed torque commands (orders) even when not actuating (e.g. STANDBY)
volatile float s_tau_cmd_R = 0.0f;
volatile float s_tau_cmd_L = 0.0f;

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

// Debug-visible gait feature/result snapshots (use in Live Expressions)
volatile GaitFeatures_t s_gait_feat;
volatile GaitRecognitionResult_t s_gait_rec;

static void GaitModeRecognition_Update(float rightThighDeg, float leftThighDeg,
                                       GaitFeatures_t* feat_out);
static void GaitModeRecognition_DetectEvents(const GaitFeatures_t* feat,
                                             GaitRecognitionResult_t* res_out);
static void adaptive_assist(const GaitRecognitionResult_t* rec,
                            bool* out_trig_R, bool* out_trig_L,
                            float* out_tau_R, float* out_tau_L);
static void ResetUserState(void);

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

    FVecDecoder_Init();
    ResetUserState();
    s_prevRecordingState = Recording;
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
    HAL_Delay(250);
    XM_SetControlMode(XM_CTRL_MONITOR);
}

static void Standby_Loop(void)
{
    static float s_tau_out_R = 0.0f;
    static float s_tau_out_L = 0.0f;

	UpdateRecordingState();
    const bool recording_started = (s_prevRecordingState != Record_ON && Recording == Record_ON);
    s_prevRecordingState = Recording;
    if (recording_started) {
        const RecordingState_t recording_saved = Recording;
        ResetUserState();
        Recording = recording_saved;
        s_prevRecordingState = Recording;
        s_tau_out_R = 0.0f;
        s_tau_out_L = 0.0f;
    }

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
    }

    // Update tau_max on each button-1 click
    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    if (XM_button1 == XM_BTN_CLICK) {
        if (tau_max_setting >= 7) tau_max_setting = 0;
        else tau_max_setting += 1;
    }

    XM_button3 = XM_GetButtonEvent(XM_BTN_3);
    const bool button3_reset = (XM_button3 == XM_BTN_CLICK);
    if (button3_reset) {
        const RecordingState_t recording_saved = Recording;
        ResetUserState();
        Recording = recording_saved;
        s_prevRecordingState = Recording;
        s_tau_out_R = 0.0f;
        s_tau_out_L = 0.0f;
    }

    // In STANDBY: compute assist "orders" (triggers + f-vector decode), but do NOT actuate.
    trig_R = false;
    trig_L = false;
    float ignored_R = 0.0f, ignored_L = 0.0f;

    GaitModeRecognition_Update(XM.status.h10.rightThighAngle, XM.status.h10.leftThighAngle, (GaitFeatures_t*)&s_gait_feat);
    GaitModeRecognition_DetectEvents((const GaitFeatures_t*)&s_gait_feat, (GaitRecognitionResult_t*)&s_gait_rec);
    s_is_moving = s_gait_feat.is_moving;
    s_dbg_wR_f = s_gait_feat.wR_f;
    s_dbg_wL_f = s_gait_feat.wL_f;

    adaptive_assist((const GaitRecognitionResult_t*)&s_gait_rec, (bool*)&trig_R, (bool*)&trig_L, &ignored_R, &ignored_L);

    if (s_transition_trigger_R || s_transition_trigger_L) {
        if (s_transition_trigger_R) {
            FVecDecoder_InitSingle(s_fvec_buf_RH);
            s_tau_out_R = 0.0f;
            s_tau_cmd_R = 0.0f;
            XM_SetAssistTorqueRH(0.0f);
            trig_R = true;
        }
        if (s_transition_trigger_L) {
            FVecDecoder_InitSingle(s_fvec_buf_LH);
            s_tau_out_L = 0.0f;
            s_tau_cmd_L = 0.0f;
            XM_SetAssistTorqueLH(0.0f);
            trig_L = true;
        }
        s_transition_trigger_R = false;
        s_transition_trigger_L = false;
    }

    if (s_transition_trigger_R || s_transition_trigger_L) {
        if (s_transition_trigger_R) {
            FVecDecoder_InitSingle(s_fvec_buf_RH);
            s_tau_out_R = 0.0f;
            s_tau_cmd_R = 0.0f;
            trig_R = true;
        }
        if (s_transition_trigger_L) {
            FVecDecoder_InitSingle(s_fvec_buf_LH);
            s_tau_out_L = 0.0f;
            s_tau_cmd_L = 0.0f;
            trig_L = true;
        }
        s_transition_trigger_R = false;
        s_transition_trigger_L = false;
    }

    if (trig_R && !Rstop_assist) {
        float fvec[4] = { kManuscriptFVec[0], (float)tau_max_setting, kManuscriptFVec[2], kManuscriptFVec[3] };
        TriggerManuscriptFVecSingle(s_fvec_buf_RH, fvec);
    }
    if (trig_L && !Lstop_assist) {
        float fvec[4] = { kManuscriptFVec[0], (float)tau_max_setting, kManuscriptFVec[2], kManuscriptFVec[3] };
        TriggerManuscriptFVecSingle(s_fvec_buf_LH, fvec);
    }

    float tau_R = FVecDecoder_StepSingle(s_fvec_buf_RH);
    float tau_L = FVecDecoder_StepSingle(s_fvec_buf_LH);

    // Apply the same STS soft-stop shaping to the computed orders.
    const bool r_sts = (s_gait_mode == RSTS);
    const bool l_sts = (s_gait_mode == LSTS);
    const float dt_s = (CTRL_DT_MS * 0.001f);
    const float rise_time_s = 0.10f;
    const float residual_ratio = 0.01f;
    float T_decay_s = rise_time_s;
    if (rise_time_s > 0.0f && residual_ratio > 0.0f && residual_ratio < 1.0f) {
        T_decay_s = -rise_time_s / logf(residual_ratio);
    }
    if (T_decay_s < dt_s) T_decay_s = dt_s;
    const float alpha = dt_s / (T_decay_s + dt_s);

    const float target_R = r_sts ? 0.0f : tau_R;
    const float target_L = l_sts ? 0.0f : tau_L;
    s_tau_out_R = s_tau_out_R + alpha * (target_R - s_tau_out_R);
    s_tau_out_L = s_tau_out_L + alpha * (target_L - s_tau_out_L);

    s_tau_cmd_R = s_tau_out_R;
    s_tau_cmd_L = s_tau_out_L;
}

static void Active_Entry(void)
{
	XM_SetControlMode(XM_CTRL_TORQUE);
    FVecDecoder_Init();
}

static void Active_Loop(void)
{
	static float s_tau_out_R = 0.0f;
	static float s_tau_out_L = 0.0f;

	UpdateRecordingState();
	const bool recording_started = (s_prevRecordingState != Record_ON && Recording == Record_ON);
	s_prevRecordingState = Recording;
	if (recording_started) {
	    const RecordingState_t recording_saved = Recording;
	    ResetUserState();
	    Recording = recording_saved;
	    s_prevRecordingState = Recording;
	    s_tau_out_R = 0.0f;
	    s_tau_out_L = 0.0f;
	    XM_SetAssistTorqueRH(0.0f);
	    XM_SetAssistTorqueLH(0.0f);
	    s_tau_cmd_R = 0.0f;
	    s_tau_cmd_L = 0.0f;
	    return;
	}

    // Update tau_max on each button-1 click
    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    if (XM_button1 == XM_BTN_CLICK) {
        if (tau_max_setting >= 7) tau_max_setting = 0;
        else tau_max_setting += 1;
    }

    // --- USER_DEFINED_CTRL: trigger manuscript-defined f-vector and decode/actuate in real-time ---
    trig_R = false;
    trig_L = false;
    float ignored_R = 0.0f, ignored_L = 0.0f;

    GaitModeRecognition_Update(XM.status.h10.rightThighAngle, XM.status.h10.leftThighAngle, (GaitFeatures_t*)&s_gait_feat);
    GaitModeRecognition_DetectEvents((const GaitFeatures_t*)&s_gait_feat, (GaitRecognitionResult_t*)&s_gait_rec);
    s_is_moving = s_gait_feat.is_moving;

    // Mirror processed metrics for Live Expressions
    s_dbg_wR_f = s_gait_feat.wR_f;
    s_dbg_wL_f = s_gait_feat.wL_f;

    adaptive_assist((const GaitRecognitionResult_t*)&s_gait_rec, (bool*)&trig_R, (bool*)&trig_L, &ignored_R, &ignored_L);

    if (trig_R && !Rstop_assist) {
        float fvec[4] = { kManuscriptFVec[0], (float)tau_max_setting, kManuscriptFVec[2], kManuscriptFVec[3] };
        TriggerManuscriptFVecSingle(s_fvec_buf_RH, fvec);
    }
    if (trig_L && !Lstop_assist) {
        float fvec[4] = { kManuscriptFVec[0], (float)tau_max_setting, kManuscriptFVec[2], kManuscriptFVec[3] };
        TriggerManuscriptFVecSingle(s_fvec_buf_LH, fvec);
    }

    float tau_R = FVecDecoder_StepSingle(s_fvec_buf_RH);
    float tau_L = FVecDecoder_StepSingle(s_fvec_buf_LH);

    // Soft stop: when STS is detected, ramp the *actual* output torque down to 0.
    const bool r_sts = (s_gait_mode == RSTS);
    const bool l_sts = (s_gait_mode == LSTS);
    const float dt_s = (CTRL_DT_MS * 0.001f);
    const float rise_time_s = 0.10f;
    const float residual_ratio = 0.01f;
    float T_decay_s = rise_time_s;
    if (rise_time_s > 0.0f && residual_ratio > 0.0f && residual_ratio < 1.0f) {
        T_decay_s = -rise_time_s / logf(residual_ratio);
    }
    if (T_decay_s < dt_s) T_decay_s = dt_s;
    const float alpha = dt_s / (T_decay_s + dt_s);

    const float target_R = r_sts ? 0.0f : tau_R;
    const float target_L = l_sts ? 0.0f : tau_L;
    s_tau_out_R = s_tau_out_R + alpha * (target_R - s_tau_out_R);
    s_tau_out_L = s_tau_out_L + alpha * (target_L - s_tau_out_L);

    // Actuate in ACTIVE
    XM_SetAssistTorqueRH(s_tau_out_R);
    XM_SetAssistTorqueLH(s_tau_out_L);

    // Mirror for debug/telemetry
    s_tau_cmd_R = s_tau_out_R;
    s_tau_cmd_L = s_tau_out_L;
}

static void Active_Exit(void)
{

}



static inline float KNN_LPF(float x, float* y_prev, float alpha)
{
    *y_prev = (1.0f - alpha) * (*y_prev) + alpha * x;
    return *y_prev;
}

static inline float clampf_ud(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void HC_UpdateDegThresh_OnSTS(float swing_deg_meas)
{
    // Unified (shared) threshold for both left and right.
    // Spec: threshold = 0.6 * (swing-leg angle at STS-classified HC), initial 10 deg.
    const float HC_DEG_MIN = 5.0f;
    const float HC_DEG_MAX = 90.0f;
    const float DEG_ALPHA  = 0.2f;

    const float deg_meas = clampf_ud(swing_deg_meas, HC_DEG_MIN, HC_DEG_MAX);
    const float deg_target = 0.55f * deg_meas;
    s_hc_deg_thresh = (1.0f - DEG_ALPHA) * s_hc_deg_thresh + DEG_ALPHA * deg_target;
    s_dbg_hc_deg_thresh = s_hc_deg_thresh;
}

static inline void UpPeak_UpdateThresh_OnSTS(float swing_deg_meas)
{
    // Adaptive upper-peak angle threshold shared for both legs.
    // Spec: s_thres_up = 0.8 * (swing-leg angle at STS-classified HC), initial 10 deg.
    const float UP_DEG_MIN = 5.0f;
    const float UP_DEG_MAX = 90.0f;
    const float UP_ALPHA   = 0.2f;

    const float deg_meas = clampf_ud(swing_deg_meas, UP_DEG_MIN, UP_DEG_MAX);
    const float deg_target = 0.8f * deg_meas;
    s_thres_up = (1.0f - UP_ALPHA) * s_thres_up + UP_ALPHA * deg_target;
    s_dbg_thres_up = s_thres_up;
}

static inline void LowPeak_UpdateThresh_OnSTS(float swing_deg_meas)
{
    // Adaptive lower-peak baseline threshold shared for both legs.
    // Spec: s_thres_down from last STS-classified HC swing-leg angle.
    // Use same 0.8 scaling as upper threshold for consistency.
    const float DOWN_DEG_MIN = 5.0f;
    const float DOWN_DEG_MAX = 90.0f;
    const float DOWN_ALPHA   = 0.2f;

    const float deg_meas = clampf_ud(swing_deg_meas, DOWN_DEG_MIN, DOWN_DEG_MAX);
    const float deg_target = 0.8f * deg_meas;
    s_thres_down = (1.0f - DOWN_ALPHA) * s_thres_down + DOWN_ALPHA * deg_target;
    s_dbg_thres_down = s_thres_down;
}

static int knn_2label_majority_ud(const float (*xy)[2], int n, int split, float x, float y, int k, volatile float *conf_out)
{

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

    float w1 = 0.0f, w2 = 0.0f;
    const float eps = 2.2204e-16f;

    for (int i = 0; i < used; ++i) {
        float dist = sqrtf(best_d2[i]);     // best_d2가 dx^2+dy^2 이므로 sqrt
        float w = 1.0f / (dist + eps);

        if (best_lab[i] == 1u) w1 += w;
        else                   w2 += w;
    }

    // confidence = max(wSOS, wSTS) / (wSOS + wSTS)
    if (conf_out) {
        float wsum = w1 + w2;
        float wmax = (w1 >= w2) ? w1 : w2;
        *conf_out = (wsum > 0.0f) ? (wmax / wsum) : 0.0f;
    }

    int label = (w1 >= w2) ? 1 : 2;
    return label;
}

static int update_standing_moving_flag_w_ud(float wR_metric, float wL_metric)
{
    const float V_ON_THRESH   = 40.0f;
    const float V_OFF_THRESH  = 10.0f;
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
    static int s_prev_is_moving = 0;

    if (res_out) {
        memset(res_out, 0, sizeof(*res_out));
    }

    s_loop_count++;
    s_R_time_afterup_ms += CTRL_DT_MS;
    s_L_time_afterup_ms += CTRL_DT_MS;
    s_HC_time_after_R_ms += CTRL_DT_MS;
    s_HC_time_after_L_ms += CTRL_DT_MS;
    s_R_swing_time_ms += CTRL_DT_MS;
    s_L_swing_time_ms += CTRL_DT_MS;

    const int is_moving = (feat != NULL) ? feat->is_moving : 0;
    if (res_out) res_out->is_moving = is_moving;

    if (!s_prev_is_moving && is_moving && feat) {
        const float vel_R = feat->wR_f;
        const float vel_L = feat->wL_f;
        const bool right_faster = (vel_R >= vel_L);
        s_transition_trigger_R = right_faster;
        s_transition_trigger_L = !right_faster;
        Rflag_assist = false;
        Lflag_assist = false;
        if (right_faster) {
            Rstop_assist = false;
        } else {
            Lstop_assist = false;
        }
    }

    if (s_R_time_afterup_ms >= s_t_gap_R_ms) s_R_time_upcond = true;
    if (s_L_time_afterup_ms >= s_t_gap_L_ms) s_L_time_upcond = true;
    if (s_HC_time_after_R_ms >= s_t_gap_R_ms) s_HC_time_upcond_R = true;
    if (s_HC_time_after_L_ms >= s_t_gap_L_ms) s_HC_time_upcond_L = true;

    if ((s_Rdeg[2] - s_Ldeg[2]) * (s_Rdeg[1] - s_Ldeg[1]) <= 0.0f) {
        // Determine swing side for this HC candidate (same rule as before).
        // Apply HC gate time depending on swing side.
        const bool is_right_swing = (s_Rdeg[2] - s_Rdeg[1] >= s_Ldeg[2] - s_Ldeg[1]);
        const bool hc_time_ok = is_right_swing ? s_HC_time_upcond_R : s_HC_time_upcond_L;

        // Swing-leg angle gate (deg), adaptive from last STS-classified HC.
        const float swing_deg = is_right_swing ? s_Rdeg[2] : s_Ldeg[2];
        const bool swing_deg_ok = (swing_deg >= s_hc_deg_thresh);

        float stance_angle_sample = 0.0f;
        bool stance_angle_ok = true;
        if (is_right_swing) {
            stance_angle_sample = s_R_prev_stance_angle_at_dpeak;
            stance_angle_ok = (!s_R_stance_angle_sampled) || (stance_angle_sample >= s_R_stance_angle_gate);
        } else {
            stance_angle_sample = s_L_prev_stance_angle_at_dpeak;
            stance_angle_ok = (!s_L_stance_angle_sampled) || (stance_angle_sample >= s_L_stance_angle_gate);
        }

        if (is_right_swing) {
            s_dbg_R_stance_angle_sample = stance_angle_sample;
        } else {
            s_dbg_L_stance_angle_sample = stance_angle_sample;
        }

        if (!hc_time_ok || !swing_deg_ok || !stance_angle_ok) {
            // Not enough time elapsed (per swing side) or swing angle too small; ignore this HC candidate.
        } else {
            if (is_right_swing) {
                s_HC_time_upcond_R = false;
                s_HC_time_after_R_ms = 0.0f;
            } else {
                s_HC_time_upcond_L = false;
                s_HC_time_after_L_ms = 0.0f;
            }
            hc_count++;

            if (is_right_swing) {
            s_HC_Rswing4upcond = true;
            s_vel_HC = (s_Rdeg[2] - s_Rdeg[1]) / (CTRL_DT_MS * 0.001f);
            s_T_HC_ms = s_R_swing_time_ms;

            s_vel_HC_dbg = s_vel_HC;
            s_T_HC_s_dbg = s_T_HC_ms * 0.001f;

            const float swing_period_s = s_swing_period_ms * 0.001f;
            s_norm_vel_HC = (s_vel_HC * swing_period_s) / s_scaling_X;
            s_norm_T_HC = ((s_T_HC_ms * 0.001f) / swing_period_s) / s_scaling_Y;

            s_dbg_norm_vel_HC = s_norm_vel_HC;
            s_dbg_norm_T_HC = s_norm_T_HC;
            s_dbg_scaling_X = s_scaling_X;
            s_dbg_scaling_Y = s_scaling_Y;

            const int knn_label = knn_2label_majority_ud(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT, s_norm_vel_HC, s_norm_T_HC, KNN_K, &s_g_knn_conf);
            s_gait_mode = (knn_label == 1) ? RSTS : RSOS;
            s_gait_mode_latch_leg = GAIT_LATCH_R;

            // Remember the most recent HC classification (ignore left/right).
            s_last_HC_class_is_valid = true;
            s_last_HC_is_STS = (s_gait_mode == RSTS);
            // Update adaptive angle threshold only when classified as STS.
            if (s_last_HC_is_STS) {
                HC_UpdateDegThresh_OnSTS(s_Rdeg[2]);
                UpPeak_UpdateThresh_OnSTS(s_Rdeg[2]);
                LowPeak_UpdateThresh_OnSTS(s_Rdeg[2]);
            }
            if (s_gait_mode == RSTS) {
                Rstop_assist = true;
                Lstop_assist = true;
                // Trajectory decoding 즉시 중단 (양쪽 모두)
                FVecDecoder_InitSingle(s_fvec_buf_RH);
                FVecDecoder_InitSingle(s_fvec_buf_LH);
            }
        } else if (!is_right_swing) {
            s_HC_Lswing4upcond = true;
            s_vel_HC = (s_Ldeg[2] - s_Ldeg[1]) / (CTRL_DT_MS * 0.001f);
            s_T_HC_ms = s_L_swing_time_ms;

            s_vel_HC_dbg = s_vel_HC;
            s_T_HC_s_dbg = s_T_HC_ms * 0.001f;

            const float swing_period_s = s_swing_period_ms * 0.001f;
            s_norm_vel_HC = (s_vel_HC * swing_period_s) / s_scaling_X;
            s_norm_T_HC = ((s_T_HC_ms * 0.001f) / swing_period_s) / s_scaling_Y;

            s_dbg_norm_vel_HC = s_norm_vel_HC;
            s_dbg_norm_T_HC = s_norm_T_HC;
            s_dbg_scaling_X = s_scaling_X;
            s_dbg_scaling_Y = s_scaling_Y;

            const int knn_label = knn_2label_majority_ud(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT, s_norm_vel_HC, s_norm_T_HC, KNN_K, &s_g_knn_conf);
            s_gait_mode = (knn_label == 1) ? LSTS : LSOS;
            s_gait_mode_latch_leg = GAIT_LATCH_L;

            // Remember the most recent HC classification (ignore left/right).
            s_last_HC_class_is_valid = true;
            s_last_HC_is_STS = (s_gait_mode == LSTS);
            // Update adaptive angle threshold only when classified as STS.
            if (s_last_HC_is_STS) {
                HC_UpdateDegThresh_OnSTS(s_Ldeg[2]);
                UpPeak_UpdateThresh_OnSTS(s_Ldeg[2]);
                LowPeak_UpdateThresh_OnSTS(s_Ldeg[2]);
            }
            if (s_gait_mode == LSTS) {
                Rstop_assist = true;
                Lstop_assist = true;
                // Trajectory decoding 즉시 중단 (양쪽 모두)
                FVecDecoder_InitSingle(s_fvec_buf_RH);
                FVecDecoder_InitSingle(s_fvec_buf_LH);
            }
        }
        }
    }

    // Upper peak detection
    if ((s_Rdeg[2] - s_Rdeg[1]) * (s_Rdeg[1] - s_Rdeg[0]) < s_var &&
        (s_Rdeg[2] - s_Rdeg[1]) < 0.0f &&
        s_Rdeg[2] >= s_thres_up && s_R_time_upcond) {
        s_R_time_upcond = false;
        R_count_upeak++;

        // Measured swing time: from lower peak reset to this upper peak.
        const float measured_swing_ms = s_R_swing_time_ms;
        // Adapt right-side gate to the measured swing duration.
        // Use clamp + 1st-order LPF to suppress outliers.
        // NOTE: this is a gate time used for event de-bounce, not the normalization swing period.
        const float GAP_MIN_MS = 150.0f;
        const float GAP_MAX_MS = 1200.0f;
        const float GAP_ALPHA  = 0.2f;
        const float meas_clamped = clampf_ud(measured_swing_ms, GAP_MIN_MS, GAP_MAX_MS);
        const float prev_gap_R = s_t_gap_R_ms;
        float updated_gap_R = (1.0f - GAP_ALPHA) * prev_gap_R + GAP_ALPHA * meas_clamped;
        if (prev_gap_R > 0.0f) {
            const float min_gap_R = prev_gap_R * 0.7f;
            const float max_gap_R = prev_gap_R * 1.3f;
            updated_gap_R = clampf_ud(updated_gap_R, min_gap_R, max_gap_R);
        }
        updated_gap_R = clampf_ud(updated_gap_R, 200.0f, 800.0f);
        s_t_gap_R_ms = updated_gap_R;
        s_dbg_t_gap_R_ms = s_t_gap_R_ms;

        // Store to SOS/STS bucket based on the most recent HC classification.
        if (s_last_HC_class_is_valid) {
            if (s_last_HC_is_STS) {
                s_T_swing_STS_ms = measured_swing_ms;
                T_swing_STS_ms = s_T_swing_STS_ms;
            } else {
                s_T_swing_SOS_ms = measured_swing_ms;
                T_swing_SOS_ms = s_T_swing_SOS_ms;
            }
        }
        TswingRecording_ms = measured_swing_ms;

        // Normalization uses mean of the most recent SOS/STS swing times (regardless of confidence).
        s_swing_period_ms = 0.5f * (s_T_swing_SOS_ms + s_T_swing_STS_ms);
        T_swing_ms = s_swing_period_ms;
        s_R_time_afterup_ms = 0.0f;
        s_HC_Rswing4upcond = false;

        if (s_last_HC_class_is_valid && !s_last_HC_is_STS) {
            const float raw_peak = fabsf(s_Rdeg[2]);
            const float target_thresh = clampf_ud(0.3f * raw_peak, 5.0f, 120.0f);
            const float ALPHA = 0.2f;
            s_R_stance_angle_gate = (1.0f - ALPHA) * s_R_stance_angle_gate + ALPHA * target_thresh;
            s_dbg_R_stance_angle_gate = s_R_stance_angle_gate;
        }

        // Clear latched gait mode after the swing leg's next upper peak.
        if (s_gait_mode_latch_leg == GAIT_LATCH_R) {
            s_gait_mode_latch_leg = GAIT_LATCH_NONE;
            s_gait_mode = NONE;
        }
    }

    if ((s_Ldeg[2] - s_Ldeg[1]) * (s_Ldeg[1] - s_Ldeg[0]) < s_var &&
        (s_Ldeg[2] - s_Ldeg[1]) < 0.0f &&
        s_Ldeg[2] >= s_thres_up && s_L_time_upcond) {
        s_L_time_upcond = false;
        L_count_upeak++;

        // Measured swing time: from lower peak reset to this upper peak.
        const float measured_swing_ms = s_L_swing_time_ms;
        // Adapt left-side gate to the measured swing duration.
        // Use clamp + 1st-order LPF to suppress outliers.
        // NOTE: this is a gate time used for event de-bounce, not the normalization swing period.
        const float GAP_MIN_MS = 150.0f;
        const float GAP_MAX_MS = 1200.0f;
        const float GAP_ALPHA  = 0.2f;
        const float meas_clamped = clampf_ud(measured_swing_ms, GAP_MIN_MS, GAP_MAX_MS);
        const float prev_gap_L = s_t_gap_L_ms;
        float updated_gap_L = (1.0f - GAP_ALPHA) * prev_gap_L + GAP_ALPHA * meas_clamped;
        if (prev_gap_L > 0.0f) {
            const float min_gap_L = prev_gap_L * 0.7f;
            const float max_gap_L = prev_gap_L * 1.3f;
            updated_gap_L = clampf_ud(updated_gap_L, min_gap_L, max_gap_L);
        }
        updated_gap_L = clampf_ud(updated_gap_L, 200.0f, 800.0f);
        s_t_gap_L_ms = updated_gap_L;
        s_dbg_t_gap_L_ms = s_t_gap_L_ms;

        // Store to SOS/STS bucket based on the most recent HC classification.
        if (s_last_HC_class_is_valid) {
            if (s_last_HC_is_STS) {
                s_T_swing_STS_ms = measured_swing_ms;
                T_swing_STS_ms = s_T_swing_STS_ms;
            } else {
                s_T_swing_SOS_ms = measured_swing_ms;
                T_swing_SOS_ms = s_T_swing_SOS_ms;
            }
        }
        TswingRecording_ms = measured_swing_ms;

        // Normalization uses mean of the most recent SOS/STS swing times (regardless of confidence).
        s_swing_period_ms = 0.5f * (s_T_swing_SOS_ms + s_T_swing_STS_ms);
        T_swing_ms = s_swing_period_ms;

        s_L_time_afterup_ms = 0.0f;
        s_HC_Lswing4upcond = false;

        if (s_last_HC_class_is_valid && !s_last_HC_is_STS) {
            const float raw_peak = fabsf(s_Ldeg[2]);
            const float target_thresh = clampf_ud(0.3f * raw_peak, 5.0f, 120.0f);
            const float ALPHA = 0.2f;
            s_L_stance_angle_gate = (1.0f - ALPHA) * s_L_stance_angle_gate + ALPHA * target_thresh;
            s_dbg_L_stance_angle_gate = s_L_stance_angle_gate;
        }

        // Clear latched gait mode after the swing leg's next upper peak.
        if (s_gait_mode_latch_leg == GAIT_LATCH_L) {
            s_gait_mode_latch_leg = GAIT_LATCH_NONE;
            s_gait_mode = NONE;
        }
    }

    // Lower peak detection

    if (s_Rdeg[2] > s_R_lowpeak_val + 10.0f) {
        s_R_lowpeak_val = s_thres_down;
    }
    if ((s_Rdeg[2]-s_Rdeg[1])*(s_Rdeg[1]-s_Rdeg[0]) <= s_var &&
        (s_Rdeg[2] - s_Rdeg[1]) >= 0.0f &&
        s_Rdeg[2] <= s_R_lowpeak_val + 0.5f) {
        R_count_dpeak++;
        if (is_moving) {
            s_R_lowpeak_val = s_Rdeg[2];
        }
        s_R_prev_stance_angle_at_dpeak = s_Ldeg[2];
        s_R_stance_angle_sampled = true;
        s_dbg_R_stance_angle_sample = s_R_prev_stance_angle_at_dpeak;
        s_R_swing_time_ms = 0.0f;
        // stop_assist는 오직 lower peak에서만 false로 바뀌고, 그 외에는 true 유지
        if (is_moving) {
            // 좌우 중 한쪽이라도 stop_assist가 true면, 둘 다 동시에 false로 바꾼다 (soft-stop이 0에 충분히 수렴했을 때)
            if (Rstop_assist || Lstop_assist) {
                if (fabsf(s_tau_cmd_R) < 0.01f && fabsf(s_tau_cmd_L) < 0.01f) {
                    Rstop_assist = false;
                    Lstop_assist = false;
                }
            }
            Rflag_assist = true;
        }
    }

    if (s_Ldeg[2] > s_L_lowpeak_val + 10.0f) {
        s_L_lowpeak_val = s_thres_down;
    }
    if ((s_Ldeg[2]-s_Ldeg[1])*(s_Ldeg[1]-s_Ldeg[0]) <= s_var &&
        (s_Ldeg[2] - s_Ldeg[1]) >= 0.0f &&
        s_Ldeg[2] <= s_L_lowpeak_val + 0.5f) {
        L_count_dpeak++;
        if (is_moving) {
            s_L_lowpeak_val = s_Ldeg[2];
        }
        s_L_prev_stance_angle_at_dpeak = s_Rdeg[2];
        s_L_stance_angle_sampled = true;
        s_dbg_L_stance_angle_sample = s_L_prev_stance_angle_at_dpeak;
        s_L_swing_time_ms = 0.0f;
        // stop_assist는 오직 lower peak에서만 false로 바뀌고, 그 외에는 true 유지
        if (is_moving) {
            // 좌우 중 한쪽이라도 stop_assist가 true면, 둘 다 동시에 false로 바꾼다 (soft-stop이 0에 충분히 수렴했을 때)
            if (Rstop_assist || Lstop_assist) {
                if (fabsf(s_tau_cmd_R) < 0.01f && fabsf(s_tau_cmd_L) < 0.01f) {
                    Rstop_assist = false;
                    Lstop_assist = false;
                }
            }
            Lflag_assist = true;
        }
    }

    if (res_out) {
        res_out->r_lower_peak_event = (R_count_dpeak != s_prev_count_dpeak_R);
        res_out->l_lower_peak_event = (L_count_dpeak != s_prev_count_dpeak_L);

        // Rising-after-lower should only fire once when the flag was armed and then the rise condition is met.
        const bool r_rise_now = (Rflag_assist && (s_Rdeg[2] > s_R_lowpeak_val + 5.0f));
        const bool l_rise_now = (Lflag_assist && (s_Ldeg[2] > s_L_lowpeak_val + 5.0f));
        res_out->r_uprise_after_lower = (r_rise_now && s_prev_Rflag_assist);
        res_out->l_uprise_after_lower = (l_rise_now && s_prev_Lflag_assist);

        res_out->r_stop_assist = Rstop_assist;
        res_out->l_stop_assist = Lstop_assist;
    }

    s_prev_count_dpeak_R = R_count_dpeak;
    s_prev_count_dpeak_L = L_count_dpeak;
    s_prev_Rflag_assist = Rflag_assist;
    s_prev_Lflag_assist = Lflag_assist;
    s_prev_is_moving = is_moving;
}

static void adaptive_assist(const GaitRecognitionResult_t* rec,
                            bool* out_trig_R, bool* out_trig_L,
                            float* out_tau_R, float* out_tau_L)
{
    if (out_trig_R) *out_trig_R = false;
    if (out_trig_L) *out_trig_L = false;
    if (out_tau_R) *out_tau_R = 0.0f;
    if (out_tau_L) *out_tau_L = 0.0f;

    const bool r_stop = (rec != NULL) ? rec->r_stop_assist : false;
    const bool l_stop = (rec != NULL) ? rec->l_stop_assist : false;

    if (rec != NULL) {
        if (rec->r_uprise_after_lower) {
            if (out_trig_R) *out_trig_R = true;
            Rflag_assist = false;
        }
        if (rec->l_uprise_after_lower) {
            if (out_trig_L) *out_trig_L = true;
            Lflag_assist = false;
        }
    }

    // Smoothly decay torque to 0 on STS/stop, instead of hard cutting.
    // Requirement: when stop happens, decay should settle to (almost) 0 within the mode's rise time.
    // Use 1st-order exponential: e(t) = exp(-t/T). Choose T such that e(Tr) = r.
    // => T = -Tr / ln(r)
    // Then discrete update: tau[k] = tau[k-1] + alpha*(target - tau[k-1]), alpha = dt/(T+dt)
    const float dt_s = (CTRL_DT_MS * 0.001f);

    // Mode rise time (seconds). If you have a real s_mode table elsewhere, wire it here.
    // For now, satisfy the stated requirement: s_mode==0 has 0.1s rise time.
    const float rise_time_s = 0.10f;

    // Residual ratio at Tr (how close to target we want by the end of rise_time_s).
    // 0.01 => within 1% (pretty much converged).
    const float residual_ratio = 0.01f;
    float T_decay_s = rise_time_s;
    if (rise_time_s > 0.0f && residual_ratio > 0.0f && residual_ratio < 1.0f) {
        T_decay_s = -rise_time_s / logf(residual_ratio);
    }
    if (T_decay_s < dt_s) T_decay_s = dt_s;
    const float alpha = dt_s / (T_decay_s + dt_s);

    const float target_R = r_stop ? 0.0f : assist_level;
    const float target_L = l_stop ? 0.0f : assist_level;
    s_tau_cmd_R = s_tau_cmd_R + alpha * (target_R - s_tau_cmd_R);
    s_tau_cmd_L = s_tau_cmd_L + alpha * (target_L - s_tau_cmd_L);

    if (out_tau_R) *out_tau_R = s_tau_cmd_R;
    if (out_tau_L) *out_tau_L = s_tau_cmd_L;
}


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTIONS
 * - 상세 구현은 파일 끝으로 이동
 *------------------------------------------------------------
 */

static void ResetUserState(void)
{
    Rflag_assist = false;
    Lflag_assist = false;
    Rstop_assist = false;
    Lstop_assist = false;

    s_loop_count = 0;
    memset(s_Rdeg, 0, sizeof(s_Rdeg));
    memset(s_Ldeg, 0, sizeof(s_Ldeg));

    Recording = Record_OFF;
    SmartAssist = SmartAssist_OFF;
    sync_signal = XM_LOW;
    sync_signal_pre = XM_LOW;

    s_y_prev_R = 0.0f;
    s_y_prev_L = 0.0f;
    s_w_prev_R = 0.0f;
    s_w_prev_L = 0.0f;

    s_R_time_upcond = false;
    s_L_time_upcond = false;
    s_HC_time_upcond_R = false;
    s_HC_time_upcond_L = false;
    s_HC_Rswing4upcond = false;
    s_HC_Lswing4upcond = false;

    R_count_upeak = 0;
    R_count_dpeak = 0;
    L_count_upeak = 0;
    L_count_dpeak = 0;
    hc_count = 0;

    s_t_gap_R_ms = 300.0f;
    s_t_gap_L_ms = 300.0f;
    s_dbg_t_gap_R_ms = s_t_gap_R_ms;
    s_dbg_t_gap_L_ms = s_t_gap_L_ms;

    s_thres_up = 10.0f;
    s_thres_down = 10.0f;
    s_R_lowpeak_val = 10.0f;
    s_L_lowpeak_val = 10.0f;
    s_dbg_thres_up = s_thres_up;
    s_dbg_thres_down = s_thres_down;

    s_R_stance_angle_gate = 20.0f;
    s_L_stance_angle_gate = 20.0f;
    s_R_prev_stance_angle_at_dpeak = 0.0f;
    s_L_prev_stance_angle_at_dpeak = 0.0f;
    s_R_stance_angle_sampled = false;
    s_L_stance_angle_sampled = false;
    s_dbg_R_stance_angle_gate = s_R_stance_angle_gate;
    s_dbg_L_stance_angle_gate = s_L_stance_angle_gate;
    s_dbg_R_stance_angle_sample = 0.0f;
    s_dbg_L_stance_angle_sample = 0.0f;

    s_var = 0.0f;

    s_R_time_afterup_ms = 0.0f;
    s_L_time_afterup_ms = 0.0f;
    s_HC_time_after_R_ms = 0.0f;
    s_HC_time_after_L_ms = 0.0f;

    s_swing_period_ms = 300.0f;
    s_T_swing_SOS_ms = 198.0f;
    s_T_swing_STS_ms = 245.0f;

    s_last_HC_class_is_valid = false;
    s_last_HC_is_STS = false;

    T_swing_ms = s_swing_period_ms;
    T_swing_SOS_ms = s_T_swing_SOS_ms;
    T_swing_STS_ms = s_T_swing_STS_ms;

    s_hc_deg_thresh = 10.0f;
    s_dbg_hc_deg_thresh = s_hc_deg_thresh;

    s_vel_HC = 0.0f;
    s_R_swing_time_ms = 0.0f;
    s_L_swing_time_ms = 0.0f;
    s_T_HC_ms = 0.0f;
    s_norm_vel_HC = 0.0f;
    s_norm_T_HC = 0.0f;
    s_vel_HC_dbg = 0.0f;
    s_T_HC_s_dbg = 0.0f;

    s_dbg_norm_vel_HC = 0.0f;
    s_dbg_norm_T_HC = 0.0f;
    s_dbg_scaling_X = s_scaling_X;
    s_dbg_scaling_Y = s_scaling_Y;

    TswingRecording_ms = 0.0f;

    s_tau_cmd_R = 0.0f;
    s_tau_cmd_L = 0.0f;

    s_gait_mode = NONE;
    s_g_knn_conf = 0.0f;
    s_gait_mode_latch_leg = GAIT_LATCH_NONE;
    trig_R = false;
    trig_L = false;
    s_is_moving = 0;
    s_dbg_wR_f = 0.0f;
    s_dbg_wL_f = 0.0f;
    s_transition_trigger_R = false;
    s_transition_trigger_L = false;

    memset((void*)&s_gait_feat, 0, sizeof(s_gait_feat));
    memset((void*)&s_gait_rec, 0, sizeof(s_gait_rec));

    s_dataSaveLoopCnt = 0;
    memset(&SavingData, 0, sizeof(SavingData));

    FVecDecoder_Init();
}

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
