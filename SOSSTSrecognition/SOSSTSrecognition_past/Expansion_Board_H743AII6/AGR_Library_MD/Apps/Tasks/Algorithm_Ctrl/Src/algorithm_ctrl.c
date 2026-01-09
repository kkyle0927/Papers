#include "algorithm_ctrl.h"
#include "KNN_ref_dataset_arb.h"

/* ------------------- Default Variables ------------------ */
TaskObj_t algorithmCtrlTask;

uint8_t CM_connect_signal = 0;
uint8_t CM_disconnect_signal = 0;

uint8_t motionMap_selection = 99;
uint8_t startPvector_decoding = 0;

//ControlMode controlMode = DEFAULT_CONTRL_MODE;
ControlMode controlMode = USER_DEFINED_CTRL;

float EMG_Rawsignal = 0.0f;
float EMG_R1_Rawsignal = 0.0f;
float EMG_L1_Rawsignal = 0.0f;

uint16_t fsr1_R1 = 0;
uint16_t fsr2_R2 = 0;
uint16_t fsr1_L1 = 0;
uint16_t fsr2_L2 = 0;

float free_var1 = -1.0f;
float free_var2 = -1.0f;

RobotData_t robotDataObj_RH;
RobotData_t robotDataObj_LH;
GravComp gravCompDataObj_RH;
GravComp gravCompDataObj_LH;
ImpedanceCtrl impedanceCtrl_RH;
ImpedanceCtrl impedanceCtrl_LH;
StepCurr StepCurr_RH;
StepCurr StepCurr_LH;
UserDefinedCtrl UserDefinedCtrl_RH;
UserDefinedCtrl UserDefinedCtrl_LH;

/* For Code Time Check */
static uint32_t STUDENTcodeStartTick = 0;
static uint32_t STUDENTcodeEndTick = 0;
static uint32_t algorithmCtrlLoopCnt;
static float algorithmCtrlTimeElap;

bool isFirstPos_RH = true;
bool isFirstPos_LH = true;
bool isFirstImp_RH = true;
bool isFirstImp_LH = true;

bool pVectorTrig_RH = false;
bool pVectorTrig_LH = false;
bool fVectorTrig_RH = false;
bool fVectorTrig_LH = false;
uint8_t MotionMap_ID_RH = 0;
uint8_t MotionMap_ID_LH = 0;

float RightHipFlexionTorque = 0.0f;
float RightHipExtensionTorque = 0.0f;
float LeftHipFlexionTorque = 0.0f;
float LeftHipExtensionTorque = 0.0f;

static uint32_t lastUpdateCnt = 0;
static uint8_t currentAmp = 0;
static bool done = false;

uint8_t ackSignal = 0;

// Force application 관련

float f_vector_input_RH = 0.0f;
float f_vector_input_LH = 0.0f;

float assist_level = 1.0f;

bool Rflag_assist = false;
bool Lflag_assist = false;

bool Rstop_assist = false;
bool Lstop_assist = false;

// KNN algorithm - start

uint8_t loop_count = 0;
float Rdeg_now = 0;
float Ldeg_now = 0;
float Rdeg_filtered = 0;
float Ldeg_filtered = 0;
float Rdeg[3] = {0.0f, 0.0f, 0.0f};
float Ldeg[3] = {0.0f, 0.0f, 0.0f};

// 필터 계수 정의 - theta 용 (float 또는 double로 필요에 맞게 설정)
float y_prev_R = 0.0f;
float y_prev_L = 0.0f;
float fs = 1000.0f;
float fc = 10.0f;
float alpha = 0.0591174f; // RC = 1.0f / (2.0f * 3.1415926f * fc), T = 1.0f / 1000, alpha = T / (T + RC), 10 Hz

// 필터 계수 정의 - w 용
float fc_omega = 25.0f;
float alpha_omega = 0.1357552; // RC = 1.0f / (2.0f * 3.1415926f * fc), T = 1.0f / 1000, alpha = T / (T + RC), 25 Hz
float w_prev_R = 0.0f;
float w_prev_L = 0.0f;

static float LPF(float x, float* y_prev, float alpha);

bool R_time_upcond = 0;
bool L_time_upcond = 0;
bool HC_time_upcond = 0;
bool HC_Rswing4upcond = 0;
bool HC_Lswing4upcond = 0;

int R_count_upeak = 0;
int R_count_dpeak = 0;
int L_count_upeak = 0;
int L_count_dpeak = 0;
int HC_count = 0;

int t_gap = 400;
float thres_up = 10;
float thres_down = 10;
float R_lowpeak_val = 10;
float L_lowpeak_val = 10;
float R_uppeak_val = 50;
float L_uppeak_val = 50;
float var = 0; // -3.0e-8

int R_time_afterupFlag = 0;
int L_time_afterupFlag = 0;
int HC_time_afterFlag = 0;
int swing_period = 400;

// Indexes
float vel_HC = 0;
int R_swing_time = 0;
int L_swing_time = 0;
int T_HC = 0;
float norm_vel_HC = 0;
float norm_T_HC = 0;

float scaling_X = 105.6426;
float scaling_Y = 835.8209;

// KNN algorithm variable
#ifndef KNN_REF_COUNT
#  define KNN_REF_COUNT    REF_COUNT          // 헤더 값 사용
#endif
#ifndef KNN_REF_SPLIT
#  define KNN_REF_SPLIT    REF_TYPE1_COUNT    // 헤더 값 사용
#endif
#ifndef KNN_K
#  define KNN_K            9                 // 기본 K (필요시 프로젝트 설정/코드에서 변경)
#endif

static int R_g_knn_label = 0;	// 1 또는 2
static int L_g_knn_label = 0;	// 1 또는 2
static float g_knn_conf = 0.0f;	// 0~1 확신도

static int knn_2label_majority(const float (*xy)[2], int n, int split, float x, float y, int k, float* conf_out); // 비가중 KNN
static int update_standing_moving_flag_w(float wR_metric, float wL_metric);


// KNN algorithm - end

/* -------------------------------------------------------- */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

// Updata Data
static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj);
static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj);

// PIF Vector Trigger
static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);
static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft);
static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft);

// Position Controller
static void InitPositionControl(PIDObject* posCtrl);
static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj);
static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl);

// Gravity Compensator
static void InitGravityCompensation(GravComp* gravComp);
static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj);

// Impedance Controller
static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl);
static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp);
static void error_filter2(ImpedanceCtrl *impedanceCtrl);

// Input Saturation
static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input);
/* -------------------------------------------------------- */

/*---------- (1) START of STUDENT CODE (Declare the functions to use) -----------*/

/*---------- (1) END of STUDENT CODE (Declare the functions to use) -------------*/

DOP_COMMON_SDO_CB(algorithmCtrlTask)

void InitAlgorithmCtrl(void)
{
    InitTask(&algorithmCtrlTask);

	InitFvectorMaxTorque(&fvectorObj_RH, RH_MOTOR);
	InitFvectorMaxTorque(&fvectorObj_LH, LH_MOTOR);

	/* State Definition */
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,		StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&algorithmCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_STUDENTS);

	// PDO
	/* For PDO setting */

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_STUDENTS)

	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM3) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM3, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunAlgorithmCtrl, NULL);
}

void RunAlgorithmCtrl(void* params)
{
	/* Loop Start Time Check */
	STUDENTcodeStartTick = DWT->CYCCNT;

	ackSignal = !ackSignal; // 0과 1 토글

	/* Run Device */
	RunTask(&algorithmCtrlTask);

	/* Elapsed Time Check */
	STUDENTcodeEndTick = DWT->CYCCNT;
	if (STUDENTcodeEndTick < STUDENTcodeStartTick) {
		algorithmCtrlTimeElap = ((4294967295 - STUDENTcodeStartTick) + STUDENTcodeEndTick) / 480;	// in microsecond (Roll-over)
	}
	else {
		algorithmCtrlTimeElap = (DWT->CYCCNT - STUDENTcodeStartTick) / 480;							// in microsecond
	}
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
	StateTransition(&algorithmCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	EntRoutines(&algorithmCtrlTask.routine);

	InitPositionControl(&posCtrl_RH);
	InitPositionControl(&posCtrl_LH);

	InitGravityCompensation(&gravCompDataObj_RH);
	InitGravityCompensation(&gravCompDataObj_LH);

	InitImpedanceSetting(&impedanceCtrl_RH);
	InitImpedanceSetting(&impedanceCtrl_LH);

	algorithmCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{

	if (CM_connect_signal == 1) {
		Send_ExtensionBoardEnable();
		CM_connect_signal = 0;
	}

	RunRoutines(&algorithmCtrlTask.routine);

	/*---------------------------- Data gathering (DO NOT CHANGE THIS) ----------------------------*/
	UpdateRobotData(&robotDataObj_RH, &StudentsDataObj_RH);
	UpdateRobotData(&robotDataObj_LH, &StudentsDataObj_LH);
	UpdateExtensionBoardData(&ExtPackDataObj);
	/*---------------------------------------------------------------------------------------------*/

	/*------------------------- Control Sample Code ------------------------*/
	if ((SUIT_State_curr >= 3 && SUIT_State_curr <= 17) || (SUIT_State_curr >= 33 && SUIT_State_curr <= 45) ||
		(SUIT_State_curr >= 66 && SUIT_State_curr <= 75)) {
		if (controlMode == POSITION_CTRL) {	// 1 - Position Control
			// Init Position For Safety
			if (isFirstPos_RH == true) {
				InitPosCtrlHoming(&posCtrl_RH, &pVectorTrig_RH, &robotDataObj_RH, &pvectorObj_RH);
				isFirstPos_RH = false;
			} else {
				PvectorTrigger(&pvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &pVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_RH, &posCtrl_RH);
			}
			if (isFirstPos_LH == true) {
				InitPosCtrlHoming(&posCtrl_LH, &pVectorTrig_LH, &robotDataObj_LH, &pvectorObj_LH);
				isFirstPos_LH = false;
			} else {
				PvectorTrigger(&pvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &pVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
				PositionCtrl_Sample(&robotDataObj_LH, &posCtrl_LH);
			}
			
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == GRAVITY_COMPENSATION) {	// 2 - Gravity Compensation
			GravityCompensation_Sample(&gravCompDataObj_RH, &robotDataObj_RH);
			GravityCompensation_Sample(&gravCompDataObj_LH, &robotDataObj_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		} else if (controlMode == IMPEDANCE_CTRL) {   // 3 - Impedance Control
			ImpedanceControl_Sample(&impedanceCtrl_RH, &robotDataObj_RH, &posCtrl_RH, &isFirstImp_RH);
			ImpedanceControl_Sample(&impedanceCtrl_LH, &robotDataObj_LH, &posCtrl_LH, &isFirstImp_LH);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == TORQUE_CTRL) {   // 4 - Torque Control
			FvectorTrigger(&fvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &fVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
			FvectorTrigger(&fvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &fVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
		} else if (controlMode == STEP_CURRENT_CTRL) {   // 5 - Step Current Control
		    // 매 루프에서 전류 입력 지정 (초기화 방지)
		    posCtrl_RH.control_input = 0.0f;
		    posCtrl_LH.control_input = 0.0f;
		    gravCompDataObj_RH.control_input = 0.0f;
		    gravCompDataObj_LH.control_input = 0.0f;
		    impedanceCtrl_RH.control_input = 0.0f;
		    impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
		    UserDefinedCtrl_RH.control_input = 0.0f;
		    UserDefinedCtrl_LH.control_input = 0.0f;

		    StepCurr_RH.control_input = (float)currentAmp;

		    if (!done && (algorithmCtrlLoopCnt - lastUpdateCnt) >= 3000)
		    {
		        if (currentAmp < 4) {
		            currentAmp++;
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		        else {
		            currentAmp = 0;     // 3초 유지 후 0A로
		            done = true;        // 이후에는 다시 증가하지 않음
		            lastUpdateCnt = algorithmCtrlLoopCnt;
		        }
		    }

		} else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control

			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;

			assist_level = 1.5f;

			MotionMap_ID_RH = 1;
			MotionMap_ID_LH = 0;
			FvectorTrigger(&fvectorObj_RH, &MotionMap_File, &robotDataObj_RH, &fVectorTrig_RH, &MotionMap_ID_RH, RH_MOTOR);
			FvectorTrigger(&fvectorObj_LH, &MotionMap_File, &robotDataObj_LH, &fVectorTrig_LH, &MotionMap_ID_LH, LH_MOTOR);

			loop_count++;
			R_time_afterupFlag++;
			L_time_afterupFlag++;
			HC_time_afterFlag++;   // ← 추가
			R_swing_time++;
			L_swing_time++;

			Rdeg_now = robotDataObj_RH.thighTheta_act;
			Rdeg_filtered = LPF(Rdeg_now, &y_prev_R, alpha);
			Ldeg_now = robotDataObj_LH.thighTheta_act;
			Ldeg_filtered = LPF(Ldeg_now, &y_prev_L, alpha);

			Rdeg[0] = Rdeg[1];
			Ldeg[0] = Ldeg[1];
			Rdeg[1] = Rdeg[2];
			Ldeg[1] = Ldeg[2];
			Rdeg[2] = Rdeg_filtered;
			Ldeg[2] = Ldeg_filtered;

			float wR = (Rdeg[2] - Rdeg[1]) * fs;
			float wL = (Ldeg[2] - Ldeg[1]) * fs;

			float wR_abs = fabsf(wR);
			float wL_abs = fabsf(wL);
			float wR_f = LPF(wR_abs, &w_prev_R, alpha_omega);
			float wL_f = LPF(wL_abs, &w_prev_L, alpha_omega);

			int is_moving = update_standing_moving_flag_w(wR_f, wL_f);

			free_var1 = -1.0f;
			free_var2 = -1.0f;

			// Time gap check

			if (R_time_afterupFlag >= t_gap) {
				R_time_upcond = 1;
			}
			if (L_time_afterupFlag >= t_gap) {
				L_time_upcond = 1;
			}

			if (HC_time_afterFlag >= t_gap) {
				HC_time_upcond = 1;
			}

			// HC check
			if (HC_time_upcond && (Rdeg[2] - Ldeg[2])*(Rdeg[1] - Ldeg[1]) <= 0 && Rdeg[2] > thres_down && Ldeg[2] > thres_down){
				HC_time_upcond = 0;
				HC_time_afterFlag = 0;
				HC_count++;
				if (Rdeg[2] - Rdeg[1] >= Ldeg[2] - Ldeg[1] && L_uppeak_val > 40){ // if it is Rswing
					HC_Rswing4upcond = 1;
					vel_HC = (Rdeg[2] - Rdeg[1]) / 0.001f;
					T_HC = R_swing_time;

					norm_vel_HC = (vel_HC * (swing_period * 0.001f)) / scaling_X;
					norm_T_HC = (T_HC / (swing_period * 0.001f)) / scaling_Y;

					float x = norm_vel_HC;
					float y = norm_T_HC;

					R_g_knn_label = (int)knn_2label_majority(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT, x, y, KNN_K, &g_knn_conf);

					free_var2 = (int)R_g_knn_label; // 1 또는 2

					if (R_g_knn_label == 1){
						Rstop_assist = true;
					}
					else if (R_g_knn_label == 2){
					}
				}
				else if (Ldeg[2] - Ldeg[1] >= Rdeg[2] - Rdeg[1] && R_uppeak_val > 40){ // if it is Lswing
					HC_Lswing4upcond = 1;
					vel_HC = (Ldeg[2] - Ldeg[1]) / 0.001f;
					T_HC = L_swing_time;

					norm_vel_HC = (vel_HC * (swing_period * 0.001f)) / scaling_X;
					norm_T_HC = (T_HC / (swing_period * 0.001f)) / scaling_Y;

					float x = norm_vel_HC;
					float y = norm_T_HC;

					L_g_knn_label = (int)knn_2label_majority(ref_xy, KNN_REF_COUNT, KNN_REF_SPLIT, x, y, KNN_K, &g_knn_conf);

					free_var2 = (int)L_g_knn_label; // 1 또는 2

					if (L_g_knn_label == 1){
						Lstop_assist = true;
					}
					else if (L_g_knn_label == 2){
					}
				}
			}

			if (Rstop_assist){
				f_vector_input_RH = 0.0f;
			}
			else if (Lstop_assist) {
				f_vector_input_LH = 0.0f;
			}

			// Upper peak detection

//			if ((Rdeg[2] - Rdeg[1])*(Rdeg[1]-Rdeg[0]) < var && Rdeg[2] - Rdeg[1] < 0 && (Rdeg[2] >= thres_up) && R_time_upcond && HC_Rswing4upcond) {
			if ((Rdeg[2] - Rdeg[1])*(Rdeg[1]-Rdeg[0]) < var && Rdeg[2] - Rdeg[1] < 0 && (Rdeg[2] >= thres_up) && R_time_upcond) {
				R_uppeak_val = Rdeg[2];
				R_time_upcond = 0;
				R_count_upeak++;
				swing_period = R_swing_time;
				R_time_afterupFlag = 0;
				HC_Rswing4upcond = 0;
			}

//			if ((Ldeg[2] - Ldeg[1])*(Ldeg[1]-Ldeg[0]) < var && Ldeg[2] - Ldeg[1] < 0 && Ldeg[2] >= thres_up && L_time_upcond && HC_Lswing4upcond) {
				if ((Ldeg[2] - Ldeg[1])*(Ldeg[1]-Ldeg[0]) < var && Ldeg[2] - Ldeg[1] < 0 && Ldeg[2] >= thres_up && L_time_upcond) {
				L_uppeak_val = Ldeg[2];
				L_time_upcond = 0;
				L_count_upeak++;
				swing_period = L_swing_time;
				L_time_afterupFlag = 0;
				HC_Lswing4upcond = 0;
			}

			// Lower peak detection
			if (Rdeg[2] > R_lowpeak_val + 10) {
				R_lowpeak_val = thres_down;
			}
			if ((Rdeg[2]-Rdeg[1])*(Rdeg[1]-Rdeg[0]) < var && Rdeg[2] - Rdeg[1] >= 0 && Rdeg[2] <= R_lowpeak_val + 3) {
				R_count_dpeak = R_count_dpeak + 1;
				R_lowpeak_val = Rdeg[2];
				R_swing_time = 0;
				if (is_moving) {
					R_g_knn_label = 0;
					Rstop_assist = false;
					Rflag_assist = 1;
				}
			}

			if (Ldeg[2] > L_lowpeak_val + 10) {
				L_lowpeak_val = thres_down;
			}
			if ((Ldeg[2]-Ldeg[1])*(Ldeg[1]-Ldeg[0]) < var && Ldeg[2] - Ldeg[1] >= 0 && Ldeg[2] <= L_lowpeak_val + 3) {
				L_count_dpeak = L_count_dpeak + 1;
				L_lowpeak_val = Ldeg[2];
				L_swing_time = 0;
				if (is_moving) {
					L_g_knn_label = 0;
					Lstop_assist = false;
					Lflag_assist = 1;
				}
			}

			if (Rflag_assist && Rdeg[2] > R_lowpeak_val + 5) {
				fVectorTrig_RH = true;
				Rflag_assist = 0;
				free_var1 = fVectorTrig_RH;
			}
			else if (Lflag_assist && Ldeg[2] > L_lowpeak_val + 5) {
				fVectorTrig_LH = true;
				Lflag_assist = 0;
				free_var1 = fVectorTrig_LH;
			}

		} else {
			// default : SUIT H10 Assist Mode
			posCtrl_RH.control_input = 0.0f;
			posCtrl_LH.control_input = 0.0f;
			gravCompDataObj_RH.control_input = 0.0f;
			gravCompDataObj_LH.control_input = 0.0f;
			impedanceCtrl_RH.control_input = 0.0f;
			impedanceCtrl_LH.control_input = 0.0f;
			f_vector_input_LH = 0.0f;
			f_vector_input_RH = 0.0f;
			StepCurr_RH.control_input = 0.0f;
			StepCurr_LH.control_input = 0.0f;
			UserDefinedCtrl_RH.control_input = 0.0f;
			UserDefinedCtrl_LH.control_input = 0.0f;
			isFirstPos_RH = true;
			isFirstPos_LH = true;
			isFirstImp_RH = true;
			isFirstImp_LH = true;
		}
	}
	/*---------------------- END of Control Sample Code ---------------------*/

	/*--------------------- (2) START of STUDENT CODE (Write your code in this section) --------------------*/

	/*------------------------------------ (2) END of STUDENT CODE-----------------------------------------*/
	// Control Input Saturation (Do not Delete)
	ControlInputSaturation(&robotDataObj_RH, &posCtrl_RH, &gravCompDataObj_RH, &impedanceCtrl_RH, &StepCurr_RH , &UserDefinedCtrl_RH, f_vector_input_RH);
	ControlInputSaturation(&robotDataObj_LH, &posCtrl_LH, &gravCompDataObj_LH, &impedanceCtrl_LH, &StepCurr_LH , &UserDefinedCtrl_LH, f_vector_input_LH);
	algorithmCtrlLoopCnt++;
}

// KNN algorithm function - start
static inline float clamp01f(float v){ return (v<0.0f)?0.0f : (v>1.0f?1.0f:v); }

static float LPF(float x, float* y_prev, float alpha) {
    *y_prev = (1.0f - alpha) * (*y_prev) + alpha * x;
    return *y_prev;
}

static int knn_2label_majority(const float (*xy)[2], int n, int split,
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
        uint8_t lab = (i < split) ? 1 : 2;

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

    // 비가중 다수결
    int c1 = 0, c2 = 0;
    for (int i = 0; i < used; ++i) {
        if (best_lab[i] == 1) ++c1; else ++c2;
    }

    int label = (c1 >= c2) ? 1 : 2;      // 동률이면 1 선택(기존 로직과 동일한 타이브레이크)
    if (conf_out) {
        int maxc = (c1 >= c2) ? c1 : c2;
        *conf_out = (used > 0) ? ((float)maxc / (float)used) : 0.0f;  // 확신도=다수 비율
    }
    return label;
}

static int update_standing_moving_flag_w(float wR_metric, float wL_metric)
{
    /* === 튜닝 파라미터 ===
       - V_ON/OFF: 속도 히스테리시스(상→동, 동→정)
       - *_HOLD_TICKS: 전환 전 연속 유지시간(1kHz 기준) */
    const float V_ON_THRESH   = 40.0f;   /* 이 이상이면 moving 증거 */
    const float V_OFF_THRESH  = 15.0f;   /* 이 이하면 standing 증거 */
    const int   ON_HOLD_TICKS = 10;      /* STANDING→MOVING 전환 대기 */
    const int   OFF_HOLD_TICKS= 50;     /* MOVING→STANDING 전환 대기 */

    typedef enum { GAIT_STANDING = 0, GAIT_MOVING = 1 } gait_state_t;

    /* 내부 상태(호출 간 유지) */
    static gait_state_t s_state = GAIT_STANDING;
    static int s_on_cnt = 0, s_off_cnt = 0;

    /* 두 다리 중 큰 속도 지표 사용 */
    float v_metric = (wR_metric > wL_metric) ? wR_metric : wL_metric;

    /* 히스테리시스 증거 */
    int evidence_move  = (v_metric >= V_ON_THRESH);
    int evidence_stand = (v_metric <= V_OFF_THRESH);

    /* 상태머신 */
    if (s_state == GAIT_STANDING) {
        if (evidence_move) {
            if (++s_on_cnt >= ON_HOLD_TICKS) {
                s_state = GAIT_MOVING; s_on_cnt = 0; s_off_cnt = 0;
            }
        } else {
            s_on_cnt = 0;
        }
    } else { /* GAIT_MOVING */
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


// KNN algorithm function - end

static void StateEnable_Ext(void)
{
    ExtRoutines(&algorithmCtrlTask.routine);
}

static void StateError_Run(void)
{

}

static void UpdateRobotData(RobotData_t* robotDataObj, StudentsData_t* StudentsDataObj)
{
	robotDataObj->thighTheta_act = StudentsDataObj->thighThetaAct;
	robotDataObj->position_act = StudentsDataObj->positionAct;
	robotDataObj->accX = StudentsDataObj->accX;
	robotDataObj->accY = StudentsDataObj->accY;
	robotDataObj->gyrZ = StudentsDataObj->gyrZ;
}

static void UpdateExtensionBoardData(Extpack_Data_t* ExtPackDataObj)
{
	// EMG_Rawsignal = ExtPackDataObj->emg_data.emg_R2_rawSign[0] / 2048.0f;
	EMG_R1_Rawsignal = ExtPackDataObj->emg_data.emg_R1_rawSign[0] / 2048.0f;
	EMG_L1_Rawsignal = ExtPackDataObj->emg_data.emg_L1_rawSign[0] / 2048.0f;

	fsr2_R2 = ExtPackDataObj->fsr_data.fsr_R2_raw;
	fsr1_L1 = ExtPackDataObj->fsr_data.fsr_L1_raw;
	fsr2_L2 = ExtPackDataObj->fsr_data.fsr_L2_raw;
}

static void PvectorTrigger(P_Vector_Decoder* pvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*pvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].p_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}
	}
}

static void InitFvectorMaxTorque(F_Vector_Decoder* fvectorObj, uint8_t isLeft)
{
	memset(fvectorObj, 0, sizeof(*fvectorObj));

	// Max 8~10Nm이나 매우 위험할 수 있으므로 Test시에는 2~3Nm로 실험 하세요
	// default 8Nm
	// 로봇 설정 최대치 10Nm
	if (isLeft) {
		LeftHipFlexionTorque = 2.0f;	// 3Nm, saturation
		LeftHipExtensionTorque = 2.0f;	// 3Nm
	} else {
		RightHipFlexionTorque = 2.0f;	// 3Nm
		RightHipExtensionTorque = 2.0f;	// 3Nm
	}
}

static void FvectorTrigger(F_Vector_Decoder* fvectorObj, MotionMapFileInfo* MotionMap_File, 
							RobotData_t* robotDataObj, bool* triggerSignal, uint8_t* MM_ID, uint8_t isLeft)
{
	if (*triggerSignal == true) {
		*triggerSignal = false;			// Reset

		if (*MM_ID >= 0 && *MM_ID < 40) {
			*fvectorObj = MotionMap_File->MS[*MM_ID].MD[isLeft].f_vector_decoder;		// Assign
			// *MM_ID = 99;		// Reset
		}

		// global variable ID에 따른 Torque Max assign
		float t_tauMax = 0.0f;
		for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
			if (fvectorObj->f_buffer[i].globalVariableID == 0x01) {
				t_tauMax = RightHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x02) {
				t_tauMax = RightHipExtensionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x03) {
				t_tauMax = LeftHipFlexionTorque;
			}
			if (fvectorObj->f_buffer[i].globalVariableID == 0x04) {
				t_tauMax = LeftHipExtensionTorque;
			}

			// Torque -> Current Input Conversion
			fvectorObj->f_buffer[i].tau_max = (t_tauMax / GEAR_RATIO / MOTOR_TORQUE_CONSTANT) * (fvectorObj->f_buffer[i].coefficient * 0.01);

			// F vector Reset Logic
			// if (fvectorObj->f_buffer[i].mode_idx == 255) {
			// 	if (isLeft) f_vector_input_LH = 0.0f;
			// 	else f_vector_input_RH = 0.0f;
			// 	fvectorObj->f_buffer[i].mode_idx = 0;
			// 	fvectorObj->f_buffer[i].tau_max = 0;
			// 	fvectorObj->f_buffer[i].delay = 0;
			// 	fvectorObj->f_buffer[i].u = 0;
			// 	fvectorObj->f_buffer[i].u_old1 = 0;
			// 	fvectorObj->f_buffer[i].u_old2 = 0;
			// 	fvectorObj->f_buffer[i].tau = 0;
			// 	fvectorObj->f_buffer[i].tau_old1 = 0;
			// 	fvectorObj->f_buffer[i].tau_old2 = 0;
			// 	fvectorObj->f_buffer[i].t_end = 0;
			// 	fvectorObj->f_buffer[i].time_stamp = 0;
			// 	fvectorObj->f_buffer[i].is_full = 0;
			// }
		}
	}
}

static void InitPositionControl(PIDObject* posCtrl)
{
	memset(posCtrl, 0, sizeof(*posCtrl));

	posCtrl->Kp = 1.5;
	posCtrl->Kd = 0.2;
}

static void InitPosCtrlHoming(PIDObject* posCtrl, bool* triggerSignal, RobotData_t* robotDataObj, P_Vector_Decoder* pvectorObj)
{
	pvectorObj->yd_f = robotDataObj->position_act * M_PI / 180.0f;
	pvectorObj->N = 1;
	pvectorObj->p_buffer->yd = 0;
	pvectorObj->p_buffer->s0 = 60;
	pvectorObj->p_buffer->sd = 60;
	pvectorObj->p_buffer->L = 3000;
}

static void PositionCtrl_Sample(RobotData_t* robotDataObj, PIDObject* posCtrl)
{
	posCtrl->err = (posCtrl->ref) - (robotDataObj->position_act * M_PI / 180);
	posCtrl->err_diff = (posCtrl->err - posCtrl->err_prev) / DT;
	posCtrl->err_prev =  posCtrl->err;

	posCtrl->control_input = posCtrl->Kp * posCtrl->err + posCtrl->Kd * posCtrl->err_diff;
}

static void InitGravityCompensation(GravComp* gravComp)
{
	memset(gravComp, 0, sizeof(*gravComp));

	gravComp->grav_gain = 0.5;
	gravComp->grav_alpha = 0.99;
}

static void GravityCompensation_Sample(GravComp* gravComp, RobotData_t* robotDataObj)
{
	if (robotDataObj->thighTheta_act > 0) 
	{
		gravComp->grav_comp_torque =  gravComp->grav_gain * (sin((robotDataObj->thighTheta_act) * M_PI / 180));
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}
	else
	{
		gravComp->grav_comp_torque = 0;
		gravComp->f_grav_comp_torque = gravComp->grav_alpha * gravComp->f_grav_comp_torque + (1 - gravComp->grav_alpha) * gravComp->grav_comp_torque;
	}

	gravComp->control_input = gravComp->f_grav_comp_torque;
}

static void InitImpedanceSetting(ImpedanceCtrl* impedanceCtrl)
{
	memset(impedanceCtrl, 0, sizeof(*impedanceCtrl));
	
	impedanceCtrl->epsilon = 5.0f * M_PI / 180.0f;
	impedanceCtrl->Kp = 1.5f;
	impedanceCtrl->Kd = 0.2f;
	impedanceCtrl->lambda = 1.0f;
	impedanceCtrl->duration = 300.0f;
}

static void ImpedanceControl_Sample(ImpedanceCtrl* impedanceCtrl, RobotData_t* robotDataObj, PIDObject* posCtrl, bool* isFirstImp)
{
	float t_epsilon = 0.0f;
	float t_Kp = 0.0f;
	float t_Kd = 0.0f;
	float t_lambda = 0.0f;

	if (*isFirstImp == true && impedanceCtrl->ON == 0) {
		posCtrl->ref = robotDataObj->position_act * M_PI / 180.0f;
		t_epsilon = impedanceCtrl->epsilon; // unit: rad   (0.001745329252 = 0.1 * pi/180)
		t_Kp      = impedanceCtrl->Kp;
		t_Kd      = impedanceCtrl->Kd;
		t_lambda  = impedanceCtrl->lambda;

		if (impedanceCtrl->duration > 0) {
			float invT      = 1/impedanceCtrl->duration;

			impedanceCtrl->gap_epsilon = (t_epsilon - impedanceCtrl->epsilon) * invT;
			impedanceCtrl->gap_Kp      = (t_Kp      - impedanceCtrl->Kp)      * invT;
			impedanceCtrl->gap_Kd      = (t_Kd      - impedanceCtrl->Kd)      * invT;
			impedanceCtrl->gap_lambda  = (t_lambda  - impedanceCtrl->lambda)  * invT;
		}

		impedanceCtrl->i  = 0; // initialize 1ms counter
		impedanceCtrl->ON = 1;
		*isFirstImp = false;
	}

	if (impedanceCtrl->ON == 1) {
		if (impedanceCtrl->duration == 0) {
			impedanceCtrl->epsilon = t_epsilon;
			impedanceCtrl->Kp      = t_Kp;
			impedanceCtrl->Kd      = t_Kd;
			impedanceCtrl->lambda  = t_lambda;
		} else {
			impedanceCtrl->epsilon = impedanceCtrl->epsilon + impedanceCtrl->gap_epsilon;
			impedanceCtrl->Kp      = impedanceCtrl->Kp      + impedanceCtrl->gap_Kp;
			impedanceCtrl->Kd      = impedanceCtrl->Kd      + impedanceCtrl->gap_Kd;
			impedanceCtrl->lambda  = impedanceCtrl->lambda  + impedanceCtrl->gap_lambda;
			impedanceCtrl->i++;
		}

		if (impedanceCtrl->i >= impedanceCtrl->duration)	{
			impedanceCtrl->ON = 0;
			impedanceCtrl->i = 0;
		}
	}

	/* Impedance Controller */
	impedanceCtrl->e = posCtrl->ref - (robotDataObj->position_act * M_PI / 180.0f);

	error_filter2(impedanceCtrl);

	float t_ef_diff = 0.0;

	if (((impedanceCtrl->ef > 0) & (impedanceCtrl->ef_diff > 0)) | ((impedanceCtrl->ef <= 0) & (impedanceCtrl->ef_diff <= 0))) {
		t_ef_diff = +impedanceCtrl->ef_diff;
	} else {
		t_ef_diff = -impedanceCtrl->ef_diff;
	}

	impedanceCtrl->control_input = impedanceCtrl->Kp * impedanceCtrl->ef + impedanceCtrl->Kd * t_ef_diff;
}

static void error_filter2(ImpedanceCtrl *impedanceCtrl)
{
	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
	float t_abs_e = 0.0;
	float t_sign_e = 0.0;
	float t_max = 0.0;
	float t_diff = 0.0;
	float y_ef = 0.0;

	/* Calculate 'sign(e) & |e|' */
	if (impedanceCtrl->e > 0)	{t_abs_e = +impedanceCtrl->e; t_sign_e = +1; }
	else						{t_abs_e = -impedanceCtrl->e; t_sign_e = -1; }

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0;}

	y_ef = (impedanceCtrl->lambda * impedanceCtrl->e) + (1 - impedanceCtrl->lambda) * t_sign_e * t_max;

	impedanceCtrl->ef_diff = (y_ef - impedanceCtrl->ef) * 0.001;

	impedanceCtrl->ef = y_ef;
}

static void ControlInputSaturation(RobotData_t *robotDataObj, PIDObject *posCtrl, GravComp *gravComp, ImpedanceCtrl *impedanceCtrl, StepCurr *StepCurr, UserDefinedCtrl *UserDefinedCtrl, float f_vector_input)
{
	// Saturate control input to ±9 using fminf and fmaxf
	robotDataObj->u_totalInput = fminf(fmaxf((posCtrl->control_input + /* control input of PID position controller */
								gravComp->control_input + /* control input of gravity compensation */
								impedanceCtrl->control_input + /* control input of impedance controller */
								StepCurr->control_input + /* control input of step current input controller */
								UserDefinedCtrl->control_input + /* control input of user defined controller */
								f_vector_input) * assist_level, -9.0f), 9.0f); /* control input of F-vector decoded */
								// assist_level 0 ~ 1.0 (0~100%, 0.1, 10%단위)
}

/*----------- (3) START of STUDENT CODE (Define the functions to use) ------------*/





/*------------ (3) END of STUDENT CODE (Define the functions to use) -------------*/
