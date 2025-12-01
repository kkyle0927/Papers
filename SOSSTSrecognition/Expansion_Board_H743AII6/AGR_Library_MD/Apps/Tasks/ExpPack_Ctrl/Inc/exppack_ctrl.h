#ifndef EXPPACK_CTRL_INC_EXPPACK_CTRL_H_
#define EXPPACK_CTRL_INC_EXPPACK_CTRL_H_

#include "module.h"

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ioif_tim_common.h"
#include "error_dictionary.h"

#include "msg_hdlr.h"
#include "algorithm_ctrl.h"
#include "data_object_common.h"
#include "ioif_adc_common.h"
#include "ioif_ms5607_02ba03.h"
#include "ioif_ra30p.h"
#include "ioif_psl_iemg2.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* Define the sensors to use */
// (1) pMMG sensor //
//#define PMMG_1A_ENABLE
//#define PMMG_1B_ENABLE
//#define PMMG_2A_ENABLE
//#define PMMG_2B_ENABLE
//#define PMMG_3A_ENABLE
//#define PMMG_3B_ENABLE
//#define PMMG_4A_ENABLE
//#define PMMG_4B_ENABLE

// (2) FSR sensor //
#define FSR_L1_ENABLE
#define FSR_L2_ENABLE
//#define FSR_L3_ENABLE
//#define FSR_L4_ENABLE
#define FSR_R1_ENABLE
#define FSR_R2_ENABLE
//#define FSR_R3_ENABLE
//#define FSR_R4_ENABLE

// (3) EMG sensor //
#define EMG_L1_ENABLE
//#define EMG_L2_ENABLE
//#define EMG_L3_ENABLE
//#define EMG_L4_ENABLE
#define EMG_R1_ENABLE
//#define EMG_R2_ENABLE
//#define EMG_R3_ENABLE
//#define EMG_R4_ENABLE


/* Define the Sampling Period for USB CDC */
//#define USB_CDC_ACTIVATE
#define PERIOD_USB_CDC    				3		// ms
#define USB_CDC_TERMINATE_PYTHON		78		// can be changed


/* For Data Save To Hex (for 2Byte Data Processing) */
#define DATA_CONV_CONST_UINT16              65535       // uint16 conversion constant
#define DATA_CONV_CONST_INT16               32768       // int16 conversion constant
#define PMMG_SCALING_FACTOR					200 		// 0~200kPa (It can be changed)
#define EMG_SCALING_FACTOR					4096		// ADC 12Bit Resolution
#define EMG_NORM_SCALING_FACTOR				1			// Normalized EMG [0,1]
#define EMG_RAWSIGN_SCALING_FACTOR			2			// Normalized EMG [-1,1]
#define EMG_RAW_12BIT_SCALING_FACTOR		4096		// Raw EMG [-2048,2047]
#define FSR_SCALING_FACTOR					4096		// ADC 12Bit Resolution

// Have to change //
#define CONTROL_INPUT_SCALING_FACTOR		30			// [-15,15]


// For MD->CM->EXTboard //
#define DEG_SCALING_FACTOR              	720         // (deg) -360 ~ +360
#define ACC_SCALING_FACTOR              	78.4532     // (m/s^2) 8 * g(9.80665) -39.24 ~ +39.24
#define GYR_SCALING_FACTOR              	1000        // (deg/s) -500 ~ +500

/* For processing */
#define EMG_MA_BUFF_SIZE					100

/* Mode Selection (Button Mode: for independently using of Expansion Board) */
//#define BUTTON_MODE

#define EXTPACK_CONTROL_PERIOD				0.001		// 1ms
#define EXTPACK_CONTROL_FREQUENCY			1000		// 1ms

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef struct _pMMG_pressure_t {
	float pMMG1A_press;
	float pMMG1B_press;
	float pMMG2A_press;
	float pMMG2B_press;
	float pMMG3A_press;
	float pMMG3B_press;
	float pMMG4A_press;
	float pMMG4B_press;
} pMMG_pressure_t;

typedef struct _pMMG_temperature_t {
	float pMMG1A_temp;
	float pMMG1B_temp;
	float pMMG2A_temp;
	float pMMG2B_temp;
	float pMMG3A_temp;
	float pMMG3B_temp;
	float pMMG4A_temp;
	float pMMG4B_temp;
} pMMG_temperature_t;

typedef struct _pMMG_err_t {
	uint32_t err1A;
	uint32_t err1B;
	uint32_t err2A;
	uint32_t err2B;
	uint32_t err3A;
	uint32_t err3B;
	uint32_t err4A;
	uint32_t err4B;
	uint32_t totalErr;
} pMMG_err_t;

typedef struct _fsr_data_t {
	uint16_t fsr_L1_raw;
	uint16_t fsr_L2_raw;
	uint16_t fsr_L3_raw;
	uint16_t fsr_L4_raw;
	uint16_t fsr_R1_raw;
	uint16_t fsr_R2_raw;
	uint16_t fsr_R3_raw;
	uint16_t fsr_R4_raw;
} fsr_data_t;

typedef struct _emg_data_t {
	/* [0, 4095] */
	uint16_t emg_L1_raw;
	uint16_t emg_L2_raw;
	uint16_t emg_L3_raw;
	uint16_t emg_L4_raw;
	uint16_t emg_R1_raw;
	uint16_t emg_R2_raw;
	uint16_t emg_R3_raw;
	uint16_t emg_R4_raw;

	/* [-2048, 2047] */
	int16_t emg_L1_rawSign[3];
	int16_t emg_L2_rawSign[3];
	int16_t emg_L3_rawSign[3];
	int16_t emg_L4_rawSign[3];
	int16_t emg_R1_rawSign[3];
	int16_t emg_R2_rawSign[3];
	int16_t emg_R3_rawSign[3];
	int16_t emg_R4_rawSign[3];

	/* [0, 2048] */
	float emg_L1_rect;
	float emg_L2_rect;
	float emg_L3_rect;
	float emg_L4_rect;
	float emg_R1_rect;
	float emg_R2_rect;
	float emg_R3_rect;
	float emg_R4_rect;

	/* [0, 1] */
	float emg_L1_norm;
	float emg_L2_norm;
	float emg_L3_norm;
	float emg_L4_norm;
	float emg_R1_norm;
	float emg_R2_norm;
	float emg_R3_norm;
	float emg_R4_norm;

	uint8_t emg_L1_LPFstart;
	uint8_t emg_L2_LPFstart;
	uint8_t emg_L3_LPFstart;
	uint8_t emg_L4_LPFstart;
	uint8_t emg_R1_LPFstart;
	uint8_t emg_R2_LPFstart;
	uint8_t emg_R3_LPFstart;
	uint8_t emg_R4_LPFstart;

	uint8_t emg_L1_BPFstart;
	uint8_t emg_L2_BPFstart;
	uint8_t emg_L3_BPFstart;
	uint8_t emg_L4_BPFstart;
	uint8_t emg_R1_BPFstart;
	uint8_t emg_R2_BPFstart;
	uint8_t emg_R3_BPFstart;
	uint8_t emg_R4_BPFstart;

	/* [-2048, 2047] */
	float emg_L1_LPF[2];
	float emg_L2_LPF[2];
	float emg_L3_LPF[2];
	float emg_L4_LPF[2];
	float emg_R1_LPF[2];
	float emg_R2_LPF[2];
	float emg_R3_LPF[2];
	float emg_R4_LPF[2];

	float emg_L1_BPF[3];
	float emg_L2_BPF[3];
	float emg_L3_BPF[3];
	float emg_L4_BPF[3];
	float emg_R1_BPF[3];
	float emg_R2_BPF[3];
	float emg_R3_BPF[3];
	float emg_R4_BPF[3];

	/* [0, 1] */
	float emg_L1_MA;
	float emg_L2_MA;
	float emg_L3_MA;
	float emg_L4_MA;
	float emg_R1_MA;
	float emg_R2_MA;
	float emg_R3_MA;
	float emg_R4_MA;

	float emg_L1_MA_sum;
	float emg_L2_MA_sum;
	float emg_L3_MA_sum;
	float emg_L4_MA_sum;
	float emg_R1_MA_sum;
	float emg_R2_MA_sum;
	float emg_R3_MA_sum;
	float emg_R4_MA_sum;

	uint16_t emg_L1_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_L2_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_L3_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_L4_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_R1_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_R2_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_R3_MA_Buff[EMG_MA_BUFF_SIZE];
	uint16_t emg_R4_MA_Buff[EMG_MA_BUFF_SIZE];

	uint8_t emg_L1_MA_index;
	uint8_t emg_L2_MA_index;
	uint8_t emg_L3_MA_index;
	uint8_t emg_L4_MA_index;
	uint8_t emg_R1_MA_index;
	uint8_t emg_R2_MA_index;
	uint8_t emg_R3_MA_index;
	uint8_t emg_R4_MA_index;

	uint8_t emg_L1_MA_count;
	uint8_t emg_L2_MA_count;
	uint8_t emg_L3_MA_count;
	uint8_t emg_L4_MA_count;
	uint8_t emg_R1_MA_count;
	uint8_t emg_R2_MA_count;
	uint8_t emg_R3_MA_count;
	uint8_t emg_R4_MA_count;

} emg_data_t;


typedef struct _Exppack_Data_t {
	fsr_data_t fsr_data;
	emg_data_t emg_data;

	pMMG_pressure_t pMMG_press;
	pMMG_temperature_t pMMG_temp;
	pMMG_err_t pMMG_err;

	uint8_t pMMG_activated_A;
	uint8_t pMMG_activated_B;

	float EMG_raw;			// raw EMG data [-1,1]	(received from EXTboard)
	float EMG_processed;	// processed EMG data

	uint8_t fvector_trigger_1;
	uint8_t fvector_trigger_2;
	uint8_t fvector_trigger_3;
	uint8_t fvector_trigger_4;
	uint8_t fvector_trigger_5;

	uint8_t SUIT_control_mode;
} Extpack_Data_t;


/* For PDO sending */
typedef struct _ScaledData_t {
	// 16bit Scaled Data (For Sending Large PDO Datas)
	uint16_t pMMG_1A_scaled;
	uint16_t pMMG_1B_scaled;
	uint16_t pMMG_2A_scaled;
	uint16_t pMMG_2B_scaled;
	uint16_t pMMG_3A_scaled;
	uint16_t pMMG_3B_scaled;
	uint16_t pMMG_4A_scaled;
	uint16_t pMMG_4B_scaled;

	int16_t EMG_L1_scaled;
	int16_t EMG_L2_scaled;
	int16_t EMG_L3_scaled;
	int16_t EMG_L4_scaled;
	int16_t EMG_R1_scaled;
	int16_t EMG_R2_scaled;
	int16_t EMG_R3_scaled;
	int16_t EMG_R4_scaled;

	uint16_t FSR_L1_scaled;
	uint16_t FSR_L2_scaled;
	uint16_t FSR_L3_scaled;
	uint16_t FSR_L4_scaled;
	uint16_t FSR_R1_scaled;
	uint16_t FSR_R2_scaled;
	uint16_t FSR_R3_scaled;
	uint16_t FSR_R4_scaled;

	/* For HAR_Demo_4 */
	int16_t EMG_RAWSIGN;
	uint16_t EMG_ENVELOPE;
	uint16_t EMG_MA;

	/* For KW Univ */
	int16_t control_input_RH;
	int16_t control_input_LH;
	int16_t EMG_RAW;
	int16_t EMG_R1_RAW;
	int16_t EMG_L1_RAW;
	uint16_t EMG_PROCESSED;

	float free_var1;
	float free_var2;
	float free_var3;
	float free_var4;
	float free_var5;

	float Pvector_ref_RH;
	float Pvector_ref_LH;

	uint8_t control_mode;
	uint8_t ack_signal;
} ScaledData_t;



/* Data for KW University students */
typedef struct _ReceivedDataFromCM_t {
	int16_t thighTheta_RH_scaled;
	int16_t thighTheta_LH_scaled;
	int16_t theta_RH_scaled;
	int16_t theta_LH_scaled;
	int16_t accX_RH_scaled;
	int16_t accX_LH_scaled;
	int16_t accY_RH_scaled;
	int16_t accY_LH_scaled;
	int16_t gyrZ_RH_scaled;
	int16_t gyrZ_LH_scaled;
	int16_t assist_level;

	int16_t u_RH;
	int16_t u_LH;

	uint8_t SUIT_state_curr;
	uint8_t SUIT_state_prev;

	uint8_t contentsFileSendFinished;
} ReceivedDataFromCM_t;


typedef struct _StudentsData_t {
	float thighThetaAct;
	float positionAct;
	
	float accX;
	float accY;
	float gyrZ;

	float u_input;
} StudentsData_t;

/* For P-vector and F-vector */
#define MAX_N_MOTION_SET  			40   // Define Maximum Available Number of Motion Set for a Single Motion Map
#define P_VECTOR_BUFF_SIZE 			10
#define F_VECTOR_BUFF_SIZE			10
#define F_MODE_NUM					10

typedef struct _P_Vector{
	int16_t yd;  // desired position  (deg)
	uint16_t L;  // trajectory length (ms)
	uint8_t s0;  // acceleration      (deg/s^2)
	uint8_t sd;  // deceleration      (deg/s^2)
} P_Vector;

typedef struct _I_Vector{
	uint8_t epsilon_target;     // Half width of the Corridor      (x10)
	uint8_t Kp_target;          // Magnitude of Virtual Spring
	uint8_t Kd_target;          // Magnitude of Virtual Damper
	uint8_t lambda_target;      // Impedance Ratio in the Corridor (x100)
	uint16_t duration;          // Duration for translation
} I_Vector;

typedef struct _P_Vector_Decoder{

	P_Vector p_buffer[P_VECTOR_BUFF_SIZE];
	uint8_t N;   // (=BufferCount) Number of Trajectory in Buffer
	uint8_t ON;  // (=TrajectoryON)

	float y0;    // initial position

	double a0, a2, a3, a4 ,a5; // coefficient of k'th step trajectory
	double b0, b2, b3, b4 ,b5; // coefficient of k+1'th step trajectory

	double t1;
	double t2;
	double t3;
	double t4;
	double t5;

	double L;
	double L_inv;

	uint16_t count;

	float yd_f; // yd(k-1)

	uint8_t durationCompleted;

} P_Vector_Decoder;

/************************ F Vector ************************/
typedef struct _F_Vector{
	uint8_t  mode_idx;     // mode
	int16_t  tau_max;      // 100*Nm
	uint16_t delay;        // delay (ms)

	float tau, tau_old1, tau_old2, u, u_old1, u_old2;
	uint32_t t_end;
	uint32_t time_stamp;
	uint8_t is_full;       // if full 1, else 0

	int16_t coefficient;
	uint16_t globalVariableID;
} F_Vector;


typedef struct _F_Mode_Param{

	double wn, b0, b1, b2, a0, a1, a2;
	float  tp;

} F_Mode_Param;

typedef struct _F_Vector_Decoder {

	F_Vector f_buffer[F_VECTOR_BUFF_SIZE];
	F_Mode_Param mode_param[F_MODE_NUM];

	uint8_t temp_idx;
	float   tp[F_MODE_NUM]; // Tp to Mode Table

	float input;
} F_Vector_Decoder;

typedef struct _PIDObject {
	float ref;  // yd(k)
	float ref1; // yd(k+1)
	float ref2;	// yd(k+2)

	float act;
	float Ctrl_BW_Hz;

	float Kp;
	float Ki;
	float Kd;

	float R;		// Input penalty in LQ,   q1=1, q2=0

	float control_input;

	float err;
	float err_prev;
	float err_sum;
	float err_diff;
	float err_diff_f;
	float err_diff_ff;
	float err_diff_raw;
	float err_diff_raw_f;
	float err_diff_raw_ff;
} PIDObject;



/*-------------------------------------------- For MotionMap --------------------------------------------*/
#define MAX_N_MOTION_SET  40   // Define Maximum Available Number of Motion Set for a Single Motion Map
#define MAX_N_P_VECTORS   10   // Define Maximum Number of P Vectors for a Single Motion Set
#define NUMEL_P_VECTOR    4    // Define Number of Element of a Single P Vector

#define MAX_N_MD          2   // Max number of MD that can be handled


//typedef struct _SingleJointMotionSet {
//	P_Vector_Decoder     p_vector_decoder;
//} __attribute__((packed))SingleJointMotionSet;
//
//typedef struct _RobotMotionSet {
//	SingleJointMotionSet MD[MAX_N_MD];
//	uint8_t MS_ID;
//	uint8_t  max_p_vectors_len;
//} __attribute__((packed))RobotMotionSet;
//
//typedef struct _MotionMapFileInfo {
//	uint8_t     robot_id;
//	uint8_t     file_version;
//
//	RobotMotionSet MS[MAX_N_MOTION_SET];
//
//	uint16_t  num_ms;
//	uint16_t  cnt;
//	uint8_t   send_state;
//
//	uint8_t MM_idx;
//} __attribute__((packed))MotionMapFileInfo;


typedef struct _SingleJointMotionSet {
	P_Vector_Decoder     p_vector_decoder;
	F_Vector_Decoder     f_vector_decoder;
} SingleJointMotionSet;

typedef struct _RobotMotionSet {
	SingleJointMotionSet MD[MAX_N_MD];
} RobotMotionSet;

typedef struct _MotionMapFileInfo {
	uint8_t MM_idx;
	RobotMotionSet MS[MAX_N_MOTION_SET];
} MotionMapFileInfo;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern Extpack_Data_t ExtPackDataObj;
extern char usbTxBufChar[100];
extern uint8_t usbTxBufSize;
extern uint8_t usbTxUpdate;
extern uint32_t timeUSBCDC;

extern StudentsData_t StudentsDataObj_RH;
extern StudentsData_t StudentsDataObj_LH;
extern PIDObject posCtrl_RH;
extern PIDObject posCtrl_LH;

extern uint8_t SUIT_State_curr;
extern P_Vector_Decoder pvectorObj_RH;
extern P_Vector_Decoder pvectorObj_LH;
extern F_Vector_Decoder fvectorObj_RH;
extern F_Vector_Decoder fvectorObj_LH;
extern MotionMapFileInfo MotionMap_File;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitExppackCtrl(void);
void RunExppackCtrl(void* params);

/* Data Size scaling for PDO */
int16_t ScaleFloatToInt16(float value, float scaleFactor);
uint16_t ScaleFloatToUInt16(float value, float scaleFactor);
float ScaleInt16ToFloat(int16_t value, float scaleFactor);

#endif /* EXPPACK_CTRL_INC_EXPPACK_CTRL_H_ */

