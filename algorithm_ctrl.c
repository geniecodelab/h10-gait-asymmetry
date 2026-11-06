#include "algorithm_ctrl.h"
#include "gait_phase.h"
#include <math.h>


/* ------------------- Default Variables ------------------ */
TaskObj_t algorithmCtrlTask;

uint8_t CM_connect_signal = 0;
uint8_t CM_disconnect_signal = 0;

uint8_t motionMap_selection = 99;
uint8_t startPvector_decoding = 0;

ControlMode controlMode = USER_DEFINED_CTRL;   // B안 (원래: DEFAULT_CONTRL_MODE)

float EMG_Rawsignal = 0.0f;
float EMG_R1_Rawsignal = 0.0f;
float EMG_L1_Rawsignal = 0.0f;

uint16_t fsr1_R1 = 0;
uint16_t fsr2_R2 = 0;
uint16_t fsr1_L1 = 0;
uint16_t fsr2_L2 = 0;

float free_var1 = 0.0f;
float free_var2 = 0.0f;
float free_var3 = 0.0f;
float free_var4 = 0.0f;
float free_var5 = 0.0f;

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

float f_vector_input_RH = 0.0f;
float f_vector_input_LH = 0.0f;

float assist_level = 1.0f;
float CalculateAutoAssistGain(float asym);
float CalculatePersonalImprovement(float off, float current);
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

GaitPhase_t gpRH, gpLH;
static bool gpInited = false;

volatile float   gp_theta_RH = 0, gp_phase_RH = 0, gp_thn_RH = 0, gp_thdn_RH = 0, gp_f_RH = 0;
volatile uint8_t gp_valid_RH = 0;
volatile float   gp_theta_LH = 0, gp_phase_LH = 0, gp_thn_LH = 0, gp_thdn_LH = 0, gp_f_LH = 0;
volatile uint8_t gp_valid_LH = 0;



/* ---------------------------------------------------------------------
 * [A] (1) START of STUDENT CODE 영역, 기존 gp_ 변수 선언 아래에 붙여넣기
 * --------------------------------------------------------------------- */

/* Live Expressions 에서 바꾸는 값 */
volatile uint8_t assist_on        = 0;      /* 0: 보조 OFF, 1: 보조 ON                    */
volatile uint8_t assist_side      = 1;      /* 보조할 다리(모래주머니 쪽) 0: RH, 1: LH    */
volatile float   assist_gain_Nm   = 0.0f;   /* 최대 보조 토크 [Nm]. 0 에서 시작!          */
volatile float   assist_dir_RH    = 1.0f;   /* 굴곡 방향 부호. 허공 시험 후 +1/-1 결정    */
volatile float   assist_dir_LH    = 1.0f;
volatile float   assist_win_start = 50.0f;  /* 허벅지 가장 뒤 (phase 50 %) */
volatile float   assist_win_end   = 78.0f;  /* 중립을 살짝 지난 지점 (중립 = 75 %) */
volatile float   phase_offset     = 0.0f;   /* 허벅지 각도 기준 phase 그대로 */

/* 관찰용(읽기 전용) */
volatile float   assist_tau_RH = 0.0f, assist_tau_LH = 0.0f;   /* 실제로 낸 토크 [Nm] */
volatile float   assist_phase_hs_RH = 0.0f, assist_phase_hs_LH = 0.0f; /* 보정된 phase */

#define ASSIST_MAX_NM   10.0f     /* 원래 3.0f → 6.0f → 10.0f */
#define ASSIST_RAMP_MS  1000.0f   /* ON/OFF 전환 시 1초에 걸쳐 서서히 */

static float assist_ramp = 0.0f;  /* 0 ~ 1 */

static float AssistTorque(float phase_pct, uint8_t valid, float* phase_hs_out) __attribute__((unused));

/* ---- [A2] B안: 사건 트리거 + 시간 기준 보조 (H10 F-vector 원리) ---- */
volatile float   assist_dur_frac  = 0.40f;  /* 보조 지속 = 보행주기 x 0.40 (82 BPM 약 0.58 s) */
volatile float   assist_delay_ms  = 0.0f;   /* 최대 신전 감지 후 시작 지연 [ms] */
volatile uint8_t assist_pulse_cnt = 0;      /* 관찰용: 보조가 시작될 때마다 +1 */
volatile float   assist_arm_dps   = 10.0f;  /* 뒤로 가는 속도가 이보다 빠르면 다음 돌아섬에서 보조 시작 [deg/s]. 작은 보폭이면 낮추기 */

static uint8_t  pulse_active_L = 0, armed_back_L = 0;
static uint32_t pulse_t_L = 0, pulse_len_L = 1, pulse_delay_L = 0;
static float    prev_thd_L = 0.0f, tau_f_L = 0.0f;
static uint32_t since_trig_L = 100000;      /* 마지막 보조 시작 후 지난 샘플 수 */
static float    thd_min_L = 0.0f;           /* 이번 뒤로 가는 동안의 최대 뒤쪽 속도(음수) */
volatile float   assist_lead_frac = 0.3f;   /* 조기 시작: 뒤로 가는 최고속도의 30 %까지 줄면 시작 (0 = 완전히 멈춘 순간) */
volatile float   assist_back_n    = -0.3f;  /* 시작 허용: 정규화 각도가 이보다 뒤쪽(-1=가장 뒤)일 때만 */
volatile float   assist_refr_frac = 0.6f;   /* 한 번 시작 후 보행주기 x 이 비율 동안 재시작 금지 */
volatile uint8_t assist_trig_src  = 3;      /* 0: 왼발 사건, 1: 오른발 사건, 2: 둘 중 먼저, 3: 두 다리 벌어짐 각(권장) */
volatile uint8_t assist_trigL_cnt = 0, assist_trigR_cnt = 0;  /* 관찰용: 어느 쪽 사건으로 시작했는지 */
static uint8_t  armed_fwd_R = 0;
static float    prev_thd_R = 0.0f, thd_max_R = 0.0f;
/* 두 다리 벌어짐 각 D = 오른 허벅지 - 왼 허벅지 (상체 움직임은 양쪽에 똑같이 들어가서 빼면 사라짐) */
volatile float   split_deg = 0.0f, split_dps = 0.0f, split_mid = 0.0f;   /* 관찰용 */
volatile uint8_t assist_trigS_cnt = 0;
static float    sp1 = 0.0f, sp2 = 0.0f, sp_prev = 0.0f, spd_prev = 0.0f, spd_max = 0.0f;
static uint8_t  sp_init = 0, sp_armed = 0, sp_wait_narrow = 0;   /* 한 번 시작 후 다리가 다시 좁아질 때까지 대기 */
static float    T_split = 0.0f;                 /* 벌어짐 사건 사이 간격으로 잰 보행 주기 [s] */
static uint32_t since_trig_S = 100000;
/* ---- 안전/부드러운 시작 (강도 6 이상에서 첫 걸음 튐 방지) ---- */
volatile float   assist_max_dur_s = 0.80f;  /* 한 번 보조의 최대 길이 [s]. 추가로 보행주기의 절반도 넘지 않게 제한 */
volatile float   assist_T_default = 1.46f;  /* 주기를 아직 못 쟀을 때 쓰는 보행 주기 [s] (82 BPM) */
volatile float   assist_max_deg   = 35.0f;  /* 왼 허벅지가 이 각도보다 앞이면 즉시 중단 */
volatile float   assist_max_dps   = 250.0f; /* 왼 허벅지가 이 속도보다 빨리 앞으로 가면 즉시 중단 */
volatile float   assist_min_split_dps = 30.0f; /* 벌어지는 최고속도가 이보다 작으면(체중 이동, 흔들림) 무시 */
volatile uint8_t assist_warm_n    = 3;      /* 걷기 시작 후 처음 n번은 힘을 40->60->80 %로 서서히 */
volatile uint8_t assist_stop_cnt  = 0;      /* 관찰용: 안전 조건으로 중단된 횟수 */
static uint8_t  warm_cnt = 0, T_split_n = 0;
static float    pulse_scale = 1.0f;
/* ---- 필요한 만큼 보조 (벌어짐 비대칭에 따라 보조 길이/세기 자동 증가) ----
 * split_asym = (오른발 앞 최대 벌어짐 + 왼발 앞 최대 벌어짐(음수)) / 전체 범위
 *   0 = 대칭, + = 왼발이 앞으로 덜 나감(오른쪽으로 치우침)
 *   A 측정값: 0 kg 약 -0.04, 3 kg 약 +0.09 */
volatile float   split_asym       = 0.0f;   /* 관찰용: 걸음마다 갱신되는 비대칭 (평활) */
volatile float   assist_sym_ref   = -0.04f; /* 이 사람의 정상(0 kg) 값. 0 kg 걸을 때 split_asym 을 보고 넣기 */
volatile float   assist_k_dur     = 2.0f;   /* 비대칭 0.1 당 보조 길이 +20 % (0 이면 끔) */
volatile float   assist_k_gain    = 0.0f;   /* 비대칭 0.1 당 세기 +k*10 % (기본 0: 세기는 손으로 정함) */
/* 개인별 보조 OFF 기준 측정 */
volatile uint8_t baseline_collect = 0;
volatile uint8_t baseline_ready = 0;
volatile uint32_t baseline_count = 0;
volatile float baseline_mean = 0.0f;
/* 보조 전후 비대칭 비교 */
volatile float asym_current = 0.0f; //현재 측정된 비대칭
volatile float asym_improvement = 0.0f; //	OFF 기준 대비 개선율(%)
volatile float personal_improvement = 0.0f;
volatile uint8_t personal_recommend_status = 0;
/* 보조 ON 상태 비대칭 측정 */
volatile uint8_t on_collect = 0;
volatile uint8_t on_ready = 0;
volatile uint32_t on_count = 0;
volatile float on_mean = 0.0f;

/* OFF 대비 ON 비대칭 개선율 */
volatile float on_improvement = 0.0f;
/* 자동 보조 강도 추천 - 모터 출력과 연결하지 않음 */
volatile float assist_recommended_Nm = 0.0f; //알고리즘이 추천하는 보조 강도
volatile float assist_target_improvement = 30.0f; //목표 비대칭 감소율 30%
volatile uint8_t assist_recommend_status = 0; //추천 결과 상태
/* 추천 알고리즘 테스트 결과 */
volatile float recommend_test_improvement = 0.0f;
volatile uint8_t recommend_test_status = 0;
volatile float recommend_test_Nm = 0.0f;
/**/
volatile float   assist_dur_eff_ms = 0.0f, assist_gain_eff = 0.0f;  /* 관찰용: 실제 적용된 길이/세기 */
static float    sp_hi = -1.0e9f, sp_lo = 1.0e9f;
static int8_t   sp_state = 0;
static uint8_t  asym_init = 0;



/*---------- (1) END of STUDENT CODE (Declare the functions to use) -------------*/

DOP_COMMON_SDO_CB(algorithmCtrlTask)

void TestAssistRecommendation(void)
{
    /* 이전 실험 데이터 */
    float off = 0.454f;
    float on = 0.142f;
    float test_gain = 5.0f;

    float target = assist_target_improvement;

    recommend_test_improvement =
        (fabsf(off) - fabsf(on)) / fabsf(off) * 100.0f;

    recommend_test_status = 0;
    recommend_test_Nm = 0.0f;

    if (recommend_test_improvement >= target)
    {
        /* 목표 달성: 현재 강도 유지 후보 */
        recommend_test_Nm = test_gain;
        recommend_test_status = 1;
    }
    else if (recommend_test_improvement >= 0.0f)
    {
        /* 목표 미달: 새로운 강도 자동 결정 보류 */
        recommend_test_Nm = 0.0f;
        recommend_test_status = 2;
    }
    else
    {
        /* 악화: 강도 증가 추천 금지 */
        recommend_test_Nm = 0.0f;
        recommend_test_status = 3;
    }
}

/* 실시간 비대칭 기반 보조 강도 추천 */
float CalculateAutoAssistGain(float asym)
{
    float a = fabsf(asym);

    float ratio = (a - 0.10f) / 0.35f;

    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    return 3.0f + 2.0f * ratio;
}

/* 개인별 기준 대비 개선율 계산 */
float CalculatePersonalImprovement(float off, float current)
{
    float ref = fabsf(off);

    if (ref <= 0.01f)
        return 0.0f;

    return (ref - fabsf(current)) / ref * 100.0f;
}

/* 테스트 결과를 Live Expressions에서 확인하기 위한 함수 */
void RunAssistRecommendationTest(void)
{
    TestAssistRecommendation();
}

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

	if (!gpInited) {
		gait_phase_init(&gpRH, 1.0f / DT);
		gait_phase_init(&gpLH, 1.0f / DT);
		gpLH.min_amp_deg = 2.0f;   /* 작은 보폭에서도 valid (기본 3 deg) */
		gpInited = true;
		TestAssistRecommendation(); /* 가상 데이터로 추천 알고리즘 1회 테스트 */
	}

	gait_phase_update(&gpRH, robotDataObj_RH.thighTheta_act);
	gp_theta_RH = robotDataObj_RH.thighTheta_act;  gp_phase_RH = gpRH.phase_pct;
	gp_thn_RH = gpRH.theta_n;  gp_thdn_RH = gpRH.thetadot_n;  gp_f_RH = gpRH.f_gait_hz;  gp_valid_RH = gpRH.valid;

	gait_phase_update(&gpLH, robotDataObj_LH.thighTheta_act);
	gp_theta_LH = robotDataObj_LH.thighTheta_act;  gp_phase_LH = gpLH.phase_pct;
	gp_thn_LH = gpLH.theta_n;  gp_thdn_LH = gpLH.thetadot_n;  gp_f_LH = gpLH.f_gait_hz;  gp_valid_LH = gpLH.valid;

	/* 로그 기록용: valid일 때만 phase(0~100), 아니면 -1 */
	/* 로그 1번 칸: 두 다리 벌어짐 비대칭 split_asym 을 1000 + x1000 으로 기록 (예: +0.29 -> 1290)
	 * (예전에는 오른발 phase 를 기록. 1000 을 더해 옛 로그(0~100, -1)와 구분) */
	free_var1 = 1000.0f + 1000.0f * split_asym;
	free_var2 = gpLH.valid ? gpLH.phase_pct : -1.0f;


	/* ---------------------------------------------------------------------
	 * [B] (2) START of STUDENT CODE 영역, gait_phase 업데이트와
	 *     free_var1/2 기록 줄 "다음"에 붙여넣기
	 * --------------------------------------------------------------------- */

		{
			/* 보조가 허용되는 조건: 모드 6 + 로봇이 보행 가능 상태(기존 제어 코드와 같은 범위) */
			bool stateOK = (SUIT_State_curr >= 3 && SUIT_State_curr <= 17) ||
			               (SUIT_State_curr >= 33 && SUIT_State_curr <= 45) ||
			               (SUIT_State_curr >= 66 && SUIT_State_curr <= 75);
			bool enable  = (controlMode == USER_DEFINED_CTRL) && stateOK && (assist_on == 1);

			/* 부드러운 ON/OFF 전환 */
			float step = (DT * 1000.0f) / ASSIST_RAMP_MS;
			if (enable) { assist_ramp += step; if (assist_ramp > 1.0f) assist_ramp = 1.0f; }
			else        { assist_ramp -= step; if (assist_ramp < 0.0f) assist_ramp = 0.0f; }

			/* (v1 위상 구간 방식은 algorithm_ctrl.c.bak_v1 에 보관) */
			/* ===== [B2] B안: 왼발 또는 오른발 사건에 시작, 정해진 시간 동안 끝까지 왼다리 보조 ===== */
			{
				float gain = assist_gain_Nm;
				if (gain < 0.0f) gain = 0.0f;
				if (gain > ASSIST_MAX_NM) gain = ASSIST_MAX_NM;

				/* 1) 보조 시작 사건 (assist_trig_src: 0=왼발만, 1=오른발만, 2=둘 중 먼저)
				 *    왼발 사건: 왼 허벅지가 뒤로 가다가 돌아서기 직전 (최대 신전 근처)
				 *    오른발 사건: 오른 허벅지가 앞으로 가다가 멈추기 직전 (최대 굴곡 = 오른발 딛기 직전)
				 *                 -> 이때 왼발은 뒤에서 밀고 떠날 준비 = 왼다리 보조 시작 시점 */
				if (since_trig_L < 100000) since_trig_L++;
				bool trigL = false, trigR = false;

				/* 왼발 사건 */
				float thdL = gpLH.thetadot;
				if (thdL < -assist_arm_dps) {
					armed_back_L = 1;
					if (thdL < thd_min_L) thd_min_L = thdL;           /* 뒤로 가는 최고속도 기록 */
				}
				float trig_lvl = assist_lead_frac * thd_min_L;            /* 음수 또는 0 */
				if (armed_back_L && thdL > prev_thd_L && prev_thd_L < trig_lvl && thdL >= trig_lvl) {
					armed_back_L = 0;  thd_min_L = 0.0f;
					bool walkL  = gpLH.valid && (gpLH.f_gait_hz > 0.4f) && (gpLH.f_gait_hz < 1.3f);
					bool behind = (gpLH.theta_n < assist_back_n);
					trigL = walkL && behind && (assist_trig_src == 0 || assist_trig_src == 2);
				}
				prev_thd_L = thdL;

				/* 오른발 사건 (건강한 쪽이 크고 규칙적으로 움직이므로 기준으로 쓰기 좋음) */
				float thdR = gpRH.thetadot;
				if (thdR > assist_arm_dps) {
					armed_fwd_R = 1;
					if (thdR > thd_max_R) thd_max_R = thdR;           /* 앞으로 가는 최고속도 기록 */
				}
				float trig_lvlR = assist_lead_frac * thd_max_R;           /* 양수 또는 0 */
				if (armed_fwd_R && thdR < prev_thd_R && prev_thd_R > trig_lvlR && thdR <= trig_lvlR) {
					armed_fwd_R = 0;  thd_max_R = 0.0f;
					bool walkR = gpRH.valid && (gpRH.f_gait_hz > 0.4f) && (gpRH.f_gait_hz < 1.3f);
					bool fwdR  = (gpRH.theta_n > 0.3f);                    /* 오른 허벅지가 실제로 앞쪽 */
					trigR = walkR && fwdR && (assist_trig_src == 1 || assist_trig_src == 2);
				}
				prev_thd_R = thdR;

				/* 두 다리 벌어짐 각 사건: 오른발이 앞, 왼발이 뒤로 가장 벌어지기 직전 = 왼발이 떠날 때
				 * - 다리가 나란히(0도)를 지나지 않아도 됨: 평균(split_mid) 기준으로 넓을 때/좁을 때를 판단
				 * - 두 다리 움직임을 합친 신호라 왼발 움직임이 작아도 크게 잡힘 */
				bool trigS = false;
				{
					float Draw = robotDataObj_RH.thighTheta_act - robotDataObj_LH.thighTheta_act;
					const float a_lp = DT / (0.04f + DT);              /* 약 4 Hz 2차 저역통과 */
					if (!sp_init) { sp1 = sp2 = sp_prev = Draw; split_mid = Draw; sp_init = 1; }
					sp1 += a_lp * (Draw - sp1);
					sp2 += a_lp * (sp1 - sp2);
					float spd = (sp2 - sp_prev) / DT;  sp_prev = sp2;
					split_deg = sp2;  split_dps = spd;
					split_mid += (DT / 3.0f) * (sp2 - split_mid);       /* 3초 평균 = 벌어짐의 중심 */

					/* 걸음마다 최대/최소 벌어짐 -> 비대칭 갱신 (중심을 아래->위로 지날 때 한 걸음 끝) */
					if (sp2 > sp_hi) sp_hi = sp2;
					if (sp2 < sp_lo) sp_lo = sp2;
					{
						int8_t st = (sp2 > split_mid) ? 1 : -1;
						if (st == 1 && sp_state == -1) {
							float rng = sp_hi - sp_lo;
							if (rng > 15.0f) {
								float A = (sp_hi + sp_lo) / rng;
								split_asym = asym_init ? (0.7f * split_asym + 0.3f * A) : A;
								asym_init = 1;
								/* 개인별 OFF 기준 대비 현재 개선율 */
								if (baseline_ready == 1 &&
								    baseline_count == 10 &&
								    fabsf(baseline_mean) > 0.01f)
								{
								    personal_improvement =
								        CalculatePersonalImprovement(
								            baseline_mean,
								            split_asym
								        );
								    /* 개인별 개선율에 따른 추천 상태 */
								    if (personal_improvement >= assist_target_improvement)
								    {
								        personal_recommend_status = 1;
								    }
								    else if (personal_improvement >= 0.0f)
								    {
								        personal_recommend_status = 2;
								    }
								    else
								    {
								        personal_recommend_status = 3;
								    }
								}
								/* 개인 기준 측정 완료 후 자동 추천값 갱신 */
								if (baseline_ready == 1)
								{
								    assist_recommended_Nm =
								        CalculateAutoAssistGain(split_asym);
								}
								/* 개인별 OFF 기준 대비 비대칭 개선율 */
								if (baseline_ready == 1 && baseline_collect == 0)
								{
								    asym_current = split_asym;

								    float ref = fabsf(baseline_mean);

								    if (ref > 0.01f)
								    {
								        asym_improvement =
								            (ref - fabsf(asym_current)) / ref * 100.0f;
								    }
								    else
								    {
								        asym_improvement = 0.0f;
								    }
								}
								/* 보조 OFF 상태에서 개인별 비대칭 기준값 수집 */
								if (baseline_collect == 1 && assist_on == 0 && baseline_ready == 0)
								{
								    baseline_count++;

								    baseline_mean += (A - baseline_mean) / (float)baseline_count;

								    if (baseline_count >= 10)
								    {
								        baseline_ready = 1;
								        baseline_collect = 0;
								    }
								}
								/* 보조 ON 상태에서 10주기 평균 측정 */
								if (on_collect == 1 &&
								    baseline_ready == 1 &&
								    assist_on == 1 &&
								    on_ready == 0)
								{
								    on_count++;

								    on_mean += (A - on_mean) / (float)on_count;

								    if (on_count >= 10)
								    {
								        on_ready = 1;
								        on_collect = 0;

								        float ref = fabsf(baseline_mean);

								        if (ref > 0.01f)
								        {
								            on_improvement =
								                (ref - fabsf(on_mean)) / ref * 100.0f;
								        }
								        else
								        {
								            on_improvement = 0.0f;
								        }
								        /* 자동 보조 강도 추천 상태 계산 */
								        assist_recommended_Nm = 0.0f;
								        assist_recommend_status = 0;

								        if (ref > 0.01f)
								        {
								            if (on_improvement >= assist_target_improvement)
								            {
								                /* 목표 개선율 달성 */
								                assist_recommended_Nm = assist_gain_Nm;
								                assist_recommend_status = 1;
								            }
								            else if (on_improvement >= 0.0f)
								            {
								                /* 개선됐지만 목표 미달 */
								                assist_recommend_status = 2;
								            }
								            else
								            {
								                /* 비대칭 악화 */
								                assist_recommend_status = 3;
								            }
								        }
								    }
								}
							}
							sp_hi = -1.0e9f;  sp_lo = 1.0e9f;
						}
						sp_state = st;
					}

					if (sp_wait_narrow && sp2 < split_mid) sp_wait_narrow = 0;   /* 다리가 다시 좁아짐 -> 다음 걸음 준비 */
					if (spd > assist_arm_dps && !sp_wait_narrow) {      /* 벌어지는 중 (오른발 앞으로 / 왼발 뒤로) */
						sp_armed = 1;
						if (spd > spd_max) spd_max = spd;
					}
					float lvl = assist_lead_frac * spd_max;
					if (sp_armed && spd < spd_prev && spd_prev > lvl && spd <= lvl && sp2 > split_mid) {   /* 넓게 벌어진 쪽에서 벌어짐이 멈추기 직전 */
						bool brisk = (spd_max > assist_min_split_dps);  /* 실제 걸음만 (체중 이동/흔들림 제외) */
						sp_armed = 0;  spd_max = 0.0f;  sp_wait_narrow = 1;
						if (brisk) {
							float iv = (float)since_trig_S * DT;
							if (iv > 0.6f && iv < 2.5f) { T_split = (T_split > 0.0f) ? 0.7f * T_split + 0.3f * iv : iv; if (T_split_n < 10) T_split_n++; }
							since_trig_S = 0;
							trigS = (assist_trig_src == 3);
						}
					}
					spd_prev = spd;
					if (since_trig_S < 100000) since_trig_S++;
				}

				/* 보행 주기: 오른발 추정이 유효하면 우선 사용 (더 안정적) */
				float fg = (gpRH.valid && gpRH.f_gait_hz > 0.4f && gpRH.f_gait_hz < 1.3f) ? gpRH.f_gait_hz : gpLH.f_gait_hz;
				if (assist_trig_src == 3) fg = (T_split_n >= 2) ? 1.0f / T_split : 1.0f / assist_T_default;   /* 2번 이상 잰 뒤에만 측정 주기 사용 */
				if (fg < 0.4f) fg = 0.4f;
				if (fg > 1.3f) fg = 1.3f;
				bool refrOK = ((float)since_trig_L * DT > assist_refr_frac / fg);

				if ((trigL || trigR || trigS) && refrOK && !pulse_active_L) {
					if ((float)since_trig_L * DT > 3.0f || assist_ramp < 0.99f) warm_cnt = 0;   /* 멈췄다 다시 걸으면 처음부터 */
					since_trig_L = 0;
					float T = 1.0f / fg;                                   /* 보행 주기 [s] */
					/* 비대칭이 정상보다 클수록 길게(그리고 선택적으로 세게) */
					float asym = asym_init ? (split_asym - assist_sym_ref) : 0.0f;
					float mdur = 1.0f + assist_k_dur  * asym;  if (mdur < 0.7f) mdur = 0.7f;  if (mdur > 1.6f) mdur = 1.6f;
					float mgn  = 1.0f + assist_k_gain * asym;  if (mgn  < 0.5f) mgn  = 0.5f;  if (mgn  > 1.5f) mgn  = 1.5f;
					float dur = assist_dur_frac * T * mdur;
					if (dur > 0.5f * T) dur = 0.5f * T;                    /* 왼발 딛기 전에는 반드시 끝 */
					if (dur > assist_max_dur_s) dur = assist_max_dur_s;    /* 너무 길게 미는 것 방지 */
					if (dur < 0.2f) dur = 0.2f;
					assist_dur_eff_ms = dur * 1000.0f;
					pulse_len_L   = (uint32_t)(dur / DT);
					/* 걷기 시작(또는 3초 이상 멈춘 뒤) 처음 몇 걸음은 약하게 */
					if (warm_cnt < assist_warm_n) { pulse_scale = 0.4f + 0.6f * (float)warm_cnt / (float)assist_warm_n; warm_cnt++; }
					else pulse_scale = 1.0f;
					pulse_scale *= mgn;
					assist_gain_eff = gain * pulse_scale;
					pulse_delay_L = (uint32_t)(assist_delay_ms * 0.001f / DT);
					pulse_t_L = 0;  pulse_active_L = 1;  assist_pulse_cnt++;
					if (trigS) assist_trigS_cnt++; else if (trigR && !trigL) assist_trigR_cnt++; else assist_trigL_cnt++;
				}

				/* 2) 시간 기준 모양: 20 % 동안 올리고, 유지, 마지막 30 % 동안 내림 */
				float tauCmdL = 0.0f;
				if (pulse_active_L) {
					if (pulse_t_L >= pulse_delay_L) {
						float s = (float)(pulse_t_L - pulse_delay_L) / (float)pulse_len_L;
						if (s >= 1.0f) {
							pulse_active_L = 0;
						} else {
							float sh;
							if      (s < 0.2f) sh = 0.5f - 0.5f * cosf((float)M_PI * s / 0.2f);
							else if (s > 0.7f) sh = 0.5f - 0.5f * cosf((float)M_PI * (1.0f - s) / 0.3f);
							else               sh = 1.0f;
							tauCmdL = gain * pulse_scale * sh;
							if (tauCmdL > ASSIST_MAX_NM) tauCmdL = ASSIST_MAX_NM;
						}
					}
					pulse_t_L++;
				}

				/* 안전: 허벅지가 너무 앞으로 들리면 이번 보조 즉시 종료 */
				if (pulse_active_L && (gpLH.theta_f > assist_max_deg || robotDataObj_LH.thighTheta_act > assist_max_deg ||
				                       gpLH.thetadot > assist_max_dps)) {
					pulse_active_L = 0;  tauCmdL = 0.0f;  assist_stop_cnt++;
				}
				if (gpLH.theta_f > assist_max_deg) tauCmdL = 0.0f;

				/* 3) 30 ms 저역통과 + 1초 ON/OFF 램프 */
				tau_f_L += (tauCmdL * assist_ramp - tau_f_L) * (DT / (0.03f + DT));

				assist_tau_RH = 0.0f;                                   /* 오른쪽은 보조 안 함 */
				assist_tau_LH = (assist_side == 1) ? tau_f_L : 0.0f;
			}

			if (controlMode == USER_DEFINED_CTRL) {
				/* 토크[Nm] -> 전류 입력 변환 (FvectorTrigger 와 같은 식) */
				UserDefinedCtrl_RH.control_input = assist_dir_RH * assist_tau_RH / GEAR_RATIO / MOTOR_TORQUE_CONSTANT;
				UserDefinedCtrl_LH.control_input = assist_dir_LH * assist_tau_LH / GEAR_RATIO / MOTOR_TORQUE_CONSTANT;
			}
		}


	/*------------------------------------ (2) END of STUDENT CODE-----------------------------------------*/
	// Control Input Saturation (Do not Delete)
	ControlInputSaturation(&robotDataObj_RH, &posCtrl_RH, &gravCompDataObj_RH, &impedanceCtrl_RH, &StepCurr_RH , &UserDefinedCtrl_RH, f_vector_input_RH);
	ControlInputSaturation(&robotDataObj_LH, &posCtrl_LH, &gravCompDataObj_LH, &impedanceCtrl_LH, &StepCurr_LH , &UserDefinedCtrl_LH, f_vector_input_LH);
	algorithmCtrlLoopCnt++;
}

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

	fsr1_R1 = ExtPackDataObj->fsr_data.fsr_R1_raw;
	fsr2_R2 = ExtPackDataObj->fsr_data.fsr_R2_raw;
	fsr1_L1 = ExtPackDataObj->fsr_data.fsr_L1_raw;
	fsr2_L2 = ExtPackDataObj->fsr_data.fsr_L2_raw;

	//free_var1 = robotDataObj_RH.accX;
	//free_var2 = robotDataObj_RH.accY;
	free_var3 = robotDataObj_RH.accX;
	// free_var4 = robotDataObj_RH.accY;
	// free_var5 = robotDataObj_RH.accX;
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


/* ---------------------------------------------------------------------
 * [C] (3) START of STUDENT CODE 영역(파일 맨 아래)에 붙여넣기
 * --------------------------------------------------------------------- */

static float AssistTorque(float phase_pct, uint8_t valid, float* phase_hs_out)
{
	/* 발 닿는 시점 기준 phase 로 옮기기 (0~100 % 로 감기) */
	float ph = phase_pct + phase_offset;
	while (ph < 0.0f)    ph += 100.0f;
	while (ph >= 100.0f) ph -= 100.0f;
	*phase_hs_out = ph;

	if (!valid) return 0.0f;                       /* phase 가 불확실하면 보조 안 함 */

	float gain = assist_gain_Nm;
	if (gain < 0.0f)          gain = 0.0f;
	if (gain > ASSIST_MAX_NM) gain = ASSIST_MAX_NM; /* 안전 상한 */

	float a = assist_win_start, b = assist_win_end;
	if (b <= a || ph < a || ph > b) return 0.0f;

	float s = (ph - a) / (b - a);                   /* 0 ~ 1 */
	return gain * sinf((float)M_PI * s);            /* 반 사인: 시작/끝에서 0 */
}




/*------------ (3) END of STUDENT CODE (Define the functions to use) -------------*/
