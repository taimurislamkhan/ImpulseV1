// AUTO CYCLE SCREEN EXTERNAL VARIABLES

uint8_t Tip_1_Enabled=0;
uint8_t Tip_1_Energy_Up_Pressed;
uint8_t Tip_1_Energy_Down_Pressed;
uint16_t Tip_1_Energy_Val = 2000;
uint16_t HOME_Tip_1_Energy=0;
double Tip_1_Distance;
double HMI_Tip_1_Distance;
uint8_t Tip_1_Distance_Up_Pressed;
uint8_t Tip_1_Distance_Down_Pressed;
double Tip_1_Distance_Val = 5.1;


uint8_t Tip_2_Enabled=0;
uint8_t Tip_2_Energy_Up_Pressed;
uint8_t Tip_2_Energy_Down_Pressed;
uint16_t Tip_2_Energy_Val = 2000;
uint16_t HOME_Tip_2_Energy = 0;
double Tip_2_Distance;
double HMI_Tip_2_Distance;
uint8_t Tip_2_Distance_Up_Pressed;
uint8_t Tip_2_Distance_Down_Pressed;
double Tip_2_Distance_Val = 5.1;



uint8_t Tip_3_Enabled=0;
uint8_t Tip_3_Energy_Up_Pressed;
uint8_t Tip_3_Energy_Down_Pressed;
uint16_t Tip_3_Energy_Val = 2000;
uint16_t HOME_Tip_3_Energy = 0;
double Tip_3_Distance;
double HMI_Tip_3_Distance;
uint8_t Tip_3_Distance_Up_Pressed;
uint8_t Tip_3_Distance_Down_Pressed;
double Tip_3_Distance_Val = 5.1;



uint8_t Tip_4_Enabled=0;
uint8_t Tip_4_Energy_Up_Pressed;
uint8_t Tip_4_Energy_Down_Pressed;
uint16_t Tip_4_Energy_Val = 2000;
uint16_t HOME_Tip_4_Energy = 0;
double Tip_4_Distance;
double HMI_Tip_4_Distance;
uint8_t Tip_4_Distance_Up_Pressed;
uint8_t Tip_4_Distance_Down_Pressed;
double Tip_4_Distance_Val = 5.1;



uint16_t HOME_Tip_5_Energy=0;
uint16_t HOME_Tip_6_Energy=0;
uint16_t HOME_Tip_7_Energy=0;
uint16_t HOME_Tip_8_Energy=0;

double Tip_4_Distance=0;
double Tip_5_Distance=0;
double Tip_6_Distance=0;
double Tip_7_Distance=0;
double Tip_8_Distance=0;

// Declare flags for each fault case
uint8_t FAULT_ESTOP_FLAG = 0;
uint8_t FAULT_TE112_FLAG = 0;
uint8_t FAULT_TE212_FLAG = 0;
uint8_t FAULT_TE312_FLAG = 0;
uint8_t FAULT_TE412_FLAG = 0;
uint8_t FAULT_SSR_1_FLAG = 0;
uint8_t FAULT_SE108_FLAG = 0;
uint8_t FAULT_SE208_FLAG = 0;
uint8_t FAULT_SE308_FLAG = 0;
uint8_t FAULT_SE408_FLAG = 0;
uint8_t FAULT_MAX_DISTANCE_FLAG = 0;
uint8_t FAULT_CLEAR_FLAG = 0;
uint8_t FAULT_BOSS_NOT_PRESENT_FLAG_TIP1=0;
uint8_t FAULT_BOSS_NOT_PRESENT_FLAG_TIP2=0;
uint8_t FAULT_BOSS_NOT_PRESENT_FLAG_TIP3=0;
uint8_t FAULT_BOSS_NOT_PRESENT_FLAG_TIP4=0;

uint8_t  Home_Machine;

// HMI MANUAL Controls Screen
uint8_t HMI_MANUAL_PLATEN_UP=0;
uint8_t HMI_MANUAL_PLATEN_DOWN=0;
uint8_t HMI_MANUAL_COOLING=0;
uint8_t HMI_MANUAL_TIP_1=0;
uint8_t HMI_MANUAL_TIP_2=0;
uint8_t HMI_MANUAL_TIP_3=0;
uint8_t HMI_MANUAL_TIP_4=0;
uint8_t HMI_MANUAL_TIP_5=0;
uint8_t HMI_MANUAL_TIP_6=0;
uint8_t HMI_MANUAL_TIP_7=0;
uint8_t HMI_MANUAL_TIP_8=0;
double HMI_MANUAL_PLATEN_DISTANCE=0;


char HMI_Error_State[10]="None\0";
uint8_t HMI_CYCLE_STATE=0;
uint8_t HMI_Start_State=0;
uint8_t HMI_Stop_State=0;
uint8_t HMI_Reset_State=0;
uint8_t HMI_Up_State=0;
uint8_t HMI_Down_State=0;
uint8_t HMI_SaveUp_State=0;
uint8_t HMI_SaveDown_State=0;
uint8_t HMI_UpArrowExtrude_State=0;
uint8_t HMI_DownArrowExtrude_State=0;
uint8_t HMI_UpArrowSteps_State=0;
uint8_t HMI_DownArrowSteps_State=0;
uint8_t HMI_1Rev_State=0;
double HMI_Extrude_Length=0;
uint16_t HMI_Steps_Per_mm=0;
uint16_t HMI_Total_Cycles=0;
uint32_t HMI_SaveUp_Counter=0;
uint32_t HMI_SaveDown_Counter=0;
double HMI_Distance_Counter=0;
double HMI_Cycle_time=0;
double HMI_Total_time=0;



// SSR TEST SCREEN EXTERNAL VARIABLES
uint8_t HMI_SSR_START_State=0;
uint8_t HMI_TIP1_State=0;
uint8_t HMI_TIP2_State=0;
uint8_t HMI_TIP3_State=0;
uint8_t HMI_TIP4_State=0;
double HMI_TIP1_AMPS=0;
double HMI_TIP2_AMPS=0;
double HMI_TIP3_AMPS=0;
double HMI_TIP4_AMPS=0;
double HMI_TIP1_ENERGY=0;
double HMI_TIP2_ENERGY=0;
double HMI_TIP3_ENERGY=0;
double HMI_TIP4_ENERGY=0;

uint16_t HMI_TIP1_ENERGY_SET=0;
uint16_t HMI_TIP1_COOLING_SET=0;
uint16_t HMI_TIP2_ENERGY_SET  = 0;
uint16_t HMI_TIP2_COOLING_SET = 0;
uint16_t HMI_TIP3_ENERGY_SET  = 0;
uint16_t HMI_TIP3_COOLING_SET = 0;
uint16_t HMI_TIP4_ENERGY_SET  = 0;
uint16_t HMI_TIP4_COOLING_SET = 0;

// HMI TEST PRESS SCREEN EXTERNAL VARIABLES

uint8_t HMI_TEST_PRESS_PLATEN_UP=0;
uint8_t HMI_TEST_PRESS_PLATEN_DOWN=0;
uint8_t HMI_TEST_PRESS_SAVE_UP=0;
uint8_t HMI_TEST_PRESS_SAVE_DOWN=0;
uint8_t HMI_TEST_PRESS_START_CYCLE=0;
double HMI_TEST_PRESS_UP_POSITION=0;
double HMI_TEST_PRESS_DOWN_POSITION=0;
double HMI_TEST_PRESS_PLATEN_DISTANCE=0;

// HMI MONITOR SCREEN EXTERNAL VARIABLES

uint8_t HMI_MONITOR_LEFT_START=0;
uint8_t HMI_MONITOR_RIGHT_START=0;
uint8_t HMI_MONITOR_ESTOP_CLEAR=0;
uint8_t HMI_MONITOR_OVERTRAVEL=0;

// HMI COOLING TIME SCREEN

uint8_t HMI_COOLING_TIME_UP=0;
uint8_t HMI_COOLING_TIME_DOWN=0;
double HMI_COOLING_TIME_SET=40;

// HMI SETTINGS WORK POSITION SCREEN

uint8_t HMI_SETTINGS_PLATEN_UP=0;
uint8_t HMI_SETTINGS_PLATEN_DOWN=0;
uint8_t HMI_SETTINGS_WORK_POSITION=0;
uint8_t HMI_AUTOFIND=0;

// HMI HOME SCREEN EXTERNAL VARIABLES


const char STRING_HOME_MACHINE[] = "HOME MACHINE";
const char STRING_GOING_HOME[] = "GOING HOME";
const char STRING_EMERGENCY_STOP[] = "EMERGENCY STOP";
const char STRING_WORK_HEIGHT[] = "SET WORK HEIGHT";
const char STRING_ANTI_TIE_DOWN_ACTIVE[] = "SYSTEM READY";
const char STRING_IN_CYCLE[] = "IN CYCLE";
const char STRING_FAULT_TE112[] = "FAULT TE112 - TIP 1 DISTANCE NOT REACHED";
const char STRING_FAULT_TE212[] = "FAULT TE212 - TIP 2 DISTANCE NOT REACHED";
const char STRING_FAULT_TE312[] = "FAULT TE312 - TIP 3 DISTANCE NOT REACHED";
const char STRING_FAULT_TE412[] = "FAULT TE412 - TIP 4 DISTANCE NOT REACHED";
const char STRING_ENCODER_1_FAIL[] = "ENCODER 1 FAIL";
const char STRING_ENCODER_2_FAIL[] = "ENCODER 2 FAIL";
const char STRING_ENCODER_3_FAIL[] = "ENCODER 3 FAIL";
const char STRING_ENCODER_4_FAIL[] = "ENCODER 4 FAIL";
const char STRING_SSR_1_FAIL[] = "SSR FAIL";
const char STRING_FAULT_SE108[] = "INTERFERENCE DETECTED";
const char STRING_FAULT_SE208[] = "INTERFERENCE DETECTED";
const char STRING_FAULT_SE308[] = "INTERFERENCE DETECTED";
const char STRING_FAULT_SE408[] = "INTERFERENCE DETECTED";
const char STRING_FAULT_MAX_DISTANCE[] = "MAX DISTANCE REACHED";
const char STRING_FAULT_BOSS_NOT_PRESENT_TIP1[] = "BOSS NOT PRESENT";
const char STRING_FAULT_BOSS_NOT_PRESENT_TIP2[] = "BOSS NOT PRESENT";
const char STRING_FAULT_BOSS_NOT_PRESENT_TIP3[] = "BOSS NOT PRESENT";
const char STRING_FAULT_BOSS_NOT_PRESENT_TIP4[] = "BOSS NOT PRESENT";
const char STRING_FAULT_HANDS_RELEASED[] = "HANDS RELEASED";
char cycle_time_string[20];
char milliseconds_str[3];
char seconds_str[3];

char HMI_HOME_BANNER_TEXT[50] ="";

uint8_t HMI_BANNER_COLOR =0; // 0 green, 1 red, 2 yellow


// Modal Error Windows

uint8_t MODAL_OK_BUTTON=0;
uint8_t FAULT_ACTIVE_FLAG;


// GLOBAL SETTINGS SCREEN

uint8_t HMI_ENCODER_BYPASS=1; // 1 Means off
double HMI_BOSS_HEIGHT_TOLERANCE_VAL=25;
uint8_t HMI_PREHEAT_ENERGY_VAL=25;
uint8_t HMI_PREHEAT_DISTANCE_VAL=25;
uint8_t HMI_PREHEAT_ENABLE=1;
uint8_t HMI_PLATEN_SPEED_CURVE_VAL=0;


int yourFlag;
int inactivityCounter;


// GLOBAL VARS
uint32_t cycle_start_time;
uint32_t elapsed_time;
uint8_t FLAG_IN_PRE_CYCLE=0;
uint8_t FLAG_CYCLE_COMPLETED=0;
static uint32_t lastFlashTick = 0; // Declare this at a global scope


uint64_t tip1_heating_start_time=0;
uint64_t tip1_heating_time_passed_ms=0;
double tip1_iteration_time_passed_s=0;
uint16_t tip1_energy_val_set=0;
double Tip1_Boss_Start_Height=0;
uint8_t FLAG_TIP1_HEATING_IN_PROGRESS=0;
uint8_t FLAG_TIP1_HEATING_START=0;
double Tip1_Collapse=0;
uint8_t FAULT_ENCODER_1_FLAG = 0;
int16_t linear_encoder_1=0;
double Tip_1_Compression=0;
double Tip_1_Energy_Consumed=0;

uint64_t tip2_heating_start_time = 0;
uint64_t tip2_heating_time_passed_ms = 0;
double tip2_iteration_time_passed_s = 0;
uint16_t tip2_energy_val_set = 0;
double Tip2_Boss_Start_Height = 0;
uint8_t FLAG_TIP2_HEATING_IN_PROGRESS = 0;
uint8_t FLAG_TIP2_HEATING_START = 0;
double Tip2_Collapse = 0;
uint8_t FAULT_ENCODER_2_FLAG = 0;
int16_t linear_encoder_2 = 0;
double Tip_2_Compression=0;
double Tip_2_Energy_Consumed=0;

uint64_t tip3_heating_start_time = 0;
uint64_t tip3_heating_time_passed_ms = 0;
double tip3_iteration_time_passed_s = 0;
uint16_t tip3_energy_val_set = 0;
double Tip3_Boss_Start_Height = 0;
uint8_t FLAG_TIP3_HEATING_IN_PROGRESS = 0;
uint8_t FLAG_TIP3_HEATING_START = 0;
double Tip3_Collapse = 0;
uint8_t FAULT_ENCODER_3_FLAG = 0;
int16_t linear_encoder_3 = 0;
double Tip_3_Compression=0;
double Tip_3_Energy_Consumed=0;

uint64_t tip4_heating_start_time = 0;
uint64_t tip4_heating_time_passed_ms = 0;
double tip4_iteration_time_passed_s = 0;
uint16_t tip4_energy_val_set = 0;
double Tip4_Boss_Start_Height = 0;
uint8_t FLAG_TIP4_HEATING_IN_PROGRESS = 0;
uint8_t FLAG_TIP4_HEATING_START = 0;
double Tip4_Collapse = 0;
uint8_t FAULT_ENCODER_4_FLAG = 0;
int16_t linear_encoder_4 = 0;
double Tip_4_Compression=0;
double Tip_4_Energy_Consumed=0;


uint8_t FLAG_MANUAL_COOLING_PRESSED=0;
uint8_t FLAG_CYCLE_ABORTED=0;
uint8_t FLAG_HEATING_STARTED=0;
uint8_t FLAG_BOSS_NOT_PRESENT[TOTAL_TIPS];
uint8_t FLAG_INTERFERENCE[TOTAL_TIPS];

#define ENABLED 1
#define DISABLE 0

#define SSR_1_ID 0
#define SSR_2_ID 1
#define TOTAL_SSR 2

#define SSR_1_Address 48
#define SSR_2_Address 0x49

#define TOTAL_ENCODERS 4

typedef struct {
	uint8_t Enable;
    uint8_t ID;
    uint8_t Address;
    uint8_t Transmit[9];
    uint8_t T_Length;
    uint8_t Recieve[12];
    uint8_t R_Length;
    uint8_t Fail;
    uint8_t Tips[4]; // Output
    float Current[4]; // Input
	uint8_t Output[6];
	uint8_t Input[4];
	uint8_t Button_Color; // Output

} SSR;

SSR ssrs[] = {
    {
        .Enable = ENABLED,
        .ID = SSR_1_ID,
        .Address = SSR_1_Address,
        .Transmit = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        .T_Length = 9,
        .Recieve = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .R_Length = 11,
        .Fail = 0,
        .Tips = {0, 0, 0, 0},
        .Current = {0, 0, 0, 0},
        .Output = {0, 0, 0, 0, 0, 0},
        .Input = {0, 0, 0, 0},
        .Button_Color = 0
    },
    {
        .Enable = DISABLE,
        .ID = SSR_2_ID,
        .Address = SSR_2_Address,
        .Transmit = {0, 0, 0, 0, 0, 0, 0, 0, 0},
        .T_Length = 9,
        .Recieve = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .R_Length = 11,
        .Fail = 0,
        .Tips = {0, 0, 0, 0},
        .Current = {0, 0, 0, 0},
        .Output = {0, 0, 0, 0, 0, 0},
        .Input = {0, 0, 0, 0},
        .Button_Color = 0
    }
};

typedef struct {
	uint8_t Enable;
	uint8_t ID;
	uint8_t Address;
	uint8_t Transmit;
	uint8_t Recieve[4];
	uint8_t R_Length;
	uint8_t Fail;
	double  Distance;
	uint8_t Color;
	uint8_t Reset;

} ENCODER;

ENCODER encoders[] = {
    {
        .Enable = ENABLED,
        .ID = 0,
        .Address = LINEAR_ENCODER_1_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = DISABLE,
        .ID = 1,
        .Address = LINEAR_ENCODER_2_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = ENABLED,
        .ID = 2,
        .Address = LINEAR_ENCODER_3_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = ENABLED,
        .ID = 3,
        .Address = LINEAR_ENCODER_4_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    }
};


typedef struct {
    uint8_t Enable;
    uint8_t Heat;
    uint8_t Color;
    uint8_t Reset_Distance;

} CONTROL;

typedef struct {
    double Amps;
    double Energy;
    double Distance;

} STATUS;

typedef struct {
    double Energy;
    double Distance;

} SETTINGS;

typedef struct {
    CONTROL Control;
    STATUS Status;
    SETTINGS Settings;
} TIP;

TIP tipArray[3] = {
    {
        .Control = {0, 0, 0, 0},
        .Status = {0.0, 0.0, 0.0},
        .Settings = {300.0, 0.0}
    },
    {
        .Control = {0, 0, 0, 0},
        .Status = {0.0, 0.0, 0.0},
        .Settings = {300.0, 0.0}
    },
    {
        .Control = {0, 0, 0, 0},
        .Status = {0.0, 0.0, 0.0},
        .Settings = {300.0, 0.0}
    }
};


double HMI_BOSS_HEIGHT=0;
double HMI_CURRENT_HEIGHT=0;

static uint32_t lastTriggerTime = 0; // To store the time of the last trigger
uint8_t FLAG_WORK_POSTION_SAVED=0;
uint8_t FLAG_TIP_1_BOSS_HEIGHT_SAVED=0;
uint8_t FLAG_TIP_2_BOSS_HEIGHT_SAVED=0;
uint8_t FLAG_TIP_3_BOSS_HEIGHT_SAVED=0;
uint8_t FLAG_TIP_4_BOSS_HEIGHT_SAVED=0;
double steps_count;
double TIP_1_BOSS_TRIGGERED;
double TIP_2_BOSS_TRIGGERED;
double TIP_3_BOSS_TRIGGERED;
double TIP_4_BOSS_TRIGGERED;
uint8_t FLAG_TIP1_BOSS_TRIGGERED_SAVED=0;
uint8_t FLAG_TIP2_BOSS_TRIGGERED_SAVED=0;
uint8_t FLAG_TIP3_BOSS_TRIGGERED_SAVED=0;
uint8_t FLAG_TIP4_BOSS_TRIGGERED_SAVED=0;
uint8_t FLAG_FIRST_CYCLE=1;
double TIP_1_CURRENT_BOSS_HEIGHT=0;
double TIP_2_CURRENT_BOSS_HEIGHT=0;
double TIP_3_CURRENT_BOSS_HEIGHT=0;
double TIP_4_CURRENT_BOSS_HEIGHT=0;

static uint32_t lastCounterValue = 0;  // To store the last known counter value
uint8_t FLAG_MACHINE_START=1;
double slow_down_distance=0;

// Define the buffer size for the last 10 values
#define BUFFER_SIZE 50

// Define arrays to store the last 10 values for each tip distance
double tip_1_distance_buffer[BUFFER_SIZE];
double tip_2_distance_buffer[BUFFER_SIZE];
double tip_3_distance_buffer[BUFFER_SIZE];
double tip_4_distance_buffer[BUFFER_SIZE];



// Initialize variables for tracking the current index and sum of values
int current_index = 0;
double tip_1_sum = 0.0;
double tip_2_sum = 0.0;
double tip_3_sum = 0.0;
double tip_4_sum = 0.0;

double average_tip_1;
double average_tip_2;
double average_tip_3;
double average_tip_4;

//// Define a delay threshold of 500ms in milliseconds
//#define INTERFERENCE_DELAY_THRESHOLD 1500
//
//// Define variables to store the timestamp when the condition was first met
//uint32_t tip_1_condition_start_time = 0;
//uint32_t tip_2_condition_start_time = 0;
//uint32_t tip_3_condition_start_time = 0;
uint8_t FLAG_CYCLE_SUCCESS=0;
uint32_t Tip_1_On_Start_Time=0;
uint32_t Tip_2_On_Start_Time=0;
uint32_t Tip_3_On_Start_Time=0;
uint32_t Tip_4_On_Start_Time=0;

// Define variables to store the timestamp when the condition was first met
uint32_t tip_1_condition_start_time = 0;
uint32_t tip_2_condition_start_time = 0;
uint32_t tip_3_condition_start_time = 0;
uint32_t tip_4_condition_start_time = 0;

uint8_t FLAG_RUN_BUZZER=0;
uint32_t Buzzer_duration;
uint16_t Buzzer_beeps;
uint32_t Buzzer_delay;
uint8_t FAULT_HANDS_RELEASED_FLAG=0;
uint8_t FLAG_CYCLE_STARTED=0;


uint8_t HMI_MODAL_LOCK_NUM_1=0;
uint8_t HMI_MODAL_LOCK_NUM_2=0;
uint8_t HMI_MODAL_LOCK_NUM_3=0;
uint8_t HMI_MODAL_LOCK_NUM_4=0;
uint8_t HMI_MODAL_LOCK_NUM_5=0;
uint8_t HMI_MODAL_LOCK_NUM_6=0;
uint8_t HMI_MODAL_LOCK_NUM_7=0;
uint8_t HMI_MODAL_LOCK_NUM_8=0;
uint8_t HMI_MODAL_LOCK_NUM_9=0;
uint8_t FLAG_SCREEN_LOCKED=1;
uint8_t keypad_index=0;

uint8_t FLAG_HOMING_WITH_COOLING=0;

uint8_t HMI_Tip_1_Offset_Up;
uint8_t HMI_Tip_2_Offset_Up;
uint8_t HMI_Tip_3_Offset_Up;
uint8_t HMI_Tip_4_Offset_Up;

uint8_t HMI_Tip_1_Offset_Down;
uint8_t HMI_Tip_2_Offset_Down;
uint8_t HMI_Tip_3_Offset_Down;
uint8_t HMI_Tip_4_Offset_Down;

double HMI_Tip_1_Offset;
double HMI_Tip_2_Offset;
double HMI_Tip_3_Offset;
double HMI_Tip_4_Offset;


static uint8_t tip1_heating_state = STATE_HEATING;
static uint32_t tip1_delay_start_time = 0;
static float last_delay_energy_value = 0;
static uint8_t tip2_heating_state = STATE_HEATING;
static uint32_t tip2_delay_start_time = 0;
static float last_delay_energy_value_2 = 0;
static uint8_t tip3_heating_state = STATE_HEATING;
static uint32_t tip3_delay_start_time = 0;
static float last_delay_energy_value_3 = 0;
static uint8_t tip4_heating_state = STATE_HEATING;
static uint32_t tip4_delay_start_time = 0;
static float last_delay_energy_value_4 = 0;

uint8_t FLAG_REPEAT_CYCLE=0;

double Tip_1_Collapse_Rate=0;
double Tip_2_Collapse_Rate=0;
double Tip_3_Collapse_Rate=0;
double Tip_4_Collapse_Rate=0;

uint8_t Brass_Mode_Enabled;

double Brass_Mode_Distance_Val = 4.9;
double AUTO_CYCLE_STOP_HEIGHT= 8.0;
double DISTANCE_STOP_TOLERANCE =  0.5;
