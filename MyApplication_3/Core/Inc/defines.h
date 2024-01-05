


#define STEPPER_1_PIN_DIRECTION Step_1_Pin
#define STEPPER_1_PIN_DIRECTION_PORT Step_1_GPIO_Port
#define STEPPER_1_PIN_PULSE STEP_2_Pin
#define STEPPER_1_PIN_PULSE_PORT STEP_2_GPIO_Port

//// Stepper motor control pin
//#define STEP_PIN Step_Pul_Pin
//#define STEP_PORT Step_Pul_GPIO_Port

// Stepper motor direction pin
#define STEP_DIR Step_Dir_Pin
#define STEP_DIR_PORT Step_Dir_GPIO_Port

//Max Values
#define MAX_ENERGY 5000
#define MAX_TOLERANCE 0.9
#define MAX_DISTANCE 20000
#define MIN_WORK_POSITION 500
#define MAX_DISTANCE_TIP 8

#define SSR_VOLTAGE 110 //in volts rms
#define SSR_PING_TIME 20 // defines how long before a ping packet is sent to SSR board
uint8_t SSRDeviceAddress = 0x48;

#define STEPS_PER_REVOLUTION 1000 // Number of steps per full revolution for the stepper motor
const uint32_t STEPPER_NUMBER_OF_STEPS = 1000*2; //  steps per rev
const uint32_t MM_PER_REV = 1000; // 10.00mm multiple by 10 to get whole number


#define STEPPER_UP_DIRECTION_SET 1
#define STEPPER_DOWN_DIRECTION_SET 0

//#define PU_HOME 90
//#define AF_HIGH 180
//#define AF_SLOW 30
//#define CD_HIGH 320
//#define CD_LOW  60
//#define CU_LOW  60
//#define CU_HIGH 320

#define PU_HOME 180
#define AF_HIGH 200
#define AF_SLOW 120
#define CD_HIGH 720
#define CD_LOW  120
#define CU_LOW  120
#define CU_HIGH 720

#define STEPPER_SPEED_LOW 40 // RPM
#define STEPPER_SPEED_MID_LOW 90
#define STEPPER_SPEED_MID 90 // RPM
#define STEPPER_SPEED_HIGH 180 // RPM


#define DELAY_AFTER_HEATING 3000 // Distance change tolerance must happen within this time
#define DISTANCE_CHANGE_TOLERANCE 0.25 // The change to happen within delay after heating

#define DISTANCE_STOP_TOLERANCE_PLASTIC 0.5
#define DISTANCE_STOP_TOLERANCE_BRASS 0.25

#define PREHEAT_START_DISTANCE 0.5 // Percentace of distance from workheight where preheat would start

#define FAULT_TE012 0
#define FAULT_ENCODER_1 1
#define FAULT_SSR_1 2
#define FAULT_SE008 3
#define FAULT_MAX_DISTANCE 4
#define FAULT_CLEAR 254
#define FAULT_ESTOP 5
#define FAULT_BOSS_NOT_PRESENT 6


#define RED 'D'
#define GREEN 'G'
#define BLUE 'B'
#define OFF 'O'
#define RESET_ENCODER 'R'

#define LINEAR_ENCODER_1_ADDRESS 0x8
#define LINEAR_ENCODER_2_ADDRESS 0x9
#define LINEAR_ENCODER_3_ADDRESS 0xA
#define LINEAR_ENCODER_4_ADDRESS 0xB

#define AUTOFIND_STOP_HEIGHT 3.0
#define AUTOCYCLE_STOP_HEIGHT 8.0

#define GLOBAL_TOLERANCE_VAL (HMI_BOSS_HEIGHT_TOLERANCE_VAL/100)

#define TOTAL_TIPS 4

uint32_t SCREEN_REFRESH_RATE=20;

int arr=40;
uint8_t FLAG_RUN_STEPPER=0;
uint8_t FLAG_STEPPER_RUNNING=0;
uint8_t FLAG_STEPPER_DOWN=0;

#define BOSS_SENSE_HEIGHT 3
#define MOVE_UP_HITTING_BOSS_DISTANCE 1500 // 15mm

#define PULSE_DISTANCE 1  // Assuming each pulse corresponds to a 0.1 unit movement, adjust accordingly
#define MAX_COUNTER_VALUE 0xFFFF  // Max value for a 16-bit timer

#define INTERFERENCE_TOLERANCE 100 // 0.8MM
#define BOSS_NOT_PRESENT_TOLERANCE 300 //100MM
#define BRASS_TOLERANCE 0.25

// Define a delay threshold of 500ms in milliseconds
#define INTERFERENCE_DELAY_THRESHOLD 1500

#define TIP_HEATING_TIMEOUT 1000


#define FAST	1
#define MEDIUM	2
#define SLOW    3

#define HOMING_WITH_COOLING_DELAY 100

#define MAX_TIP_DISTANCE_OFFSET 1.0

#define HEATING_DELAY_PER_STEP 2000
#define JOULES_PER_STEP  200

// Define state constants
#define STATE_HEATING 0
#define STATE_DELAY 1


#define REPEAT_CYCLE_DELAY 10000

#define COOLING_BURST 10
#define HEATING_BURST 500

#define ENERGY_INCREMENTS 5

#define PLATEN_RETRACT_AFTER_COOLING 200 //2.0mm

