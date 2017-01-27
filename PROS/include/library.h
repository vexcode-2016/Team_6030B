////////////////////////////////////////////
//// Macros                             ////
//// #define <identifier> <token>       ////
////////////////////////////////////////////

//Motors
#define MOTOR_WHEEL_LB                  2
#define MOTORS_CLAPPER                  3   //Y-Cabled; right motor is wiring-reversed
#define MOTOR_WHEEL_RB                  4
#define MOTORS_ARM_L_HIGH               5   //To Power Expander A & B via Y-Cable
#define MOTORS_ARM_R_HIGH               6   //To Power Expander C & D via Y-Cable
#define MOTORS_ARM_LR_LOW               7   //Y-Cabled; right-side motor is wiring-reversed
#define MOTOR_WHEEL_LF                  8
#define MOTOR_WHEEL_RF                  9

//Analog Sensors
#define SENSOR_POT_ARM                  1
#define SENSOR_POT_CLAPPER              2
#define SENSOR_GYRO                     3
#define SENSOR_ACCEL_LX                 5
#define SENSOR_ACCEL_LY                 6
#define SENSOR_ACCEL_RX                 7
#define SENSOR_ACCEL_RY                 8

//Digital Sensors
#define JUMPER_SKILLS                  11
#define JUMPER_AUTON                   12

//Motor Groups
#define MOTORGROUP_WHEELS_L             1
#define MOTORGROUP_WHEELS_R             2
#define MOTORGROUP_ARM                  3
#define MOTORGROUP_CLAPPER              4

//QwikScore Modes
#define QWIKSCORE_INACTIVE              0
#define QWIKSCORE_GRAB                  1
#define QWIKSCORE_ROTATE                2
#define QWIKSCORE_DRIVE                 3
#define QWIKSCORE_THROW                 4
#define QWIKSCORE_DONE                  5



///////////////////////////////
//// Variables             ////
//// extern <type> <name>; ////
///////////////////////////////

//Autonomous
extern int autonMode;

//Arm
extern const int armFloorGrab;
extern const int armThrow;
extern const int armFence;
extern int armTarget;
extern int armKpUp;
extern int armKpDown;

//Clapper
extern const int clapperHold;
extern const int clapperReady;
extern const int clapperFence;
extern const int clapperBack;
extern int clapperTarget;
extern int clapperKp;

//QwikScore
extern int qwikScoreMode;
extern int qwikScoreXtraIter;
extern Gyro gyro;



////////////////////////////////
//// Functions              ////
//// <type> <name>(<args>); ////
////////////////////////////////

/**
 * Sets a group of motors to the same speed and in the correct directions
 * @param motorGroup MOTORGROUP_WHEELS_L, MOTORGROUP_WHEELS_R, MOTORGROUP_ARM,
 * MOTORGROUP_CLAPPER, or MOTORGROUP_HANGER
 * @param speed the desired signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSet (unsigned char motorGroup, int speed);

/**
 * Runs arm with PID to reach/maintain target angle
 * @param target the potentiometer reading that corresponds to the target height, divided by 10
 */
void armToAngle (int target);

/**
 * Runs clapper with PID to reach/maintain target openness
 * @param target the potentiometer reading that corresponds to the target height, divided by 10
 */
void clapperToOpenness (int target);

/**
 * Runs drivetrain with PID to reach target position/rotation on the field
 * @param left the target output value for the left IME
 * @param right the target output value for the right IME
 */
void robotToPosition (int left, int right);

/**
 * Closes the clapper, raises the arm, rotates, and drives as necessary to score in 1 graceful motion
 * ENCLOSE IN WHILE LOOP AND RESET 'qwikScoreMode' TO 'QWIK_SCORE_INACTIVE' AFTER USE!!!
 * @param autoDrive whether or not the robot should autonomously rotate and drive to the fence (1 = yes, 0 = no)
 */
void qwikScore (int autoDrive);