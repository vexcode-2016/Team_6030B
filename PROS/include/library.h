////////////////////////////////////////////
//// Macros                             ////
//// #define <identifier> <token>       ////
////////////////////////////////////////////

//Motors
#define MOTOR_HANGER_L                  1
#define MOTOR_WHEEL_LB                  2
#define MOTOR_CLAPPER_L                 3
#define MOTOR_WHEEL_RB                  4
#define MOTORS_ARM_L                    5   //To Power Expander A & B via Y-Cable
#define MOTOR_CLAPPER_R                 6
#define MOTORS_ARM_R                    7   //To Power Expander C & D via Y-Cable
#define MOTOR_WHEEL_LF                  8
#define MOTOR_WHEEL_RF                  9
#define MOTOR_HANGER_R                 10

//Analog Sensors
#define SENSOR_POT_ARM                  1
#define SENSOR_POT_CLAPPER              2
#define SENSOR_GYRO                     3

//Digital Sensors
#define SENSOR_BUMPER_LOW1              1
#define SENSOR_BUMPER_LOW2              2
#define SENSOR_BUMPER_HIGH1             3
#define SENSOR_BUMPER_HIGH2             4

//IMEs (I2C)
#define SENSOR_IME_WHEEL_LF             0
#define SENSOR_IME_WHEEL_RF             1

//Motor Groups
#define MOTORGROUP_WHEELS_L             1
#define MOTORGROUP_WHEELS_R             2
#define MOTORGROUP_ARM                  3
#define MOTORGROUP_CLAPPER              4
#define MOTORGROUP_HANGER               5

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

//Clapper
extern const int clapperHold;
extern const int clapperReady;
extern const int clapperFence;
extern int clapperTarget;

//QwikScore
extern int qwikScoreMode;
extern int qwikScoreXtraIter;
extern Gyro gyro;



////////////////////////////////
//// Functions              ////
//// <type> <name>(<args>); ////
////////////////////////////////

/**
 * Sets a group of motors to the same speed and in the correct directions with slew rate
 * @param motorGroup MOTORGROUP_WHEELS_L, MOTORGROUP_WHEELS_R, MOTORGROUP_ARM,
 * MOTORGROUP_CLAPPER, or MOTORGROUP_HANGER
 * @param speed the desired signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSlew(unsigned char motorGroup, int speed);

/**
 * Sets the speed of the specified motor channel with slew rate
 * @param channel the motor channel to modify from 1-10
 * @param speed the desired signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorSlew (unsigned char channel, int speed);

/**
 * Background task for slew rate control
 * DO NOT RUN DIRECTLY AS A FUNCTION!
 * Use ONLY when creating the background task in initialize()
 */
void slewControlTask (void * parameter);

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