////////////////////////////////////////////
//// Macros                             ////
//// #define <identifier> <token>       ////
////////////////////////////////////////////

//Motors
#define MOTOR_CLAPPER_L                 1
#define MOTOR_WHEEL_LB                  2
#define MOTOR_HANGER_L                  3
#define MOTOR_WHEEL_RB                  4
#define MOTORS_ARM_L                    5   //To Power Expander A & B via Y-Cable
#define MOTORS_ARM_R                    6   //To Power Expander C & D via Y-Cable
#define MOTOR_HANGER_R                  7
#define MOTOR_WHEEL_LF                  8
#define MOTOR_WHEEL_RF                  9
#define MOTOR_CLAPPER_R                10

//Analog Sensors
#define SENSOR_POT_ARM                  1
#define SENSOR_POT_CLAPPER              2

//IMEs
#define SENSOR_IME_WHEEL_LF             1
#define SENSOR_IME_WHEEL_RF             2

//Motor Groups
#define MOTORGROUP_WHEELS_L             1
#define MOTORGROUP_WHEELS_R             2
#define MOTORGROUP_ARM                  3
#define MOTORGROUP_CLAPPER              4
#define MOTORGROUP_HANGER               5

//Autonomous Modes
#define AUTON_NONE                      0
#define AUTON_NORMAL                    1
#define AUTON_TIMER                     2



///////////////////////////////
//// Variables             ////
//// extern <type> <name>; ////
///////////////////////////////

extern int autonMode;



////////////////////////////////
//// Functions              ////
//// <type> <name>(<args>); ////
////////////////////////////////

/**
 * Sets a group of motors to the same power and in the correct directions
 * @param motorGroup MOTORGROUP_WHEELS_L, MOTORGROUP_WHEELS_R, MOTORGROUP_ARM,
 * MOTORGROUP_CLAPPER, or MOTORGROUP_HANGER
 * @param speed the new signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSet(unsigned char motorGroup, int speed);

/**
* Runs arm with PID to reach/maintain target angle
* @param target the potentiometer reading that corresponds to the target height, divided by 10
*/
void armToAngle(int target);

/**
* Runs clapper with PID to reach/maintain target openness
* @param target the potentiometer reading that corresponds to the target height, divided by 10
*/
void clapperToOpenness(int target);