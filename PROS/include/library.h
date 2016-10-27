////////////////////////////////////////////
//// Commonly Used Tokens               ////
//// #define <identifier> <token>       ////
////////////////////////////////////////////

//Motor Ports
#define MOTOR_FOURBAR_R    2
#define MOTOR_FOURBAR_L    3
#define MOTOR_WHEEL_LF     4
#define MOTOR_WHEEL_RF     5
#define MOTOR_WHEEL_LB     6
#define MOTOR_WHEEL_RB     7
#define MOTOR_WHEEL_M      8
#define MOTOR_CLAW         9

//Analog Sensor Ports
#define SENSOR_FOURBAR_POT    1

//Motor Groups
#define MOTORGROUP_WHEELS_L    1
#define MOTORGROUP_WHEELS_R    2
#define MOTORGROUP_FOURBAR     3



///////////////////////////////
//// Variables             ////
//// extern <type> <name>; ////
///////////////////////////////

//Four-Bar
extern const int fourBarMax;
extern const int fourBarFenceHigh;
extern const int fourBarFenceLow;
extern const int fourBarMin;
extern int fourBarPreset;



////////////////////////////////
//// Functions              ////
//// <type> <name>(<args>); ////
////////////////////////////////

/**
 * Sets a group of motors to the same power and in the correct directions
 * @param motorGroup MOTORGROUP_WHEELS_L, MOTORGROUP_WHEELS_R, or MOTORGROUP_FOURBAR
 * @param speed the new signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSet(unsigned char motorGroup, int speed);

/**
 * Runs four-bar with PID to reach/maintain target height
 * @param target the potentiometer reading that corresponds to the target height, divided by 10; -1
 * will disable PID control but will continue recording sensor readings for derivative calculation
 */
void fourBarToHeight(int target);