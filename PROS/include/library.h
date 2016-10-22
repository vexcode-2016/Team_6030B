///////////////////////////////
//// Ports                 ////
//// #define <name> <port> ////
///////////////////////////////

//Motor
#define motorFourBarR     2
#define motorFourBarL     3
#define motorWheelLF      4
#define motorWheelRF      5
#define motorWheelLB      6
#define motorWheelRB      7
#define motorWheelM       8
#define motorClaw         9

//Analog
#define sensorFourBarPot    1



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
 * @param motorGroup 1 for wheelsL, 2 for wheelsR, 3 for fourBar", or 4 for grabber
 * @param speed the new signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSet(int motorGroup, int speed);

/**
 * Runs four-bar with PID to reach/maintain target height
 * @param target the potentiometer reading that corresponds to the target height, divided by 10; -1
 * will disable PID control but will continue recording sensor readings for derivative calculation
 */
void fourBarToHeight(int target);