///////////////////////////////
//// Ports                 ////
//// #define <name> <port> ////
///////////////////////////////

//Motor
#define motorFourBarL     2
#define motorFourBarR     3
#define motorWheelLF      4
#define motorWheelRF      5
#define motorWheelLB      6
#define motorWheelRB      7
#define motorWheelM       8
#define motorGrabberL     9
#define motorGrabberR    10

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
 * @param motorGroup "wheelsL", "wheelsR", "fourBar", or "grabber"
 * @param speed the new signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction, with 0 being off
 */
void motorGroupSet(char motorGroup[], int speed);

/**
 * Runs four-bar with PID to reach/maintain target height
 * @param height the potentiometer reading that corresponds to the target height; -1 will
 * disable PID control but will continue recording sensor readings for derivative calculation
 */
void fourBarToHeight(int height);