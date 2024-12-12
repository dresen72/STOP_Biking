/*

 @authors: George Fang, Luke Andresen

 @date: 12/12/2024

 @brief: Configurations for the main code

*/

// COMPUTATION VARIABLES
#define CIRC           2.03481 // Circumfrence of the bike wheel
#define CIRC_8TH       0.25435 // 1/8 of the circumfrece
#define MPH_CONV       2.237   // M/s to MPH conversion
#define MAX_SPEED      20.0    // Max speed for display dial
#define DIAL_RAD       60      // Radius for the speed dial
#define HIST_LEN       5
#define JUMP_FACTOR    1.5
#define SERVO_MIN      100
#define SERVO_MAX      10 // 30
#define BRAKE_FACTOR   0.3464 // 0.335 (w/ 1.4 dist metric) //0.3464 was good// 0.309105 (.4 was too much)
#define BRAKE_ADJUSTMENT 0.15 // 0.075 (w/ 1.4 dist metric)// 0.2 was good
#define BRAKE_FACTOR_SMALL 0.33
#define BRAKE_ADJUSTMENT_SMALL 0.1

// LIDAR VARIABLES
#define FAST_I2C


// PIN DECLARATIONS
#define HALL           33 // Hall effect sensor
#define BUZZER         14 // Buzzer
#define BRAKE          5  // Servo

#define TFT_CS        10 // LCD CS
#define TFT_RST        9 // LCD RST
#define TFT_DC         8 // LCD  DC


