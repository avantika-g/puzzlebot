/*
 *  This header contains params
 *  Author: shayi@cs.cmu.edu
 */

#ifndef Esp32Motor_h
#define Esp32Motor_h

#define PI 3.1415926535897932384626433832795
#define R 0.002 // radius of wheel 
#define TH 2 // threshold for pid reset
#define MAX_PWM 255
#define MIN_PWM 10
#define PID_SUP_UPPER 280
#define PID_UPPER 150
#define PID_LOWER 70

// wifi related setup
const uint16_t port   = 8080;             // we use the same port number for host and client
// NSH lab wifi
const char* host      = "192.168.2.222";     // host computer ip
const char* ssid      = "RoboSAR2";     // Change this to your WiFi SSID RoboSAR2
const char* password  = "robosar2022"; // Change this to your WiFi password robosar2022
// Wean lab wifi

// const char* host      = "192.168.1.16";     // host computer ip
// const char* ssid      = "NETGEAR15";     // Change this to your WiFi SSID
// const char* password  = "stronghippo553"; // Change this to your WiFi password

// motor related const
const int motor0_m0 = A0;
const int motor0_m1 = A1;
const int motor0_encoderA = D4;
const int motor0_encoderB = D5;
const int motor1_m0 = A3;
const int motor1_m1 = A2;
const int motor1_encoderA = D7;
const int motor1_encoderB = D8;

const double gear_ratio = 30;
const double wheel_gear_ratio = 42/10;
const double ticksPerRev = 12;

// pid
// for pwm super high
const double kP_sh[2] = {0.3, 0.15};
const double kI_sh[2] = {0.01};
const double kD_sh[2] = {0};
// for pwm high cmd
const double kP_h[2] = {0.2, 0.1};
const double kI_h[2] = {0.01};
const double kD_h[2] = {0}; 
// for pwm low cmd
const double kP_l[2] = {0.05, 0.03};
const double kI_l[2] = {0.002, 0.002};
const double kD_l[2] = {0}; 

#endif