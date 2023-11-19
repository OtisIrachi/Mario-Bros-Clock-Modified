//*************************************************************************
// defines.h
//
//
//
//
// Modified by RCI
//*************************************************************************

#ifndef DEFINES_H_
#define DEFINES_H_

// Stepper Driver Pins for ESP Stepper I/O Board
#define PIN_ENABLE_MOTORS 5        
#define PIN_MOTOR1_DIR 19    
#define PIN_MOTOR1_STEP 18    
#define PIN_MOTOR2_DIR 16    
#define PIN_MOTOR2_STEP 17   
#define PIN_SERVO 4             
#define PIN_WIFI_LED 2        
#define PIN_BUZZER 33 
// FOR BLINKIN EYES.H
//#define DATA_PIN  25      // IO25  
//#define CS_PIN    26      // or D0
//#define CLK_PIN   27      // IO27

         
// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 150  // was 550
#define MAX_STEERING 80  // was 140
#define MAX_TARGET_ANGLE 14   // was 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32

#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

// These were tested with EVO-2 design, very stable
#define KP 0.15   // was .3
#define KD 0.08
#define KP_THROTTLE 0.1  // was .080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.06
#define KD_POSITION 0.45
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.08
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500   // was 500
#define ITERM_MAX_ERROR 30   // was 30 Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate robot own weight distribution)

// Servo Arm defaults
#define SERVO_AUX_NEUTRO 4800  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH SERVO_AUX_NEUTRO - 2700
#define SERVO_MAX_PULSEWIDTH SERVO_AUX_NEUTRO + 2700

// Servo 2 defaults
#define SERVO2_NEUTRO 4200
#define SERVO2_RANGE 8400

#define ZERO_SPEED 0xffffff


#define MICROSTEPPING 8   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

// AUX definitions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#endif /* DEFINES_H_ */
