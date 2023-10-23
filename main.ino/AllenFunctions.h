#ifndef ALLENFUNCTIONS_H
#define ALLENFUNCTIONS_H

#define PWM1 2
#define AIN1 26
#define AIN2 24
#define PWM2 3
#define BIN1 30
#define BIN2 32
#define STANDBY 28

#define TRIG_L 20 // Left Ultrasonic
#define ECHO_L 22 
#define TRIG_R 31 // Right ultrasonic
#define ECHO_R 29
#define ECHO_B 49 // Back ultrasonic
#define TRIG_B 47

#define BR_S 43 // Back right sensor
#define FL_S 25 // Fron left sensor
#define BL_S 45 // Back left sensor
#define FR_S 23 // Front right sensor

// Put function declarations here
void blinkLED(int ledPin, int delayTime); //this is just an example

#endif