#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "../src/epos4/epos4_def.h"

// #define SIMULATE

#define IP_ADDR "10.2.1.177"
#define MASTER_ADDR "10.2.1.1"
#define PORT 8888
#define CS_PIN 10

#define NUM_BYTES_PER_VALUE sizeof(int32_t)   // 32 bit integer
#define MAX_BUFFER_SIZE 1024

const OpMode OP_MODE[] = { CyclicSyncPosition };
const int ENCODER_TICKS_PER_TURN[] = { 2048 };  // ticks per 90 deg turn
const int MOTOR_POLARITY[] = { 0 };  // Direction of rotation 0: ccw, 1 cw
const int HOMING_METHOD[] = { CurrentThresholdNegative };
const int HOMING_THRESHOLD[] = { 5000 };

#define PDO_RATE 10    // ms

#define MAX_FOLLOW_ERROR 30000
const int EC45_DATA[] = { 3210, 5000, 36900, 296, 8, 0, 781067, 1386330, 3303872, 11508462, 79040, 0, 630 };

#define GT2_PULLEY_RADIUS 5.05
// PID control params
const float Kp = 0.001;
const float Kd = 0.0001;
const float Ki = 1;
#define POSITION_OFFSET 0
#define MIN_POSITION_MM 0.f
#define MAX_POSITION_MM 50.f

#endif // DEFINITIONS_H