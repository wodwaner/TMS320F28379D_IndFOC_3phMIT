/*
 * Encoder.h
 *
 *  Created on: 15 de jul de 2024
 *      Author: wodwa
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "Peripheral_Setup.h"
#define MECH_SCALER     1.0/40000.0   // Parameter: 0.9999/total count, total count = 4000 (Q26)
#define POLE_PAIRS      4.0
#define BASE_RPM        6000.0    // Scaler converting GLOBAL_Q speed to rpm
#define SPEED_SCALER    250

#define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,16776,2,0,0x0,250,0,6000,0,0,0,0,}

typedef struct {
    uint8_t Direction;      // Output: Motor rotation direction
    int16_t theta_raw;     // Variable: Raw angle from Timer 2 (Q0)
    int16_t cal_angle;      // Parameter: Raw angular offset between encoder and phase a (Q0)
    float theta_mech;       // Output: Motor Mechanical angle
    float theta_elec;         // Output: Motor Electrical angle
    //float oldpos;            // Input: Electrical angle (pu)
    float Speed_pr;           // Output :  speed in per-unit Low Speed
    float Speed_fr;           // Output :  speed in per-unit High Speed (Position Delta/100ms) * 60 rpm
    float SpeedRpm_pr;
    float SpeedRpm_fr;
}  st_Encoder;

void Encoder_Calc(st_Encoder *obj);
#endif /* ENCODER_H_ */
