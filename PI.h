/*
 * PI.h
 *
 *  Created on: 27 de mai de 2023
 *      Author: waner
 */

#ifndef PI_H_
#define PI_H_

typedef struct {
    float  Err;             // Variable: Error
    float  Kp;              // Parameter: Proportional gain
    float  Up;              // Variable: Proportional output
    float  Ui;              // Variable: Integral output
    float  OutPreSat;    // Variable: Pre-saturated output
    float  OutMax;          // Parameter: Maximum output
    float  OutMin;          // Parameter: Minimum output
    float  Out;             // Output: PID output
    float  SatErr;          // Variable: Saturated difference
    float  Ki;              // Parameter: Integral gain
    float  Kc;              // Parameter: Integral correction gain
    float  Err1;           // History: Previous proportional output
    float  Feedforward;   // History: Previous proportional output
    void  (*calc)();        // Pointer to calculation function
    void  (*reset)();       // Pointer to reset function
} PIDREG;

void PI_Calc(PIDREG *v);
void PI_Reset(PIDREG *v);
#define PIDREG_DEFAULTS {0,0,0,0,0,0,0,0,0,0,1.0,0,0,(void (*)(unsigned int))PI_Calc,(void (*)(unsigned int))PI_Reset}

#pragma FUNC_ALWAYS_INLINE(In_PI_Calc)
inline void In_PI_Calc(PIDREG *v){
    v->Up = v->Kp*v->Err;
    v->Ui = v->Ui + v->Ki*(v->Err+v->Err1)*0.5 + (v->Kc*v->SatErr);
    v->OutPreSat = v->Up + v->Ui +v->Feedforward;
    if (v->OutPreSat > v->OutMax)  v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin) v->Out =  v->OutMin;
    else v->Out = v->OutPreSat;
    v->SatErr = v->Out - v->OutPreSat;
    v->Err1 = v->Err;
}
#pragma FUNC_ALWAYS_INLINE(In_PI_Reset)
inline void In_PI_Reset(PIDREG *v){
    v->Err = 0;
    v->Out = 0;
    v->OutPreSat = 0;
    v->Ui = 0;
    v->Err1 = 0;
    v->SatErr = 0;
    v->Out = 0;
}


#endif /* PI_H_ */
