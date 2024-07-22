/*
 * PI.c
 *
 *  Created on: 27 de mai de 2023
 *      Author: waner
 */
#include "PI.h"


void PI_Calc(PIDREG *v){
    v->Up = v->Kp*v->Err;
    v->Ui = v->Ui + v->Ki*(v->Err + v->Err)*0.5 + (v->Kc*v->SatErr);
    v->OutPreSat = v->Up + v->Ui+v->Feedforward;
     v->Out = v->OutPreSat;
    if (v->OutPreSat > v->OutMax)  v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin) v->Out =  v->OutMin;
    v->SatErr = v->Out - v->OutPreSat;
    v->Err1 = v->Err;
}
void PI_Reset(PIDREG *v){
    v->Err = 0;
    v->Out = 0;
    v->OutPreSat = 0;
    v->Ui = 0;
    v->Err1 = 0;
    v->SatErr = 0;
    v->Out = 0;
}

