/*
 * 3ph_ABC_DQ0.c
 *
 *  Created on: 27 de mai de 2023
 *      Author: waner
 */

#include "3ph_ABC_DQ0.h"

void ABC_DQ0_Init(DQ0_ABC *v){
    v->a=0;
    v->b=0;
    v->c=0;
    v->alpha=0;
    v->beta=0;
    v->z=0;
    v->d=0;
    v->q=0;
}

void DQ0_to_ABC(DQ0_ABC *v){
    v->alpha = v->d*v->cos_ - v->q*v->sin_;
    v->beta  = v->d*v->sin_ + v->q*v->cos_;
    v->a = v->alpha + 0.5*v->z;
    v->b = -0.5*v->alpha + 0.8660254*v->beta + 0.5*v->z;
    v->c = -0.5*v->alpha - 0.8660254*v->beta + 0.5*v->z;
}

void ABC_to_DQ0(DQ0_ABC *v){
    v->alpha=(0.6666666667)*(v->a-0.5*(v->b+v->c));
    v->beta=(0.57735026913)*(v->b-v->c);
    v->z =0.57735026913*(v->a+v->b+v->c);
    v->d=v->alpha*v->cos_+v->beta*v->sin_;
    v->q=-v->alpha*v->sin_+v->beta*v->cos_;
}

void SPLL_3ph_SRF_Init(int Grid_freq, float DELTA_T, SPLL_3ph_SRF *spll_obj){
    spll_obj->v_q[0]=(0.0);
    spll_obj->v_q[1]=(0.0);
    spll_obj->ylf[0]=(0.0);
    spll_obj->ylf[1]=(0.0);
    spll_obj->fo=(0.0);
    spll_obj->fn=(Grid_freq);
    spll_obj->theta[0]=(0.0);
    spll_obj->theta[1]=(0.0);
    // loop filter coefficients for 36kHz
    spll_obj->lpf_coeff.B0_lf=(222.51);
    spll_obj->lpf_coeff.B1_lf=(-221.81);
    spll_obj->lpf_coeff.A1_lf=(-1.0);
    spll_obj->delta_T=DELTA_T;
}

void SPLL_3ph_SRF_Func(SPLL_3ph_SRF *spll_obj){
    //update v_q[0] before calling the routine
    // Loop Filter
    spll_obj->ylf[0]=spll_obj->ylf[1] + (spll_obj->lpf_coeff.B0_lf*spll_obj->v_q[0]) + (spll_obj->lpf_coeff.B1_lf*spll_obj->v_q[1]);
    spll_obj->ylf[1]=spll_obj->ylf[0];
    spll_obj->v_q[1]=spll_obj->v_q[0];
    spll_obj->ylf[0]=(spll_obj->ylf[0]>(200.0))?(200.0):spll_obj->ylf[0];
    // VCO
    spll_obj->fo=spll_obj->fn + spll_obj->ylf[0];
    spll_obj->theta[0]=spll_obj->theta[1] + ((spll_obj->fo*spll_obj->delta_T)*(2.0*3.1415926));

    if(spll_obj->theta[0] > (2.0*3.1415926)) spll_obj->theta[0]=spll_obj->theta[0] - (2.0*3.1415926);
    else if(spll_obj->theta[0] < (-2.0*3.1415926)) spll_obj->theta[0] = spll_obj->theta[0] + (2.0*3.1415926);

    spll_obj->theta[1]=spll_obj->theta[0];
}

