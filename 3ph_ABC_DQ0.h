/*
 * 3ph_ABC_DQ0.h
 *
 *  Created on: 27 de mai de 2023
 *      Author: waner
 */

#ifndef ABC_DQ0_H_
#define ABC_DQ0_H_

typedef struct{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float sin_;
    float cos_;
    float d;
    float q;
    float z;
}DQ0_ABC;

typedef struct{
    float   B1_lf;
    float   B0_lf;
    float   A1_lf;
}SPLL_3ph_SRF_LPF_COEFF;

typedef struct{
    float v_q[2];
    float ylf[2];
    float fo; // output frequency of PLL
    float fn; //nominal frequency
    float theta[2];
    float delta_T;
    SPLL_3ph_SRF_LPF_COEFF lpf_coeff;
}SPLL_3ph_SRF;

void ABC_DQ0_Init(DQ0_ABC *v);
void DQ0_to_ABC(DQ0_ABC *v);
void ABC_to_DQ0(DQ0_ABC *v);
void SPLL_3ph_SRF_Init(int Grid_freq, float DELTA_T, SPLL_3ph_SRF *spll_obj);
void SPLL_3ph_SRF_Func(SPLL_3ph_SRF *spll_obj);

//obj->sin = CLAsin(obj->theta);
//obj->cos = CLAcos(obj->theta);

#pragma FUNC_ALWAYS_INLINE(In_DQ0_to_ABC)
inline void In_DQ0_to_ABC(DQ0_ABC *v){
    v->alpha = v->d*v->cos_ - v->q*v->sin_;
    v->beta  = v->d*v->sin_ + v->q*v->cos_;
    v->a = v->alpha + 0.5*v->z;
    v->b = -0.5*v->alpha + 0.8660254*v->beta + 0.5*v->z;
    v->c = -0.5*v->alpha - 0.8660254*v->beta + 0.5*v->z;
}

#pragma FUNC_ALWAYS_INLINE(In_ABC_to_DQ0)
inline void In_ABC_to_DQ0(DQ0_ABC *v){
    v->alpha=(0.6666666667)*(v->a-0.5*(v->b+v->c));
    v->beta=(0.57735026913)*(v->b-v->c);
    v->z =0.57735026913*(v->a+v->b+v->c);
    v->d=v->alpha*v->cos_+v->beta*v->sin_;
    v->q=-v->alpha*v->sin_+v->beta*v->cos_;
}

#pragma FUNC_ALWAYS_INLINE(In_SPLL_3ph_SRF_Func)
inline void In_SPLL_3ph_SRF_Func(SPLL_3ph_SRF *v){
    v->ylf[0]=v->ylf[1] + (v->lpf_coeff.B0_lf*v->v_q[0]) + (v->lpf_coeff.B1_lf*v->v_q[1]);
    v->ylf[1]=v->ylf[0];
    v->v_q[1]=v->v_q[0];
    v->ylf[0]=(v->ylf[0]>(200.0))?(200.0):v->ylf[0];
    // VCO
    v->fo=v->fn + v->ylf[0];
    v->theta[0]=v->theta[1] + ((v->fo*v->delta_T)*(2.0*3.1415926));
    if(v->theta[0] > (2.0*3.1415926))
        v->theta[0]=v->theta[0] - (2.0*3.1415926);
    v->theta[1]=v->theta[0];
}

#endif /* 3PH_ABC_DQ0_H_ */
