/*
 * Dlog.c
 *
 *  Created on: 15 de fev de 2022
 *      Author: wodwa
 */
#include "Dlog.h"

void DLOG_Init(DLOG *v){
    v->prev_value = 0;
    v->trig_value = 0;
    v->status = NO_TRIGGER;
    v->pre_scalar = 0;
    v->skip_count = 0;
    v->size = SIZE_LOG;
    v->count = 0;
}

//*********** Function Definition ********//
void DLOG_Func(DLOG *v){
    switch(v->status){
    case WAIT_TRIGER: // wait for trigger
        if((*v->input_trigger > v->trig_value) && (v->prev_value < v->trig_value)){
            // rising edge detected start logging data*
            v->status = TRIGGERED;
        }

        v->skip_count++;
        if(v->skip_count >= v->pre_scalar){
            v->skip_count = 0;
            v->log_ch1[v->count]=*v->input_ch1;
            v->log_ch2[v->count]=*v->input_ch2;
            v->log_ch3[v->count]=*v->input_ch3;
            v->count++;
            if(v->count == 20 && v->status == WAIT_TRIGER) v->count = 0;
        }

        break;
    case TRIGGERED:
        v->skip_count++;
        if(v->skip_count >= v->pre_scalar){
            v->skip_count = 0;
            v->log_ch1[v->count]=*v->input_ch1;
            v->log_ch2[v->count]=*v->input_ch2;
            v->log_ch3[v->count]=*v->input_ch3;
            v->count++;
            if(v->count == v->size){
                v->count = 0;
                v->status = CAPTURED;
            }
        }
        break;
    case NO_TRIGGER: // always sampled
        v->skip_count++;
        if(v->skip_count >= v->pre_scalar){
            v->skip_count = 0;
            v->log_ch1[v->count] = *v->input_ch1;
            v->log_ch2[v->count] = *v->input_ch2;
            v->log_ch3[v->count] = *v->input_ch3;

            v->count = (v->count == (v->size - 1)) ? 0 : (v->count+1);
        }
    }
    v->prev_value = *v->input_trigger;
}
