/*
 * Encoder.c
 *
 *  Created on: 15 de jul de 2024
 *      Author: wodwa
 */
#include "Encoder.h"

float theta0 = 0, theta1 = 0;
float freqsignal = 0;

void Encoder_Calc(st_Encoder *obj){
    static float oldp = 0;
    float Tmp1, tmp, newp;
    uint16_t pos16bval, temp1;
    // Position calculation - mechanical and electrical motor angle
    obj->Direction = EQep1Regs.QEPSTS.bit.QDF;      // Motor direction: 0=CCW/reverse, 1=CW/forward
    pos16bval = (uint16_t)EQep1Regs.QPOSCNT;        // capture position once per QA/QB period
    obj->theta_raw = pos16bval + obj->cal_angle;    // raw theta = current pos. ang. offset from QA

    // The following lines calculate
    // theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
    // where mech_scaler = 4000 cnts/revolution
    obj->theta_mech = MECH_SCALER * (float)(obj->theta_raw);
    if(obj->theta_mech > 6.28318530718) obj->theta_mech = obj->theta_mech - 6.28318530718;
    else if(obj->theta_mech < -6.28318530718) obj->theta_mech = obj->theta_mech + 6.28318530718;

    // The following lines calculate elec_mech
    obj->theta_elec = POLE_PAIRS*obj->theta_mech;
    if(obj->theta_elec > 6.28318530718) obj->theta_elec = obj->theta_elec - 6.28318530718;
    else if(obj->theta_elec < -6.28318530718) obj->theta_elec = obj->theta_elec + 6.28318530718;

    theta0 = obj->theta_elec;
    freqsignal = (theta0-theta1)*10.0*523.0;
    theta1 = theta0;
    // Check an index occurrence
    if(EQep1Regs.QFLG.bit.IEL == 1) EQep1Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
    // High Speed Calculation using QEP Position counter, check unit Time out-event for speed calculation:
    // Unit Timer is configured for 10Hz in INIT function
    if(EQep1Regs.QFLG.bit.UTO == 1){    // If unit timeout (one 10Hz period)
        // Differentiator
        // The following lines calculate position = (x2-x1)/40000 (position in 1 revolution)
        pos16bval = (uint16_t)EQep1Regs.QPOSLAT;   // Latched POSCNT value
        tmp = MECH_SCALER *(float)pos16bval;
        newp = tmp;
        //oldp = obj->oldpos;
        if(obj->Direction == 0){                // POSCNT is counting down
            if(newp > oldp) Tmp1 = newp - oldp -1.0;    // x2-x1 should be negative
            else Tmp1 = newp - oldp;
        }else if(obj->Direction == 1){          // POSCNT is counting up
            if(newp<oldp) Tmp1 = 1.0 + newp - oldp;
            else Tmp1 = newp - oldp;            // x2-x1 should be positive
        }
        if(Tmp1> 1.0) obj->Speed_fr = 1.0;
        else if(Tmp1 < -1.0) obj->Speed_fr = -1.0;
        else obj->Speed_fr = Tmp1;
        //obj->oldpos = newp;                     // Update the electrical angle
        oldp = newp;                     // Update the electrical angle
        // Change motor speed from pu value to rpm value
        obj->SpeedRpm_fr = BASE_RPM * obj->Speed_fr;
        EQep1Regs.QCLR.bit.UTO = 1;               // Clear __interrupt flag
    }
    // Low-speed computation using QEP capture counter
    if(EQep1Regs.QEPSTS.bit.UPEVNT == 1){       // Unit position event
        if(EQep1Regs.QEPSTS.bit.COEF == 0) temp1 = (uint16_t)EQep1Regs.QCPRDLAT; // temp1 = t2-t1, no Capture overflow
        else temp1 = 0xFFFF;                    // Capture overflow, saturate the result
        // Speed_pr = p->SpeedScaler/temp1
        obj->Speed_pr = SPEED_SCALER/((float)temp1);
        Tmp1 = obj->Speed_pr;
        if (Tmp1> 1.0) obj->Speed_pr = 1.0;
        else obj->Speed_pr = Tmp1;
        // Convert p->Speed_pr to RPM
        if(obj->Direction == 0) obj->SpeedRpm_pr = - BASE_RPM*obj->Speed_pr;    // Reverse direction = negative
        else obj->SpeedRpm_pr = BASE_RPM*obj->Speed_pr;                         // Forward direction = positive
        EQep1Regs.QEPSTS.all = 0x88;            // Clear Unit position event flag
    }
}
