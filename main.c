#include "Peripheral_Setup.h"
#include "Structs.h"
#include "PI.h"
#include "FT.h"
#include "Dlog.h"
#include "3ph_ABC_DQ0.h"
#include "Encoder.h"

#define PI2     6.28318530718
#define VDCBUS  500.0
#define Rr      0.01
#define Lr      0.1E-3
#define INV_Taur (Rr/Lr)
#define N_PAIR_POLES    3
#define RPM_to_RADS (0.1047187*N_PAIR_POLES)

// ====== Local functions ==================
void Run_Ctrl(void);
void SVPWM(float *da, float *db, float *dc);
void PWM_Enable(st_Control *obj);
void Config_Controllers(void);

// ======= Interruptions ===================
__interrupt void isr_cpu_timer0(void);
__interrupt void isr_adc(void);

// ======= Local variables =================
//#pragma SET_DATA_SECTION("CLAData")

//#pragma SET_DATA_SECTION()

uint16_t counter = 0;
st_Measures Measures = {0,0,0,0};
st_Control Control = {0};
st_Offset Offset = {0,0,0,0,Reset};

PIDREG C_iLd = PIDREG_DEFAULTS; // Id current compesator
PIDREG C_iLq = PIDREG_DEFAULTS; // Iq current compesator
PIDREG C_RPM = PIDREG_DEFAULTS; // DC voltage compesator
st_FT Int_w_rads = FT_DEFAULTS;

DQ0_ABC iL_dq0_abc;     // Grid current transform dq0->abc
DQ0_ABC iL_abc_dq0;     // Grid current transform abc -> dq0
st_Encoder Encoder = {0};

DLOG dlog;
float RPM_ref;

int main(void){

    InitSysCtrl();                              // Initialize System Control:
    DisablePeripheralClocks();
    DINT;                                       // Disable CPU interrupts
    InitPieCtrl();                              // Initialize the PIE control registers to their default state
    IER = 0x0000;                               // Disable CPU interrupts
    IFR = 0x0000;                               // Clear all CPU interrupt flags:
    InitPieVectTable();                         // Initialize the PIE vector table

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;       // Enable timer clock
    PieVectTable.TIMER0_INT = &isr_cpu_timer0;  // Redirect function to interruption
    PieVectTable.ADCA1_INT = &isr_adc;          // Redirect function to interruption
    EDIS;

    // pg. 102 PIE Channel Mapping spruhm8i.pdf - Technical reference
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;          // Enable PieVector to Timer 0 interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;          // Enable PieVector to ADCA1 interrupt
    IER |= (M_INT1);                   // Enable lines of interrupt

    Setup_GPIO();                               // Configure GPIOs
    Setup_ePWM();                               // Configure ePWM
    PWM_Enable(&Control);
    Setup_ADC_A();
    Setup_eQep1();
    InitCpuTimers();                            // Initializae cpu timer
    ConfigCpuTimer(&CpuTimer0, 200, 100000);   // Configure cpu timer0 with 200MHz and 1s
    CpuTimer0Regs.TCR.all = 0x4000;             // Enable timer 0
    //========================================

    DLOG_Init(&dlog);
    dlog.input_ch1 = &Measures.iLa;
    dlog.input_ch2 = &Measures.iLb;
    dlog.input_trigger = &Measures.iLa;
    dlog.pre_scalar = 10;
    dlog.skip_count = 0;
    Config_Controllers();
    EINT;                                       // Enable Global interrupt INTM
    ERTM;                                       // Enable Global realtime interrupt DBGM
    GpioDataRegs.GPADAT.bit.GPIO31 = 0;         // Turn off LED

    while(1){
        PWM_Enable(&Control);
        if(Control.Run_Ctrl) Run_Ctrl();
        counter++;
    }// Infinite loop
    return 0;
}

void Run_Ctrl(void){
    static float theta = 0;
    float dutya = 0, dutyb = 0, dutyc = 0;
    float Slip;
    float w_rads;
    float Id_ref, Iq_ref;

    // iLabc to iLdq0
    iL_abc_dq0.a = Measures.iLa;
    iL_abc_dq0.b = Measures.iLb;
    iL_abc_dq0.c = Measures.iLc;
    iL_abc_dq0.sin_ = __sin(theta);
    iL_abc_dq0.cos_ = __cos(theta);
    ABC_to_DQ0(&iL_abc_dq0);

    if(Control.PWM == 1){
        // Speed control
        if(Control.DIR == 1) C_RPM.Err = RPM_ref - Encoder.SpeedRpm_pr;
        else C_RPM.Err = -RPM_ref - Encoder.SpeedRpm_fr;

        C_RPM.calc(&C_RPM);
        Iq_ref = C_RPM.Out;
        Id_ref = 4.55;

        // iLdq control
        C_iLd.Err = Id_ref - iL_abc_dq0.d;
        C_iLd.calc(&C_iLd);
        C_iLq.Err = Iq_ref - iL_abc_dq0.q; // verificar se -1*iL_abc_dq0.q
        C_iLq.calc(&C_iLq);

        // Theta
        Slip = INV_Taur*Iq_ref/Id_ref;
        w_rads = Slip + RPM_to_RADS*Encoder.SpeedRpm_fr;

        Int_w_rads.x0 = w_rads;
        FT_Calc(&Int_w_rads);
        if(Int_w_rads.y0 > PI2) Int_w_rads.y0 -= PI2;
        else if(Int_w_rads.y0 < -PI2) Int_w_rads.y0 += PI2;
        Int_w_rads.y1 = Int_w_rads.y0;

        theta = Int_w_rads.y0;

        // iLdq0 to iLabc
        iL_dq0_abc.d = C_iLd.Out;
        iL_dq0_abc.q = C_iLq.Out;
        iL_dq0_abc.z = 0;
        iL_dq0_abc.sin_ = iL_abc_dq0.sin_;
        iL_dq0_abc.cos_ = iL_abc_dq0.cos_;
        DQ0_to_ABC(&iL_dq0_abc);

        dutya = iL_dq0_abc.a/VDCBUS;
        dutyb = iL_dq0_abc.b/VDCBUS;
        dutyc = iL_dq0_abc.c/VDCBUS;

    }else{
        FT_Reset(&Int_w_rads);
        C_iLd.reset(&C_iLd);
        C_iLq.reset(&C_iLq);
        C_RPM.reset(&C_RPM);
    }

    if(dutya > 0.98) dutya = 0.98;
    else if(dutya < -0.98) dutya = -0.98;
    if(dutyb > 0.98) dutyb = 0.98;
    else if(dutyb < -0.98) dutyb = -0.98;
    if(dutyc > 0.98) dutyc = 0.98;
    else if(dutyc < -0.98) dutyc = -0.98;

    SVPWM(&dutya, &dutyb, &dutyc);

    dutya = (dutya+1.0)*0.5*PWM_PERIOD;
    dutyb = (dutyb+1.0)*0.5*PWM_PERIOD;
    dutyc = (dutyc+1.0)*0.5*PWM_PERIOD;

    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)dutya;
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)dutyb;
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)dutyc;

    Control.Run_Ctrl = 0;
    DLOG_Func(&dlog);
    return;
}

void SVPWM(float *da, float *db, float *dc){
    float dmin = 0;
    float dmax = 0;
    //Calc da seq zero para o SVPWM
    if(*da < *db && *da < *dc && *db>*dc){
        dmin = *da;
        dmax = *db;
    }else if((*da < *db) && (*da < *dc) && (*dc >*db)){
        dmin = *da;
        dmax = *dc;
    }else if((*db<*da) && (*db<*dc) && (*da>*dc)){
        dmin = *db;
        dmax = *da;
    }else if((*db<*da) && (*db<*dc) && (*dc>*da)){
        dmin = *db;
        dmax = *dc;
    }else if((*dc<*da) && (*dc<*db) && (*da>*db)){
        dmin = *dc;
        dmax = *da;
    }else if((*dc<*da) && (*dc<*db) && (*db>*da)){
        dmin = *dc;
        dmax = *db;
    }
    *da = -0.5*(dmin+dmax) + *da;
    *db = -0.5*(dmin+dmax) + *db;
    *dc = -0.5*(dmin+dmax) + *dc;
}

void PWM_Enable(st_Control *obj){
    static uint8_t pwm = 1;
    if(obj->PWM == pwm) return;
    pwm = obj->PWM;
    if(pwm == 0){
        EALLOW;
        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZFRC.bit.OST = 1;
        EDIS;

    }else{
        EALLOW;
        EPwm1Regs.TZCLR.bit.OST = 1;
        EPwm2Regs.TZCLR.bit.OST = 1;
        EPwm3Regs.TZCLR.bit.OST = 1;
        EDIS;
    }
}

void Config_Controllers(void){
    C_iLd.Kp = C_iLq.Kp = 0.1;
    C_iLd.Ki = C_iLq.Ki = 100.0/20400.0;
    C_iLd.OutMax = C_iLq.OutMax = 800.0;
    C_iLd.OutMin = C_iLq.OutMin = -800.0;

    C_RPM.Kp = 0.8;
    C_RPM.Ki = 1.6/20400.0;
    C_RPM.OutMax = 100.0;
    C_RPM.OutMin = -100.0;

    C_iLd.reset(&C_iLd);
    C_iLq.reset(&C_iLq);
    C_RPM.reset(&C_RPM);

    Int_w_rads.a1 = -1.0;
    Int_w_rads.b0 = 0.5/20400.0;
    Int_w_rads.b1 = Int_w_rads.b0;
}

// ======= Interruptions ===================
__interrupt void isr_cpu_timer0(void){

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void isr_adc(void){    // Run at 2x fpwm, fpwm = 10kHz
    static float avg[] = {0,0,0};    // auxiliary variable to reset offset
    static uint16_t cnt_isr = 0;

    if(Control.Run_Ctrl == 1) Control.Over_Run = 1;
    Control.Run_Ctrl = 1;

    if(Offset.Calc == Reset){
        Offset.Calc = Running;
        Offset.cnt = 0;
        Offset.ch0 = Offset.ch1 = Offset.ch2 = 0;
        avg[0] = avg[1] = avg[2] = 0.0;
    }

    Measures.iLa  = -0.007326*(float)((int)AdcaResultRegs.ADCRESULT0 - 2252) - Offset.ch0;
    Measures.iLb  = -0.007326*(float)((int)AdcaResultRegs.ADCRESULT1 - 2252) - Offset.ch1;
    Measures.iLc  = -0.007326*(float)((int)AdcaResultRegs.ADCRESULT2 - 2252) - Offset.ch2;

    if(Offset.cnt < N_SAMPLE_OFFSET){
        Offset.cnt++;
        avg[0] += Measures.iLa;
        avg[1] += Measures.iLb;
        avg[2] += Measures.iLc;
        if(Offset.cnt == N_SAMPLE_OFFSET){
            Offset.Calc = OK;
            Offset.ch0 = avg[0]/QNT_SAMPLE_OFFSET;
            Offset.ch1 = avg[1]/QNT_SAMPLE_OFFSET;
            Offset.ch2 = avg[2]/QNT_SAMPLE_OFFSET;
        }
    }

    cnt_isr++;
    if(cnt_isr == 204){
        Encoder_Calc(&Encoder); // Run at 2xfpwm/204 = 100 Hz
        cnt_isr = 0;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//========== Interruption CLA ============
