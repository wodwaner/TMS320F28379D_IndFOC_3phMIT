/*
 * Peripheral_Setup.c
 */
#include "Peripheral_Setup.h"

void Setup_GPIO(void){
    // pg 959 Table Mux, pg 965 Registers and  spruhm8i.pdf - Technical reference
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    EALLOW;
    // LED 31 A, 2
    // LED 34 B, 1
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;

    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;

    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;

    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;

    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1;

    //======== Encoder ================
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO10 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO11 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;   // Sync GPIO10 to SYSCLK  (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;   // Sync GPIO11 to SYSCLK  (EQEP1B)

    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0;    // Configure GPIO10 as EQEP1A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0;    // Configure GPIO11 as EQEP1B
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;    // Configure GPIO10 as EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;    // Configure GPIO11 as EQEP1B

    //PWM 3ph 1ab 2ab 3ab
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

    EDIS;

}

void Setup_ePWM(void){
    // pg 1978 spruhm8i.pdf - Technical reference
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;

    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;

    EPwm1Regs.TBPRD = PWM_PERIOD;                // Set timer period
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter

    EPwm1Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWMxA
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Hi complementary
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm1Regs.DBFED.bit.DBFED = 90;                 // FED = 20 TBCLKs
    EPwm1Regs.DBRED.bit.DBRED = 90;                 // RED = 20 TBCLKs

    Conf_Reg_ePWM(&EPwm1Regs, &EPwm2Regs);
    Conf_Reg_ePWM(&EPwm1Regs, &EPwm3Regs);

    //Trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   // Dispara ADC no topo
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;            // Trigger on every event
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Conf_Reg_ePWM(volatile struct EPWM_REGS *reg1, volatile struct EPWM_REGS *reg2){
    EALLOW;
    reg2->TZFRC.bit.OST = reg1->TZFRC.bit.OST;
    reg2->TZCTL.bit.TZA = reg1->TZCTL.bit.TZA;
    reg2->TZCTL.bit.TZB = reg1->TZCTL.bit.TZB;
    EDIS;

    reg2->TBPRD = reg1->TBPRD;
    reg2->CMPA.bit.CMPA = reg1->CMPA.bit.CMPA;
    reg2->TBCTR = reg1->TBCTR;
    reg2->TBPHS.bit.TBPHS = reg1->TBPHS.bit.TBPHS;
    reg2->TBCTL.bit.PHSEN = reg1->TBCTL.bit.PHSEN;
    reg2->TBCTL.bit.PHSDIR = reg1->TBCTL.bit.PHSDIR;
    reg2->TBCTL.bit.SYNCOSEL = reg1->TBCTL.bit.SYNCOSEL;
    reg2->TBCTL.bit.CTRMODE = reg1->TBCTL.bit.CTRMODE;
    reg2->TBCTL.bit.HSPCLKDIV = reg1->TBCTL.bit.HSPCLKDIV;
    reg2->TBCTL.bit.CLKDIV = reg1->TBCTL.bit.CLKDIV;
    reg2->CMPCTL.bit.SHDWAMODE = reg1->CMPCTL.bit.SHDWAMODE;
    reg2->CMPCTL.bit.LOADAMODE = reg1->CMPCTL.bit.LOADAMODE;
    reg2->CMPCTL.bit.SHDWBMODE = reg1->CMPCTL.bit.SHDWBMODE;
    reg2->CMPCTL.bit.LOADBMODE = reg1->CMPCTL.bit.LOADBMODE;

    reg2->AQCTLA.bit.PRD = reg1->AQCTLA.bit.PRD;
    reg2->AQCTLA.bit.ZRO = reg1->AQCTLA.bit.ZRO;
    reg2->AQCTLA.bit.CAU = reg1->AQCTLA.bit.CAU;
    reg2->AQCTLA.bit.CAD = reg1->AQCTLA.bit.CAD;

    reg2->AQCTLB.bit.PRD = reg1->AQCTLB.bit.PRD;
    reg2->AQCTLB.bit.ZRO = reg1->AQCTLB.bit.ZRO;
    reg2->AQCTLB.bit.CBU = reg1->AQCTLB.bit.CBU;
    reg2->AQCTLB.bit.CBD = reg1->AQCTLB.bit.CBD;
    reg2->AQCTLB.bit.CAU = reg1->AQCTLB.bit.CAU;
    reg2->AQCTLB.bit.CAD = reg1->AQCTLB.bit.CAD;

    reg2->DBCTL.bit.OUT_MODE = reg1->DBCTL.bit.OUT_MODE;
    reg2->DBCTL.bit.POLSEL = reg1->DBCTL.bit.POLSEL;
    reg2->DBFED.bit.DBFED = reg1->DBFED.bit.DBFED;
    reg2->DBRED.bit.DBRED = reg1->DBRED.bit.DBRED;
}

void Setup_ADC_A(void){
    // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;
    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC
    DELAY_US(1000);                             // delay for 1ms to allow ADC time to power up

    // determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION) acqps = 20; // 75ns
    else acqps = 63; // 320ns

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14;          //SOC0 will convert pin ADCIN14
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;      //sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;  //trigger on ePWM1 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;          //SOC1 will convert pin ADCINA3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = AdcaRegs.ADCSOC0CTL.bit.TRIGSEL;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0;          //SOC1 will convert pin ADCINA0
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = AdcaRegs.ADCSOC0CTL.bit.TRIGSEL;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;      // end of SOC2 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared
    EDIS;
}

void  Setup_eQep1(void){
    EALLOW;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;
    EDIS;
    EQep1Regs.QUPRD = 2000000;              // Unit Timer for 100Hz at 200 MHz
    // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 0x00; //00; // QEP quadrature dir mode pg 2199
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;     // pg 2201
    EQep1Regs.QEPCTL.bit.PCRM = 1;         // PCRM=00 mode - QPOSCNT reset on
    // index event
    EQep1Regs.QEPCTL.bit.UTE = 1;           // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;          // Latch on unit time out
    EQep1Regs.QPOSMAX = 0x0000FFFF;
    EQep1Regs.QEPCTL.bit.QPEN = 1;          // QEP enable
    EQep1Regs.QCAPCTL.bit.UPPS = 7;         // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS = 7;         // 1/64 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN = 1;          // QEP Capture Enable
}

