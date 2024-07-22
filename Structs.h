/*
 * Structs.h
 *
 *  Created on: 17 de jul de 2023
 *      Author: wodwa
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#define N_SAMPLE_OFFSET 65535
#define QNT_SAMPLE_OFFSET 65535.0

// ===========================

typedef struct{
    float iLa;
    float iLb;
    float iLc;
    float Vdc;
} st_Measures;

typedef struct{
    uint8_t PWM:1;
    uint8_t DIR:1;
    uint8_t Run_Ctrl:1;
    uint8_t Over_Run:1;
    uint8_t Run_Protections:1;
    uint8_t Over_Current:1;
    uint8_t Over_Voltage:1;
    uint8_t RESERVED:1;
} st_Control;
//==========================

typedef enum {OK = 0, Reset = 1, Running} en_Offset;
typedef struct{
    float ch0;
    float ch1;
    float ch2;
    uint16_t cnt;
    en_Offset Calc;
} st_Offset;
#endif /* STRUCTS_H_ */
