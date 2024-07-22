#ifndef FT_H_
#define FT_H_

typedef struct {        //filtro / ressonante
    float a1;           //a - denominador
    float a2;           //b - numerador
    float a3;
    float b0;
    float b1;
    float b2;
    float b3;
    float x0;           //entrada atual
    float x1;           //entrada anterior
    float x2;
    float x3;
    float y0;           //sa�da atual
    float y1;           //sa�da anterior
    float y2;
    float y3;
    //void  (*calc)();      // Pointer to calculation function
    //void  (*reset)();     // Pointer to reset function
} st_FT;

#define FT_DEFAULTS {0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0};

#pragma FUNC_ALWAYS_INLINE(FT_Calc)
inline void FT_Calc(st_FT *v){
    v->y0 = v->b0*v->x0 + v->b1*v->x1 + v->b2*v->x2 + v->b3*v->x3 - (v->a1*v->y1 + v->a2*v->y2 + v->a3*v->y3);

    v->x3 = v->x2;
    v->x2 = v->x1;
    v->x1 = v->x0;

    v->y3 = v->y2;
    v->y2 = v->y1;
    v->y1 = v->y0;
}
#pragma FUNC_ALWAYS_INLINE(FT_Reset)
inline void FT_Reset(st_FT *v){
    v->x0 = 0;
    v->x1 = 0;
    v->x2 = 0;
    v->x3 = 0;

    v->y0 = 0;
    v->y1 = 0;
    v->y2 = 0;
    v->y3 = 0;
}
#endif /* FT_H_ */
