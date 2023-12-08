#include "arm_math.h"
 
void Park(float alpha, float beta, float theta, float *d, float *q);
void InvPark(float d, float q, float theta, float *alpha, float *beta);
void Clarke(float a, float b, float *alpha, float *beta);
void InvClarke(float alpha, float beta, float *a, float *b, float *c);

void dq0(float sf, float cf, float a, float b, float c, float *d, float *q);
void abc(float sf, float cf, float d, float q, float *a, float *b, float *c);
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);