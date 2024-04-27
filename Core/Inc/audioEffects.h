#include "main.h"

/************************
* EXTERN VARIABLES
************************/
extern float l_a0, l_a1, l_a2, l_b1, l_b2, lin_z1, lin_z2, lout_z1, lout_z2;
extern float r_a0, r_a1, r_a2, r_b1, r_b2, rin_z1, rin_z2, rout_z1, rout_z2;

/************************
* FUNCTION PROTOTYPES
************************/

int Calc_IIR_Left (int inSample);

int Calc_IIR_Right (int inSample);