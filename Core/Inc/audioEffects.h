#include "main.h"
#include <math.h>
/************************
* DEFINES
************************/
#define BufSize 1000
#define Overlap 100

//Schroeder delays from 25k->96k interpolated
//*2 delay extension -> not more possible without external ram
#define l_CB0 3460
#define l_CB1 2988*2
#define l_CB2 3882*2
#define l_CB3 4312*2
#define l_AP0 480*2
#define l_AP1 161*2
#define l_AP2 46*2

/************************
* EXTERN VARIABLES
************************/

/************************
* STRUCTS
************************/

/************************
* FUNCTION PROTOTYPES
************************/
int Do_HighPass (int inSample);
int Do_PitchShift(float sum);
float Do_Comb0(float inSample);
float Do_Comb1(float inSample); 
float Do_Comb2(float inSample); 
float Do_Comb3(float inSample);
float Do_Allpass0(float inSample);
float Do_Allpass1(float inSample);
float Do_Allpass2(float inSample);
int Do_Reverb(float inSample);
float Do_Chorus(float inSample);