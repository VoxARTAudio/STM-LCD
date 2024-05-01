#include "main.h"
#include <math.h>
/************************
* DEFINES
************************/
#define OVERLAP 100
#define RING_BUFF_SIZE 1000

/************************
* EXTERN VARIABLES
************************/

/************************
* STRUCTS
************************/
typedef struct {
	float crossfade;
	int writePtr;
	float readPtr;
	float shift;
	int Buff[RING_BUFF_SIZE];
} effect_pitchShift;

/************************
* FUNCTION PROTOTYPES
************************/
int pitchShift(int lSample, int rSample);
