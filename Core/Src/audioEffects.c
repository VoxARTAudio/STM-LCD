#include "audioEffects.h"

effect_pitchShift ps = {1.0, 0, 0.0, 1.4, {0}};

int pitchShift(int lSample, int rSample) {
	int sum = lSample + rSample;
	
	ps.Buff[ps.writePtr] = sum;

	//read fractional readpointer and generate 0° and 180° read-pointer in integer
	int RdPtr_Int = roundf(ps.readPtr);
	int RdPtr_Int2 = 0;
	if (RdPtr_Int >= RING_BUFF_SIZE/2) RdPtr_Int2 = RdPtr_Int - (RING_BUFF_SIZE/2);
	else RdPtr_Int2 = RdPtr_Int + (RING_BUFF_SIZE/2);

	//read the two samples...
	float Rd0 = (float) ps.Buff[RdPtr_Int];
	float Rd1 = (float) ps.Buff[RdPtr_Int2];

	//Check if first readpointer starts OVERLAP with write pointer?
	// if yes -> do cross-fade to second read-pointer
	if (OVERLAP >= (ps.readPtr-RdPtr_Int) && (ps.readPtr-RdPtr_Int) >= 0 && ps.shift!=1.0f) {
		int rel = ps.readPtr-RdPtr_Int;
		ps.crossfade = ((float)rel)/(float)OVERLAP;
	}
	else if (ps.readPtr-RdPtr_Int == 0) ps.crossfade = 0.0f;

	//Check if second readpointer starts OVERLAP with write pointer?
	// if yes -> do cross-fade to first read-pointer
	if (OVERLAP >= (ps.readPtr-RdPtr_Int2) && (ps.readPtr-RdPtr_Int2) >= 0 && ps.shift!=1.0f) {
			int rel = ps.readPtr-RdPtr_Int2;
			ps.crossfade = 1.0f - ((float)rel)/(float)OVERLAP;
		}
	else if (ps.readPtr-RdPtr_Int2 == 0) ps.crossfade = 1.0f;


	//do cross-fading and sum up
	sum = (Rd0*ps.crossfade + Rd1*(1.0f-ps.crossfade));

	//increment fractional read-pointer and write-pointer
	ps.readPtr += ps.shift;
	ps.readPtr++;
	if (ps.readPtr == RING_BUFF_SIZE) ps.readPtr = 0;
	if (roundf(ps.readPtr) >= RING_BUFF_SIZE) ps.readPtr = 0.0f;

	return sum;
}


