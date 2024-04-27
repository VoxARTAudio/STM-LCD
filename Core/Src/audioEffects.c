#include "audioEffects.h"

int Calc_IIR_Left (int inSample) {
	float inSampleF = (float)inSample;
	float outSampleF =
			l_a0 * inSampleF
			+ l_a1 * lin_z1
			+ l_a2 * lin_z2
			- l_b1 * lout_z1
			- l_b2 * lout_z2;
	lin_z2 = lin_z1;
	lin_z1 = inSampleF;
	lout_z2 = lout_z1;
	lout_z1 = outSampleF;

	return (int) outSampleF;
}

int Calc_IIR_Right (int inSample) {
	float inSampleF = (float)inSample;
	float outSampleF =
			r_a0 * inSampleF
			+ r_a1 * rin_z1
			+ r_a2 * rin_z2
			- r_b1 * rout_z1
			- r_b2 * rout_z2;
	rin_z2 = rin_z1;
	rin_z1 = inSampleF;
	rout_z2 = rout_z1;
	rout_z1 = outSampleF;

	return (int) outSampleF;
}