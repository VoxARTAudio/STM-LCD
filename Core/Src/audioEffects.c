/*
* Audio Effects Library by Phil
* https://github.com/YetAnotherElectronicsChannel/
*/

#include "audioEffects.h"

/************************
* PITCH SHIFT
************************/
int Buf[BufSize];

int WtrP = 0;
float Rd_P = 0.0f;
float Shift = 1.0f;
float CrossFade = 3.0f;
float a0, a1, a2, b1, b2, hp_in_z1, hp_in_z2, hp_out_z1, hp_out_z2;

int Do_HighPass (int inSample) {
	//300Hz high-pass, 96k
	a0 = 0.9862117951198142f;
	a1 = -1.9724235902396283f;
	a2 = 0.9862117951198142f;
	b1 = -1.972233470205696f;
	b2 = 0.9726137102735608f;

	float inSampleF = (float)inSample;
	float outSampleF =
			a0 * inSampleF
			+ a1 * hp_in_z1
			+ a2 * hp_in_z2
			- b1 * hp_out_z1
			- b2 * hp_out_z2;
	hp_in_z2 = hp_in_z1;
	hp_in_z1 = inSampleF;
	hp_out_z2 = hp_out_z1;
	hp_out_z1 = outSampleF;

	return (int) outSampleF;
}

int Do_PitchShift(int lSample, int rSample) {
	int sum = lSample + rSample;
	//sum up and do high-pass
	//sum=Do_HighPass(sum);

	//write to ringbuffer
	Buf[WtrP] = sum;

	//read fractional readpointer and generate 0° and 180° read-pointer in integer
	int RdPtr_Int = roundf(Rd_P);
	int RdPtr_Int2 = 0;
	if (RdPtr_Int >= BufSize/2) RdPtr_Int2 = RdPtr_Int - (BufSize/2);
	else RdPtr_Int2 = RdPtr_Int + (BufSize/2);

	//read the two samples...
	float Rd0 = (float) Buf[RdPtr_Int];
	float Rd1 = (float) Buf[RdPtr_Int2];

	//Check if first readpointer starts overlap with write pointer?
	// if yes -> do cross-fade to second read-pointer
	if (Overlap >= (WtrP-RdPtr_Int) && (WtrP-RdPtr_Int) >= 0 && Shift!=1.0f) {
		int rel = WtrP-RdPtr_Int;
		CrossFade = ((float)rel)/(float)Overlap;
	}
	else if (WtrP-RdPtr_Int == 0) CrossFade = 0.0f;

	//Check if second readpointer starts overlap with write pointer?
	// if yes -> do cross-fade to first read-pointer
	if (Overlap >= (WtrP-RdPtr_Int2) && (WtrP-RdPtr_Int2) >= 0 && Shift!=1.0f) {
			int rel = WtrP-RdPtr_Int2;
			CrossFade = 1.0f - ((float)rel)/(float)Overlap;
		}
	else if (WtrP-RdPtr_Int2 == 0) CrossFade = 1.0f;


	//do cross-fading and sum up
	sum = (Rd0*CrossFade + Rd1*(1.0f-CrossFade));

	//increment fractional read-pointer and write-pointer
	Rd_P += Shift;
	WtrP++;
	if (WtrP == BufSize) WtrP = 0;
	if (roundf(Rd_P) >= BufSize) Rd_P = 0.0f;

	return sum;

}

/************************
* REVERB
************************/

//define wet 0.0 <-> 1.0
float wet = 1.3f;
//define time delay 0.0 <-> 1.0 (max)
float time = 1.0f;

//define pointer limits = delay time
int cf0_lim, cf1_lim, cf2_lim, cf3_lim, ap0_lim, ap1_lim, ap2_lim;

//define buffer for comb- and allpassfilters
float cfbuf0[l_CB0], cfbuf1[l_CB1], cfbuf2[l_CB2], cfbuf3[l_CB3];
float apbuf0[l_AP0], apbuf1[l_AP1], apbuf2[l_AP2];
//feedback defines as of Schroeder
float cf0_g = 0.805f, cf1_g=0.827f, cf2_g=0.783f, cf3_g=0.764f;
float ap0_g = 0.7f, ap1_g = 0.7f, ap2_g = 0.7f;
//buffer-pointer
int cf0_p=0, cf1_p=0, cf2_p=0, cf3_p=0, ap0_p=0, ap1_p=0, ap2_p=0;

float Do_Comb0(float inSample) {
	float readback = cfbuf0[cf0_p];
	float n = readback*cf0_g + inSample;
	cfbuf0[cf0_p] = n;
	cf0_p++;
	if (cf0_p==cf0_lim) cf0_p = 0;
	return readback;
}

float Do_Comb1(float inSample) {
	float readback = cfbuf1[cf1_p];
	float n = readback*cf1_g + inSample;
	cfbuf1[cf1_p] = n;
	cf1_p++;
	if (cf1_p==cf1_lim) cf1_p = 0;
	return readback;
}
float Do_Comb2(float inSample) {
	float readback = cfbuf2[cf2_p];
	float n = readback*cf2_g + inSample;
	cfbuf2[cf2_p] = n;
	cf2_p++;
	if (cf2_p==cf2_lim) cf2_p = 0;
	return readback;
}
float Do_Comb3(float inSample) {
	float readback = cfbuf3[cf3_p];
	float new = readback*cf3_g + inSample;
	cfbuf3[cf3_p] = new;
	cf3_p++;
	if (cf3_p==cf3_lim) cf3_p = 0;
	return readback;
}

float Do_Allpass0(float inSample) {
	float readback = apbuf0[ap0_p];
	readback += (-ap0_g) * inSample;
	float n = readback*ap0_g + inSample;
	apbuf0[ap0_p] = n;
	ap0_p++;
	if (ap0_p == ap0_lim) ap0_p=0;
	return readback;
}

float Do_Allpass1(float inSample) {
	float readback = apbuf1[ap1_p];
	readback += (-ap1_g) * inSample;
	float n = readback*ap1_g + inSample;
	apbuf1[ap1_p] = n;
	ap1_p++;
	if (ap1_p == ap1_lim) ap1_p=0;
	return readback;
}
float Do_Allpass2(float inSample) {
	float readback = apbuf2[ap2_p];
	readback += (-ap2_g) * inSample;
	float n = readback*ap2_g + inSample;
	apbuf2[ap2_p] = n;
	ap2_p++;
	if (ap2_p == ap2_lim) ap2_p=0;
	return readback;
}

float Do_Reverb(float inSample) {
	float newsample = (Do_Comb0(inSample) + Do_Comb1(inSample) + Do_Comb2(inSample) + Do_Comb3(inSample))/4.0f;
	newsample = Do_Allpass0(newsample);
	newsample = Do_Allpass1(newsample);
	newsample = Do_Allpass2(newsample);
	return newsample;
}
