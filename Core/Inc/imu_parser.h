#include <stdbool.h>

#define SHIFT_MAX 1.9f
#define SHIFT_MIN 0.5f


typedef enum {
	NO,
	POS,
	NEG
} lock;

enum effectState {
	NONE,
	PITCH,
	REVERB,
	CHORUS
};

typedef struct imuMovement {
	int x;
	int y;
	int z;
	lock l;
} imuMovement;

void parseReverbData(void);
void parseChorusData(void);
void setStates(void);
void pitchAdjuster(float angle);
