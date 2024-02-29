#include <stdbool.h>

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
