#include <stdbool.h>

typedef struct imuMovement {
	int x;
	int y;
	int z;
	bool lock;
} imuMovement;

enum effectState {
	NONE,
	PITCH,
	REVERB,
	CHORUS
};

void parseReverbData(void);
void parseChorusData(void);
void setStates(void);
