typedef struct circle {
	int radius;
	int xPos;
	int yPos;
} circle;

void updateCircle(int x, int y, int scaler);

void serialPrint(char* msg);

void serialPrintln(char* msg);

void serialPrintIMU(void);
