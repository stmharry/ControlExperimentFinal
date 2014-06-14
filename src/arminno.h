class MotorRunnerC {
	public:
		MotorRunnerC(int) {}
		void Forward(short) {}
		void Backward(short) {}
		void Stop() {}
};

class SonarA {
	public:
		SonarA(int) {}
		void SetFloorLevel(int) {}
		void Ranging() {}
		short GetDistance(int, short) {return 0;}
};

class CompassB {
	public:
		CompassB(int) {}
		void GetAngle(short) {}
};

class LCD2X16A {
	public:
		LCD2X16A(int) {}
		void BacklightOn(int) {}
		void SetBacklight(int) {}
		void Clear() {}
		void Display(int) {}
		void Display(char*) {}
};

typedef enum {
	PD8,
	PD9,
	PA0,
	PA1,
	PA2,
	PA3,
	PA4
} Button;

void Pause(int) {}
void SetUart0(int, int) {}
void SendUart0Data(char*, int) {}
void InitialGpioState(Button, int, int) {}
void SetTm2Decoder(int, int, int) {}
void SetTm3Decoder(int, int, int) {}
void SetTm4Counter(int, int, int, int) {}
void SetAdc(Button) {}
short GetAdc(int) {return 0;}
void GetTm4CounterValue(unsigned short) {}
void GetTm2DecoderValue(short) {}
void GetTm3DecoderValue(short) {}
short GetUart0TxState() {return 0;}

