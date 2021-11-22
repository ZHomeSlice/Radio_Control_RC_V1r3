#ifndef RV_V1_h
#define RC_V1_h

//#if ARDUINO >= 100
#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
#define LIBRARY_VERSION	1.1.1
#define runEvery(t) for (static unsigned long _ETimer; millis() >= _ETimer; _ETimer += (t))
//runEvery(t) Advantages and Disadvantages
// Advantages
// the delay is always calculated to be a known value
// great for keeping track of seconds minutes hours etc
// Disadvantages
// any delay will cause it to wind up and repeat itself rapidly

#define runAfter(t) for (static unsigned long _ATimer; (unsigned long)(millis() - _ATimer) >= (t); _ATimer = millis())
// runAfter(t) Advantages and Disadvantages
// Advantages
// great for Serial.Print or LCD Print type functions where time is not critical but delay between events is
// Immune to wind up
// Disadvantages
// the delay is calculated with millis() value causing

#define runAt(t)    for (static unsigned long _EATimer; millis() >= _EATimer; _EATimer += (t) * (1 +((int)  (millis()- _EATimer) / (t))))
// runAt(t) Advantages and Disadvantages
// Advantages
// great for Serial.Print or LCD Print type functions
// will not wind up
// Disadvantages
// can skip counts and always calculates as close to possible on the delay
// takes more memory


#include <avr/pgmspace.h>
class RC {
	public:
	//Commonly used functions **************************************************************************
	//Available but not commonly used functions ********************************************************
	//Display functions ********************************************************************************
	RC();
	void pciSetupRC(uint8_t pin);
	void pciSetupEncoder(uint8_t ClockPin,uint8_t DataPin, uint8_t Mode = 0, uint8_t InputType = INPUT);
	void pciSetupPing(uint8_t PingPin,uint8_t TriggerPin);
	void pciSetupSwitch(uint8_t SwitchPin, uint8_t InputType = INPUT_PULLUP);
	void Print(uint16_t SpamDelay = 100);
	void Timers(uint8_t pin, uint8_t group, uint8_t StartPin, uint8_t EndPin);
	uint8_t Alive();
	int16_t PinTime(uint8_t pin, uint16_t offset = 1000,bool MinMaxADJ = false);
	int16_t PinTimeX(uint8_t pin);
	int16_t PosTime(uint8_t PinCount, uint16_t offset = 1000,bool MinMaxADJ = false);
	int16_t PosTimeX(uint8_t PinCount);

	int16_t PinEncode(uint8_t pin,uint8_t Reset = 0);
	int16_t PosEncode(uint8_t pin,uint8_t Reset = 0);

	int16_t PinPing(uint8_t pin);
	int16_t PosPing(uint8_t pin);
	int16_t PinSwitch(uint8_t pin);
	int16_t PosSwitch(uint8_t pin);
	void YawOffsetGenerator(float *YawOffset, float *DesiredDegrees, float YawDegrees,  float SetpointShift,int16_t TurnLimit = 90);
	void YawGyroPiD(int16_t *Turn, float YawInput,int16_t MaxOut, float kp, float kd, int8_t controlDirection, uint8_t TurnControlEnable);
	void XYInput(uint8_t Xpin, uint8_t Ypin,uint8_t Throttlepin, int16_t* X, int16_t* Y);
	
	void XYtoTank(int16_t X, int16_t Y, int16_t* LeftDrive, int16_t* RightDrive,int16_t Throttle);
	void XYtoTankPWM(uint8_t Xpin, uint8_t Ypin, uint8_t Throttlepin, uint8_t LeftPin, uint8_t Forward_LeftPin , uint8_t  Reverse_LeftPin, uint8_t RightPin, uint8_t  Forward_RightPin, uint8_t Reverse_RightPin );
	void TankPWM(int16_t LeftDrive,int16_t RightDrive, int16_t Breaking, uint8_t LeftPin, uint8_t Forward_LeftPin , uint8_t  Reverse_LeftPin, uint8_t RightPin, uint8_t  Forward_RightPin, uint8_t Reverse_RightPin );
	int16_t TankDeadzone(int16_t V,uint16_t deadzone = 0);
	
	int16_t ZeroDeadzone(int16_t V, uint16_t deadzone);
	void Deadzone(uint16_t DZone = 0);
	void MaxThrottle(uint16_t TMax = 1000);
	void Ping();

	uint8_t ValuePinCtr = 0;
	uint8_t SwitchPinCtr = 0;

	RC & RC::onRCIsAlive(void (*CallBack)(void));
    RC & RC::onRCIsNotAlive(void (*CallBack)(void));
	private:
	static void nothing(void){}                 // Z added Create an empty function
	typedef void (*voidFuncPtr)(void);                 // Z added Create a type to point to a funciton.
	volatile voidFuncPtr onRCIsAliveFuncPtr = nothing;    // Z Callback for onRepeat
	volatile voidFuncPtr onRCIsNotAliveFuncPtr = nothing;    // Z Callback for onRepeat
	uint32_t PinLock = 0;
	uint16_t throttleMax = 1000;
	uint16_t deadZone = 0;
	uint8_t TankControls = 0;
	uint16_t TankLeftDrive;
	uint16_t TankRightDrive;
	uint16_t offset = 1000; // Pulse Minimum   1000ms  and max is assumed to be 2000ms or CenterPos * 2
	bool FailSafe = 1; // Sets all inputs to 1500 when RC Signal fails
	uint8_t  PinNumber[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // List of pins that could be used as ping inputs:
	void pciSetup(uint8_t pin, uint8_t InputType = INPUT);
	//void SetMask(uint8_t Pin, uint8_t *  xmask);

};
#endif
