// Fixed issues with RC portion of code 9/13/16
// Added Tank Controls and Arc Controller Code;
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "RC_V1r3.h"
// All
volatile uint8_t  GroupStartPin[3]  = {0,8,14};
volatile uint8_t  GroupEndPin[3]    = {7,13,19};
union Mask{
	volatile uint32_t All;
	volatile uint8_t  Port[4];
};

Mask Now;
volatile uint32_t Changed; //volatile uint8_t  mask[3];						// Place holder for general Mask Calculations
volatile uint32_t M;    //Localizes altered Mask
volatile uint32_t PCintLast; //volatile uint8_t  PCintLast[3];					// looking fo pin changes using bianary
volatile uint8_t  PinArray[20];                 // Key = Pin Val = Position List of pins that could be used as ping inputs:
volatile uint8_t  PinNumber[22] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, -1, -1, 14, 15, 16, 17, 18, 19};                 // Key = Bit Val = Position List of pins that could be used as ping inputs:
volatile uint8_t  BitNumber[20] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21};                 // Key = Pin Val = Position List of pins that could be used as ping inputs:
volatile uint16_t RCedgeTime[20];      // Pulse HIGH start time for RCInputs & PingInputs

//RC Only
#define RCInputs 6
volatile uint32_t rcPinMask; //volatile uint8_t  rcPinMask[3]; // RC Pin Mask for active RC Pins
volatile uint16_t rcValue[RCInputs];            // interval [1000;2000]
volatile uint16_t rcValueX[RCInputs];           // interval [0;65535]
volatile uint16_t rcValueMin[RCInputs];         //
volatile uint16_t rcValueMax[RCInputs];         //
volatile uint16_t rcValueCenter[RCInputs];      //
volatile uint32_t AliveBits = 0;                // a reading Since Last Test,
volatile int8_t   RCIsAlive = 0;
volatile bool     FailSafe = true;
volatile uint8_t  RCPinCtr = 0;                 // Number of Pins Activated
volatile uint16_t RCEdgeTime[RCInputs];      // Pulse HIGH start time for RCInputs & PingInputs
volatile uint8_t  RCPinLink[RCInputs];

// Encoder Only
#define EncoderInputs 0
volatile uint8_t EncodeModeVal = 0; // 0 Single Step on Clock Pin Rise only
// 1 2 Steps on Clock Pin Rise and Fall
// 2 4 Setps on Clock and Data Pins Rise and Fall
volatile uint8_t EncoderPinCtr = 0;
volatile uint8_t EncoderPinList[10];
volatile uint32_t EncodePinMask; //volatile uint8_t  EncodePinMask[3];				// Encode Pin Mask for active Encode Pins
volatile uint32_t DataPin; //volatile uint8_t  EncodePinMask[3];				// Encode Pin Mask for active Encode Pins
volatile int16_t EncodeValue[EncoderInputs];   // [0;65535]
volatile uint8_t  DataPinLink[EncoderInputs];

// Ping Only
#define PingInputs 0
volatile uint32_t PingPinMask; //volatile uint8_t  PingPinMask[3] ;				// Ping Pin Mask for active Ping Pins
volatile uint16_t PingTime[PingInputs];         // rolling average of the last 3 pings
volatile uint16_t PingTimeX[4][PingInputs];     //
volatile uint8_t  PingCount[PingInputs];        // interval [0;65535]
volatile uint8_t  ToCompleteCtr;
volatile uint16_t PingEdgeTime[PingInputs];      // Pulse HIGH start time for RCInputs & PingInputs
volatile uint8_t PingPinCtr = 0;
volatile uint8_t PingPinList[PingInputs];
// Switch Only
#define SwitchInputs 1
volatile uint32_t SwitchPinMask; //volatile uint8_t SwitchPinMask[3];				// Switch Pin Mask for active Switch Pins
volatile uint32_t DebounceTime[SwitchInputs];
volatile uint8_t SwitchValue[SwitchInputs];
volatile uint8_t SwitchPinList[SwitchInputs];
volatile uint8_t SwitchPinCtr = 0;
volatile uint16_t SwitchEdgeTime[PingInputs];


/************ static functions common to all instances ***********************/

static inline void RCTimers(uint16_t cTime) {
	//cTime Note micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
	M = Changed & rcPinMask;// only keep RC Data
	if(!M)return;                    //No RC Data move on
	for (uint8_t Position = 0; (Position < RCPinCtr); Position++) {	// Cycle throug all the pins
		uint8_t Bit = BitNumber[RCPinLink[Position]];

		if (M >> Bit & 1) {        // See if the pin has changed

			if ((Now.All >> Bit & 1)) RCEdgeTime[Position] = cTime; //Pulse went HIGH store the start time
			else {                         // Pulse Went low calculate the duratoin
				uint16_t dTime = cTime - RCEdgeTime[Position]; // Calculate the change in time
				if (900 < dTime && dTime < 2100){    // Time is within proper RC range
					rcValue[Position] = dTime;		 // only save proper PWM Pulse
					rcValueMin[Position] = min(rcValueMin[Position], dTime); // only save proper PWM Pulse
					rcValueMax[Position] = max(rcValueMax[Position], dTime); // only save proper PWM Pulse

					rcValueMin[Position] = min(rcValueMin[Position], rcValueCenter[Position] - (rcValueMax[Position] - rcValueCenter[Position]) ); // Keep Center Centered
					rcValueMax[Position] = max(rcValueMax[Position], rcValueCenter[Position] + (rcValueCenter[Position] - rcValueMin[Position])); // Keep Center Centered
				}
				rcValueX[Position] = dTime; // Lets Store any duration up to 65535 micro seconds
				AliveBits = AliveBits | 1 << Position;
			}
		}
	}
}


static inline void Encode() {
	M = Changed & EncodePinMask;				// only keep Encoder Data
	if (!M)return;                              // no encoder data move on
	for (uint8_t Position = 0; (Position < EncoderPinCtr); Position++) {	// Cycle throug all the pins
		uint8_t Bit = BitNumber[EncoderPinList[Position]];
		if (M >> Bit & 1) {  // See if the pin has changed
			uint8_t DataBit = BitNumber[DataPinLink[Position]];
			if ((Now.All >> Bit & 1)) {             //CLK Pulse went HIGH
				if (Now.All >> DataBit & 1) EncodeValue[Position]--;         // Check if Data Pin is High at the moment of change
				else EncodeValue[Position]++;
				} else if (EncodeModeVal >= 1) { // half step code higher resolution
				if (Now.All >> DataBit & 1)  EncodeValue[Position]++;        // Check if Data Pin is High at the moment of change
				else  EncodeValue[Position]--;
			}
		}
	}
}
static inline void PingTimers(uint16_t cTime ){
	M = Changed & PingPinMask;					// only keep Encoder Data
	if(!M)return;											// no encoder data move on
	ToCompleteCtr++;
	sei();
	for (uint8_t Position = 0; (Position < PingPinCtr); Position++) {	// Cycle throug all the pins
		uint8_t Bit = BitNumber[PingPinList[Position]];
		for (uint8_t Bit = 0; (Bit < 22); Bit++) { // Cycle throug all the pins
			if (M >> Bit & 1) {									// pin has changed
				if ((Now.All >> Bit & 1))PingEdgeTime[Position] = cTime;			//Pulse went HIGH store the start time
				else {			// Pulse Went low calculate the duratoin
					uint16_t dTime = cTime - PingEdgeTime[Position];				// Calculate the change in time
					PingTimeX[PingCount[Position]][Position] = dTime; // Lets Store any duration up to 65535 micro seconds
					PingTime[Position] = (uint16_t)((PingTimeX[0][Position]+PingTimeX[1][Position]+PingTimeX[2][Position]+PingTimeX[3][Position])*.25);
					PingCount[Position]++;
					if(PingCount[Position] >=4) PingCount[Position] = 0;
				}
			}
		}
		ToCompleteCtr--;  //when all interupts are complete this will return to zero
	}
}

static inline void Switch (uint16_t cTime){
	M = Changed & SwitchPinMask;       // only keep Switch Data
	if(!M)return; // no swith pins defigned
	for (uint8_t Position = 0; (Position < SwitchPinCtr); Position++) {	// Cycle throug all the pins
		uint8_t Bit = BitNumber[SwitchPinList[Position]];
		if (M >> Bit & 1) { // SW pin Changed
			if ((cTime - SwitchEdgeTime[Position]) >= 10 ) {
				SwitchEdgeTime[Position] = cTime;
				if ((Now.All >> Bit & 1)) { //SW Pulse went HIGH
					//          Serial.println(F("SW Pulse went HIGH"));
					SwitchValue[Position]++;
				}
				//else { // SW Pulse went low
				//         Serial.println(F("SW Pulse went LOW"));
				//}
			}
			return;
		}
	}
}

// port change Interrupt
ISR(PCINT2_vect) { //this ISR pins 0-7
	Now.Port[0] = PIND; // 0-7
	Now.Port[1] = PINB; // 8-13
	Now.Port[2] = PINC; // A0-A6
	Changed = Now.All ^ PCintLast;// doing a ^ between the current interruption and the last one indicates wich pin changed
	uint16_t cTime = micros();    // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
	PingTimers(cTime);   // Calculate Ping Data
	sei();                        // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
	RCTimers(cTime);        // Calculate RC Data
	Encode();    // Calculate Encoder Data
	Switch (cTime);
	PCintLast = Now.All;          // we memorize the current state of all PINs in group

}

ISR(PCINT0_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a pins 8-13 only need 9-11
	Now.Port[0] = PIND; // 0-7
	Now.Port[1] = PINB; // 8-13
	Now.Port[2] = PINC; // A0-A6
	Changed = Now.All ^ PCintLast;// doing a ^ between the current interruption and the last one indicates wich pin changed
	uint16_t cTime = micros();    // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
	PingTimers(cTime);   // Calculate Ping Data
	sei();                        // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
	RCTimers(cTime);	      // Calculate RC Data
	Encode();    // Calculate Encoder Data
	Switch (cTime);
	PCintLast = Now.All;          // we memorize the current state of all PINs in group
}

ISR(PCINT1_vect) { //this ISR s A0~A5
	Now.Port[0] = PIND; // 0-7
	Now.Port[1] = PINB; // 8-13
	Now.Port[2] = PINC; // A0-A6
	Changed = Now.All ^ PCintLast;// doing a ^ between the current interruption and the last one indicates wich pin changed
	uint16_t cTime = micros();    // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
	PingTimers(cTime);   // Calculate Ping Data
	sei();                        // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
	RCTimers(cTime);        // Calculate RC Data
	Encode();    // Calculate Encoder Data
	Switch (cTime);
	PCintLast = Now.All;          // we memorize the current state of all PINs in group
}

SIGNAL(TIMER0_COMPA_vect){ // Timer to check for signal loss
	sei();
	static int MSec=0;
	if(MSec++ < 20)return; // Check every 10 Miliseconds for change
	MSec = 0;
	RCIsAlive = (AliveBits > 0) ?   RCIsAlive+1 : RCIsAlive-1  ;
	RCIsAlive = constrain(RCIsAlive,-20,+20);// Must have 20 tests good or 20 tests bad for RCIsAlive to change states
	if ((RCIsAlive <= 0) && FailSafe){
		for (uint8_t c = 0;c < RCPinCtr ;c++) rcValue[c] = rcValueCenter[c];
	}
	AliveBits = 0;
}

/****************** end of static functions ******************************/
RC::RC(){
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
}

// Install Pin change interrupt for a pin, can be called multiple times
void RC::pciSetupRC(uint8_t pin){
	if(bitRead(PinLock,BitNumber[pin])) return;
	bitWrite(PinLock, BitNumber[pin], 1);
	bitWrite(rcPinMask, BitNumber[pin], 1);
	RC::pciSetup( pin,  INPUT);
	PinArray[pin] = RCPinCtr;
	RCPinLink[RCPinCtr] = pin;
	rcValue[RCPinCtr] = 1500;
	rcValueMin[RCPinCtr] = 1200;
	rcValueMax[RCPinCtr] = 1800;
	rcValueCenter[RCPinCtr] = 1500;
	RCPinCtr++;
}
void RC::pciSetupEncoder(uint8_t ClockPin,uint8_t DataPin, uint8_t Mode, uint8_t InputType) {
	EncodeModeVal = Mode;
	if(bitRead(PinLock,ClockPin)) return;
	if(bitRead(PinLock,DataPin)) return;
	bitWrite(PinLock, ClockPin, 1);
	bitWrite(PinLock, DataPin, 1);
	bitWrite(EncodePinMask, BitNumber[ClockPin], 1);
	RC::pciSetup( ClockPin);
	PinArray[ClockPin] = EncoderPinCtr;
	DataPinLink[EncoderPinCtr] = DataPin;
	EncoderPinList[EncoderPinCtr] = ClockPin;
	EncoderPinCtr++;
	if(EncodeModeVal == 2){
		bitWrite(EncodePinMask, BitNumber[DataPin], 1);
		RC::pciSetup( DataPin);
		PinArray[DataPin] = EncoderPinCtr;
		DataPinLink[EncoderPinCtr] = ClockPin;
		EncoderPinList[EncoderPinCtr] = DataPin;
		EncoderPinCtr++;
	}
}
void RC::pciSetupPing(uint8_t PingPin,uint8_t TriggerPin){
	if(bitRead(PinLock,PingPin)) return;
	if(bitRead(PinLock,TriggerPin)) return;
	bitWrite(PinLock, PingPin, 1);
	bitWrite(PinLock, TriggerPin, 1);
	bitWrite(PingPinMask, BitNumber[PingPin], 1);
	RC::pciSetup( PingPin);
	PinArray[PingPin] = PingPinCtr;
	PingPinList[PingPinCtr] = PingPin;
	PingPinCtr++;
}
void RC::pciSetupSwitch(uint8_t SwitchPin, uint8_t InputType) {
	if(bitRead(PinLock,SwitchPin)) return;
	bitWrite(PinLock, SwitchPin, 1);
	bitWrite(SwitchPinMask, BitNumber[SwitchPin], 1);
	RC::pciSetup( SwitchPin,InputType);
	PinArray[SwitchPin] = SwitchPinCtr;
	SwitchPinList[SwitchPinCtr] = SwitchPin;
	SwitchPinCtr++;
}
void RC::pciSetup(uint8_t pin, uint8_t InputType){
	pinMode(pin, InputType);// enable interrupt for pin...
	*digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}






void RC::Print(uint16_t SpamDelay) {
	runAfter(SpamDelay){ // print after SpamDelay miliseconds (No Windup)
		if (RCPinCtr){
			Serial.print(RCIsAlive);
			Serial.print(" ");
			if(!Alive()) {
				Serial.print("No Signal ");


				} else {
				for (int Ctr = 0; Ctr < RCPinCtr; Ctr++) {
					Serial.print(Ctr);
					Serial.print(F("-"));
					Serial.print(RCPinLink[Ctr]);
					Serial.print(F(": "));
					Serial.print(PosTime(Ctr,0));
					Serial.print(F("\t"));
				}
				if(RC::TankControls){
					Serial.print(F("LeftDrive: "));
					Serial.print(TankLeftDrive);
					Serial.print(F("RightDrive: "));
					Serial.print(TankRightDrive);
					Serial.print(F("\t"));
				}
			}

		}
		if (EncoderPinCtr){
			for (int ECtr = 0; ECtr < EncoderPinCtr; ECtr++) {
				Serial.print(ECtr);
				Serial.print(F(": "));
				Serial.print(RC::PosEncode(ECtr));
				Serial.print(F("\t"));
				if(EncodeModeVal == 2)ECtr++; // Skip the data pin
			}
		}
		if (PingPinCtr){
			for (int PCtr = 0; PCtr < PingPinCtr; PCtr++) {
				Serial.print(PCtr);
				Serial.print(F(": "));
				Serial.print(PingTime[PCtr]);
				Serial.print(F("\t"));
			}
		}
		if (SwitchPinCtr){
			for (int SCtr = 0; SCtr < SwitchPinCtr; SCtr++) {
				Serial.print(SCtr);
				Serial.print(F(": "));
				Serial.print(PingTime[SCtr]);
				Serial.print(F("\t"));
			}
		}
		Serial.println();
	}
}

RC & RC::onRCIsAlive(void (*CallBack)(void)){
	onRCIsAliveFuncPtr = CallBack;
	return * this;
}
RC & RC::onRCIsNotAlive(void (*CallBack)(void)){
	onRCIsNotAliveFuncPtr = CallBack;
	return * this;
}
uint8_t RC::Alive() {
	if(RCIsAlive > 0){
		if (onRCIsAliveFuncPtr) {
			onRCIsAliveFuncPtr(); // call the function we assigned to the once empty function pointer
		}
		} else {
		if (onRCIsNotAliveFuncPtr) {
			onRCIsNotAliveFuncPtr(); // call the function we assigned to the once empty function pointer
		}
	}
	return(RCIsAlive > 0);
}
int16_t RC::PinValue(uint8_t pin){
	return(PinTime(pin, 1000, true));
}
int16_t RC::PinTime(uint8_t pin, uint16_t offset,bool MinMaxADJ) {
	return(PosTime(PinArray[pin], offset , MinMaxADJ));
}
int16_t RC::PinTimeX(uint8_t pin) {
	return(PosTimeX(PinArray[pin]));
}
int16_t RC::PosTime(uint8_t PinCount, uint16_t offset ,bool MinMaxADJ) {
	if(!MinMaxADJ) return(rcValue[PinCount] - offset);


	//int16_t Pos;
	//Pos = map((rcValue[PinCount] - offset),-400, 400, -500, 500);
	//Serial.print("rcValue,rcValueMin,rcValueMax "); Serial.print(rcValue[PinCount]);Serial.print(",");Serial.print(rcValueMin[PinCount]);Serial.print(",");Serial.print(rcValueMax[PinCount]);Serial.print("\t");

	//todo Adjust rcValue to 1000ms ~ 2000ms based on rcValueMin and rcValueMax
	// adjust for slight overage +- and constrain to fit before Mapping Each
	
	int16_t Pos;
	Pos = constrain(rcValue[PinCount] - offset,0,1000);
	return(Pos);
}
int16_t RC::PosTimeX(uint8_t PinCount) {
	return(rcValueX[PinCount]);
}

void RC::Deadzone(uint16_t DZone){
	deadZone = DZone;
}

void RC::MaxThrottle(uint16_t TMax){
	throttleMax = TMax;
}

void RC::XYtoTankPWM(uint8_t Xpin, uint8_t Ypin, uint8_t Throttlepin, uint8_t LeftPin, uint8_t Forward_LeftPin , uint8_t  Reverse_LeftPin, uint8_t RightPin, uint8_t  Forward_RightPin, uint8_t Reverse_RightPin ){
	int16_t LeftDrive;
	int16_t RightDrive;
	int16_t X;
	int16_t Y;
	int16_t Throttle = 255;
	int16_t Breaking = 0;
	XYInput( Xpin,  Ypin, Throttlepin , &X, &Y);
	XYtoTank( X,  Y, &LeftDrive, &RightDrive,Throttle);
	TankPWM(LeftDrive, RightDrive, Breaking, LeftPin,  Forward_LeftPin ,   Reverse_LeftPin,  RightPin,   Forward_RightPin,  Reverse_RightPin );
}



void RC::XYInput(uint8_t Xpin, uint8_t Ypin,uint8_t Throttlepin, int16_t* X, int16_t* Y ){
	static int16_t XMinMax = 300;
	static int16_t YMinMax = 300;
	static int16_t throttleMinMax = 300;
	int16_t XVal,YVal,Throttle;
	if(!RC::Alive()) return;
	RC::TankControls = 1;

	Throttle = (int16_t)(Throttlepin == -1) ? 500 : constrain(PinTime(Throttlepin, offset, true)*.5,0,500);// full throttle or 0 ~ 500
	Throttle = map(Throttle,0,500,0,(int16_t)(throttleMax * .5));
	throttleMinMax = max(abs(Throttle),throttleMinMax);
	Throttle = constrain(map (Throttle, 0, XMinMax, 0, 500),0,500);

	XVal = PinTime(Xpin, (rcValueCenter[PinArray[Xpin]]),true); //Retrieves value and Shifts to center around Zero.
	YVal = PinTime(Ypin, (rcValueCenter[PinArray[Ypin]]),true); //Retrieves value and Shifts to center around Zero.
	XMinMax = max(abs(XVal),XMinMax);
	YMinMax = max(abs(YVal),YMinMax);
	*X = constrain(map (XVal, -XMinMax, XMinMax, -500, 500),-Throttle,Throttle);
	*Y = constrain(map (YVal, -YMinMax, YMinMax, -500, 500),-Throttle,Throttle);

}



// PID adjust YawDegrees to Match Setpoint adjusted by Y
// Setpoint = 0 always
// We offset the input instead.
// y represents Yaw Offset
// YawOffset = actual YawMPU + (Shift over time) (limited to turnLimit);
// YawOffset is the desired heading it is unknown
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
#define printintx(Name,Variable,EndTxt) print(Name); Serial.print(F(" ")); Serial.print(Variable);Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
/*
void xYawOffsetGenerator(float *NewDeg, float *DesiredDegrees, float YawDegrees,  float SetpointShift,int16_t TurnLimit = 90){
	if (abs((*DesiredDegrees - SetpointShift) - YawDegrees )<170) *DesiredDegrees = *DesiredDegrees - SetpointShift;
	//if(abs(SetpointShift) < 1) *DesiredDegrees = YawDegrees;
	if ( *DesiredDegrees < -180 ) *DesiredDegrees += 360;
	if ( *DesiredDegrees > 180 ) *DesiredDegrees -= 360;
	float YawErrorDeg = (*DesiredDegrees - YawDegrees );
	float absYawErrorDeg = abs(YawErrorDeg);
	float Limited_absYawError;
	float Limited_YawError;
	float Output;
	Limited_absYawError = (absYawErrorDeg > TurnLimit) ?TurnLimit: absYawErrorDeg;
	Limited_YawError = (YawErrorDeg >= 0) ? -Limited_absYawError :  Limited_absYawError; // Restores a FullRange Value +-
	//Output = YawDegrees + Limited_YawError;
	Output = -(YawDegrees - (YawDegrees + Limited_YawError));
	*NewDeg = Output;
}
*/
void RC::YawOffsetGenerator(float *NewDeg, float *DesiredDegrees, float YawDegrees,  float SetpointShift,int16_t TurnLimit = 90){
	*NewDeg = YawDegrees - *DesiredDegrees;
	if ( *NewDeg < -180 ) *NewDeg += 360;
	if ( *NewDeg > 180 ) *NewDeg -= 360;
}
// yaw offset could be generated from GPS or other AI source
// The goal is to make YawMPU == YawOffset by adjusting the Turn output between +- 500 to match what might happen with the RC Reciever
// Set point is always zero output is zero when set point is reached so Integral is irrelevant
// YawMPU is the direction the Gyro says we are pointing.
// YawOffset is the direction we should be pointing
// Turn is a value between +-500
// kp = 25 kd = 3 controlDirection = -1 past experience
void RC::YawGyroPiD(int16_t *Turn, float YawInput,int16_t MaxOut, float kp, float kd, int8_t controlDirection, uint8_t TurnControlEnable) {
	if (TurnControlEnable) {
		static float lastInput;
	//	Serial.printfloatx(F("YawInput ")  , YawInput, 4, 0, F(" "));

		if (YawInput > 180) YawInput -= 360; // rollover to negative
		if (YawInput < -180) YawInput += 360; // rollover to positive
		// PD no Integral

		float error = 0 - YawInput;// Calculate error
		float PTerm = (kp *  controlDirection) * error;// Proportional term
		float DTerm = 0;
		if(kd != 0){
			
			static uint32_t lastTime;
			float DeltaTS;
			unsigned long Now;
			Now = micros();
			DeltaTS = ((float)(Now - lastTime)) * 0.000001;
			lastTime = Now;
			DTerm = (-kd *  controlDirection) * (((YawInput - lastInput) * .5)  / DeltaTS); //Derivative term  that avoids infinity errors

			if(abs(error) < 3)DTerm = 0;// Shut down DTerm while near set point if we could use DTerm then we are out of tune anyway and overshooting set point.
		}
		lastInput = YawInput;
		int16_t Output = PTerm + DTerm; //Compute PID Output
		Output = (Output < -MaxOut )? -MaxOut:((Output > MaxOut)? MaxOut : Output); // Force Limits RC Pulse width 1000us
		*Turn =  Output;
		} else{
		*Turn = 0;
	}
}


void RC::XYtoTank(int16_t X, int16_t Y, int16_t* LeftDrive, int16_t* RightDrive,int16_t Throttle){
	int16_t LeftD,RightD,V;
	float DriveScaler;
	//mix throttle and direction
	LeftD = (X - Y) ;
	RightD = (X + Y) ;
	DriveScaler = (max(1, max(abs((float)LeftD / 500.0), abs((float)RightD / 500.0))));
	V = (int16_t)constrain((float)LeftD / DriveScaler, -500, 500);
	if(deadZone)V = TankDeadzone(V, deadZone);
	V = (Throttle == -1) ? V: map(V,0,500,0,Throttle); // Adjust Throttle Controlls Max Speed
	*LeftDrive =  V ;
	V = (int16_t)constrain((float)RightD / DriveScaler, -500, 500);
	if(deadZone)V = TankDeadzone(V, deadZone);
	V = (Throttle == -1) ? V: map(V,0,500,0,Throttle); // Adjust Throttle Controlls Max Speed
	*RightDrive =  V;
	TankLeftDrive = *LeftDrive;
	TankRightDrive = *RightDrive;
}
int16_t RC::TankDeadzone(int16_t V, uint16_t deadzone){
	int16_t absV = abs(V);
	absV = (absV < 3) ? 0 : map(absV, 3, 500, deadzone ,500) ; // Inserts a Deadzone
	V = (V >= 0) ? absV :  -absV; // Restores a FullRange Value +-
	return(V);
}

int16_t RC::ZeroDeadzone(int16_t Val, int16_t DZone){
    int16_t absV ;
	absV = abs(Val);
	absV = absV - DZone;
	if (absV < 0) absV = 0;
	absV  = map((absV) , 0, (500 - DZone ), 0 ,500);
	Val = (Val >= 0) ? absV :  -absV; // Restores a FullRange Value +-
	return(Val);
}

void RC::TankPWM(int16_t LeftDrive,int16_t RightDrive, int16_t Breaking, uint8_t LeftPin, uint8_t Forward_LeftPin , uint8_t  Reverse_LeftPin, uint8_t RightPin, uint8_t  Forward_RightPin, uint8_t Reverse_RightPin ){
	
	static int16_t LastLeftDrive;
	static int16_t LastRightDrive;
	static int16_t LastLeftDriveDirection;
	static int16_t LastRightDriveDirection;
	static int16_t ActualLeftDrive;
	static int16_t ActualRightDrive;
	static int16_t LeftDriveRampUP = 0;
	static int16_t RightDriveRampUP = 0;
	ActualLeftDrive = abs(LeftDrive);
	LastLeftDrive = ActualLeftDrive;
	
	ActualRightDrive = abs(RightDrive);
	LastRightDrive = ActualRightDrive;
	
    if ((LastLeftDriveDirection <= 0) ==  (LeftDrive > 0)) LeftDriveRampUP = 0;
    ActualLeftDrive =  (ActualLeftDrive > LeftDriveRampUP) ? (int16_t) ((LeftDriveRampUP++) ) : (LeftDriveRampUP =  ActualLeftDrive);
    if ((LastRightDriveDirection <= 0) ==  (RightDrive >= 0)) RightDriveRampUP = 0;
    ActualRightDrive =  (ActualRightDrive > RightDriveRampUP) ? (int16_t) ((RightDriveRampUP++) ) : (RightDriveRampUP =  ActualRightDrive);

	if ((LeftDrive == 0) ||((LastLeftDriveDirection > 0) != (LeftDrive > 0))){
		digitalWrite(Forward_LeftPin, LOW);
		digitalWrite(Reverse_LeftPin, LOW);
		ActualLeftDrive = min(255,abs(Breaking));
		// Add Breaking
	}
	if ((RightDrive == 0) ||((LastRightDriveDirection > 0) != (RightDrive > 0))){
		digitalWrite(Forward_RightPin, LOW);
		digitalWrite(Reverse_RightPin, LOW);
		ActualRightDrive = min(255,abs(Breaking));
		// Add Breaking
	}

	LastLeftDriveDirection = LeftDrive;
	LastRightDriveDirection = RightDrive;
	//	LastLeftDriveDirection = min(-1,max(1,LeftDrive));// -1 Reverse , 0 stop, +1 Forward
	//	LastRightDriveDirection = min(-1,max(1,RightDrive)); // -1 Reverse , 0 stop, +1 Forward
	//Add Slow Ramp up to Speed to Prevent sudden changes in direction of motor

	analogWrite(LeftPin,constrain(map(ActualLeftDrive,0,500,0,255),0,255));
	analogWrite(RightPin,constrain(map(ActualRightDrive,0,500,0,255),0,255));

	if(LeftDrive < 0){
		digitalWrite(Forward_LeftPin, HIGH);
		digitalWrite(Reverse_LeftPin, LOW);
	}
	if(RightDrive < 0){
		digitalWrite(Forward_RightPin, HIGH);
		digitalWrite(Reverse_RightPin, LOW);
	}
	if(LeftDrive > 0){
		digitalWrite(Forward_LeftPin, LOW);
		digitalWrite(Reverse_LeftPin, HIGH);
	}
	if(RightDrive > 0){
		digitalWrite(Forward_RightPin, LOW);
		digitalWrite(Reverse_RightPin, HIGH);
	}


}




int16_t RC::PinEncode(uint8_t pin,uint8_t Reset) {

	return(PosEncode(PinArray[pin],Reset));

}
int16_t RC::PosEncode(uint8_t PinCount,uint8_t Reset) {

	if(EncodeModeVal>1){
		int16_t V = EncodeValue[PinCount] - EncodeValue[PinCount+1];
		if(Reset){
			EncodeValue[PinCount] = 0;
			EncodeValue[PinCount+1] = 0;
		}
		return(V);
	}
	int16_t V = EncodeValue[PinCount];
	if(Reset)EncodeValue[PinCount]=0;
	return(V);
}
void RC::Ping(){

}
int16_t RC::PinPing(uint8_t pin) {
	return(PosPing(PinArray[pin]));
}
int16_t RC::PosPing(uint8_t PinCount) {
	return(PingTime[PinCount]);
}
int16_t RC::PinSwitch(uint8_t pin) {
	return(PosSwitch(PinArray[pin]));
}
int16_t RC::PosSwitch(uint8_t PinCount) {
	return(SwitchValue[PinCount]);
}

