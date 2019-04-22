#ifndef Axis_h
#define Axis_h

#include "SparkFunAutoDriver.h"
#include "Keyframe.h"

//Maximum number of keyframes
#define MAXKEYFRAMES 6

struct MotorConfig {
	float voltage;		// rated voltage [V]
	float current;		// rated current [A]
	float resistance;	// phase resistance [Ohm]
	float inductance;	// phase inductance in [mH]
	float torque;		// torque [Ncm]
};

struct AxisConfig {
	float maxAccel;		// maximum acceleration [steps/s/s]
	float maxDecel;		// maximum deceleration [steps/s/s]
	long maxSpeed;		// maximum speed [steps/s]
	byte accPower;		// acceleration power ratio [%]
	byte decPower;		// deceleration power raqtio [%]
	byte runPower;		// running power ratio [%]
	byte holdPower;		// holding power ratio [%]
	MotorConfig motorConfig;	// axis motor configuration
};

class Axis :public AutoDriver {
	// Program state 
public:
	enum MotionState { 
		STOPPED,		// program stopped
		INITIALIZING,	// axes are moving to first keyframe position
		INITCOMPLETE,	// initializytion complete and waiting for start
		RUNNING,		// programm is running
		LASTKEYFRAME,	// last keyframe in program
		STOPPING		// axis is decelerating to stop position
 	};
	enum ResultCode { SUCCESS, NOKEYFRAMES };

	Axis(int position, int CSPin, int resetPin);
	Axis(int position, int CSPin, int resetPin, int busyPin);

	int init(AxisConfig*);

	void setAxisNumber(int number) { this->axisNumber = number; }
	int getAxisNumber() { return this->axisNumber; }

	void setNumberOfKeyframes(int number) { this->numberOfKeyframes = number; }
	int getNumberOfKeyframes() { return this->numberOfKeyframes; }
	int addKeyframe(Keyframe kf);	// add keyframe
	int setKeyframe(Keyframe kf, byte position); // set keyframe at position
	void printKeyframes();

	MotionState getMotionState() { return motionState; }

	Keyframe* getKeyframe(int index) { return &keyframes[index - 1]; }       //get keyframe at index

	int initKeyframeSequence(int framerate);
	float calculateSpeed(float T, float acc, float dec, long x);
	void nextKeyframe();              //switches to next keyframe
	int startKeyframeSequence();     //starts motion programme consisting of keyframe sequence
	void stopKeyframeSequence();      //stops and resets motion programme
	void controlKeyframeSequence();   //controls motion programme, including sequencing of keyframes

	byte getDirection(Keyframe* kf1, Keyframe* kf2);
	byte getDirection();
	void markStartSoftStop();
	long getStartSoftStop();
	void markEndSoftStop();
	long getEndSoftStop();
	void setStartSoftStop(long pos);
	void setEndSoftStop(long pos);
	void stop();
	long getFullPosition();
	long getMicroStepPosition(long position);
	

private:
	int axisNumber;                        //axis number
	float busVoltage = 24.0;				// bus voltage
	AxisConfig* axisConfig;					// stores axis configuration information
	Keyframe keyframes[MAXKEYFRAMES];     //keyframe sequence
	int numberOfKeyframes = 0;             //number of keyframes in keyframe sequence
	int currentKeyframeIndex = 0;         //sequence number of current keyframe
	Keyframe* currentKeyframe = &keyframes[0];  //pointer to current keyframe
	byte currentDir;						// direction from previous to current keyframe position
	long switchingPosition;					// position at which to switch keyframes
	MotionState motionState = STOPPED;     //states of the motion program
	long startSoftStop = 0L;                //start position
	long endSoftStop = 0L;                  //end position
	bool stopsEnabled = false;
	long lastPosition = 0L;					// stores last position to detect movement

	void debug(String message) { Serial.println(message); }
};

//Status register
//#define STATUS_HIZ    0x01
//#define STATUS_BUSY   0x02
//#define STATUS_SW_F   0x04
//#define STATUS_SW_EVN 0x08
//#define STATUS_DIR    0x10

//#define STATUS_DIR 0b00010000

#endif
