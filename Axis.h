#ifndef Axis_h
#define Axis_h

#include "SparkFunAutoDriver.h"
#include "Keyframe.h"

//Maximum number of keyframes
#define MAXKEYFRAMES 6

class Axis :public AutoDriver {
	// Program state 
public:
	enum MotionState { STOPPED, INIT, SEQUENCING, LASTKEYFRAME, MANUAL };
	enum ResultCode { SUCCESS, NOKEYFRAMES };

	Axis(int position, int CSPin, int resetPin);
	Axis(int position, int CSPin, int resetPin, int busyPin);

	void setAxisNumber(int number) { this->axisNumber = number; }
	int getAxisNumber() { return this->axisNumber; }

	void setNumberOfKeyframes(int number) { this->numberOfKeyframes = number; }
	int getNumberOfKeyframes() { return this->numberOfKeyframes; }
	int addKeyframe(Keyframe kf);
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
	void setMaxSpeed(float speed);	//Overloading AutoDriver method
	void setAcc(float acc);			//Overloading AutoDriver method
	void setDec(float dec);			//Overloading AutoDriver method

	

private:
	int axisNumber;                        //axis number
	Keyframe keyframes[MAXKEYFRAMES];     //keyframe sequence
	int numberOfKeyframes = 0;             //number of keyframes in keyframe sequence
	int currentKeyframeIndex = 0;         //sequence number of current keyframe
	Keyframe* currentKeyframe = &keyframes[0];  //pointer to current keyframe
	MotionState motionState = STOPPED;     //states of the motion program
	long startSoftStop = 0L;                //start position
	long endSoftStop = 0L;                  //end position
	bool stopsEnabled = false;

	void debug(String message) { Serial.println(message); }

	float maxSpeedBuffer = 0;
	float accBuffer = 0;
	float decBuffer = 0;
};

//Status register
//#define STATUS_HIZ    0x01
//#define STATUS_BUSY   0x02
//#define STATUS_SW_F   0x04
//#define STATUS_SW_EVN 0x08
//#define STATUS_DIR    0x10

//#define STATUS_DIR 0b00010000

#endif
