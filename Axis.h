#ifndef Axis_h
#define Axis_h

#include "SparkFunAutoDriver.h"
#include "Keyframe.h"

//Maximum number of keyframes
#define MAXKEYFRAMES 6

class Axis:public AutoDriver {
  // Program state 
  public:
    enum MotionState {STOPPED, INIT, SEQUENCING, LASTKEYFRAME, MANUAL};
    enum ResultCode {SUCCESS, NOKEYFRAMES};
  
    Axis(int CSPin, int resetPin);
    Axis(int CSPin, int resetPin, int busyPin);

    void setAxisNumber(int number) {this->axisNumber = number;}
    int getAxisNumber() {return this->axisNumber;}

    void setNumberOfKeyframes(int number) {this->numberOfKeyframes = number;}
    int getNumberOfKeyframes() {return this->numberOfKeyframes;}

    MotionState getMotionState() {return motionState;}
    
    Keyframe* getKeyframe(int index) {return &keyframes[index-1];}       //get keyframe at index

    void nextKeyframe();              //switches to next keyframe
    int startKeyframeSequence();     //starts motion programme consisting of keyframe sequence
    void stopKeyframeSequence();      //stops and resets motion programme
    void controlKeyframeSequence();   //controls motion programme, including sequencing of keyframes

    byte getDirection(Keyframe* kf1, Keyframe* kf2);
    byte getDirection();
    void markStart();
    long getStart();
    void markEnd();
    long getEnd();
    
    void stop();

  private:
    int axisNumber;                        //axis number
    Keyframe keyframes[MAXKEYFRAMES];     //keyframe sequence
    int numberOfKeyframes = 0;             //number of keyframes in keyframe sequence
    int currentKeyframeIndex  = 0;         //sequence number of current keyframe
    Keyframe* currentKeyframe = &keyframes[0];  //pointer to current keyframe
    MotionState motionState = STOPPED;     //states of the motion program
    long startPosition = -2000000;                //start position
    long endPosition = 2000000;                  //end position
    
    void debug(String message) {Serial.println(message);
      }
    float maxSpeed = 600;
};

//Status register
//#define STATUS_HIZ    0x01
//#define STATUS_BUSY   0x02
//#define STATUS_SW_F   0x04
//#define STATUS_SW_EVN 0x08
#define STATUS_DIR    0x10

//#define STATUS_DIR 0b00010000

#endif
