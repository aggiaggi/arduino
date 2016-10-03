#include "Axis.h"

Axis::Axis(int position, int CSPin, int resetPin) : AutoDriver(position,CSPin,resetPin) {};

Axis::Axis(int position, int CSPin, int resetPin, int busyPin) : AutoDriver(position,CSPin,resetPin,busyPin) {};

int Axis::startKeyframeSequence() {
  debug("Starting up keyframe sequence");
  //if (++currentKeyframeIndex <= numberOfKeyframes) {
  currentKeyframeIndex = 0;
  
  //Update motion state
  motionState = SEQUENCING;

  nextKeyframe();
  
  return SUCCESS;
  
}

void Axis::nextKeyframe() {
  //Increase keyframe index
  currentKeyframeIndex++;
  currentKeyframe = getKeyframe(currentKeyframeIndex);
  
  //Get current stepper position
  byte stepMode = this->getStepMode();
  long currentPos = this->getPos()>>(stepMode%8);

  //Get new position & speed
  long newPos = currentKeyframe->getPosition();
  float newSpeed = currentKeyframe->getSpeed();
  float newAcc = currentKeyframe->getAcc();
  float newDec = currentKeyframe->getDec();
    
  //Determine direction
  byte dir = FWD;
  if(currentPos > newPos)
    dir = REV;

  //Setup acceleration and deceleration
  this->setAcc(newAcc);
  this->setDec(newDec);
        
  //Start motion
  if (currentKeyframeIndex < numberOfKeyframes) {
    debug("Next Keyframe!");
    Keyframe* nextKeyframe = getKeyframe(currentKeyframeIndex+1);
    if (dir == getDirection(currentKeyframe, nextKeyframe)) {
      
      //If not last keyframe AND keyframe after next keyframe requires same direction of movement 
      //-> run command required
      this->run(dir, newSpeed);
      
      debug("Run " + String(newPos));
    }
    else {
      //Keyframe after  next keyframe requires switching of direction
      //-> goto command required to stop at exact position
      this->setMaxSpeed(newSpeed);
      this->goTo(((unsigned long)newPos)<<(stepMode%8));
      
      debug("Goto " + String(newPos));
    }
  }
  else {
    //If last keyframe: goto command required to reach exact stop position
    
    this->setMaxSpeed(newSpeed);
    this->goTo(((unsigned long)newPos)<<(stepMode%8));
    debug("Last Keyframe!");
    debug("Goto " + String(newPos) + " @" + String(newSpeed));

    //Update motion state to indicate last keyframe
    motionState = LASTKEYFRAME;
  }
  debug("Speed: " + String(newSpeed)
          + " / Accelleration: " + String(newAcc)
          + " / Deceleration: " + String(newDec)
          + " /Direction: " + String(dir));
}

void Axis::stopKeyframeSequence() {
  //Immediatly stop stepper
  //this->hardHiZ();
  this->setMaxSpeed(maxSpeed);
  
  //Reset keyframe index
  currentKeyframeIndex = 0;

  //Update motion state
  motionState = STOPPED;

  debug("Keyframe sequence stopped!");
}

void Axis::controlKeyframeSequence() {
  //Determine if keyframe position has been reached or surpassed
  
  //Get current stepper position
  byte stepMode = this->getStepMode();
  long currentPos = this->getPos()>>(stepMode%8);
  
  //Get target keyframe position
  long targetPos = currentKeyframe->getPosition();

  //Get status register
  int currentStatus = this->getStatus();

  //Get direction from status register
  bool stepperDir = (currentStatus & STATUS_DIR) == STATUS_DIR;

  //Check if target position has been reached or surpassed, depending on stepper direction
  //dir = FWD -> position increases
  if (!this->busyCheck() && (((currentPos >= targetPos) && stepperDir) || ((currentPos <= targetPos) && !stepperDir))) {

    if (motionState != LASTKEYFRAME) {
      //select next keyframe in sequence
      nextKeyframe();
      //give stepper some time to start moving before doing next position check
      //delay(200);
    }
    else
      stopKeyframeSequence();
    
  }

  debug(String(currentPos) + "/" + String(targetPos) + "/" + String(stepperDir) + "/" +String(this->busyCheck()));
  
}

byte Axis::getDirection(Keyframe* kf1, Keyframe* kf2) {
  if ((kf2->getPosition() - kf1->getPosition()) >= 0  )
    return FWD;
  else
    return REV;
}

void Axis::markStart() {
  this->startPosition = this->getPos();
  debug(String(this->getDirection()));
}

long Axis::getStart() {
  return startPosition;
}

void Axis::markEnd() {
   this->endPosition = this->getPos();
   debug(String(this->getDirection()));
}

long Axis::getEnd() {
  return endPosition;
}

byte Axis::getDirection() {
  int status = this->getStatus();
  if ((status & STATUS_DIR) == STATUS_DIR)
    return FWD;
  else
    return REV;
}

void Axis::stop() {
   this->softHiZ();
}


