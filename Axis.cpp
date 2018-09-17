#include "Axis.h"

Axis::Axis(int position, int CSPin, int resetPin) : AutoDriver(position, CSPin, resetPin) {};

Axis::Axis(int position, int CSPin, int resetPin, int busyPin) : AutoDriver(position, CSPin, resetPin, busyPin) {};


// Add keyframe to keyframes array
int Axis::addKeyframe(Keyframe kf) {
	// too many keyframes?
	if (numberOfKeyframes == 0) {
		// first keyframe to be created
		keyframes[0] = kf;
		numberOfKeyframes++;
		return 1;
	}
	else if (numberOfKeyframes + 1 > MAXKEYFRAMES) {
		// no keyframe is created
		debug("Too many keyframes");
		return 0;
	}
	else
	{
		//Insert keyframe in array based on frame number
		unsigned long newFrame = kf.getFrame();
		
		unsigned long currentFrame = keyframes[0].getFrame();
		int i = 0;
		do  {
			i++;
			currentFrame = keyframes[i].getFrame();
		} while (newFrame <= currentFrame);

		//Is there already a keyframe at this frame
		if (newFrame == currentFrame) {
			// replace keyframe
			debug("Replacing keyframe at frame " + String(newFrame));
			keyframes[i] = kf;
			return 1;
		} 
		else {
			// insert new keyframe into array
			// calculate number of keyframes to be moved to make space for new keyframe
			
			int move = numberOfKeyframes - i;
			debug("Inserting Keyframe " + String(i) + " and shifting " + String(move) + " keyframes");
			for (int j = 0; j < move; j++) {
				// start with last keyframe
				int index = i + move - j - 1;
				if (index + 1 > MAXKEYFRAMES) {
					debug("Error while adding keyframe");
					return 0;
				}
				keyframes[index + 1] = keyframes[index];
				
			}
			keyframes[i] = kf;
		}
		numberOfKeyframes++;
		return 1;
	}
}

void Axis::printKeyframes() {
	debug("Number of keyframes: " + String(numberOfKeyframes));
	for (int i = 0; i < MAXKEYFRAMES; i++) {
		Keyframe kf = keyframes[i];
		debug("F" + String(kf.getFrame()) + "P" + String(kf.getPosition()) + "S" + String(kf.getSpeed()));
	}
}

// Calculate speed based on quadratic equation via p/q formula:
// v^2 - 2T/(1/acc + 1/dec) * v + 2x/(1/acc + 1/dec) = 0
// v1,2 = -p/2 +- sqrt( (p/2)^2 - q )
// p = 2T/(1/acc + 1/dec) ; q = 2x/(1/acc + 1/dec)
float Axis::calculateSpeed(float T, float acc, float dec, long x) {
	float p = -2 * T / (1 / acc + 1 / dec);
	float q = 2 * x / (1 / acc + 1 / dec);
	debug("p:" + String(p) + " / q:" + String(q));

	float v = -p / 2 - sqrt(sq(p/2) - q);
       
	return v;
}


int Axis::initKeyframeSequence(int framerate) {
	// set speed for first keyframe
	float maxSpeed = maxSpeedBuffer;
	keyframes[0].setSpeed(maxSpeed);
	unsigned long lastFrame = keyframes[0].getFrame();
	long lastPosition = keyframes[0].getPosition();

	// set keyframe speeds based on framerate
	if (numberOfKeyframes > 1) {
		for (int i = 1; i < numberOfKeyframes; i++) {
			unsigned long currentFrame = keyframes[i].getFrame();
			float time = ((float)(currentFrame - lastFrame)) / ((float)framerate);
			long currentPosition = keyframes[i].getPosition();
			float speed = calculateSpeed(time, keyframes[i].getAcc(), keyframes[i].getDec(), abs(currentPosition - lastPosition));
			debug(String(speed));
			//float speed = ((float)(currentPosition - lastPosition)) / time;
			if (speed > maxSpeed) {
				debug("Max speed exceeded for keyframe at frame " + String(currentFrame) + ": " + String(speed));
				return -1;
			}
			keyframes[i].setSpeed(speed);
		}
	}
	
	return 0;
}

int Axis::startKeyframeSequence() {
	debug("Starting up keyframe sequence");
	//if (++currentKeyframeIndex <= numberOfKeyframes) {

	debug("Buffers: maxspeed " + String(this->maxSpeedBuffer) + " / acc " + String(this->accBuffer) + " / dec " + String(this->decBuffer));

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
	long currentPos = this->getFullPosition();

	//Get new position & speed
	long newPos = currentKeyframe->getPosition();
	float newSpeed = currentKeyframe->getSpeed();
	float newAcc = currentKeyframe->getAcc();
	float newDec = currentKeyframe->getDec();

	//Determine direction
	byte dir = FWD;
	if (currentPos > newPos)
		dir = REV;

	//Setup acceleration and deceleration
	this->AutoDriver::setAcc(newAcc);
	this->AutoDriver::setDec(newDec);

	//Start motion
	if (currentKeyframeIndex < numberOfKeyframes) {
		debug("Next Keyframe!");
		//Keyframe* nextKeyframe = getKeyframe(currentKeyframeIndex + 1);
		//if (dir == getDirection(currentKeyframe, nextKeyframe)) {
		//  
		//  //If not last keyframe AND keyframe after next keyframe requires same direction of movement 
		//  //-> run command required
		//  this->run(dir, newSpeed);
		//  
		//  debug("Run " + String(newPos));
		//}
		//else {
		  //Keyframe after  next keyframe requires switching of direction
		  //-> goto command required to stop at exact position
		this->AutoDriver::setMaxSpeed(newSpeed);
		byte stepMode = this->getStepMode();
		this->goTo(((unsigned long)newPos) << (stepMode % 8));

		debug("Goto " + String(newPos));
		//}
	}
	else {
		//If last keyframe: goto command required to reach exact stop position

		this->AutoDriver::setMaxSpeed(newSpeed);
		byte stepMode = this->getStepMode();
		this->goTo(((unsigned long)newPos) << (stepMode % 8));
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

	// Restore original values
	this->setMaxSpeed(this->maxSpeedBuffer);
	this->setAcc(this->accBuffer);
	this->setDec(this->decBuffer);

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
	long currentPos = this->getPos();

	//Get target keyframe position in microsteps
	long targetPos = (currentKeyframe->getPosition()) << (stepMode % 8);

	//Get status register
	int currentStatus = this->getStatus();

	//Get direction from status register
	bool stepperDir = (currentStatus & STATUS_DIR) == STATUS_DIR;

	//Check if target position has been reached or surpassed, depending on stepper direction
	//dir = FWD -> position increases
	//if (!this->busyCheck() && (((currentPos >= targetPos) && stepperDir) || ((currentPos <= targetPos) && !stepperDir))) {
	if (
		((currentPos >= targetPos) && stepperDir) || ((currentPos <= targetPos) && !stepperDir)
		) {


		if (motionState != LASTKEYFRAME) {
			//select next keyframe in sequence
			nextKeyframe();
			//give stepper some time to start moving before doing next position check
			//delay(200);
		}
		else
			stopKeyframeSequence();

	}

	debug(String(currentPos) + "/" + String(targetPos) + "/" + String(stepperDir) + "/" + String(this->busyCheck()));

}

byte Axis::getDirection(Keyframe* kf1, Keyframe* kf2) {
	if ((kf2->getPosition() - kf1->getPosition()) >= 0)
		return FWD;
	else
		return REV;
}

void Axis::markStartSoftStop() {
	this->startSoftStop = this->getPos();
	debug(String(this->startSoftStop));
}

void Axis::setStartSoftStop(long pos) {
	this->startSoftStop = pos;
	debug(String(pos));
}

long Axis::getStartSoftStop() {
	return startSoftStop;
}

void Axis::markEndSoftStop() {
	this->endSoftStop = this->getPos();
	debug(String(this->endSoftStop));
}

void Axis::setEndSoftStop(long pos) {
	this->endSoftStop = pos;
	debug(String(pos));
}

long Axis::getEndSoftStop() {
	return endSoftStop;
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

/*bool Axis::getStopsEnabled() {
	return this->stopsEnabled;
}
*/

long Axis::getMicroStepPosition(long position) {
	byte stepMode = this->getStepMode();
	return position << (stepMode % 8);
}

long Axis::getFullPosition() {
	byte stepMode = this->getStepMode();
	long position = this->AutoDriver::getPos() >> (stepMode % 8);
	return  position;
}

void Axis::setMaxSpeed(float speed) {
	this->maxSpeedBuffer = speed;
	this->AutoDriver::setMaxSpeed(speed);
}

void Axis::setAcc(float acc) {
	this->accBuffer = acc;
	this->AutoDriver::setAcc(acc);
}

void Axis::setDec(float dec) {
	this->decBuffer = dec;
	this->AutoDriver::setDec(dec);
}
