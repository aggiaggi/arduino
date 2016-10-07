#include <SPI.h>
#include "Axis.h"

//Buffer REST commands
#define COMMANDBUFFERLENGTH 512
char commandBuffer[COMMANDBUFFERLENGTH];
//Buffer for REST values
#define VALUEBUFFERLENGTH 255
char valueBuffer[VALUEBUFFERLENGTH];


//Storing axis information

//Autodriver setup
Axis axis1(0, 2, 4, 3);
//Axis axis2(0, 6, 8, 7);
//Axis axis3(0, 10, 12, 11);

Axis* currentAxis = &axis1;

int counter = 0;

void setup() {

	//Setup I/O pins
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(2, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(3, INPUT);

	//Set CS High
	digitalWrite(2, HIGH);

	//Reset
	digitalWrite(4, LOW);
	digitalWrite(4, HIGH);

	SPI.begin();
	SPI.setDataMode(SPI_MODE3);

	void SPIPortConnect(SPIClass *SPIPort);


	axis1.SPIPortConnect(&SPI);

	//Trinamic
	/*axis1.setAccKVAL(106);
	axis1.setDecKVAL(106);
	axis1.setRunKVAL(106);
	axis1.setHoldKVAL(106);
	axis1.setParam(INT_SPD, 6675);
	axis1.setParam(ST_SLP, 68);
	axis1.setParam(FN_SLP_ACC, 137);
	axis1.setParam(FN_SLP_DEC, 137);*/

	//Motech 24V
	/*axis1.setAccKVAL(10);
	axis1.setDecKVAL(10);
	axis1.setRunKVAL(10);
	axis1.setHoldKVAL(10);
	axis1.setParam(INT_SPD, 4806);
	axis1.setParam(ST_SLP, 59);
	axis1.setParam(FN_SLP_ACC, 95);
	axis1.setParam(FN_SLP_DEC, 95);
	*/
	//SanyoDenki 24V
   /*axis1.setAccKVAL(10);
   axis1.setDecKVAL(10);
   axis1.setRunKVAL(10);
   axis1.setHoldKVAL(0);//17
   axis1.setParam(INT_SPD, 0);
   axis1.setParam(ST_SLP, 52);
   axis1.setParam(FN_SLP_ACC, 85);
   axis1.setParam(FN_SLP_DEC, 85);
   */

	axis1.setAxisNumber(1);
	axis1.configStepMode(STEP_FS_64); //Step mode 1/64

	//Wantai 28BYGH105 17V
	axis1.setAccKVAL(20); //40
	axis1.setDecKVAL(20);
	axis1.setRunKVAL(20);
	axis1.setHoldKVAL(0);
	axis1.setParam(INT_SPD, 37382);
	axis1.setParam(ST_SLP, 37);
	axis1.setParam(FN_SLP_ACC, 42);
	axis1.setParam(FN_SLP_DEC, 42);

	// axis2.setAxisNumber(2);
	 //axis2.configStepMode(STEP_FS_64); //Step mode 1/64

	 // Set up serial port between MCU and Linux
	SerialUSB.begin(400000);
	//Set up serial console port for debugging
	Serial.begin(115200);

	debug("Axes initialisation finished ...");
}

void loop() {

	//Send JSON data to node server
	String data =
		"{\"pos1\":" + String(axis1.getPos()) + ",\"pos2\":" + String("-123") + ",\"pos3\":" + String(-123) + "}";
	SerialUSB.println(data);

	//Receive commands from node server
	String commandString = "";
	char c;
	while (SerialUSB.available()) {
		c = SerialUSB.read();
		commandString += String(c);
	}

	//Process command
	if (commandString.length() > 0) {
		process(commandString);
	}

	//Update axes
	
	/*if (axis1.getMotionState() != Axis::STOPPED )
	  axis1.controlKeyframeSequence();
	else if ((axis1.getMotionState() != Axis::MANUAL) && (axis1.getStopsEnabled())) {
	  byte dir = axis1.getDirection();
	  long pos = axis1.getPos();
	  if ((dir == FWD && pos >= axis1.getEndSoftStop()) || (dir == REV && pos <= axis1.getStartSoftStop()) )
	  {
		axis1.hardStop();
		debug("Axis Hard Stop!");
	  }
	}*/

	//if (axis2.getMotionState() != Axis::STOPPED )
	//  axis2.controlKeyframeSequence();
	

	// Poll every 100ms
	delay(300);

	//Output position info
	//if (++counter >= 10 && (axis1.getMotionState() == Axis::MANUAL || axis2.getMotionState() == Axis::MANUAL)) {
	//  debug("Axis1 pos: " + String(axis1.getPos()) + " / Axis2 pos: " + String(axis2.getPos()));
	//  counter = 0;
	//}
}

/// --------------------------
// Process GUI commands
//
/// --------------------------
void process(String commandString) {
	debug(F("Process ..."));
	debug(commandString);

	// Get rid of whitespace
	commandString.trim();

	//Parse axis
	String token = parseCommand(&commandString);
	// is "start" command?
	// arduino/start
	//switch(tokrn)
	if (token == "start") {
		debug("Start!");
		int result = initializeKeyframeSequence(&axis1);
		//result += initializeKeyframeSequence(&axis2);
		if (result == 0) {
			axis1.startKeyframeSequence();
			//axis2.startKeyframeSequence();
		}
		else
			debug(F("Problems starting motion program!"));

		return;
	}

	// is "stop" command?
	// arduino/stop
	else if (token == "stop") {
		debug("Stop!");
		axis1.hardHiZ();
		axis1.stopKeyframeSequence();

		//axis2.hardHiZ();
		//axis2.stopKeyframeSequence();

		return;
	}

	else if (token == "1")
		currentAxis = &axis1;
	else
		return;

	//Parse next token
	token = parseCommand(&commandString);

	// is "move" command?
	// arduino/<axis>/move/<'FWD'|'REV'>/<NUMBER_OF_STEPS>
	if (token == "move") {
		byte dir = FWD;
		unsigned long numberOfSteps = 0;

		if (commandString != "") {
			//debug("commandString: " + commandString);

			//Parse direction token
			token = parseCommand(&commandString);
			if (token == "REV") {
				dir = REV;
			}

			//Parse number of steps token
			if (commandString != "") {
				token = parseCommand(&commandString);
				numberOfSteps = token.toInt();
			}
			byte stepMode = currentAxis->getStepMode();
			debug("Move dir " + String(dir) + " / Steps " + String(numberOfSteps));
			currentAxis->AutoDriver::move(dir, numberOfSteps << (stepMode % 8));
		}
	}

	// is "RUN" command?
	// arduino/<axis>/run/<'FWD'|'REV'>/<STEPS_PER_SECOND>
	else if (token == "run") {
		byte dir = FWD;
		float runSpeed = 0.0;

		//Parse speed token
		if (commandString != "") {
			//debug("commandString: " + commandString);
			token = parseCommand(&commandString);
			runSpeed = token.toFloat();
			if (runSpeed < 0) {
				runSpeed = -1.0 * runSpeed;
				dir = REV;
			}
		}
		debug("Run dir " + String(dir) + " / Speed " + String(runSpeed));
		currentAxis->AutoDriver::run(dir, runSpeed);
	}

	// is "GO" command?
	// arduino/<axis>/go/<POS>
	else if (token == "go") {
		
		long pos = 0;
		//Parse speed token
		if (commandString != "") {
			//debug("commandString: " + commandString);
			token = parseCommand(&commandString);
			pos = token.toInt();
		}
		byte dir;
		if (pos >= currentAxis->getPos())
			dir = FWD;
		else
			dir = REV;
		debug("Go Position " + String(pos) + " / Dir " + String(dir));
		currentAxis->AutoDriver::goToDir(dir, pos);
	}


	// is Acceleration command?
	// arduino/<axis>/acc/<?PARAM?>
	else if (token == "acc") {
		if (commandString != "") {
			//Parse parameter token
			String paramString = parseCommand(&commandString);
			float param = paramString.toFloat();

			debug("Set acceleration for axis" + String(currentAxis->getAxisNumber()) + " to " + paramString);
			currentAxis->setAcc(param);
		}
		else {
			//return current accelleration parameter
			//client.print(currentAxis->getAcc());
		}
	}

	// is Deceleration command?
	// arduino/<axis>/dec/<?PARAM?>
	else if (token == "dec") {
		if (commandString != "") {
			//Parse parameter token
			String paramString = parseCommand(&commandString);
			float param = paramString.toFloat();

			debug("Set deceleration for axis" + String(currentAxis->getAxisNumber()) + " to " + paramString);
			currentAxis->setDec(param);
		}
		else {
			//return current deceleration parameter
			//client.print(currentAxis->getDec());
		}
	}

	// is Speed command?
	// arduino/<axis>/speed/<?PARAM?>
	else if (token == "speed") {
		if (commandString != "") {
			//Parse parameter token
			String paramString = parseCommand(&commandString);
			float param = paramString.toFloat();

			debug("Set max speed to " + paramString);
			currentAxis->setMaxSpeed(param);
		}
		else {
			//return current max speed parameter
			//client.print(currentAxis->getMaxSpeed());
		}
	}

	// is "stop" command?
	// arduino/<axis>/stop
	else if (token == "stop") {
		debug("Stop!");
		currentAxis->stop();
		currentAxis->stopKeyframeSequence();

		return;
	}

	//Mark home command
	else if (token == "sethome") {
		debug("Set Home!");
		currentAxis->resetPos();
	}

	//Go home command
	else if (token == "gohome") {
		debug("Go Home!");
		currentAxis->goHome();
	}

	//Mark start soft stop command
	else if (token == "markstartsoftstop") {
		debug("Mark Start Stop!");
		currentAxis->markStartSoftStop();
	}

	//Mark end soft stop command
	else if (token == "markendsoftstop") {
		debug("Mark End Stop!");
		currentAxis->markEndSoftStop();
	}

	//Go to start soft stop
	else if (token == "gostartsoftstop") {
		debug("Go Start Soft Stop!");
		currentAxis->goToDir(REV, currentAxis->getStartSoftStop());
	}

	//Go to end soft stop
	else if (token == "goendsoftstop") {
		debug("Go End Soft Stop!");
		currentAxis->goToDir(FWD, currentAxis->getEndSoftStop());
	}

	//Delete  start soft stop
	else if (token == "deletestartsoftstop") {
		debug("Delete Start Soft Stop!");
		currentAxis->setStartSoftStop(0L);
	}

	//Delete end soft stop
	else if (token == "deleteendsoftstop") {
		debug("Delete End Soft Stop!");
		currentAxis->setEndSoftStop(0L);
	}


	//Set Keyframe command
	else if (token == "setkeyframe") {
		debug("Set Keyframe!");

	}

	//Go Keyframe command
	else if (token == "gokeyframe") {
		debug("Go Keyframe!");

	}
}

///--------------------------------
// Initialise keyframe sequence from datastore
//
// Datastore scheme:
// axis1.numberofkeyframes
// axis1.kf1.position
// axis1.kf1.speed
//
///--------------------------------
int initializeKeyframeSequence(Axis* axis) {
	int axisNumber = axis->getAxisNumber();

	//Getting number of keyframes
	//Constructing datastore command string
	String command = "axis" + String(axisNumber) + ".numberofkeyframes";

	//Convert command string to char array
	command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
	debug(commandBuffer);
	//Get values from datastore
	//Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);

	//Convert value char buffer to String
	String valueString = (String)valueBuffer;

	//Convert value string to int
	int numberOfKeyframes = valueString.toInt();
	axis->setNumberOfKeyframes(numberOfKeyframes);
	debug(String(numberOfKeyframes));

	for (int i = 1; i <= numberOfKeyframes; i++) {
		//read keyframe position
		command = "axis" + String(axisNumber) + ".kf" + String(i) + ".position";
		command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
		debug(commandBuffer);

		//Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
		String positionString = (String)valueBuffer;
		unsigned long position = positionString.toInt();
		debug(String(position));

		//read keyframe speed
		command = "axis" + String(axisNumber) + ".kf" + String(i) + ".speed";
		command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
		debug(commandBuffer);
		//Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
		String speedString = (String)valueBuffer;
		float speed = speedString.toFloat();
		debug(String(speed));

		//read keyframe accelleration
		command = "axis" + String(axisNumber) + ".kf" + String(i) + ".acc";
		command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
		debug(commandBuffer);
		//Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
		String accString = (String)valueBuffer;
		float acc = accString.toFloat();
		debug(String(acc));

		//read keyframe deceleration
		command = "axis" + String(axisNumber) + ".kf" + String(i) + ".dec";
		command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
		debug(commandBuffer);
		//Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
		String decString = (String)valueBuffer;
		float dec = decString.toFloat();
		debug(String(dec));

		axis->getKeyframe(i)->setPosition(position);
		axis->getKeyframe(i)->setSpeed(speed);
		axis->getKeyframe(i)->setAcc(acc);
		axis->getKeyframe(i)->setDec(dec);
	}
	return 0;
}

/// --------------------------
// parseCommand
// Returns next command as String, or "" if no command left
// The parsed token is removed from the String
/// --------------------------
String parseCommand(String *commandString)
{
	String command = "";

	//Does commandString contain more tokens separated by "/"?
	int pos = commandString->indexOf('/');
	if (pos != -1) { //Yes
	  //Extract command token
		command = commandString->substring(0, commandString->indexOf("/"));

		//Remove token from commandString
		*commandString = commandString->substring((commandString->indexOf("/") + 1));
	}
	else { //No
		command = *commandString;
		*commandString = "";
	}

	return command;
}

/// --------------------------
// debug
// Print debug text
/// --------------------------
void debug(String message)
{
	Serial.println(message);
}
