#include <SPI.h>
#include "Axis.h"
#include "ArduinoJson.h"

//Buffer for commands
#define COMMANDBUFFERLENGTH 512
char commandBuffer[COMMANDBUFFERLENGTH];
//Buffer for REST values
#define VALUEBUFFERLENGTH 255
char valueBuffer[VALUEBUFFERLENGTH];

//Dynamic JSON Buffer
DynamicJsonBuffer  jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& jsonAxis1 = root.createNestedObject("axis1");
JsonObject& jsonAxis2 = root.createNestedObject("axis2");
JsonObject& jsonAxis3 = root.createNestedObject("axis3");

//Autodriver setup
//position, cs, reset, busy
Axis axis1(0, 3, 2, 4); //3. motor
Axis axis2(1, 3, 2, 4); //2. motor
Axis axis3(2, 3, 2, 4); //Tian shield motor

Axis* currentAxis = &axis1;

int counter = 0;

void setup() {

	//Setup I/O pins
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);

	//Setup Driver Interface
	pinMode(2, OUTPUT); //Reset
	pinMode(3, OUTPUT); //CS
	pinMode(4, INPUT); //Busy

	//Set CS High
	digitalWrite(3, HIGH);

	//Reset all axes
	digitalWrite(2, LOW);
	digitalWrite(2, HIGH);

	//Setup Axis 1
	axis1.setAxisNumber(1);
	axis1.configStepMode(STEP_FS_64); //Step mode 1/64

	//Wantai 28BYGH105 24V
	axis1.setAccKVAL(20); //28
	axis1.setDecKVAL(20);
	axis1.setRunKVAL(20);
	axis1.setHoldKVAL(20);
	axis1.setParam(INT_SPD, 37382);
	axis1.setParam(ST_SLP, 27);
	axis1.setParam(FN_SLP_ACC, 30);
	axis1.setParam(FN_SLP_DEC, 30);

	
	//Setup Axis 2
	axis2.setAxisNumber(2);
	axis2.configStepMode(STEP_FS_64); //Step mode 1/64

	//Wantai 28BYGH105 24V
	axis2.setAccKVAL(20); //40
	axis2.setDecKVAL(20);
	axis2.setRunKVAL(20);
	axis2.setHoldKVAL(20);
	axis2.setParam(INT_SPD, 37382);
	axis2.setParam(ST_SLP, 27);
	axis2.setParam(FN_SLP_ACC, 30);
	axis2.setParam(FN_SLP_DEC, 30);

	//Setup Axis 3
	axis3.setAxisNumber(3);
	axis3.configStepMode(STEP_FS_64); //Step mode 1/64

	//Motech 24V
	axis3.setAccKVAL(10); //40
	axis3.setDecKVAL(10);
	axis3.setRunKVAL(5);
	axis3.setHoldKVAL(0);
	axis3.setParam(INT_SPD, 4806);
	axis3.setParam(ST_SLP, 59);
	axis3.setParam(FN_SLP_ACC, 95);
	axis3.setParam(FN_SLP_DEC, 95);
	

	 // Set up serial port between MCU and MIPS
	SerialUSB.begin(400000);
	//Set up serial console port for debugging
	Serial.begin(115200);

	debug("Axes initialisation finished ...");
}

void loop() {

	//Send JSON data to node server
	//debug(String(axis1.getPos()));
	//debug(String(axis2.getPos()));

	jsonAxis1["pos"] = axis1.getPos();
	jsonAxis1["stop1"] = axis1.getStartSoftStop();
	jsonAxis1["stop2"] = axis1.getEndSoftStop();
	
	jsonAxis2["pos"] = axis2.getPos();
	jsonAxis2["stop1"] = axis2.getStartSoftStop();
	jsonAxis2["stop2"] = axis2.getEndSoftStop();

	jsonAxis3["pos"] = axis3.getPos();
	jsonAxis3["stop1"] = axis3.getStartSoftStop();
	jsonAxis3["stop2"] = axis3.getEndSoftStop();

	root.printTo(SerialUSB);

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

		axis2.hardHiZ();
		axis2.stopKeyframeSequence();

		axis3.hardHiZ();
		axis3.stopKeyframeSequence();

		return;
	}

	else if (token == "1")
		currentAxis = &axis1;
	else if (token == "2")
		currentAxis = &axis2;
	else if (token == "3")
		currentAxis = &axis3;
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
