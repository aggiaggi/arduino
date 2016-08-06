#include <Process.h>
#include <Bridge.h>
#include <Console.h>
#include <SPI.h>
#include "Axis.h"

//Buffer REST commands
#define COMMANDBUFFERLENGTH 128
char commandBuffer[COMMANDBUFFERLENGTH];
//Buffer for REST values
#define VALUEBUFFERLENGTH 255
char valueBuffer[VALUEBUFFERLENGTH];


//Storing axis information

//Autodriver setup
Axis axis1(2, 4, 3);
Axis axis2(6, 8, 7);
Axis axis3(10, 12, 11);

Axis* currentAxis = &axis1;
int counter = 0;

Process nodejs;    // make a new Process for calling Node

void setup() {
  
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
  axis1.setAccKVAL(10);
  axis1.setDecKVAL(10);
  axis1.setRunKVAL(10);
  axis1.setHoldKVAL(0);//17
  axis1.setParam(INT_SPD, 0);
  axis1.setParam(ST_SLP, 52);
  axis1.setParam(FN_SLP_ACC, 85);
  axis1.setParam(FN_SLP_DEC, 85);
  
 
  axis1.setAxisNumber(1);
  axis1.configStepMode(STEP_FS_64); //Step mode 1/64
  
  //Wantai 28BYGH105 24V
  axis2.setAccKVAL(56); //28
  axis2.setDecKVAL(56);
  axis2.setRunKVAL(56);
  axis2.setHoldKVAL(56);
  axis2.setParam(INT_SPD, 37382);
  axis2.setParam(ST_SLP, 27);
  axis2.setParam(FN_SLP_ACC, 30);
  axis2.setParam(FN_SLP_DEC, 30);
  
  axis2.setAxisNumber(2);
  axis2.configStepMode(STEP_FS_64); //Step mode 1/64

  //Wait for Linux
  Serial.begin(115200); //or set it to an appropriate value (same value as set in device manager on windows)).
  Serial1.begin(250000); 

  //Check if Linux is booting
  setupUnix(2000,20000); // void setupUnix(initTimeout, bootTimeout)
  
  // Bridge startup
  Bridge.begin();
  Console.begin();
  //while(!Console);
  //Console.println("Console ready ...");
  //Console.println("OC Shutdown: " + String(axis1.getOCShutdown()));
  //Console.println("OC Threshold: " + String(axis1.getOCThreshold()));

  //Timer1.stop();
  //Timer1.initialize(100000); // set timer in microseconds
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  
  while(!nodejs.running()) {
    debug(F("Starting websocket server ..."));
    nodejs.runShellCommandAsynchronously("node /mnt/sda1/arduino/webapp/server/websockets.js");
  }
  debug(F("Websocket server started ..."));
}

void loop() {
  //Communication from arduino to node server
  //String data pos1':" + String(axis1.getPos()) + "}";
  String data = 
    "{\"pos1\":" + String(axis1.getPos()) + ",\"pos2\":" + String(axis2.getPos()) + ",\"pos3\":" + String(axis3.getPos()) + "}";
  int len = data.length();
  data.toCharArray(valueBuffer, len+1);
  if (nodejs.running()) {
    for (int i=0; i<len; i++) {
      nodejs.write(valueBuffer[i]);
    }
    //debug(valueBuffer);
    nodejs.write('\n');
  }
  

  //Communication from node server to arduino
  String commandString = "";
  char c;
  while (nodejs.available()) {
    c = nodejs.read();
    Console.write(c);
    commandString += String(c);
  }

  if (commandString != "") {
    process(commandString);
  }
  
  //Update axes
  if (axis1.getMotionState() != Axis::STOPPED )
    axis1.controlKeyframeSequence();
  else if (axis1.getMotionState() != Axis::MANUAL) {
    byte dir = axis1.getDirection();
    long pos = axis1.getPos();
    if ((dir == FWD && pos >= axis1.getEnd()) || (dir == REV && pos <= axis1.getStart()) ) 
    {
      axis1.hardStop();
      debug("Axis Hard Stop!");
    }
  }

  if (axis2.getMotionState() != Axis::STOPPED )
    axis2.controlKeyframeSequence();

  // Poll every 100ms
  delay(100);

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
  if (token == "start") {
    debug("Start!");
    int result = initializeKeyframeSequence(&axis1);
    result += initializeKeyframeSequence(&axis2);
    if (result == 0) {
      axis1.startKeyframeSequence();
      axis2.startKeyframeSequence();
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

    return;
  }
  
  else if (token == "1")
    currentAxis = &axis1;
  else if (token == "2")
    currentAxis = &axis2;
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
      if  (runSpeed < 0) {
        runSpeed = -1.0 * runSpeed;
        dir = REV;
      }
    }
    debug("Run dir " + String(dir) + " / Speed " + String(runSpeed));
    currentAxis->AutoDriver::run(dir, runSpeed);
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

  //Mark start command
  else if (token == "markstart") {
    debug("Mark Start!");
    currentAxis->markStart();
  }

  //Mark end command
  else if (token == "markend") {
    debug("Mark End!");
    currentAxis->markEnd();
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
  Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);

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

    Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
    String positionString = (String)valueBuffer;
    unsigned long position = positionString.toInt();
    debug(String(position));

    //read keyframe speed
    command = "axis" + String(axisNumber) + ".kf" + String(i) + ".speed";
    command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
    debug(commandBuffer);
    Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
    String speedString = (String)valueBuffer;
    float speed = speedString.toFloat();
    debug(String(speed));

    //read keyframe accelleration
    command = "axis" + String(axisNumber) + ".kf" + String(i) + ".acc";
    command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
    debug(commandBuffer);
    Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
    String accString = (String)valueBuffer;
    float acc = accString.toFloat();
    debug(String(acc));

    //read keyframe deceleration
    command = "axis" + String(axisNumber) + ".kf" + String(i) + ".dec";
    command.toCharArray(commandBuffer, COMMANDBUFFERLENGTH);
    debug(commandBuffer);
    Bridge.get(commandBuffer, valueBuffer, VALUEBUFFERLENGTH);
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
  if (pos  != -1) { //Yes
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
  Console.println(message);
}



/// --------------------------
// Booting
// 
/// --------------------------
//This function Loops untill all boot data is processed and bootTimeout is reached.
// http://forum.arduino.cc/index.php?topic=306674.0

void loopWhileBooting()
{
   bool useConsole = false;
   do //loop as long as we are in console mode. (enter console with enterkey. Exit console by sendeing a tilde (~)
   {
         //Send data from USB to UART
         int c = -1;
         c = Serial.read();
         if (c != -1) { 
               useConsole |= (c == 13); //enter console mode (13 = enter key)
               useConsole &= !(c == '~' ); //exit console mode
              if (useConsole) Serial1.write(c);
         }
       
         //Send data from UART to USB
         c = Serial1.read();
         if (c != -1) { Serial.write(c); } 
        
        //loops until bootTime timed-out.
       // if ((millis() - ledBlinkTime) > 500) //every ~500ms
       // { 
      //    digitalWrite(bootLed, !digitalRead(bootLed)); //toggle led D13 (Blink)
       //   ledBlinkTime = millis();
      //  }        
    } while (useConsole);
}

void setupUnix(unsigned int initTimeOut, unsigned int bootTimeOut)
{
  long unsigned initTimes[] = { millis(), millis() }; //set timers
  do { //init timeout    
    if(Serial1.available() > 0) { //run the next boot timeout block only when data is available
      while (Serial1.available() > 0 || ((initTimes[1] + bootTimeOut) > millis())) { //boot Timeout         
       if (Serial1.available() > 0) { initTimes[1] = millis();} //Flag boot timer        
          loopWhileBooting(); //process serial data (or just loop until timeout is reached)
      } //end while bootTimer    
      initTimes[0] = millis(); //Flag init Timer
    } //end if
  } while (Serial1.available() > 0 || ((initTimes[0] + initTimeOut) > millis())); //serial data available within initTimeOut? (cold-boot = data / Arduino Reset = no data)
}


