
void setup() {

	//Enable MIPS
	pinMode(32, OUTPUT);
	digitalWrite(32, HIGH);

	pinMode(13, OUTPUT);
}

void loop() {
	digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);              // wait for a second
	digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);
}