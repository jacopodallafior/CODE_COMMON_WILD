/*

* Axelboard Throttle Testing Script

* Target: Arduino Nano

* Function: Manual control of the Jimny Throttle via Serial Monitor

*/



const int throttlePin = 3; // PWM output to the Throttle DAC filter

const int enablePin = 6; // Relay/Enable pin to hijack the pedal signal



void setup() {

Serial.begin(115200);


pinMode(throttlePin, OUTPUT);

pinMode(enablePin, OUTPUT);



// Start with the system DISABLED for safety

digitalWrite(enablePin, LOW);

analogWrite(throttlePin, 0);



Serial.println("--- Jimny Axelboard Throttle Test ---");

Serial.println("Enter a value 0-100 to set throttle percentage.");

Serial.println("Example: '10' for a light rev, '0' to idle.");

}



void loop() {

if (Serial.available() > 0) {

// Read the incoming string and convert to integer

int inputVal = Serial.parseInt();



// Constrain input for safety (don't let a typo redline the engine)

inputVal = constrain(inputVal, 0, 50);



if (inputVal >= 0) {

// 1. Enable the board (Click the relay to hijack signal)

digitalWrite(enablePin, HIGH);



// 2. Map 0-100% to the specific PWM range (approx 0.7V to 4.3V)

// On an 8-bit Arduino (0-255), 0.7V is ~36 and 4.3V is ~219

int pwmOutput = map(inputVal, 0, 100, 36, 219);



analogWrite(throttlePin, pwmOutput);



// 3. Feedback to user

Serial.print("Throttle Set to: ");

Serial.print(inputVal);

Serial.println("%");

}


// Clear the buffer

while(Serial.available() > 0) Serial.read();

}

}
