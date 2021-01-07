/*
  Reading a serial ASCII-encoded string.

  This sketch demonstrates the Serial parseInt() function.
  It looks for an ASCII string of comma-separated values.
  It parses them into ints, and uses those to fade an RGB LED.

  Circuit: Common-Cathode RGB LED wired like so:
  - red anode: digital pin 3
  - green anode: digital pin 5
  - blue anode: digital pin 6
  - cathode: GND

  created 13 Apr 2012
  by Tom Igoe
  modified 14 Mar 2016
  by Arturo Guadalupi

  This example code is in the public domain.
*/

// pins for the LEDs:
const int output1 = 3;
const int output2 = 5;

int RXLED = 17;

int period = 0;

void setup() {
  // initialize serial:
  Serial.begin(115200);
  // make the pins outputs:
  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  pinMode(RXLED, OUTPUT);

}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int value = Serial.parseInt();

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      period = value;
      Serial.print(value);
      Serial.println("  |  Ack.");
    }
  }

  if(period > 0)
  {

  digitalWrite(output1, HIGH);
  digitalWrite(output2, HIGH);
  digitalWrite(RXLED, HIGH);
  delay(1);
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  delay(int(period/2));
  digitalWrite(RXLED, LOW);
  delay(int(period/2)-1);
    
  }
  
}
