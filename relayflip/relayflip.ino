/*
  Relay flip
 */


// the setup function runs once when you press reset or power the board
void setup() {
  for (int i=2; i<=5; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
}

int index = 0;

// the loop function runs over and over again forever
void loop() {
  
  int pin = index + 2;
  digitalWrite(pin, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(pin, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  index = (index+1)%4;
}
