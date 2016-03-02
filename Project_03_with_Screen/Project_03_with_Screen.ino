#include <LiquidCrystal.h>
#include <Average.h>

//LiquidCrystal lcd (12, 11, 8, 7, 6, 5);
LiquidCrystal lcd (12, 11, 5, 4, 3, 2);
const int sensorPin = A0;
const float baselineTemp = 20.0;

int readVcc()
// Calculate current Vcc in mV from the 1.1V reference voltage
{
  long result;

  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  
  return(result);
}

#include "wiring_private.h"
#include "pins_arduino.h"

//uint8_t analog_reference = DEFAULT;
extern uint8_t analog_reference;

//void analogReference(uint8_t mode)
//{
//	// can't actually set the register here because the default setting
//	// will connect AVCC and the AREF pin, which would cause a short if
//	// there's something connected to AREF.
//	analog_reference = mode;
//}

int myAnalogRead(uint8_t pin)
{
	uint8_t low, high;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)
	if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#elif defined(analogPinToChannel) && (defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__))
	pin = analogPinToChannel(pin);
#else
	if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif
	
#if defined(__AVR_ATmega32U4__)
	pin = analogPinToChannel(pin);
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	sbi(ADCSRA, ADSC);

        delay(10);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
        delay(1);
	high = ADCH;
#else
	// we dont have an ADC, return 0
	low  = 0;
	high = 0;
#endif

	// combine the two bytes
	return (high << 8) | low;
}

const int numberOfSamples = 20;
Average<float> samples(numberOfSamples);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.print("oh my");
  lcd.setCursor(0,1);
  lcd.print("oh yes");
  for (int pinNumber = 6; pinNumber <= 8; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int VccMv = readVcc();
  float vcc = VccMv / 1000.0;
  Serial.print("vcc: ");
  Serial.print(vcc);
  Serial.print(", ");
  
  int sensorVal = analogRead(sensorPin);
  //int sensorVal = myAnalogRead(sensorPin);
  //sensorVal = samples.rolling(sensorVal);
  
  //delay(2);
  //sensorVal = analogRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorVal);
  
  float voltage = (sensorVal/1024.0) * vcc;
  voltage = samples.rolling(voltage);
  
  Serial.print(", Volts: ");
  Serial.print(voltage);
  
  lcd.clear();
  lcd.setCursor(0,0);
//  lcd.print("V ");
//  lcd.print(voltage);
  
  float temperatureC = (voltage - .5) * 100;
  Serial.print(", degrees C: ");
  Serial.print(temperatureC);
  
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
  Serial.print(", degrees F: ");
  Serial.print(temperatureF);

  lcd.print("T: ");
  lcd.print(int(round(temperatureF)));

  Serial.println();
  
  if (temperatureC < baselineTemp+2) {
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  } else if (temperatureC >= baselineTemp+2 &&
    temperatureC < baselineTemp+4) {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    } else if (temperatureC >= baselineTemp+4 && temperatureC < baselineTemp+6){
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    } else {
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    }
  //delay(1);
  delay(500);
}
