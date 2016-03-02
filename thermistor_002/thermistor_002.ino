// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 9870

// analog pin for room temp
#define TMPPIN A1

int samples[NUMSAMPLES];
float minread = 1024.0;
float maxread = 0.0;

float desiredTempC = (70.0 - 32.0) * 5 / 9;

#define DAMPER_RELAY_PIN 2

#define DAMPER_UNKNOWN 0
#define DAMPER_OPEN 1
#define DAMPER_OPENING 2
#define DAMPER_CLOSING 3
#define DAMPER_CLOSED 4

#define DAMPER_TRANSITION_TIME (50UL*1000UL)
//#define DAMPER_TRANSITION_TIME (5UL*1000UL)
#define MIN_DAMPER_STATE_CHANGE (60UL*1000UL)

int damperState = DAMPER_UNKNOWN;
unsigned long transitionEndTime = 0;
int consecutiveRequestsForOpen = 0;
int consecutiveRequestsForClose = 0;

unsigned long noDamperChangeUntil = 0;

int lastAction = 0; // 0 = Unknonw, 1 = HEAT, 2 = AC
int auxHeat = 0;
int auxCount = 0;
#define MIN_AUX_TIME (60L*1000L)
unsigned long auxOnUntil = 0L;

int readVcc()
// Calculate current Vcc in mV from the 1.1V reference voltage
{
  long result;

  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV

  return (result);
}

void CloseDamper()
{
  digitalWrite(DAMPER_RELAY_PIN, HIGH);
}

void OpenDamper()
{
  digitalWrite(DAMPER_RELAY_PIN, LOW);
}

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  // Set the damper relay pin to output and set it HIGH, which means closed/off
  pinMode(DAMPER_RELAY_PIN, OUTPUT);
  digitalWrite(DAMPER_RELAY_PIN, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop(void) {
  if (damperState == DAMPER_OPENING || damperState == DAMPER_CLOSING)
  {
    if (millis() < transitionEndTime)
    {
      //      Serial.println("Damper transitioning...");
      //      Serial.println(millis());
      //      Serial.println(transitionEndTime);
      digitalWrite(13, HIGH);
      Serial.print(".");
      delay(1000);
      return;
    } else digitalWrite(13, LOW);
    damperState = (damperState == DAMPER_OPENING ? DAMPER_OPEN : DAMPER_CLOSED);
    Serial.println(damperState == DAMPER_OPEN ? "DAMPER OPEN" : "DAMPER CLOSED");
  }

  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  if (average < minread) minread = average;
  if (average > maxread) maxread = average;

  //  Serial.print("Average analog reading ");
  //  Serial.print(average);
  //  Serial.print(" [");
  //  Serial.print(minread);
  //  Serial.print(" - ");
  //  Serial.print(maxread);
  //  Serial.println("]");

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  //  Serial.print("Thermistor resistance ");
  //  Serial.print(average);

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  //  Serial.print("Duct temperature ");
  //  Serial.print(steinhart);
  //  Serial.print(" *C, ");
  //  Serial.print(steinhart * 9 / 5 + 32);
  //  Serial.println(" *F");

  float vcc = readVcc() / 1000.0f;
  //vcc = 3.3f;
  //  Serial.print("vcc: ");
  //  Serial.println(vcc);

  //  for (i=0; i< NUMSAMPLES; i++) {
  //   samples[i] = analogRead(TMPPIN);
  //   delay(100);
  //  }
  //
  //  // average all the samples out
  //  average = 0;
  //  for (i=0; i< NUMSAMPLES; i++) {
  //     average += samples[i];
  //  }
  //  average /= NUMSAMPLES;

  //int sensorVal = analogRead(TMPPIN);
  average = analogRead(TMPPIN);
  //  Serial.print("TMP Sensor: ");
  //  Serial.print(average);
  float voltage = (average / 1024.0) * vcc;
  //  Serial.print(", volts: ");
  //  Serial.println(voltage);

  float roomTempC = (voltage - .5) * 100 + .5;
  float roomTempF = roomTempC * 9 / 5 + 32;

  //  Serial.print("Desired temp: ");
  //  Serial.print(desiredTempC * 9 / 5 + 32);
  //  Serial.print(", Room temp: ");
  //  Serial.println(roomTempF);

  Serial.print(roomTempF);
  Serial.print(" [");
  Serial.print(desiredTempC * 9 / 5 + 32);
  Serial.print("/");
  Serial.print(steinhart * 9 / 5 + 32);
  Serial.print("] ");
  if (auxHeat || (millis() < auxOnUntil)) Serial.print(" AUX ");
  if (damperState == DAMPER_UNKNOWN) Serial.print("Unknown");
  if (damperState == DAMPER_OPEN) Serial.print("Open");
  if (damperState == DAMPER_OPENING) Serial.print("Opening");
  if (damperState == DAMPER_CLOSING) Serial.print("Closing");
  if (damperState == DAMPER_CLOSED) Serial.print("Closed");
  if (lastAction == 0) Serial.print(" ---- ");
  else if (lastAction == 1) Serial.print(" HEAT ");
  else Serial.print(" COOL ");
  auxHeat = 0;
  if (millis() > noDamperChangeUntil)
  {
    if (roomTempC < desiredTempC)
    {
      //Serial.print("Heat needed -- ");
      if ((lastAction == 1) && (roomTempC < desiredTempC - 1))
      {
        auxCount++;
        if (auxCount == 6)
        {
          auxHeat = 1;
          auxOnUntil = millis() + MIN_AUX_TIME;
        }
      } else auxCount = 0;
      if ((steinhart > roomTempC + 5) && ((damperState == DAMPER_CLOSED) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForOpen++;
        if (consecutiveRequestsForOpen == 6)
        {
          damperState = DAMPER_OPENING;
          OpenDamper();
          transitionEndTime = millis() + DAMPER_TRANSITION_TIME;
          lastAction = 1;
          Serial.println(" Open the damper!");
          noDamperChangeUntil = millis() + MIN_DAMPER_STATE_CHANGE;
        } else Serial.println(consecutiveRequestsForOpen);
      }
      else if ((steinhart <= roomTempC + 5) && ((damperState == DAMPER_OPEN) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForClose++;
        if (consecutiveRequestsForClose == 6)
        {
          damperState = DAMPER_CLOSING;
          CloseDamper();
          transitionEndTime = millis() + DAMPER_TRANSITION_TIME;
          Serial.println(" Close the damper!");
          noDamperChangeUntil = millis() + MIN_DAMPER_STATE_CHANGE;
        } else Serial.println(consecutiveRequestsForClose);
      }
      else
      {
        Serial.println("");
        consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;
      }
    }
    else
    {
      //Serial.print("AC needed -- ");
      if ((steinhart < roomTempC - 5) && ((damperState == DAMPER_CLOSED) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForOpen++;
        if (consecutiveRequestsForOpen == 6)
        {
          damperState = DAMPER_OPENING;
          OpenDamper();
          transitionEndTime = millis() + DAMPER_TRANSITION_TIME;
          lastAction = 2;
          Serial.println(" Open the damper!");
          noDamperChangeUntil = millis() + MIN_DAMPER_STATE_CHANGE;
        } else Serial.println(consecutiveRequestsForOpen);
      }
      else if ((steinhart >= roomTempC - 5) && (damperState == DAMPER_OPEN) || (damperState == DAMPER_UNKNOWN))
      {
        consecutiveRequestsForClose++;
        if (consecutiveRequestsForClose == 6)
        {
          damperState = DAMPER_CLOSING;
          CloseDamper();
          transitionEndTime = millis() + DAMPER_TRANSITION_TIME;
          Serial.println(" Close the damper!");
          noDamperChangeUntil = millis() + MIN_DAMPER_STATE_CHANGE;
        } else Serial.println(consecutiveRequestsForClose);
      }
      else
      {
        Serial.println("");
        consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;
      }
    }
  }
  else
  {
    consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;
    Serial.println("");
  }
  delay(1000);
}
