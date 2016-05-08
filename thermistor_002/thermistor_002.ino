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
// the value of the 'other' resistor forming the voltage divider
#define SERIESRESISTOR 9870
// the output pin controlling the damper open/close relay
#define DAMPER_RELAY_PIN 2

// analog pin for room temp
#define TMPPIN A1

int samples[NUMSAMPLES];
float minread = 1024.0;
float maxread = 0.0;

// Set the desired room temperature to 70 degrees F (and convert to C)
float desiredTempC = (70.0 - 32.0) * 5 / 9;

// Potential states of the damper
#define DAMPER_UNKNOWN 0
#define DAMPER_OPEN 1
#define DAMPER_CLOSED 2

// Minimum amount of time, in milliseconds, permitted between damper (open/close) changes
#define MIN_DAMPER_STATE_CHANGE (60UL*1000UL)

// the current damper state
int damperState = DAMPER_UNKNOWN;

// Hysteresis -- number of positive, consecutive reading required before treated as fact
#define CONSECUTIVE_REQUESTS_REQUIRED 6
int consecutiveRequestsForOpen = 0;
int consecutiveRequestsForClose = 0;

// Time at which the damper was last instructed to change
unsigned long damperChangeStart = 0;

// HVAC actions that are observed
#define ACTION_UNKNOWN 0
#define ACTION_HEAT 1
#define ACTION_AC 2
int lastAction = ACTION_UNKNOWN;

// number of degrees C that the duct temp must exceed the desired room temp
// to be considered worth opening the duct
#define DUCT_TEMP_THRESHOLD 5

// Turn on auxiliar heat? (0 = no, 1 = yes)
int auxHeat = 0;

// Hysteresis -- number of positive, consecutive readings  requesting aux heat
int auxCount = 0;

// Minimum amount of time to keep aux hear on (to combat bouncing)
#define MIN_AUX_TIME (60L*1000L)

// Time at which aux heat was turned on
unsigned long auxOnStart = 0L;

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

void closeDamper()
{
  digitalWrite(DAMPER_RELAY_PIN, HIGH);
}

void openDamper()
{
  digitalWrite(DAMPER_RELAY_PIN, LOW);
}

void damperControl(int newDamperState)
{
  damperControl(newDamperState, 0);
}

void damperControl(int newDamperState, int actionObserved)
{
  damperState = newDamperState;

  newDamperState == DAMPER_OPEN ? openDamper() : closeDamper();
  if (actionObserved != 0) lastAction = actionObserved;
  damperChangeStart = millis();
}

void setup(void)
{
  Serial.begin(9600);
  analogReference(EXTERNAL);
  // Set the damper relay pin to output and set it HIGH, which means closed/off
  pinMode(DAMPER_RELAY_PIN, OUTPUT);
  digitalWrite(DAMPER_RELAY_PIN, HIGH);
  // Prepare the debug LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void turnDebugLightOn()
{
      digitalWrite(13, HIGH);
}

void turnDebugLightOff()
{
      digitalWrite(13, LOW);
}

float getDuctTemp()
{
  // reading the duct termistor can be noisy
  // so grab a few samples and average them
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

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  // compute the temperature via Steinhart-Hart equation
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert from K to C

  return steinhart;
}

float getRoomTemp()
{
  float vcc = readVcc() / 1000.0f;

  float reading = analogRead(TMPPIN);
  float voltage = (reading / 1024.0) * vcc;

  return (voltage - .5) * 100 + .5;
}

void loop(void)
{
  unsigned long loopStart = millis();

  float ductTempC = getDuctTemp();

  float roomTempC = getRoomTemp();
  float roomTempF = roomTempC * 9 / 5 + 32;

  auxHeat = 0;
  if ((millis() - damperChangeStart) >= MIN_DAMPER_STATE_CHANGE)
  {
    if (roomTempC < desiredTempC)
    {
      if ((lastAction == 1) && (roomTempC < desiredTempC - 1))
      {
        auxCount++;
        if (auxCount == CONSECUTIVE_REQUESTS_REQUIRED)
        {
          auxHeat = 1;
          auxOnStart = millis();
        }
      } else auxCount = 0;
      if ((ductTempC > roomTempC + DUCT_TEMP_THRESHOLD) && ((damperState == DAMPER_CLOSED) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForOpen++;
        if (consecutiveRequestsForOpen == CONSECUTIVE_REQUESTS_REQUIRED)
          damperControl(DAMPER_OPEN, ACTION_HEAT);
      }
      else if ((ductTempC <= roomTempC + DUCT_TEMP_THRESHOLD) && ((damperState == DAMPER_OPEN) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForClose++;
        if (consecutiveRequestsForClose == CONSECUTIVE_REQUESTS_REQUIRED)
          damperControl(DAMPER_CLOSED);
      }
      else
        consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;
    }
    else
    {
      if ((ductTempC < roomTempC - DUCT_TEMP_THRESHOLD) && ((damperState == DAMPER_CLOSED) || (damperState == DAMPER_UNKNOWN)))
      {
        consecutiveRequestsForOpen++;
        if (consecutiveRequestsForOpen == CONSECUTIVE_REQUESTS_REQUIRED)
          damperControl(DAMPER_OPEN, ACTION_AC);
      }
      else if ((ductTempC >= roomTempC - DUCT_TEMP_THRESHOLD) && (damperState == DAMPER_OPEN) || (damperState == DAMPER_UNKNOWN))
      {
        consecutiveRequestsForClose++;
        if (consecutiveRequestsForClose == CONSECUTIVE_REQUESTS_REQUIRED)
          damperControl(DAMPER_CLOSED);
      }
      else
        consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;
    }
  }
  else
    consecutiveRequestsForOpen = consecutiveRequestsForClose = 0;

  unsigned long loopExecutionTime = millis() - loopStart;
  if (loopExecutionTime < 1000) delay(1000 - loopExecutionTime);
}
