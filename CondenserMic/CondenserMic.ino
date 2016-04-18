/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED

uint32_t prevPrintTime;
uint32_t prevSampleTime;
int bufIdx;
uint16_t buffer[512];
uint32_t totalSamples;

// state for updating extrema over a window ...
uint16_t maxOver10,minOver10,maxOver100,minOver100;
int maxOver10Idx, minOver10Idx, maxOver100Idx, minOver100Idx;

#define indexDistance(cur,old,sz) ((cur)>=(old))?((cur)-(old)):((sz)-(old)+(cur))
#define decIndex(idx,sz) (idx)?((idx)-1):((sz)-1)

void FindExtremaOverWindow(uint16_t* pmax, int* maxIdx, uint16_t* pmin, int* minIdx, int windowSz);

void setup() {
  // declare the ledPin as an OUTPUT:
  //pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

  // initialize state variables for timing, sampling ...
  prevPrintTime = millis();
  prevSampleTime = micros();
  bufIdx = 0;
  buffer[bufIdx++] = maxOver10 = minOver10 = maxOver100 = minOver100 = analogRead(sensorPin);
  totalSamples = 1; // this controls how far we can search on start up ...

  // initialize extrema indecies ...
  maxOver10Idx=minOver10Idx=maxOver100Idx=minOver100Idx=0;
}

void loop() {

  uint32_t loopStart_ms = millis();
  uint32_t loopStart_us = micros();
  uint32_t sampleValue;

  if (loopStart_us-prevSampleTime > 2000U) {

    // update sample buffer ...
    buffer[bufIdx++] = analogRead(sensorPin); totalSamples++;
    if (bufIdx >= sizeof(buffer)) bufIdx = 0;

    FindExtremaOverWindow(&maxOver10,  &maxOver10Idx,  &minOver10,  &minOver10Idx,   10);
    FindExtremaOverWindow(&maxOver100, &maxOver100Idx, &minOver100, &minOver100Idx, 100);
    
    prevSampleTime = loopStart_us;
  }

  if (loopStart_ms-prevPrintTime > 1000U) {
    Serial.print("min/max over 10  "); Serial.print(minOver10);  Serial.print(" "); Serial.print(maxOver10);  Serial.print("\n");
    Serial.print("min/max over 100 "); Serial.print(minOver100); Serial.print(" "); Serial.print(maxOver100); Serial.print("\n");
    prevPrintTime = loopStart_ms;
  }
}

//
// This function is called when the *current* sample has been placed in the buffer.
// That means that bufIdx-1 (logically) is where the current sample resides and
// it also defines where the desired window of samples are in the buffer.
//
void
FindExtremaOverWindow(uint16_t* pmax, int* maxIdx, uint16_t* pmin, int* minIdx, int windowSz) {

  int windowCounter,checkIdx;
  uint16_t currentValue;

  // UPDATE MAX ....
  // first check to see if the current extrema have rolled out
  // of the current window ...
  currentValue = buffer[checkIdx];
  checkIdx = bufIdx ? (bufIdx-1) : (sizeof(buffer)-1);
  
  if (indexDistance(bufIdx,*maxIdx, sizeof(buffer))>windowSz) {
    // search for the new maxima ...
    int newMaxIdx = checkIdx; // currentValue is already set above to match.
    for (windowCounter = windowSz; windowCounter--;) {
      checkIdx = decIndex(checkIdx,sizeof(buffer));
      if (buffer[checkIdx]>currentValue) {
        currentValue = buffer[checkIdx];
        newMaxIdx = checkIdx;
      }
    }

    *pmax = currentValue;
    *maxIdx = newMaxIdx;
    
  } else {
    if (currentValue>*pmax) {
      *pmax = currentValue;
      *maxIdx = checkIdx;
    }
  }


  // UPDATE MIN ....
  // first check to see if the current extrema have rolled out
  // of the current window ...
  currentValue = buffer[checkIdx];
  checkIdx = bufIdx ? (bufIdx-1) : (sizeof(buffer)-1);
  
  if (indexDistance(bufIdx,*minIdx, sizeof(buffer))>windowSz) {
    // search for the new maxima ...
    int newMinIdx = checkIdx; // currentValue is already set above to match.
    for (windowCounter = windowSz; windowCounter--;) {
      checkIdx = decIndex(checkIdx,sizeof(buffer));
      if (buffer[checkIdx]<currentValue) {
        currentValue = buffer[checkIdx];
        newMinIdx = checkIdx;
      }
    }

    *pmin = currentValue;
    *minIdx = newMinIdx;
    
  } else {
    if (currentValue<*pmin) {
      *pmin = currentValue;
      *minIdx = checkIdx;
    }
  }
  return;
}

