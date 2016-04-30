/*
  Based on: "Analog Input"

  Created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe
  modified 30 Apr 2016
  By Dave Ilstrup

 This example code is in the public domain.
 */

int gAttack, gDecay, gThreshold;

#define COEF 0.5
#define MEAN_VALUE 340

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED

uint32_t prevPrintTime;
uint32_t prevSampleTime,prevHiSampleTime;
int bufIdx;
uint16_t buffer[512];
uint32_t totalSamples;
uint16_t prevSampleValue, filteredValue;

// state for updating extrema over a window ...
uint16_t maxOver10,minOver10,maxOver100,minOver100;
int maxOver10Idx, minOver10Idx, maxOver100Idx, minOver100Idx;

#define indexDistance(cur,old,sz) ((cur)>=(old))?((cur)-(old)):((sz)-(old)+(cur))
#define decIndex(idx,sz) (idx)?((idx)-1):((sz)-1)

void FindExtremaOverWindow(uint16_t* pmax, int* maxIdx, uint16_t* pmin, int* minIdx, int windowSz);

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

  // initialize state variables for timing, sampling ...
  prevPrintTime = millis();
  prevSampleTime = micros();
  bufIdx = 0;
  buffer[bufIdx++] = maxOver10 = minOver10 = maxOver100 = minOver100 = analogRead(sensorPin);
  totalSamples = 1; // this controls how far we can search on start up ...

  prevSampleValue = maxOver10;
  filteredValue = MEAN_VALUE;

  // initialize extrema indecies ...
  maxOver10Idx=minOver10Idx=maxOver100Idx=minOver100Idx=0;
}


//
// The original code used different constants because
// of the input range of that sensor.
//
static int gZeroBias = 340;
static int IBI = 60;
static int BPM = 60;
static boolean Pulse = false;

// return non-zero if the BPM variable has been updated ...
int UpdateBeatFollower(uint16_t Signal) {

static int rate[10];                    // used to hold last ten IBI values
static unsigned long sampleCounter = 0; // used to determine pulse timing
static unsigned long lastBeatTime = 0;  // used to find the inter beat interval
static int P = gZeroBias;               // used to find peak in pulse wave
static int T = gZeroBias;               // used to find trough in pulse wave
static int thresh = gZeroBias;          // used to find instant moment of heart beat
static int amp = 400;                   // used to hold amplitude of pulse waveform
static boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
static boolean secondBeat = true;       // used to seed rate array so we startup with reasonable BPM

    sampleCounter += 2;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
    int ret = 0; // default is no update ...

//  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
   if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
    Pulse = true;                               // set the Pulse flag when we think there is a pulse
    digitalWrite(ledPin,HIGH);                // turn on pin 13 LED
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = false;                 // clear firstBeat flag
             return ret;                        // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = false;                 // clear secondBeat flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic BPM at startup
                    rate[i] = IBI;                      
                    }
            }
          
    // keep a running total of the last 10 IBI values
    word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the rate array
          rate[i] = rate[i+1];              // and drop the oldest IBI value 
          runningTotal += rate[i];          // add up the 9 oldest IBI values
        }
        
    rate[9] = IBI;                          // add the latest IBI to the rate array
    runningTotal += rate[9];                // add the latest IBI to runningTotal
    runningTotal /= 10;                     // average the last 10 IBI values 
    BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
    ret = 1; // new BPM ...
   }                       
  }

  if (Signal < thresh && Pulse == true){    // when the values are going down, the beat is over
      digitalWrite(ledPin,LOW);             // turn off pin 13 LED
      Pulse = false;                        // reset the Pulse flag so we can do it again
      amp = P - T;                          // get amplitude of the pulse wave
      thresh = amp/2 + T;                   // set thresh at 50% of the amplitude
      P = thresh;                           // reset these for next time
      T = thresh;
     }
  
  if (N > 2500){                            // if 2.5 seconds go by without a beat
      thresh = gZeroBias;                   // set thresh default
      P = gZeroBias;                        // set P default
      T = gZeroBias;                        // set T default
      lastBeatTime = sampleCounter;         // bring the lastBeatTime up to date        
      firstBeat = true;                     // set these to avoid noise
      secondBeat = true;                    // when we get the heartbeat back
     }

  return ret;
}

void loop() {

  uint32_t loopStart_ms = millis();
  uint32_t loopStart_us = micros();
  uint16_t sampleValue;

  // we've observed the mic run on 3.3V to produce average values around 340,
  // maximum values of 684, and minimum values of 0.

   // 20kHz sampling ...
  if (loopStart_us-prevHiSampleTime > 50U) {
    sampleValue = analogRead(sensorPin);
    filteredValue = (int)(COEF*sampleValue)+(int)((1.0-COEF)*filteredValue);
    prevHiSampleTime = loopStart_us;
  }

  if (loopStart_us-prevSampleTime > 2000U) {
    //Serial.print(filteredValue); Serial.print("\n");
    prevSampleTime = loopStart_us;

	// this is the rate at which the light sensor beat follower runs.
	if (UpdateBeatFollower(filteredValue)) {
    		Serial.print(BPM); Serial.print("\n");
	}
  }


  if (Serial.available()) {

	static int readCnt = 0;
	static int readState = 0;
	static String numbuf = ""; 

	char c = Serial.read();
	//
	// A is Attack
	// D is Decay
	// T is Threshold
	//
	if(readCnt == 0) {

		switch(c) {
		case 'A': case 'D': case 'T':
			readState = c;
			readCnt++;
		break;
		}

	} else {

		if (c == '\n') {
			readCnt = 0; // reset parser.
			int num = numbuf.toInt();

			switch(readState) {
			case 'A': gAttack = num;    break;
			case 'D': gDecay = num;     break;
			case 'T': gThreshold = num; break;
			}

		} else if (readCnt<8) {
			numbuf += c;
			readCnt++;
		} else readCnt = 0; // messed up, reset...
	}
  }

#if 0
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
#endif
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

