#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "environ.h"

extern void millisecondTimer(void);

#ifdef USE_DATALOGGING
	// set up variables using the SD utility library functions:
	Sd2Card card;
	SdVolume volume;
	SdFile root;
	File testFile;
#endif

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
// Teensy 3.0: pin 10
// Audio Shield for Teensy 3.0: pin 10
const int chipSelect = 10;

// local functions
extern void updateSimulation(int);

#define DISPLAY_RATE_MS		25	// 100 works well
#define SIMULATE_RATE_MS	10

IntervalTimer myTimer;

//#define USE_INTERNAL	//2014-06-08 Internal doesn't work

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
#define FFT_LEVEL	1	// 1 is fastest
#ifdef USE_FFT_1024
	AudioAnalyzeFFT1024  myFFT(FFT_LEVEL);
#else
	AudioAnalyzeFFT256  myFFT(FFT_LEVEL);
#endif

#ifdef USE_INTERNAL
	AudioSynthWaveform sine0;
	AudioSynthWaveform sine1;
	AudioMixer4        mixer;
	AudioConnection c1(sine0, 0, mixer, 0);
	AudioConnection c2(sine1, 0, mixer, 1);
	AudioConnection c3(mixer, 0, myFFT, 0);
#else
	//const int myInput = AUDIO_INPUT_LINEIN;
	const int myInput = AUDIO_INPUT_MIC;

	// Create an object to control thfe audio shield.
	AudioControlSGTL5000 audioShield;

	AudioInputI2S       audioInput;         // audio shield: mic or line-in
	AudioOutputI2S      audioOutput;        // audio shield: headphones & line-out
	AudioConnection c1(audioInput, 0, myFFT, 0);
	AudioConnection c2(audioInput, 0, audioOutput, 0);
	AudioConnection c3(audioInput, 1, audioOutput, 1);
#endif

//#define SIMPLIFY_SETUP

//-------------------------------------------------------------------------------------------------
void setup() {
	// Audio connections require memory to work.  For more
	// detailed information, see the MemoryAndCpuUsage example
	AudioMemory(12);

	Serial.begin(115200);

#ifndef USE_INTERNAL
	// Enable the audio shield and set the output volume.
	audioShield.enable();
	audioShield.inputSelect(myInput);
	audioShield.inputLevel(100);	// Input
	audioShield.volume(100);		// Output
#endif

#ifndef SIMPLIFY_SETUP
		targetTracking.open();
		serialPort.open();
		//  target.open();

		memset(&displayData, 0, sizeof(displayData));
		randomSeed(analogRead(0));
		myTimer.begin(millisecondTimer, 1000);

		memset(&serialData, 0, sizeof(serialData));
		serialData.protocol = DEFAULT_PROTOCOL;

	#ifdef USE_INTERNAL
		// reduce the gain on mixer channels, so more than 1
		// sound can play simultaneously without clipping
		mixer.gain(0, 1.0);
		mixer.gain(1, 1.0);
	#endif

  		Serial.println("Board has been initialized");

	#ifdef USE_DATALOGGING
	#error Not Ready
		delay(2000);

		//--------------------------------------------------------------
		SPI.setMOSI(7); // Audio shield has MOSI on pin 7
		SPI.setSCK(14); // Audio shield has SCK on pin 14
   
		// Open serial communications and wait for port to open:
		Serial.begin(9600);

		Serial.print("\nInitializing SD card...");
		// On the Ethernet Shield, CS is pin 4. It's set as an output by default.
		// Note that even if it's not used as the CS pin, the hardware SS pin
		// (10 on most Arduino boards, 53 on the Mega) must be left as an output
		// or the SD library functions will not work.
		pinMode(chipSelect, OUTPUT); // change this to 53 on a mega

		// we'll use the initialization code from the utility libraries
		// since we're just testing if the card is working!
		if (!card.init(SPI_HALF_SPEED, chipSelect)) {
			Serial.println("initialization failed. Things to check:");
			Serial.println("* is a card is inserted?");
			Serial.println("* Is your wiring correct?");
			Serial.println("* did you change the chipSelect pin to match your shield or module?");
			return;
		} else {
			Serial.println("Wiring is correct and a card is present.");
		}

		// print the type of card
		Serial.print("\nCard type: ");
		switch(card.type()) {
		case SD_CARD_TYPE_SD1:
			Serial.println("SD1");
			break;
		case SD_CARD_TYPE_SD2:
			Serial.println("SD2");
			break;
		case SD_CARD_TYPE_SDHC:
			Serial.println("SDHC");
			break;
		default:
			Serial.println("Unknown");
		}

		// Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
		if (!volume.init(card)) {
			Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
			return;
		}


		// print the type and size of the first FAT-type volume
		uint32_t volumesize;
		Serial.print("\nVolume type is FAT");
		Serial.println(volume.fatType(), DEC);
		Serial.println();
  
		volumesize = volume.blocksPerCluster(); // clusters are collections of blocks
		volumesize *= volume.clusterCount(); // we'll have a lot of clusters
		volumesize *= 512; // SD card blocks are always 512 bytes
		Serial.print("Volume size (bytes): ");
		Serial.println(volumesize);
		Serial.print("Volume size (Kbytes): ");
		volumesize /= 1024;
		Serial.println(volumesize);
		Serial.print("Volume size (Mbytes): ");
		volumesize /= 1024;
		Serial.println(volumesize);

  
		Serial.println("\nFiles found on the card (name, date and size in bytes): ");
		root.openRoot(volume);
  
		// list all files in the card with date and size
		root.ls(LS_R | LS_DATE | LS_SIZE);

		// open the file. note that only one file can be open at a time,
		// so you have to close this one before opening another.
		testFile = SD.open("test.txt", FILE_WRITE);

		// if the file is available, write to it:
		if (testFile) {
			testFile.println("This is a test");
			testFile.close();
		} else {
			// if the file isn't open, pop up an error:
			Serial.println("error opening \"test.txt\"");
		}

		delay(5000);
	#else
		delay(1000);
	#endif
#endif	// SIMPLIFY_SETUP
}

//-------------------------------------------------------------------------------------------------
volatile int fftCounter = 0;
boolean readyToPrint = FALSE;
void loop() {

#ifdef SIMPLIFY_SETUP
#error SIMPLIFY_SETUP
	if (myFFT.available()) {
		Serial.print("FFT Is Available: ");
		Serial.println(fftCounter);
		targetTracking.findNewTracks();
		targetTracking.processExistingTracks();
		targetTracking.sort();
		targetTracking.updateMinimumMagnitude();
		fftCounter++;
	}
#else
	#ifdef USE_INTERNAL
		//#define TRY_THIS
		#ifdef TRY_THIS
			sine0.begin(fftData.amplitude, fftData.frequency, fftData.type);
		#else
			fftData.amplitude[0] = 0.2;
			fftData.frequency[0] = 4567.0;
			fftData.amplitude[1] = 0.1;
			fftData.frequency[1] = 1234.0;
			sine0.begin(fftData.amplitude[0], fftData.frequency[0], fftData.type);
			sine1.begin(fftData.amplitude[1], fftData.frequency[1], fftData.type);
		#endif
	#endif
  
		if (myFFT.available()) {
			readyToPrint = TRUE;
			fftCounter++;

			targetTracking.findNewTracks();
			targetTracking.processExistingTracks();
			targetTracking.sort();
			targetTracking.updateMinimumMagnitude();
		} else {
			if (readyToPrint) {
				readyToPrint = FALSE;

				serialPort.monitor();
				serialPort.updateDisplay(DISPLAY_RATE_MS);
			}
		}

		if (timer.millisecond >= 1000) {
			timer.millisecond = 0;
			systemData.fftsPerSecond = fftCounter;
			fftCounter = 0;

			// Send something to the screen at least once per second
			readyToPrint = TRUE;
		}
#endif	// SIMPLIFY_SETUP
}

//-----------------
#define TARGET_GOAL  75.0
#define PATROL_GOAL  55.0
void updateSimulation(int updateRate_ms) {
	int randomNumber = random(0, 1000);
	static int lockCounter = 0;

	switch (displayData.targetState) {
	case TARGET_LOW:
		displayData.target += (float)updateRate_ms/200;
		if (displayData.target >= TARGET_GOAL) {
			displayData.targetState = TARGET_TRACKING;
		}
		break;
	case TARGET_TRACKING:
		displayData.target = 75 + randomNumber/500;
		if (displayData.target >= (TARGET_GOAL + 10.0)) {
			displayData.targetState = TARGET_HIGH;
		}
		break;
	case TARGET_HIGH:
		displayData.target -= updateRate_ms/10000;
		if (displayData.target < (TARGET_GOAL+1.0)) {
			displayData.targetState = TARGET_TRACKING;
		}
		break;
	}

	switch (displayData.lockState) {
		case LOCK_BLANK:
		displayData.lock = 0;
		if (lockCounter > 2000) {
			lockCounter = 0;
			displayData.lockState = LOCK_VALID;
			displayData.lock = displayData.target;
		}
		break;
	case LOCK_VALID:
		if (lockCounter > 4000) {
			lockCounter = 0;
			displayData.lockState = LOCK_BLANK;
		}
		break;
	}
	lockCounter+=updateRate_ms;

	if (displayData.patrol < PATROL_GOAL) {
		displayData.patrol += (float)updateRate_ms/250;
	} else {
		displayData.patrol = PATROL_GOAL + randomNumber/400;
	}

	if (randomNumber < 500) {
		displayData.direction = 0;
	} else {
		displayData.direction = 1;
	}
}

//------------------------
void millisecondTimer(void) {
	#ifndef SIMPLIFY_SETUP
		timer.millisecond++;
		timer.displayCounter++;
		timer.simulationCounter++;
		targetTracking.simulate(1);
	#endif
}

//-------------------------------------------------------------------------------------------------
void displayFFT(void) {
#ifdef USE_FFT_1024
    Serial.print("FFT1024, ");
#else
    Serial.print("FFT256, ");
#endif
    for (int i=0; i<FFT_OUTPUT_ARRAY_SIZE; i++) {
      Serial.print(myFFT.output[i]);
      Serial.print(", ");
    }
    Serial.println();
}

