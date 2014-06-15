  //-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// SerialPort 
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

#define GLOBAL
#include "environ.h"
#include "Arduino.h"
#include <SD.h>

File myFile;

// Local Function Declarations
static void _open(void);
static void _monitor(void);
static boolean _processInput(byte);
static int _getc(void);
static void _reset(void);
static void _updateDisplay(U16);
static void displayAnalog(void);
static void displayTracking(void);
static void displaySFR(void);
static void displaySFRstate(int);
extern void displayFFT(void);

serialPortType serialPort = SERIALPORT_DEFAULTS;

//-------------------------------------------------------------------------------------------------
// This function should fill a buffer and return instantly. If it sends too much data it will wait on the buffer to empty which is not correct operation.
//-------------------------------------------------------------------------------------------------
static void _updateDisplay(U16 updateRate_ms) {
	static protocolEnumType protocol_z = SP_NONE;
	int i;

	if (timer.displayCounter < updateRate_ms) {
		return;
	}
	timer.displayCounter = 0;

	if (protocol_z != serialData.protocol) {
		protocol_z = serialData.protocol;
		Serial.println("Protocol Changed");
	}

#ifdef SKIP_THIS
	Serial.print("[");
	Serial.print(systemData.fftsPerSecond);
	Serial.print("]");
#endif

	switch (serialData.protocol) {
	default:
	case SP_NONE:
		break;
	case SP_FFT_TIMING:
		Serial.print("FFTs per second: ");
		Serial.println(systemData.fftsPerSecond);
		break;
	case SP_SIMULATED:
		Serial.print("1:Target,");
		Serial.print("2:Lock,");
		Serial.print("3:Patrol,");
		Serial.print("4:"); Serial.print(displayData.target,0); Serial.print(",");
		Serial.print("5:"); Serial.print(displayData.lock,0); Serial.print(",");
		Serial.print("6:"); Serial.print(displayData.patrol,0); Serial.print(",");
		Serial.print("7:"); Serial.print(displayData.direction); Serial.print(",");
		Serial.println("8:Faster");
		break;
	case SP_ANALOG:
		displayAnalog();
		break;
	case SP_SFR:
		// Side Firing Radar Algorithm
		displaySFR();
		break;
	case SP_FFT:
		displayFFT();
		break;
	case SP_TRACKING:
		displayTracking();
		break;
	case SP_TRACKING_SIMULATION:
		Serial.print(fftData.frequency[0]);
		Serial.print(", ");
		Serial.print(fftData.amplitude[0]);
		Serial.print(", ");
		Serial.print(fftData.frequency[1]);
		Serial.print(", ");
		Serial.print(fftData.amplitude[1]);
		Serial.print(": ");
		for (i=0; (i<FFT_OUTPUT_ARRAY_SIZE) && (i<75); i++) {
			Serial.print(fftData.fftOutputArray[i]);
			Serial.print(" ");
		}
		Serial.println();

		#ifdef USE_DATALOGGING
			if (SD.exists("datalog.txt")) {
				Serial.println("datalog.txt exists.");
			} else {
				Serial.println("datalog.txt doesn't exist.");
			}

			// open the file. note that only one file can be open at a time,
			// so you have to close this one before opening another.
			myFile = SD.open("datalog.txt", FILE_WRITE);

			// if the file is available, write to it:
			if (myFile) {
				myFile.print(fftData.frequency);
				myFile.print(", ");
				myFile.print(fftData.amplitude);
				myFile.print(", ");
				for (i=0; (i<FFT_OUTPUT_ARRAY_SIZE) && (i<50); i++) {
					myFile.print(fftData.fftOutputArray[i]);
					myFile.print(" ");
				}
				myFile.println();
				myFile.close();
			} else {
				// if the file isn't open, pop up an error:
				Serial.println("error opening datalog.txt");
			}
		#endif
		break;
	case SP_DEBUG:
		break;
	}
}  
  
//-------------------------------------------------------------------------------------------------
static void _reset(void) {
	serialData.rx.head = 0;
	serialData.rx.tail = 0;
}

//-------------------------------------------------------------------------------------------------
static void _open(void) {
	memset(&serialData, 0, sizeof(serialData));
}

//-------------------------------------------------------------------------------------------------
static void _monitor(void) {
	byte value;
	boolean commandReceived;

	commandReceived = FALSE;
	while (Serial.available()) {
		commandReceived = TRUE;
		serialPort.processInput(Serial.read());
	}
	if (commandReceived) {
		commandReceived = FALSE;

		// clear simulation data
		memset(&displayData, 0, sizeof(displayData));

		processCommands();

#ifdef DISABLE
		while(value) {
			Serial.write(value);
			value = serialPort.read();
		};
		Serial.println();
#endif
	}
}

//-------------------------------------------------------------------------------------------------
static boolean _processInput(byte incomingByte) {
	int i;

	switch (incomingByte) {
	case CARRIAGE_RETURN:
	case LINEFEED_CHAR:
	case '.':
		return(TRUE);
	}

	switch (incomingByte) {
	case 'x':
		serialData.protocol = SP_NONE;
		break;
	case 'c':
		for (i=0; i<FFT_OUTPUT_ARRAY_SIZE; i++) {
			fftData.binIsUnderInvestigation[i] = FALSE;
		}
		Serial.println("BinInvestigation has been cleared");
		break;
	case 'b':
		for (i=0; i<25; i++) {
			if (fftData.binIsUnderInvestigation[i]) {
				Serial.print(i);
				Serial.print(".");
			}
		}
		Serial.println();
		break;
	default:
		serialData.rx.buffer[serialData.rx.head++] = incomingByte;
		if (serialData.rx.head >= RS232_BUFFER_SIZE) {
			serialData.rx.head = 0;
		}
		if (serialData.rx.head == serialData.rx.tail) {
			serialData.rx.tail++;
			if (serialData.rx.tail >= RS232_BUFFER_SIZE) {
				serialData.rx.tail = 0;
			}
		}
		break;
	}

	return(FALSE);
}

//-------------------------------------------------------------------------------------------------
static int _getc(void) {
	byte returnValue = NULL;

	if (serialData.rx.tail != serialData.rx.head) {
		returnValue = serialData.rx.buffer[serialData.rx.tail++];
		if (serialData.rx.tail >= RS232_BUFFER_SIZE) {
			serialData.rx.tail = 0;
		}
	}

	return(returnValue);
}

//-------------------------------------------------------------------------------------------------
static void displayAnalog(void) {
	int i;

	Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
	for (i=0; (i<FFT_OUTPUT_ARRAY_SIZE) && (i<45); i++) {
//	for (i=0; (i<FFT_OUTPUT_ARRAY_SIZE); i++) {
#ifdef OLD_VERSION
		Serial.print(fftData.fftOutputArray[i]);
		Serial.print(" ");
#else
		Serial.println(fftData.fftOutputArray[i]);
#endif
	}
	Serial.println("===========================================================");
}

//-------------------------------------------------------------------------------------------------
static void displayTracking(void) {
	int searchIndex;
	int targetsFound;
	static int counter = 0;

	targetsFound = 0;

	for (searchIndex=0; searchIndex < MAX_NUMBER_OF_TARGETS_TRACKED; searchIndex++) {
		if ((systemData.targetTracker[searchIndex].index != INVALID_VEHICLE_ENTRY) &&
			(systemData.targetTracker[searchIndex].index > MIN_INDEX) &&
			(systemData.targetTracker[searchIndex].magnitude > MIN_MAGNITUDE) &&
			(systemData.targetTracker[searchIndex].trackCounter > MIN_TRACK)) {

			if (targetsFound == 0) {
				Serial.print(counter++);
				Serial.print(", old:");
				Serial.print(systemData.numberOfOldTargetsFound);
				Serial.print(", ");
				if (counter >= 10) {
					counter = 0;
				}
			}

			targetsFound++;


			Serial.print(searchIndex);
			Serial.print(": ");

			Serial.print("Freq:");
			targetTracking.findFrequency(systemData.targetTracker[searchIndex].index);
			Serial.print(systemData.frequency.value, 1);
			Serial.print(", Speed:");
			Serial.print(systemData.speed.value, 1);

			Serial.print(", ");
			Serial.print(": I");
//			Serial.print("Freq:");
//			Serial.print(fftData.frequency[searchIndex],0);
//			Serial.print(", ");
			Serial.print(systemData.targetTracker[searchIndex].index);
			Serial.print(", M");
			Serial.print(systemData.targetTracker[searchIndex].magnitude, 0);
//			Serial.print(systemData.targetTracker[searchIndex].direction);
//			Serial.print(", ");
			Serial.print(", T");
			Serial.print(systemData.targetTracker[searchIndex].trackCounter);
			Serial.println();
		}
	}
#ifdef SKIP_THIS
	if (targetsFound == 0) {
		Serial.print("0: ");
		Serial.print(fftData.minimumMagnitude, 2);
		Serial.println();
	}
#endif
}

//-------------------------------------------------------------------------------------------------
// Side Firing Radar Algorithm
//-------------------------------------------------------------------------------------------------
static void displaySFR(void) {
	int index;
	static int counter_z = 0;
	boolean somethingWasDisplayed = FALSE;

	if (counter_z != systemData.statistics.counter) {
		counter_z = systemData.statistics.counter;
		Serial.print("Count[");
		Serial.print(systemData.statistics.counter);
		Serial.print("]  ");
		somethingWasDisplayed = TRUE;
	}

	for (index=0; (index<MAX_NUMBER_OF_TARGETS_TRACKED) && (sfrData[index].state > SFR_WAITING_FOR_VEHICLE); index++) {
		Serial.print(index);
		Serial.print(": ");
		displaySFRstate(index);
		Serial.print(" Index:");
		Serial.print(sfrData[index].confidence.index);
		Serial.print(" Magnitude:");
		Serial.print(sfrData[index].confidence.magnitude);
		Serial.print(".");
		Serial.print(systemData.targetTracker[index].index);
		Serial.print(", ");
		somethingWasDisplayed = TRUE;
	}
	if (somethingWasDisplayed) {
		Serial.println();
	}
}

//-------------------------------------------------------------------------------------------------
static void displaySFRstate(int index) {
	switch (sfrData[index].state) {
	case SFR_INITIAL_STATE:
		Serial.print("Initial State");
		break;
	case SFR_WAITING_FOR_VEHICLE:
		Serial.print("Waiting");
		break;
	case SFR_FOUND_VEHICLE:
		Serial.print("Found");
		break;
	case SFR_TRACKING_TOWARDS:
		Serial.print("Tracking Towards");
		break;
	case SFR_DIRECTLY_IN_FRONT:
		Serial.print("In Front");
		break;
	case SFR_TRACKING_AWAY:
		Serial.print("Tracking Away");
		break;
	case SFR_PROCESS_FOUND_VEHICLE_DATA:
		Serial.print("Processing");
		break;
	case SFR_DONE:
		Serial.print("Done");
		break;
	}
}
