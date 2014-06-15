//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Environ.h
//
// Common include file for this project
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------  

#include "Arduino.h"

//#define USE_DATALOGGING

//#define USE_FFT_256	// 345 FFT's/second
#ifndef USE_FFT_256
	#define USE_FFT_1024	// 69 FFT's/second
#endif

typedef struct {
	int millisecond;
	int displayCounter;
	int simulationCounter;
} timerType;

#ifdef GLOBAL
	timerType timer;
#else
	extern timerType timer;
#endif

//-------------------------------------------------------------------------------------------------
#define CARRIAGE_RETURN  '\n'
#define LINEFEED_CHAR    '\r'
typedef unsigned char boolean;
typedef unsigned char byte;
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned int U32;

typedef enum {
  PASS,
  FAIL = -1,
  EMPTY_BUFFER = -2,
} ErrorCodeIntType;

//-------------------------------------------------------------------------------------------------
typedef enum {
	TARGET_LOW,
	TARGET_TRACKING,
	TARGET_HIGH,
} targetStateType;

typedef enum {
	LOCK_BLANK,
	LOCK_VALID,
} lockStateType;

typedef struct {
	targetStateType targetState;
	float target;
	lockStateType lockState;
	float lock;
	float patrol;
	boolean direction;
} displayDataType;

#ifdef GLOBAL
	displayDataType displayData;
#else
	extern displayDataType displayData;
#endif

//-------------------------------------------------------------------------------------------------
#ifdef _WIN32
	// A couple of things to make Windows compiling go more smoothly
	#define cregister
	#define interrupt
	#define disableInterrupts()
	#define enableInterrupts()
#endif

#define POWERUP_TIMER	500

#define LEFT_CHANNEL   0
#define RIGHT_CHANNEL  1
#define NUMBER_OF_CHANNELS 2

// For serialProtocolUpdate() function
enum {
	SEND_ZERO_VALUES_PER_ZERO_SUPPRESS_SETTING,
	ALWAYS_SEND_ZERO_VALUES,
	POLLED_RESPONSE};

// Include The Following Definition Files:
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#ifndef FALSE
	#define FALSE				0
	#define TRUE				1
	#define	NOT_TRUE_OR_FALSE	2	// Rarely used
#endif

// The default serial number
#define	INVALID_SERIAL_NUMBER	0

// Commonly defined macros
#define SETBIT(ADDRESS,BIT)		(ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT)	(ADDRESS &= ~(1<<BIT))
#define CHECKBIT(ADDRESS,BIT)	(ADDRESS & (1<<BIT))

//-------------------------------------------------------------------------------------------------
// Type definitions
//-------------------------------------------------------------------------------------------------
typedef unsigned char	boolean;
//typedef unsigned short	word;
typedef unsigned char	Uint8;
typedef unsigned int    Uint16;

//===================
// Global Data Types
//===================

typedef struct {
	int counter;
} statisticsType;

typedef struct {
	int index;								// fftOutputArray index
	float magnitude;
	float magnitude_z;
	int	direction;
	int	directionCounter;
	float	theta[NUMBER_OF_CHANNELS];
	float	theta_z[NUMBER_OF_CHANNELS];
	float	deltaTheta;						// The difference between the left and right theta's
	float	deltaTheta_z;
	float	deltaDeltaTheta;				// The change in the delta theta
	boolean directionIsLocked;
	int trackCounter;						// Increments when a vehicle is found again.
	int deltaIndex;							// The difference between the present and previous track indexes
	struct {
		int direction;						// Increments when direction is good, decrements when bad
		int acceleration;					// Increments when the present speed is tracking well with the previous speed
		int magnitude;						// Increments when the magnitude is high enough to use
		int magnitudeTrack;					// Increments when the present magnitude is tracking well with the previous magnitude
	} confidence;
} targetTrackingStructureType;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#define MAX_NUMBER_OF_TARGETS_TRACKED	4	// Was 10 before 2007-03-13
typedef struct {
	struct {
		int serialPortCommandReceived : 1;
		int waitForFFTcompletion : 1;
		int waitForFFTcompletion_z : 1;
		int systemIsUpAndReady : 1;	
		int initializationIsComplete : 1;
		int ramOnly : 1;
		int lowPowerMode : 1;
		int radarIsTransmitting : 1;
		int testMode : 1;			// Enables more detailed debugging data through the serial port
	} flags;

	// Used for min/max speeds - to avoid screwing up the Strong/Fast decision
	int minimumIndex;
	int maximumIndex;

	targetTrackingStructureType targetTracker[MAX_NUMBER_OF_TARGETS_TRACKED];
	int numberOfOldTargetsFound;
	int numberOfNewTargetsFound;
	long targetFFTIndex;			// The position in the vehicle tracker array of the speed we are displaying

	statisticsType statistics;

	struct {
		float value;
	} speed;
	struct {
		float FFTsignalLevel;
		float value;
	} frequency;


	// Test Values - delete later
	float filteredNoiseFloor[NUMBER_OF_CHANNELS];		// Tracks the noise floor level - for testing until proven!
	float	speedFormatConversionMultiplier;

	int fftsPerSecond;
} systemDataType;

//===================
// GPIO Definitions
//===================
#define DIGITAL_IO		0
#define PERIPHERAL_IO	1
#define INPUT_PIN		0
#define OUTPUT_PIN		1

#define OFF				0
#define ON				1
#define TOGGLE			2
#define PRESSED			0
#define NOT_PRESSED		1
#define HIGH			1
#define LOW				0

#define FORCE_RADAR_ON		{														\
		systemData.flags.radarIsTransmitting = TRUE;								\
		transitionToNormalOperatingMode();											\
	}
#define FORCE_RADAR_OFF		{														\
		systemData.flags.radarIsTransmitting = FALSE;								\
		transitionToLowPowerMode();													\
	}
#define RADAR_ON		{															\
	if (systemData.flags.radarIsTransmitting != TRUE) {								\
		systemData.flags.radarIsTransmitting = TRUE;								\
		transitionToNormalOperatingMode();											\
	}}
#define RADAR_OFF		{															\
	if (systemData.flags.radarIsTransmitting != FALSE) {							\
		systemData.flags.radarIsTransmitting = FALSE;								\
		transitionToLowPowerMode();													\
	}}

#define PI 3.141592654

//-------------------------------------------------------------------------------------------------
// Function Declarations that don't belong anywhere else
//-------------------------------------------------------------------------------------------------
extern ErrorCodeIntType processCommands(void);
extern systemDataType systemData;

// Includes at the end to support arduino
#include "VehicleTracker.h"
#include "serialPort.h"
//#include "ansicode.h"

/*********************************** End of File ******************************************************/

