//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Traffic Detection - Side Firing Algorithm
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//=========================
// SFR - Side Firing Radar
//=========================

#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include "environ.h"

#define N	FFT_OUTPUT_ARRAY_SIZE

sfrDataType sfrData[MAX_NUMBER_OF_TARGETS_TRACKED];

//=================================================================================================
// This function could be rewritten using the standard fftOutputArray.
//=================================================================================================
static boolean hasBeenInitialized = FALSE;
void _sideFiringAlgorithm(void) {
	#define MIN_CONFIDENCE	2
	#define MAX_CONFIDENCE	10
	int i;

	if (!hasBeenInitialized) {
		memset(sfrData, 0, sizeof(sfrData));
		hasBeenInitialized = TRUE;
	}

	for (i=0; i < MAX_NUMBER_OF_TARGETS_TRACKED; i++) {
		if ((systemData.targetTracker[i].index != INVALID_VEHICLE_ENTRY) &&
			(systemData.targetTracker[i].index > MIN_INDEX) &&
			(systemData.targetTracker[i].magnitude > MIN_MAGNITUDE) &&
			(systemData.targetTracker[i].trackCounter > MIN_TRACK)) {
			switch (sfrData[i].state) {
			case SFR_INITIAL_STATE:
				sfrData[i].state = SFR_WAITING_FOR_VEHICLE;
			case SFR_WAITING_FOR_VEHICLE:
				if (systemData.targetTracker[i].trackCounter > 2) {
					sfrData[i].index = systemData.targetTracker[i].index; 
					sfrData[i].magnitude = systemData.targetTracker[i].magnitude; 

					sfrData[i].state = SFR_FOUND_VEHICLE;
				}
				break;
			case SFR_FOUND_VEHICLE:
				// Looking for an increase in magnitude and a decrease in searchIndex
				sfrData[i].index = systemData.targetTracker[i].index; 
				if (sfrData[i].index <= sfrData[i].index_z) {
					if (sfrData[i].confidence.index < MAX_CONFIDENCE) {
						sfrData[i].confidence.index++;
					}
				} else {
					if (sfrData[i].confidence.index > 0) {
						sfrData[i].confidence.index--;
					}
				}

				sfrData[i].magnitude = systemData.targetTracker[i].magnitude; 
				if (sfrData[i].magnitude >= sfrData[i].magnitude_z) {
					if (sfrData[i].confidence.magnitude < MAX_CONFIDENCE) {
						sfrData[i].confidence.magnitude++;
					}
				} else {
					if (sfrData[i].confidence.magnitude > 0) {
						sfrData[i].confidence.magnitude--;
					}
				}

				if ((sfrData[i].confidence.index > MIN_CONFIDENCE) &&
					(sfrData[i].confidence.magnitude > MIN_CONFIDENCE)) {
					sfrData[i].state = SFR_TRACKING_TOWARDS;
					systemData.statistics.counter++;
				}
				break;
			case SFR_TRACKING_TOWARDS:
				// Wait for searchIndex to approach zero
				if (sfrData[i].index <= CUTOFF_INDEX) {
					sfrData[i].state = SFR_DIRECTLY_IN_FRONT;
					systemData.statistics.counter++;
				}
				break;
			case SFR_DIRECTLY_IN_FRONT:
				// The signal may be very high and jump around when the target is passing by the radar
				sfrData[i].confidence.index = 0;
				sfrData[i].confidence.magnitude = 0;
				if (sfrData[i].index > CUTOFF_INDEX) {
					sfrData[i].state = SFR_TRACKING_AWAY;
				}
				break;
			case SFR_TRACKING_AWAY:
				// Looking for an decrease in magnitude and an increase in searchIndex
				sfrData[i].index = systemData.targetTracker[i].index; 
				if (sfrData[i].index >= sfrData[i].index_z) {
					if (sfrData[i].confidence.index < MAX_CONFIDENCE) {
						sfrData[i].confidence.index++;
					}
				} else {
					if (sfrData[i].confidence.index > 0) {
						sfrData[i].confidence.index--;
					}
				}

				sfrData[i].magnitude = systemData.targetTracker[i].magnitude; 
				if (sfrData[i].magnitude <= sfrData[i].magnitude_z) {
					if (sfrData[i].confidence.magnitude < MAX_CONFIDENCE) {
						sfrData[i].confidence.magnitude++;
					}
				} else {
					if (sfrData[i].confidence.magnitude > 0) {
						sfrData[i].confidence.magnitude--;
					}
				}

				if ((sfrData[i].confidence.index > MIN_CONFIDENCE) &&
					(sfrData[i].confidence.magnitude > MIN_CONFIDENCE)) {
					sfrData[i].state = SFR_TRACKING_TOWARDS;
				}
				break;
			case SFR_PROCESS_FOUND_VEHICLE_DATA:
				systemData.statistics.counter++;
				sfrData[i].state = SFR_DONE;
				// Fall through on purpose
			case SFR_DONE:
				sfrData[i].state = SFR_WAITING_FOR_VEHICLE;
				break;
			}
		} else {
			sfrData[i].state = SFR_WAITING_FOR_VEHICLE;
			sfrData[i].confidence.index = 0;
			sfrData[i].confidence.magnitude = 0;
		}
		sfrData[i].index_z = sfrData[i].index;
		sfrData[i].magnitude_z = sfrData[i].magnitude;
	}
}

//=================================================================================================
#define K_HZ_PER_MPH		72.083
#define K_HZ_PER_KPH		44.7903
#define K_HZ_PER_MPS		49.1475
#define K_HZ_PER_FPS		161.2453
#define KA_HZ_PER_MPH		105.9
#define SPEED_GAIN			(1/K_HZ_PER_MPH)

#define	SAMPLE_RATE_KHZ			44100
#define SAMPLE_RATE_KHZ_LONG	44100l
#define GAIN_ADJUSTMENT			1.0
#define INDEX_PER_HZ			GAIN_ADJUSTMENT*(SAMPLE_RATE_KHZ/(FFT_OUTPUT_ARRAY_SIZE*2))
#define FREQUENCY_GAIN			INDEX_PER_HZ
#define	FREQUENCY_OFFSET		40.0
#define	SPEED_OFFSET			0.0

void _findFrequency(int index) {
	float	result,
			signalLevel,
			temporary,
			presentValue,
			previousValue1,
			previousValue2,
			previousValue3,
			nextValue1,
			nextValue2,
			nextValue3;
	float	differenceArray[6];
	float	ACsignalLevel,
			ACsignalLevel_z;

	if ((index > 0) && (index < (N - 3))) {
		presentValue	= fftData.fftOutputArray[index];

		previousValue1	= fftData.fftOutputArray[index - 1];
		previousValue2	= fftData.fftOutputArray[index - 2];
		previousValue3	= fftData.fftOutputArray[index - 3];
		nextValue1		= fftData.fftOutputArray[index + 1];
		nextValue2		= fftData.fftOutputArray[index + 2];
		nextValue3		= fftData.fftOutputArray[index + 3];

		// Sum differences * filter constant. K values must add to 1.0
#ifdef OLD_VERSION
		differenceArray[0] = -(presentValue - previousValue3) * 0.125;	// 1/8
		differenceArray[1] = -(presentValue - previousValue2) * 0.25;	// 1/4
		differenceArray[2] = -(presentValue - previousValue1) * 0.5;	// 1/2
		differenceArray[3] = (presentValue - nextValue1) * 0.5;			// 1/2
		differenceArray[4] = (presentValue - nextValue2) * 0.25;		// 1/4
		differenceArray[5] = (presentValue - nextValue3) * 0.125;		// 1/8
#else
		differenceArray[0] = -(presentValue - previousValue3) * 1.0/16;
		differenceArray[1] = -(presentValue - previousValue2) * 1.0/8;
		differenceArray[2] = -(presentValue - previousValue1) * 1.0/4;
		differenceArray[3] = (presentValue - nextValue1) * 1.0/4;
		differenceArray[4] = (presentValue - nextValue2) * 1.0/8;
		differenceArray[5] = (presentValue - nextValue3) * 1.0/16;
#endif

		// Find index
		result		=	differenceArray[0] +
						differenceArray[1] +
						differenceArray[2] +
						differenceArray[3] +
						differenceArray[4] +
						differenceArray[5];
		temporary	= result/presentValue;
		result		= index - temporary;

		// Calculate frequency
		systemData.frequency.value = (FREQUENCY_GAIN * result) + FREQUENCY_OFFSET;

		// Calculate speed
		systemData.speed.value = SPEED_GAIN * systemData.frequency.value;

	} else {
		systemData.frequency.value				= 0.0;
		systemData.speed.value					= 0.0;
		systemData.frequency.FFTsignalLevel		= 0.0;
	}
}

/*---- End Of File ----*/
