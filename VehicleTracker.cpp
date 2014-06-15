//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Traffic Detection via Slope logic
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include "environ.h"

#define IGNORE_DIRECTION

#define START_FREQ			10.0
#define STOP_FREQ			20000.0

systemDataType systemData;
extern AudioAnalyzeFFT256 myFFT;

// Local Function Declarations
static void _open(void);				// Initialize the structure
static void _reset(void);
static boolean _trackTowardsDirection(int);
static boolean _trackAwayDirection(int);
static void _trackBothDirections(int);
static void _incrementDirectionConfidence(int);
static void _zeroVehicleTrack(targetTrackingStructureType *);
static void _sort(void);
static void _updateMinimumMagnitude(void);
static void _simulate(int);

targetTrackingType		targetTracking = VEHICLE_TRACKING_STRUCT_DEFAULTS;
targetTrackingStructureType	_targetTracker[MAX_NUMBER_OF_TARGETS_TRACKED];

#define NUMBER_OF_MAGNITUDE_LEVELS	5
const float magnitudeConfidenceArray[NUMBER_OF_MAGNITUDE_LEVELS] = {
	10.0,
	50.0,
	100.0,
	200.0,
	400.0
};

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
static void _open(void) {
	int i;

	targetTracking.reset();
	targetTracking.flags.intitialized = TRUE;

	memset(&fftData, 0, sizeof(fftData));
	fftData.frequency[0] = 0.0;  
	fftData.amplitude[0] = 0.0;
	fftData.frequency[1] = 0.0;  
	fftData.amplitude[1] = 0.0;
	fftData.type = TONE_TYPE_SINE;
	fftData.minimumMagnitude = DEFAULT_MINIMUM_MAGNITUDE;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
static void _reset(void) {
	// Clear out the vehicle tracking structure
	memset(systemData.targetTracker, 0, sizeof(systemData.targetTracker));
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void _processExistingTracks(void) {
#define THREE_MPH			4		// TBD - 3 mph/second max acceleration to be tracked
	boolean matchFound = FALSE;
	int
		i,
		startIndex,
		endIndex,
		deltaIndex,
		searchIndex,
		sampleIndex,
		maximumIndex;
	int16_t	temp1,
		temp2,
		result,
		maximum,
		value;

	// Clear out investigation list
	memset(fftData.binIsUnderInvestigation,	0,	sizeof(fftData.binIsUnderInvestigation));

	//---------------------------------------------------------------------------------------------
	// Loop through list of vehicle objects that are presently being tracked
	//---------------------------------------------------------------------------------------------
	systemData.numberOfOldTargetsFound = 0;
	for (searchIndex=0; searchIndex < MAX_NUMBER_OF_TARGETS_TRACKED; searchIndex++) {

//		// Don't check if this entry is invalid.  An index of zero is invalid.
//		if (systemData.targetTracker[searchIndex].index != INVALID_VEHICLE_ENTRY) {
		// Don't check if this entry is invalid or is already under investigation.  An index of zero is invalid.
		if ((systemData.targetTracker[searchIndex].index != INVALID_VEHICLE_ENTRY) &&
			!fftData.binIsUnderInvestigation[systemData.targetTracker[searchIndex].index])  {
			matchFound = FALSE;

			// Point to array position where previous vehicle was found
			sampleIndex = systemData.targetTracker[searchIndex].index;
			value		= fftData.fftOutputArray[sampleIndex];

			// Loop around sampleIndex to find if the peak is still in the same place or within a reasonable delta
			startIndex	= sampleIndex - MAX_DELTA_SEARCH;
			if (startIndex < SAMPLE_START_LOCATION) {
				startIndex = SAMPLE_START_LOCATION;
			}

			endIndex	= sampleIndex + MAX_DELTA_SEARCH;
			if (endIndex < FFT_OUTPUT_ARRAY_SIZE) {
				endIndex = FFT_OUTPUT_ARRAY_SIZE;
			}

			maximum					= 0;
			maximumIndex			= 0;
			for (i=startIndex;i<endIndex;i++) {
				value		= fftData.fftOutputArray[i];
				if (value > maximum) {
					maximum			= value;
					maximumIndex	= i;
				}
			}

			// Assign how much the peak moved
			deltaIndex = abs(systemData.targetTracker[searchIndex].index - maximumIndex);
			if ((maximum >= fftData.minimumMagnitude) && 

				// Drops off at 1/4 the minimum as determined by the sensitivityArray
				(maximum >= (MINIMUM_MAGNITUDE_FROM_SENSITIVITY_ARRAY*0.5)) &&

				(deltaIndex < (THREE_MPH+systemData.targetTracker[searchIndex].deltaIndex))) {
				matchFound = TRUE;

				if (systemData.targetTracker[searchIndex].trackCounter < 1000) {
					systemData.targetTracker[searchIndex].trackCounter++;
				}

				// Mark these bins so that other tracks don't find them.  Reuse startIndex and endIndex.
				for (i=startIndex;i<endIndex;i++) {
					fftData.binIsUnderInvestigation[i] = TRUE;
				}

				// Track how far this index was from the previous track index
				systemData.targetTracker[searchIndex].deltaIndex = maximumIndex - systemData.targetTracker[searchIndex].index;

				//-----------------------------------------------------------
				// Set new index
				//-----------------------------------------------------------
				systemData.targetTracker[searchIndex].index = maximumIndex;
				systemData.targetTracker[searchIndex].magnitude = maximum;
	
				//-----------------------------------------------------------
				// Set confidence counters
				//-----------------------------------------------------------

				//+++++++++++
				// Magnitude
				//+++++++++++
//				deltaValue	= abs(systemData.targetTracker[searchIndex].magnitude - maximum);

				// 25 percent of previous magnitude - magnitude is always a positive number
				value = systemData.targetTracker[searchIndex].magnitude * 0.25;

				if (systemData.targetTracker[searchIndex].confidence.magnitude < MAX_CONFIDENCE_LEVEL) {
					systemData.targetTracker[searchIndex].confidence.magnitude++;
				}

				//+++++++++++
				// Direction
				//+++++++++++
				#ifdef IGNORE_DIRECTION
					#undef SUPPORT_CONFIGURED_DIRECTIONS
				#else
					temp1	= systemData.targetTracker[searchIndex].theta[RIGHT_CHANNEL];
					temp2	= systemData.targetTracker[searchIndex].theta[LEFT_CHANNEL];
					if (temp2 > temp1) {
						result		= (temp1 + 1.0) - temp2;
					} else {
						result		= temp1 - temp2;
					}

					// Add calibrated phase offset then filter into deltaTheta
				#ifdef FILTER_DELTA_THETA
					temp1	= result + configuration.flash.phaseOffset;
					temp2	= ((temp1 - systemData.targetTracker[searchIndex].deltaTheta) * 0.25) + systemData.targetTracker[searchIndex].deltaTheta;
					systemData.targetTracker[searchIndex].deltaTheta = temp2;
				#else
					systemData.targetTracker[searchIndex].deltaTheta = result + PHASE_OFFSET;
				#endif

					// deltaDeltaTheta is filtered
					temp1	= systemData.targetTracker[searchIndex].deltaTheta - systemData.targetTracker[searchIndex].deltaTheta_z;
					temp2	= ((temp1 - systemData.targetTracker[searchIndex].deltaDeltaTheta) * 0.25) + systemData.targetTracker[searchIndex].deltaDeltaTheta;
					systemData.targetTracker[searchIndex].deltaDeltaTheta	= temp2;
				#endif


				#ifdef SUPPORT_CONFIGURED_DIRECTIONS
					switch (configuration.ram.targetReport) {
					case TARGET_APPROACH:
					case TARGET_TOWARDS:
						_trackTowardsDirection(searchIndex);
						break;
					case TARGET_RECEDE:
					case TARGET_AWAY:
						_trackAwayDirection(searchIndex);
						break;
					default:
						_trackBothDirections(searchIndex);
						break;
					}
				#else
					_trackBothDirections(searchIndex);
				#endif

				//---------------------------------------------------------------------------------
				// Lock or unlock the direction status
				//---------------------------------------------------------------------------------
// DWH TBD - integrate the following code where it makes more sense to have it
				if (!systemData.targetTracker[searchIndex].directionIsLocked) {
					if (systemData.targetTracker[searchIndex].confidence.direction >= (MAXIMUM_DIRECTION_COUNTER/4)) {
						if (systemData.targetTracker[searchIndex].directionCounter > 0) {
							systemData.targetTracker[searchIndex].direction = AWAY;
						} else if (systemData.targetTracker[searchIndex].directionCounter < 0) {
							systemData.targetTracker[searchIndex].direction = TOWARDS;
						} else {
							systemData.targetTracker[searchIndex].direction = UNKNOWN_DIRECTION;
						}
					} else {
						systemData.targetTracker[searchIndex].direction = UNKNOWN_DIRECTION;
					}
				} else {
					// Unlock direction if needed
					switch (systemData.targetTracker[searchIndex].direction) {
					case TOWARDS:
						if (systemData.targetTracker[searchIndex].directionCounter > (-MAXIMUM_DIRECTION_COUNTER/4)) {
							systemData.targetTracker[searchIndex].directionIsLocked = FALSE;
						}
						break;
					case AWAY:
						if (systemData.targetTracker[searchIndex].directionCounter < (MAXIMUM_DIRECTION_COUNTER/4)) {
							systemData.targetTracker[searchIndex].directionIsLocked = FALSE;
						}
						break;
					default:
						systemData.targetTracker[searchIndex].directionIsLocked = FALSE;
						break;
					}
				}

				systemData.targetTracker[searchIndex].deltaTheta_z	= systemData.targetTracker[searchIndex].deltaTheta;
				systemData.numberOfOldTargetsFound++;
			}

			//-------------------------------------------------------------------------------------
			if ((maximum < fftData.minimumMagnitude) || !matchFound) {
				// Vehicle not found.  Bring confidence counters to zero.
				_slowlyZeroVehicleTrack(searchIndex);
			}
		}
	}

	// Sort the tracking array
	targetTracking.sort();

	// 
	targetTracking.sideFiringAlgorithm();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// The last step of this function zero's out fftOutputArray so that a peak is only found once.
// This routine needs to be the last function called in the fft.process() function.
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void _findNewTracks(void) {
	boolean
		matchFound;
	int
		i,
		newVehicleIndex,
		oldVehicleIndex,
		startIndex,
		endIndex,
		deltaIndex,
		sampleIndex,
		maximumIndex;
	float
		maximum,
		value,
		previousValue;

	// Clear out the present vehicle tracker structure
	memset(_targetTracker,	0, sizeof(_targetTracker));
	for (i=0; i<FFT_OUTPUT_ARRAY_SIZE; i++) {
		fftData.fftOutputArray[i] = myFFT.output[i];
	}

	// Outer loop goes once for each peak found, looking for progressively smaller peaks
	for (newVehicleIndex=0; newVehicleIndex < MAX_NUMBER_OF_TARGETS_TRACKED; newVehicleIndex++) {
		maximum						= 0;
		maximumIndex				= 0;
		for (sampleIndex = SAMPLE_START_LOCATION; sampleIndex < FFT_OUTPUT_ARRAY_SIZE; sampleIndex++) {
			if (!fftData.binIsUnderInvestigation[sampleIndex]) {
				value = fftData.fftOutputArray[sampleIndex];
				if (value > maximum) {
					maximum			= value;
					maximumIndex	= sampleIndex;
				}
			}
		}

		// Only find new peaks that are above our minimum noise level and the sensitivity setting
		if ((maximum >= fftData.minimumMagnitude) &&
			(maximum >= MINIMUM_MAGNITUDE_FROM_SENSITIVITY_ARRAY)) {
			//-----------------------------------------------------------------------------------------
			// Peak found, now store it away
			//-----------------------------------------------------------------------------------------
			// Mark this area as being under investigation by first finding the whole peak - start to finish
			//-----------------------------------------------------------------------------------------
			_zeroVehicleTrack(&_targetTracker[newVehicleIndex]);				// Clear it out to start from scratch
			_targetTracker[newVehicleIndex].index						= maximumIndex;
			_targetTracker[newVehicleIndex].magnitude					= maximum;
			_targetTracker[newVehicleIndex].confidence.direction		= 0;
			_targetTracker[newVehicleIndex].confidence.acceleration	= 0;
			_targetTracker[newVehicleIndex].confidence.magnitude		= 2;	// Needs to be 2 to avoid immediate dropout
			_targetTracker[newVehicleIndex].confidence.magnitudeTrack	= 2;
			_targetTracker[newVehicleIndex].trackCounter				= 1;

			// Search from peak backwards
			previousValue	= maximum;
			matchFound		= FALSE;
			startIndex		= maximumIndex;
			for (i=maximumIndex-1; (i>=SAMPLE_START_LOCATION) && !matchFound; i--) {
				value = fftData.fftOutputArray[i];
				if (value > previousValue) {
					matchFound	= TRUE;
					startIndex	= i;
				}
				previousValue = value;
			}

			// Search from peak forwards
			previousValue	= maximum;
			matchFound		= FALSE;
			endIndex		= maximumIndex;
			for (i=maximumIndex+1; (i<FFT_OUTPUT_ARRAY_SIZE) && !matchFound; i++) {
				value = fftData.fftOutputArray[i];
				if (value > previousValue) {
					matchFound	= TRUE;
					endIndex	= i;
				}
				previousValue = value;
			}

			// Start and End index of present peak has been found.  Now mark it so we don't look at it the next time through.
			for (i=startIndex;i<=endIndex;i++) {
				// Mark these bins so that other tracks don't find them.  Reuse startIndex and endIndex.
				fftData.binIsUnderInvestigation[i] = TRUE;
			}
		} else {
			// No valid signal level is present
			break;
		}
	}

	//=============================================================================================
	//=============================================================================================
	// Done finding vehicles - now go through previous list to see if we've already seen these
	// vehicles.  If we've seen them before, then we've processed them already in the 
	// processExistingTracks() function.
	//=============================================================================================
	//=============================================================================================
	systemData.numberOfOldTargetsFound = 0;
	for (newVehicleIndex=0; newVehicleIndex < MAX_NUMBER_OF_TARGETS_TRACKED; newVehicleIndex++) {
		matchFound = FALSE;
		
		// Don't check old entry if the new entry is invalid.  An index of zero is invalid.
		if (_targetTracker[newVehicleIndex].index != INVALID_VEHICLE_ENTRY) {
			for (oldVehicleIndex=0; (oldVehicleIndex < MAX_NUMBER_OF_TARGETS_TRACKED) && (matchFound == FALSE); oldVehicleIndex++) {
				if (systemData.targetTracker[oldVehicleIndex].index != INVALID_VEHICLE_ENTRY) {
					deltaIndex = abs(systemData.targetTracker[oldVehicleIndex].index - _targetTracker[newVehicleIndex].index);
					if (deltaIndex < MAX_DELTA_SEARCH) {
						matchFound = TRUE;
						// Compare to 0, not fftData.minimumMagnitude, because we want everything at 
						// this stage of the search
						if (_targetTracker[newVehicleIndex].magnitude > 0) {
							systemData.numberOfOldTargetsFound++;
						}
						// Zero the new vehicle track so we don't use it when we save it.
						_zeroVehicleTrack(&_targetTracker[newVehicleIndex]);
					}
				}
			}

			// If we went all the way through the above list and didn't find a match, then we found a new vehicle.
			if (oldVehicleIndex >= MAX_NUMBER_OF_TARGETS_TRACKED) {
				systemData.numberOfNewTargetsFound++;
			}
		}
	}

	//=============================================================================================
	//=============================================================================================
	// We're done.  Save new vehicle tracks.
	//=============================================================================================
	//=============================================================================================
	for (newVehicleIndex=0; newVehicleIndex < MAX_NUMBER_OF_TARGETS_TRACKED; newVehicleIndex++) {
		if (_targetTracker[newVehicleIndex].index != INVALID_VEHICLE_ENTRY) {
			// Find an unused spot in the system vehicle tracking structure
			matchFound = FALSE;
			for (oldVehicleIndex=0; (oldVehicleIndex<MAX_NUMBER_OF_TARGETS_TRACKED) && (matchFound == FALSE); oldVehicleIndex++) {
				if (systemData.targetTracker[oldVehicleIndex].index == INVALID_VEHICLE_ENTRY) {
					matchFound = TRUE;
					memcpy(&systemData.targetTracker[oldVehicleIndex], &_targetTracker[newVehicleIndex], sizeof(targetTrackingStructureType));
				}
			}
			// No more room for new vehicles in the systemData.targetTracker structure
			if (matchFound == FALSE) {
				break;
			}
		}
	}

	targetTracking.sort();
}

//-------------------------------------------------------------------------------------------------
// Returns TRUE if we found a vehicle going the correct direction
//-------------------------------------------------------------------------------------------------
static boolean _trackTowardsDirection(int vehicleIndex) {
	boolean returnValue	= FALSE;

	// If Towards
	if ((systemData.targetTracker[vehicleIndex].deltaTheta <= MAX_TOWARDS_PHASE_DELTA) && 
		(systemData.targetTracker[vehicleIndex].deltaTheta >= MIN_TOWARDS_PHASE_DELTA)) {
		// We found the correct direction
		returnValue	= TRUE;
		if (systemData.targetTracker[vehicleIndex].directionCounter > -MAXIMUM_DIRECTION_COUNTER) {
			systemData.targetTracker[vehicleIndex].directionCounter--;
		} else {
			systemData.targetTracker[vehicleIndex].directionIsLocked = TRUE;
		}

		_incrementDirectionConfidence(vehicleIndex);
	} else {
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
	}
	return(returnValue);
}


//-------------------------------------------------------------------------------------------------
// Returns TRUE if we found a vehicle going the correct direction
//-------------------------------------------------------------------------------------------------
static boolean _trackAwayDirection(int vehicleIndex) {
	boolean returnValue	= FALSE;

	// If Away
	if ((systemData.targetTracker[vehicleIndex].deltaTheta <= MAX_AWAY_PHASE_DELTA) && 
		(systemData.targetTracker[vehicleIndex].deltaTheta >= MIN_AWAY_PHASE_DELTA)) {
		// We found the correct direction
		returnValue	= TRUE;
		if (systemData.targetTracker[vehicleIndex].directionCounter < MAXIMUM_DIRECTION_COUNTER) {
			systemData.targetTracker[vehicleIndex].directionCounter++;
		} else {
			systemData.targetTracker[vehicleIndex].directionIsLocked = TRUE;
		}

		_incrementDirectionConfidence(vehicleIndex);
	} else {
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
	}
	return(returnValue);
}


//-------------------------------------------------------------------------------------------------
// No return value
//-------------------------------------------------------------------------------------------------
static void _trackBothDirections(int vehicleIndex) {
#ifdef IGNORE_DIRECTION
	if (systemData.targetTracker[vehicleIndex].directionCounter < MAXIMUM_DIRECTION_COUNTER) {
		systemData.targetTracker[vehicleIndex].directionCounter++;
	} else {
		systemData.targetTracker[vehicleIndex].directionIsLocked = TRUE;
	}
	_incrementDirectionConfidence(vehicleIndex);
#else
	if ((systemData.targetTracker[vehicleIndex].deltaTheta <= MAX_TOWARDS_PHASE_DELTA) && 
		(systemData.targetTracker[vehicleIndex].deltaTheta >= MIN_TOWARDS_PHASE_DELTA)) {
		if (systemData.targetTracker[vehicleIndex].directionCounter > -MAXIMUM_DIRECTION_COUNTER) {
			systemData.targetTracker[vehicleIndex].directionCounter--;
		} else {
			systemData.targetTracker[vehicleIndex].directionIsLocked = TRUE;
		}

		_incrementDirectionConfidence(vehicleIndex);
	} else if ((systemData.targetTracker[vehicleIndex].deltaTheta <= MAX_AWAY_PHASE_DELTA) && 
			(systemData.targetTracker[vehicleIndex].deltaTheta >= MIN_AWAY_PHASE_DELTA)) {
		if (systemData.targetTracker[vehicleIndex].directionCounter < MAXIMUM_DIRECTION_COUNTER) {
			systemData.targetTracker[vehicleIndex].directionCounter++;
		} else {
			systemData.targetTracker[vehicleIndex].directionIsLocked = TRUE;
		}

		_incrementDirectionConfidence(vehicleIndex);
	} else {
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
		bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
		bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
	}
#endif
}


//-------------------------------------------------------------------------------------------------
// No return value
//-------------------------------------------------------------------------------------------------
static void _incrementDirectionConfidence(int vehicleIndex) {
	int i, count;

	count	= 2;	// Minimum count set here
	for (i=0; i<NUMBER_OF_MAGNITUDE_LEVELS; i++) {
		if (systemData.targetTracker[vehicleIndex].magnitude < magnitudeConfidenceArray[i]) {
			count = i+1;
			break;
		}
	}
	systemData.targetTracker[vehicleIndex].confidence.direction += count;

	if (systemData.targetTracker[vehicleIndex].confidence.direction > (MAXIMUM_DIRECTION_COUNTER)) {
		systemData.targetTracker[vehicleIndex].confidence.direction = (MAXIMUM_DIRECTION_COUNTER);
	}
}

//-------------------------------------------------------------------------------------------------
// No return value
//-------------------------------------------------------------------------------------------------
void _slowlyZeroVehicleTrack(int vehicleIndex) {
	float tempMagnitude;

	// Reduce magnitude by 1/8
	tempMagnitude = systemData.targetTracker[vehicleIndex].magnitude*0.125;
	systemData.targetTracker[vehicleIndex].magnitude -= tempMagnitude;

	// Twice for direction!
	bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
	bringToZero(&systemData.targetTracker[vehicleIndex].confidence.direction);
	bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);
	bringToZero(&systemData.targetTracker[vehicleIndex].directionCounter);

	bringToZero(&systemData.targetTracker[vehicleIndex].confidence.acceleration);
	bringToZero(&systemData.targetTracker[vehicleIndex].confidence.magnitude);
	bringToZero(&systemData.targetTracker[vehicleIndex].confidence.magnitudeTrack);

	if ((systemData.targetTracker[vehicleIndex].confidence.direction == 0) &&
		(systemData.targetTracker[vehicleIndex].confidence.acceleration == 0) &&
		(systemData.targetTracker[vehicleIndex].confidence.magnitude == 0) &&
		(systemData.targetTracker[vehicleIndex].confidence.magnitudeTrack == 0)) {

		// If we have no confidence, this is an invalid vehicle
		_zeroVehicleTrack(&systemData.targetTracker[vehicleIndex]);
	}
}


//-------------------------------------------------------------------------------------------------
static void _zeroVehicleTrack(targetTrackingStructureType *p) {
	memset(p, 0, sizeof(targetTrackingStructureType));
}


//-------------------------------------------------------------------------------------------------
// Sort by magnitude
//-------------------------------------------------------------------------------------------------
// WARNING!  This function uses the _targetTracker structure as a temporary storage location!
//-------------------------------------------------------------------------------------------------
static void _sort(void) {
	int destinationIndex,
		maximumIndex,
		vehicleIndex;
	float maximum;

	for (destinationIndex=0; destinationIndex<MAX_NUMBER_OF_TARGETS_TRACKED; destinationIndex++) {
		// Find maximum
		maximum				= 0;
		maximumIndex		= 0;
		for (vehicleIndex=0; vehicleIndex<MAX_NUMBER_OF_TARGETS_TRACKED; vehicleIndex++) {
			if (systemData.targetTracker[vehicleIndex].index != INVALID_VEHICLE_ENTRY) {
				if (systemData.targetTracker[vehicleIndex].magnitude > maximum) {
					maximum			= systemData.targetTracker[vehicleIndex].magnitude;
					maximumIndex	= vehicleIndex;
				}
			}
		}

		// Store it away
		memcpy(&_targetTracker[destinationIndex], &systemData.targetTracker[maximumIndex], sizeof(targetTrackingStructureType));

		// Zero it so we won't find it again
		_zeroVehicleTrack(&systemData.targetTracker[maximumIndex]);
	}

	// targetTracker is sorted, now write it back
	for (vehicleIndex=0; vehicleIndex<MAX_NUMBER_OF_TARGETS_TRACKED; vehicleIndex++) {
		memcpy(&systemData.targetTracker[vehicleIndex], &_targetTracker[vehicleIndex], sizeof(targetTrackingStructureType));
	}
}

//-------------------------------------------------------------------------------------------------
// Adjust minimum noise level based on the number of vehicle tracks.
//-------------------------------------------------------------------------------------------------
static void _updateMinimumMagnitude(void) {
#ifndef SVR_COMPILE	// Skip for SVR
	int i, counter;
	float decrement;

	counter = 0;
	for (i=0; i<MAX_NUMBER_OF_TARGETS_TRACKED; i++) {
		if (systemData.targetTracker[i].index != INVALID_VEHICLE_ENTRY) {
			counter++;
		}
	}

	// Increase the minimum magnitude if our vehicle tracks are maxed out
	if (counter>=(MAX_NUMBER_OF_TARGETS_TRACKED-2)) {
		// 50.0 represents the noisiest "good" antenna that we ever expect to have to work with
		if (fftData.minimumMagnitude < 50.0) {
			fftData.minimumMagnitude += 0.1;
		}
	}

	// Reduce minimum magnitude if less than half the max vehicle tracks
	if (counter == 0) {
		// Faster recovery than the other way.  Subtracts 1/8 of present minimum.
		decrement = fftData.minimumMagnitude*0.125;
		fftData.minimumMagnitude -= decrement;
	} else if (counter<(MAX_NUMBER_OF_TARGETS_TRACKED/2)) {
		if (fftData.minimumMagnitude > 0.0) {
			fftData.minimumMagnitude -= 0.1;
		}
	}
#endif
}

//-------------------------------------------------------------------------------------------------
void bringToZero(int *pValue) {
	int value;

	value = *pValue;
	if (value > 0) {
		value--;
		*pValue = value;
	}
}

//-------------------------------------------------------------------------------------------------
// Simulate side-firing location
//-------------------------------------------------------------------------------------------------
typedef enum {
	SIM_STARTUP,
	SIM_INCREASE_AMPLITUDE,
	SIM_DECREASE_FREQ_TO_ZERO,
	SIM_HOLD,
	SIM_INCREASE_FREQ,
	SIM_DECREASE_AMPLITUDE,
	SIM_DONE
} sideFiringSimulationEnumType;
sideFiringSimulationEnumType state;
#define MIN_AMPLITUDE	0.0
#define MAX_AMPLITUDE	0.5
#define AMPLITUDE_TIME	2000

#define START_FREQUENCY	8000.0
#define END_FREQUENCY	0.0
#define FREQUENCY_TIME	2000

#define HOLD_TIME		250
#define STARTUP_TIME	1000
#define DONE_TIME		2500
static void _simulate(int updateRate_ms) {
	static U16 delayCounter  = 0;
	int channel = 0;

	switch (state) {
	case SIM_STARTUP:
		fftData.frequency[channel] = 0.0;  
		fftData.amplitude[channel] = 0.0;
		delayCounter += updateRate_ms;
		if (delayCounter >= STARTUP_TIME) {
			state = SIM_INCREASE_AMPLITUDE;
		}
		break;
	case SIM_INCREASE_AMPLITUDE:
		fftData.frequency[channel] = START_FREQUENCY;  
		fftData.amplitude[channel] += (MAX_AMPLITUDE-MIN_AMPLITUDE)/AMPLITUDE_TIME;
		if (fftData.amplitude[channel] >= MAX_AMPLITUDE) {
			state = SIM_DECREASE_FREQ_TO_ZERO;
		}
		break;
	case SIM_DECREASE_FREQ_TO_ZERO:
		fftData.frequency[channel] -= (START_FREQUENCY-END_FREQUENCY)/FREQUENCY_TIME;
		if (fftData.frequency[channel] < 100) {
			fftData.amplitude[channel] -= (MAX_AMPLITUDE-MIN_AMPLITUDE)/100;
			if (fftData.amplitude[channel] <= 0) {
				fftData.amplitude[channel] = 0;
			}
		}
		if (fftData.frequency[channel] <= END_FREQUENCY) {
			state = SIM_HOLD;
			delayCounter = 0;
		}
		break;
	case SIM_HOLD:
		delayCounter += updateRate_ms;
		if (delayCounter >= HOLD_TIME) {
			state = SIM_INCREASE_FREQ;
		}
		break;
	case SIM_INCREASE_FREQ:
		fftData.frequency[channel] += (START_FREQUENCY-END_FREQUENCY)/FREQUENCY_TIME;
		fftData.amplitude[channel] += (MAX_AMPLITUDE-MIN_AMPLITUDE)/100;
		if (fftData.amplitude[channel] >= MAX_AMPLITUDE) {
			fftData.amplitude[channel] = MAX_AMPLITUDE;
		}
		if (fftData.frequency[channel] >= START_FREQUENCY) {
			state = SIM_DECREASE_AMPLITUDE;
		}
		break;
	case SIM_DECREASE_AMPLITUDE:
		fftData.amplitude[channel] -= (MAX_AMPLITUDE-MIN_AMPLITUDE)/AMPLITUDE_TIME;
		if (fftData.amplitude[channel] <=  MIN_AMPLITUDE) {
			state = SIM_DONE;
			delayCounter = 0;
		}
		break;
	case SIM_DONE:
		delayCounter += updateRate_ms;
		if (delayCounter >= DONE_TIME) {
			state = SIM_STARTUP;
		}
		break;
	}
}

/*---- End Of File ----*/
