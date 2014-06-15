//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Vehicle Tracking
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

#ifndef SLOPE_H
#define SLOPE_H

#ifdef USE_FFT_1024
	#define FFT_OUTPUT_ARRAY_SIZE		512
#else
	#define FFT_OUTPUT_ARRAY_SIZE		128
#endif

#define DEFAULT_MINIMUM_MAGNITUDE	50.0
#define MAX_TOWARDS_PHASE_DELTA		15.0
#define MIN_TOWARDS_PHASE_DELTA		1.0
#define MAX_AWAY_PHASE_DELTA		15.0
#define MIN_AWAY_PHASE_DELTA		1.0
#define MINIMUM_MAGNITUDE_FROM_SENSITIVITY_ARRAY  2
#define PHASE_OFFSET				1.0
#define MAXIMUM_DIRECTION_COUNTER	1
#define UNKNOWN_DIRECTION			0
#define TOWARDS						1
#define AWAY						2

// Side-firing
#define MIN_INDEX					5
#define CUTOFF_INDEX				10	// Count a vehicle if it tracks lower than this
#define MIN_TRACK					10
#define MIN_MAGNITUDE				100

#ifdef SVR_COMPILE
	#define FLOAT_MINIMUM_PEAK_DELTA	1.0		// The shallowest slope on the FFT data that we'll recognize
	#define	FLOAT_MAXIMUM_PEAK_DELTA	100.0	// The upper end of the above
#else
	#ifdef	USE_HIGH_GAIN_AMPLIFIER
		#define FLOAT_MINIMUM_PEAK_DELTA	0.50	// The shallowest slope on the FFT data that we'll recognize
		#define	FLOAT_MAXIMUM_PEAK_DELTA	3000.0	// The upper end of the above
	#else
		#define FLOAT_MINIMUM_PEAK_DELTA	0.05	// The shallowest slope on the FFT data that we'll recognize
		#define	FLOAT_MAXIMUM_PEAK_DELTA	100.0	// The upper end of the above
	#endif
#endif

#define MINIMUM_PEAK_DELTA			_IQ(FLOAT_MINIMUM_PEAK_DELTA)	// The shallowest slope on the FFT data that we'll recognize
#define	MAXIMUM_PEAK_DELTA			_IQ(FLOAT_MAXIMUM_PEAK_DELTA)	// The upper end of the above
#define MAX_DELTA_SEARCH			5
#define MAX_SLOPE_COUNTER			2
#define MAGNITUDE_DIVIDER_FOR_WIDTH	200000
#define SAMPLE_START_LOCATION		1	//Start bin. Adjust for very low frequency built-in noise
#define INVALID_VEHICLE_ENTRY		0
#define	DIRECTION_COUNTER_DELTA		5

#define MAX_CONFIDENCE_LEVEL	20
#define MIN_CONFIDENCE_LEVEL	5

//-------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------
// Used in FFTProcessing only for filtering theta
extern targetTrackingStructureType targetTracker[MAX_NUMBER_OF_TARGETS_TRACKED];

typedef struct {
	boolean initialized;
	boolean busy;					// TRUE when the FFT process is in the middle of processing data
	boolean runProcess;				// Set True to signal FFT to run.  Set FALSE by calling function.
	int16_t fftOutputArray[FFT_OUTPUT_ARRAY_SIZE];
	int16_t fftOutputArray_z[FFT_OUTPUT_ARRAY_SIZE];
	int16_t fftOutputArrayNoise[FFT_OUTPUT_ARRAY_SIZE];
	boolean binIsUnderInvestigation[FFT_OUTPUT_ARRAY_SIZE];
	int	adcGainShift;
	float frequency[MAX_NUMBER_OF_TARGETS_TRACKED];
	int type;
	float amplitude[MAX_NUMBER_OF_TARGETS_TRACKED];
	float minimumMagnitude;
} fftStructType;

#ifdef GLOBAL
	fftStructType fftData;
#else
	extern fftStructType fftData;
#endif


//-------------------------------------------------------------------------------------------------
// BEGIN Definition of structure
//-------------------------------------------------------------------------------------------------
typedef struct {
	struct {
		int intitialized:1;
	} flags;
	void (*open)(void);				// Initialize the structure
	void (*reset)(void);
	void (*findNewTracks)(void);
	void (*processExistingTracks)(void);
	void (*sort)(void);
	void (*updateMinimumMagnitude)(void);
	void (*simulate)(int);
	void (*sideFiringAlgorithm)(void);
	void (*findFrequency)(int);
} targetTrackingType;

extern targetTrackingType targetTracking;

#define VEHICLE_TRACKING_STRUCT_DEFAULTS	\
{								\
	0,		/* flags */			\
	_open,						\
	_reset,						\
	_findNewTracks,				\
	_processExistingTracks,		\
	_sort,						\
	_updateMinimumMagnitude,	\
	_simulate,					\
	_sideFiringAlgorithm,		\
	_findFrequency,				\
}

// Tracking states
typedef enum {
	SFR_INITIAL_STATE,
	SFR_WAITING_FOR_VEHICLE,		// Passed initialization. Ready for work.
	SFR_FOUND_VEHICLE,				// In side-firing mode, vehicles are found first at a high speed
	SFR_TRACKING_TOWARDS,			// Vehicle is approaching. Magnitude is increasing.
	SFR_DIRECTLY_IN_FRONT,			// Vehicle is directly in front. This state may remain for a second or so depending on the length of the vehicle.
	SFR_TRACKING_AWAY,				// Vehicle has passed. Magnitude will decrease. Continue tracking until it is gone.
	SFR_PROCESS_FOUND_VEHICLE_DATA,	// Add to the datalog or display
	SFR_DONE,						// Tidy up. Return to waiting state.
} sfrTrackingStateType;

typedef struct {
	int index;
	int index_z;
	int magnitude;
	int magnitude_z;
	sfrTrackingStateType state;
	struct {
		int index;					// Increments when the index is changing in the correct direction, otherwise decrements
		int magnitude;				// Increments when the magnitude is changing in the correct direction, otherwise decrements
	} confidence;
} sfrDataType;

extern sfrDataType sfrData[MAX_NUMBER_OF_TARGETS_TRACKED];

extern void _slowlyZeroVehicleTrack(int);
extern void bringToZero(int *);
extern void _findNewTracks(void);
extern void _processExistingTracks(void);
extern void _sideFiringAlgorithm(void);
extern void _findFrequency(int);

#endif   /* #ifndef SLOPE_H */

/*********************************** End of File ******************************************************/
