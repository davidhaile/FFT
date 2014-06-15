//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Speed Module
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

#ifndef SPEED_H
#define SPEED_H

#ifdef _WIN32
	#define	INVALID_SPEED		-1.0
	#define SPEED_LOCK_DELTA	1.0
#else
	#define	INVALID_SPEED		_IQ(-1.0)
	#define SPEED_LOCK_DELTA	_IQ(1.0)
#endif

#define SPEED_DELTA_LOCK_COUNT			14
#define SPEED_INCREASE_MAXIMUM_COUNT	28

typedef enum {
	SPEED_NOT_FOUND,
	SPEED_LOCK_IN_PROGRESS,
	SPEED_LOCKED
} speedLockStateType;

typedef enum {
	SPEED_PEAK_RUN_FREE,
	SPEED_PEAK_HOLD_REQUEST,
	SPEED_PEAK_HOLD
} speedHoldStateType;

//-------------------------------------------------------------------------------------------------
// BEGIN Definition of structure
//-------------------------------------------------------------------------------------------------
typedef struct {
	boolean initialized;
	boolean speedIsNotDecreasing;	// TRUE or FALSE
	int speedIncreaseCounter;
	int speedStableCounter;
	speedHoldStateType	holdState;
	int	timer;						// Incrementented in the timer interrupt.  Represents milliseconds.
	void (*open)(void);				// Initialize the structure
	void (*update)(int);
	void (*hold)(void);
	void (*processHoldState)(int);
	void (*updateSVRfilterAndDisplay)(void);
	void (*restartHoldState)(void);
	_iq	 (*averageForFinalDisplay)(void);
	_iq	k;
	_iq	delta;
	_iq	present;
	_iq	previous;
	_iq	filtered;
	_iq	displayed;
	_iq displayedSignalLevel;
	_iq	maximum;
	boolean positiveSpeedJumpDetected;
	int	directionWhenLocked;
	int rangeCounter;
	int myVehicleIndex;				// For testing and debugging
	int myVehicleIndex_z;
	int patrolVehicleIndex;			// For testing and debugging
	int	vehicleFoundCounter;
	int vehicleNotFoundCounter;
	int svrFFTfilterShift;
	int svrFiveSecondCounter;		// Counts the number of readings that go into the 5-second display update
	int svrSixtySecondCounter;		// Counts the number of readings that go into the 60-second display update
	int valueWithinLimitsCounter;
} speedStructType;

extern speedStructType speed;

#define SPEED_STRUCT_DEFAULTS							\
{														\
	FALSE,			/* Initialization status */			\
	FALSE,			/* speedIsNotDecreasing */			\
	0,				/* speedIncreaseCounter */			\
	0,				/* speedStableCounter */			\
	SPEED_PEAK_RUN_FREE,	/* holdState */				\
	0,				/* timer */							\
	_init,												\
	_update,											\
	_hold,												\
	_processHoldState,									\
	_updateSVRfilterAndDisplay,							\
	_restartHoldState,									\
	_averageForFinalDisplay,							\
	_IQ(1.0),		/* k */								\
	_IQ(0.0),		/* delta */							\
	_IQ(0.0),		/* present */						\
	_IQ(0.0),		/* previous */						\
	_IQ(0.0),		/* filtered */						\
	_IQ(0.0),		/* displayed */						\
	_IQ(0.0),		/* displayedSignalLevel */			\
	_IQ(0.0),		/* maximum */						\
	FALSE,			/* positiveSpeedJumpDetected */		\
	UNKNOWN_DIRECTION,	/* directionWhenLocked */		\
	0,				/* rangeCounter */					\
	0,				/* myVehicleIndex */				\
	0,				/* myVehicleIndex_z */				\
	0,				/* patrolVehicleIndex */			\
	0,				/* vehicleFoundCounter */			\
	0,				/* vehicleNotFoundCounter */		\
	0,				/* svrFFTfilterShift */				\
	0,				/* svrSixtySecondCounter */			\
	0,				/* valueWithinLimitsCounter */		\
}
//-------------------------------------------------------------------------------------------------
// END Structure Definition section
//-------------------------------------------------------------------------------------------------

#endif   /* #ifndef SPEED_H */

/*********************************** End of File ******************************************************/
