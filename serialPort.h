//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Serial Port
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
#define DEFAULT_PROTOCOL	SP_SFR	//SP_TRACKING	//SP_SFR	// SP_TRACKING_SIMULATION	//SP_SFR
typedef enum {
	SP_NONE,				// 0
	SP_FFT_TIMING,			// 1 FFT's per second
	SP_SIMULATED,			// 2
	SP_ANALOG,				// 3
	SP_FFT,					// 4
	SP_TRACKING,			// 5
	SP_SFR,					// 6
	SP_TRACKING_SIMULATION,	// 7
	SP_DEBUG,				// 8
} protocolEnumType;

#define RS232_BUFFER_SIZE  32
typedef struct {
	int head;								// Incremented by the ISR
	int tail;								// Incremented by the Application
	int buffer[RS232_BUFFER_SIZE];
} rs232BufferType;

typedef struct {
	protocolEnumType protocol;
	boolean commandReceived;
	boolean txWaitingForBufferSpace;
	rs232BufferType	tx;
	rs232BufferType	rx;
} rs232PortType;

#ifdef GLOBAL
  rs232PortType serialData;
#else
  extern rs232PortType serialData;
#endif

//-------------------------------------------------------------------------------------------------
// BEGIN Definition of structure
//-------------------------------------------------------------------------------------------------
typedef struct {
	void (*open)(void);				// Initialize the structure
	boolean (*processInput)(U8);
	int (*read)(void);
	void (*reset)(void);
	void (*updateDisplay)(U16);
	void (*monitor)(void);
} serialPortType;

extern serialPortType serialPort;

#define SERIALPORT_DEFAULTS		\
{								\
	_open,						\
	_processInput,				\
	_getc,						\
	_reset,						\
    _updateDisplay,				\
	_monitor,					\
}

/*********************************** End of File ******************************************************/
