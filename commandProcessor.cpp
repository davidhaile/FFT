//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// Command Processor
// $Header: https://togwotee/ets/svn/ETS_Libraries/trunk/commandProcessor.c 810 2013-05-01 21:27:06Z dhaile $
//-------------------------------------------------------------------------------------------------
// These functions process configuration commands that arrive from the serial port.
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#include "environ.h"

// Local processing functions
void processSP(void);


#define TOKENS			" ,:"
#define TOKENS_ALLOW_SPACES	",:"
#define MAXIMUM_COMMAND_LENGTH	30

// Enumerated Types
typedef enum {
	CMD_OK,					// Does nothing, must be the first in the list.
	CMD_SP,					// Serial Protocol
	CMD_HELP				// Lists all commands.  Must be the last in this list.
} commandEnumType;
#define NUMBER_OF_COMMANDS	(CMD_HELP+1)

typedef struct {
	commandEnumType			index;		// Used to keep us honest
	char					idString[MAXIMUM_COMMAND_LENGTH];
} commandType;

//-------------------------------------------------------------------------------------------------
// Commands
//-------------------------------------------------------------------------------------------------
const commandType commands[NUMBER_OF_COMMANDS] = {
	// index,					command
	{CMD_OK,					"ok"},
	{CMD_SP,					"s"},

	// Status or Help Only
	{CMD_HELP,					"help"},
};

// A "Command String" is the above command plus the data fields that may follow it
#ifndef SERIALPORT_BUFFER_SIZE
  #define SERIALPORT_BUFFER_SIZE  64
#endif
char returnDataBuffer[SERIALPORT_BUFFER_SIZE];

//-------------------------------------------------------------------------------------------------
// The Command Processor
//-------------------------------------------------------------------------------------------------
ErrorCodeIntType processCommands(void) {
	int i, j;
	boolean matchWasFound = FALSE;
	ErrorCodeIntType returnCode = EMPTY_BUFFER;
	char *pLocal;
	char ch;
	U32 temp32;
	U8 *p8;
	char *pCommand;
	int value;

	memset(returnDataBuffer, 0, sizeof(returnDataBuffer));
	for (i=0; i<RS232_BUFFER_SIZE; i++) {
		value = serialPort.read();
		if (value != NULL) {
			returnDataBuffer[i] = value;
		} else {
			break;
		}
	}

	pCommand = returnDataBuffer;

	//-----------------------------------------------------------------------------------------
	// Initialize local variables, then find the command
	//-----------------------------------------------------------------------------------------
	matchWasFound = FALSE;
	pLocal = strtok(pCommand, TOKENS_ALLOW_SPACES);

	for (i=0; (i<NUMBER_OF_COMMANDS) && (matchWasFound == FALSE); i++) {
		if ((strlen(commands[i].idString) == strlen(pCommand)) && 
			(strncmp(pCommand, commands[i].idString, strlen(commands[i].idString)) == 0)) {
	 		matchWasFound = TRUE;

			// A match was found. Now we need to validate the data field
			switch (commands[i].index) {
			case CMD_OK:
				Serial.print("OK");
				break;
			case CMD_HELP:
				// List all the commands
				for (j=0; j<(NUMBER_OF_COMMANDS-1); j++) {
					Serial.println(commands[j].idString);
				}
				break;
			case CMD_SP:
				Serial.print("Serial Protocol");
				processSP();
				break;
			default:
				returnCode = FAIL;
				break;
			}

			returnCode = PASS;
		}
	}

	if (returnCode != PASS) {
		Serial.print("Command: ");
		Serial.print(pCommand);
	}
	Serial.println();

	return(returnCode);
}

//===========================================================================
void processSP(void) {
	char *pLocal;

	pLocal = strtok(NULL, TOKENS_ALLOW_SPACES);
	if (pLocal != NULL) {
		Serial.print(":");
		Serial.print(pLocal);
		serialData.protocol = (protocolEnumType)atoi(pLocal);
	};
}

//===========================================================================
// No more.
//===========================================================================

