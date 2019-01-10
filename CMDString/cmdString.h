

#ifndef CmdString_h
#define CmdString_h

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h"       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

#include <EEPROM.h>

#define MAX_CMD_LINE 80
#define MAX_CMD_PARAMETERS  4
#define MAX_CMDS 12
#define MAX_VALUE_LEN 12
//
#define CMD_LENGTH_STRICT 6		// 4 + room for any syntax and zero
#define MAX_PREAMBLE_SIZE 16


//
#define CMD_LOAD_STAGES (3)
#define CMD_LOAD_UPPER_STAGES (8)
#define CMD_LOAD_COMMANDS (0)
#define CMD_LOAD_PARAMETERS (1)
#define CMD_LOAD_KEYS (2)


static inline zero_n(char *dst,unsigned char n) {
	for ( int i = 0; i < n; i++ ) {
		*dst++ = 0;
	}
}

static inline void cpy_zero_commas(char *src, char *dst) {
	while ( *src ) {
		char c = *src;
		if ( c == ',' ) {
			*dst = 0;
		} else {
			*dst = c;
		}
		dst++;
		src++;
	}
}


static inline void copy_upTo(char *src, char *end, char *dst) {
	while ( src < end ) {
		*dst++ = *src++;
	}
}


class CmdString {
	public:
		CmdString(uint8_t linesize,uint8_t appMaxCmd) {
			//
			// manage commands
			_parCount = 0;
			_ready = false;
			//
			zero_n(_values[0],MAX_VALUE_LEN);
			zero_n(_values[1],MAX_VALUE_LEN);
			zero_n(_values[2],MAX_VALUE_LEN);
			zero_n(_values[3],MAX_VALUE_LEN);
			//
			_cmdCount = 0;
			_maxCMDs = min(appMaxCmd,MAX_CMDS);
			_cnum = 0;
			
			// read in cmd string
			_linesize = min(MAX_CMD_LINE,linesize);
			_readState = 0;
			_cmdSetKey[0] = 'z';
			_cmdSetKey[1] = '!';
			_cmdSetKey[2] = 'x';
			_cmdSetKey[3] = 0;
			//
			_cmdMarker = "";
			_parStart = NULL;
			reset();
		}
		virtual ~CmdString() {}
	
		void addChar(char c);
	
		bool parseCommandString(void);
		bool isCommand(char * cmd);
	
		bool commandIS(const char * cmd);
		//
		bool ready(void) { return _ready; }
	
		char 	* getCommand(void) { return _command; }
		char 	* preamble(void) { return _preamble; }
	
		bool	unloadCommand(void);
		bool 	unloadParameters(void);
	
		char * valueOf(char * pKey);
		int intValueOf(char * pKey);
		float floatValueOf(char * pKey);
		//
		//
		void addCmd(char *cmdName,const char *params[4],const uint8_t n);
		void setCommandAdder(void (* addfun)(void)) {
			_functionAdder = addfun;
		}
	
		//
		void addRefTable(char *table,uint8_t sectNum,uint8_t size) {

			if ( sectNum < CMD_LOAD_STAGES ) {
				
				_cmdTableRefs[sectNum] = table;
				zero_n(table,size);
				
				char tmpTable[MAX_CMD_LINE];
				//
				EEPROM.get(0,_readState);
				if ( (_readState > CMD_LOAD_STAGES) && (_readState < CMD_LOAD_UPPER_STAGES) ) {
					EEPROM.get((1 + sectNum*MAX_CMD_LINE),tmpTable);
					cpy_zero_commas(tmpTable,table);
				}

			}
 
		}
		//
		void setCmdMaker(char *marker) {
			_cmdMarker = marker;
		}
		//
		bool reset(void) {
			//
			_ready = false;
			//
			_lineindex = 0;
			_parStart = NULL;
			for ( uint8_t i = 0; i < MAX_CMD_LINE; i++ ) { _buffer[i] = 0; }
			//
		}
	
		void clearEEPROM() {
			for (int i = 0 ; i < EEPROM.length() ; i++) {
				EEPROM.write(i, 0);
			}
		}
	
		void bufferToEEPROM(uint8_t sectNum) {
			char tmpTable[MAX_CMD_LINE];
			tmpTable[0] = 0;
			strcpy(tmpTable,_buffer);
			EEPROM.put((1 + sectNum*MAX_CMD_LINE), tmpTable);
		}
	
/*
	void print80(const char *tag) {
		for ( int i = 0; i < 80; i++ ) {
			char c = tag[i];
			Serial.print(c ? c : '0');
		}
		Serial.println("-");
	}
*/
		void reportReadState(const char *tag) {
			Serial.print("DBG=->");
			Serial.println(tag);
			/*
			Serial.print(tag);  // must be /n as on last line.
			Serial.print("->");
			Serial.println(_readState);
			*/
			/*
			Serial.print("DBG=->");
			print80(_cmdTableRefs[0]);
			Serial.print("DBG=->");
			print80(_cmdTableRefs[1]);
			Serial.print("DBG=->");
			print80(_cmdTableRefs[2]);
			 */
		}

	protected:
		bool 			_ready;
		//
		uint8_t			_maxCMDs;
		uint8_t			_parCount;
		uint8_t			_cmdCount;
		uint8_t			_cnum;
		//
		uint8_t			_readState;
		char 			_cmdSetKey[CMD_LENGTH_STRICT];
		void 			(* _functionAdder)(void);

		//
		char 			_buffer[MAX_CMD_LINE];
		uint8_t			_linesize;
		uint8_t			_lineindex;
		//
		char			*_cmdTableRefs[CMD_LOAD_STAGES];
		char			*_cmdMarker;
		char			*_parStart;
		//
		char 			_preamble[MAX_PREAMBLE_SIZE];
		char 			_command[CMD_LENGTH_STRICT];
		//
		char *			_cmdNames[MAX_CMDS];
		char *			_cmdPars[MAX_CMDS][MAX_CMD_PARAMETERS];
		uint8_t			_cmdPartCount[MAX_CMDS];
		//
		char 			_values[MAX_CMD_PARAMETERS][MAX_VALUE_LEN];
};



#endif
