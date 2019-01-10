
#include "cmdString.h"
#include <MemoryUsage.h>



void CmdString::addChar(char c) {
	
	if ( c == '\n' ) {
		
		EEPROM.get(0,_readState);
		_buffer[_lineindex++] = 0;
		
		//Serial.print("DBG=->addChar ");
		//Serial.println(_buffer);
		//
		if ( strncmp(_buffer,_cmdSetKey,3) == 0 ) {
			if ( _readState > CMD_LOAD_STAGES ) {
				clearEEPROM();
				_readState = 0;
				if ( _buffer[3] != '1' ) return;
			}
		}

		if ( _readState > CMD_LOAD_STAGES ) {
			_ready = parseCommandString();
			_readState = 4; // something wrote over this?
		} else if ( strncmp(_buffer,_cmdSetKey,3) == 0 ) {
			//
			_buffer[_lineindex+1] = 0;
			Serial.print(_buffer);  // token that was sent
			if ( _buffer[3] == '1' ) {
				_readState = 0;
				Serial.println(":OK:1");
				strcpy(_preamble,_buffer + 4);
				EEPROM.put(0,_readState);
			}
			if ( _buffer[3] == '2' ) {
				_readState = 1;
				Serial.println(":OK:2");
				EEPROM.put(0,_readState);
			}
			if ( _buffer[3] == '3' ) {
				_readState = 2;
				Serial.println(":OK:3");
				EEPROM.put(0,_readState);
			}
			if ( _buffer[3] == '4' ) {
				_readState = 3;
				Serial.println(":OK:4");
				EEPROM.put(0,_readState);
			}
			reset();
		} else {
			//
			Serial.print(_cmdSetKey);
			Serial.print((_readState+1));
			Serial.print(_preamble);
			switch ( _readState )  {
				case CMD_LOAD_COMMANDS: {
					char *src = _buffer;
					char *dst = _cmdTableRefs[CMD_LOAD_COMMANDS];
					cpy_zero_commas(src,dst);
					Serial.println(":OK:Loaded");
					bufferToEEPROM(CMD_LOAD_COMMANDS);
					break;
				}
				case CMD_LOAD_PARAMETERS: {
					char *src = _buffer;
					char *dst = _cmdTableRefs[CMD_LOAD_PARAMETERS];
					cpy_zero_commas(src,dst);
					Serial.println(":OK:Loaded");
					bufferToEEPROM(CMD_LOAD_PARAMETERS);
					break;
				}
				case CMD_LOAD_KEYS: {
					char *src = _buffer;
					char *dst = _cmdTableRefs[CMD_LOAD_KEYS];
					cpy_zero_commas(src,dst);
					Serial.println(":OK:Loaded");
					bufferToEEPROM(CMD_LOAD_KEYS);
					break;
				}
				default: {
					(*_functionAdder)();
					_readState = 4;
					Serial.println(":OK:init");
					EEPROM.put(0,_readState);
					break;
				}
			}
			reset();
		}
	} else {
		//
		if ( _lineindex >= _linesize ) {
			_ready = parseCommandString();
		} else {
			_ready = false;
			_buffer[_lineindex++] = c;
		}
	}
}

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----


/*
String CmdString::char_asciiOf(const char *cmd) {
}
*/


void CmdString::addCmd(char * cmdName,const char *params[4],const uint8_t n) {
	if ( _cmdCount >= _maxCMDs ) return;
	_cmdNames[_cmdCount] = cmdName;
	for ( uint8_t i = 0; i < n; i++ )  {
		_cmdPars[_cmdCount][i] = params[i];
	}
	_cmdPartCount[_cmdCount] = n;
	_cmdCount++;
}



bool CmdString::isCommand(char * cmd) {
	//
	for ( uint8_t i = 0; i < _cmdCount; i++ ) {
		if ( strncmp(_cmdNames[i],cmd,4) == 0 ) {
			_cnum = i;
			return(true);
		}
	}
	//
	return(false);
}



bool CmdString::commandIS(const char *cmd) {
	if ( _cnum >= 0 && _cnum < _cmdCount ) {
		return(strcmp(cmd,_cmdNames[_cnum]) == 0);
	}
	return(false);
}


char * CmdString::valueOf(char * pKey) {
	if ( _ready ) {
		_parCount = _cmdPartCount[_cnum];
		for ( uint8_t i = 0; i < _parCount; i++ ) {
			if ( strcmp(_cmdPars[_cnum][i],pKey) == 0 ) {
				return(_values[i]);
			}
		}
	}
	return("");
}

int CmdString::intValueOf(char * pKey) {
	char *valstr = valueOf(pKey);
	return(atoi(valstr));
}

float CmdString::floatValueOf(char * pKey) {
	char *valstr = valueOf(pKey);
	return(atof(valstr));
}

bool CmdString::unloadCommand(void) {
	_command[0] = 0;
	char *cmd = strstr(_buffer,_cmdMarker);
	//
	if ( cmd != NULL ) {
		//
		zero_n(_preamble,MAX_PREAMBLE_SIZE);
		copy_upTo(_buffer,cmd-1,_preamble);
		cmd += 4;
		//
		zero_n(_command,4);
		
		if ( *cmd == ' ' ) {
			while ( *cmd && *cmd == ' ' ) cmd++;
		}
		uint8_t i = 0;
		while ( *cmd ) {
			char c = *cmd++;
			if ( !(c) || c == ',' || c == ' ' ) {
				break;
			}
			_command[i] = c;
			i++;
			if ( i >= 4 ) break;
		}
		if ( !(isCommand(_command)) ) {
			return(false);
		}
		_parStart = cmd+1;
		//
	} else {
		return false;
	}
	
	return(true);
}

bool CmdString::unloadParameters(void) {
	//
	if ( _parStart == NULL ) return(false);
	_parCount = _cmdPartCount[_cnum];
	for ( uint8_t i = 0; i < _parCount; i++ ) {
		char *pname = _cmdPars[_cnum][i];
		char *par = strstr(_parStart,pname);
		if ( par != NULL ) {
			uint8_t plen = strlen(_cmdPars[_cnum][i]);
			par += plen;
			if ( *par == ':' ) par++;
			char *end = strchr(par,',');
			if ( end != NULL ) {
				zero_n(_values[i],MAX_VALUE_LEN);
				copy_upTo(par ,end, _values[i]);
			} else {
				strcpy(_values[i],par);
			}
		} else {
			reset();  // failed to parse
			return(false);
		}
		//
	}
	//
	return(true);
}

bool CmdString::parseCommandString(void) {
	
	if ( unloadCommand() ) {
		return(unloadParameters());
	} else {
		reset();  // failed to parse
		return(false);
	}
	return true;
}

