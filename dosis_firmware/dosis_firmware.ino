#include <MemoryUsage.h>


#include <cmdString.h>
#define TASKER_MAX_TASKS 4
#include <Tasker.h>
#include <OneWire.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

//#include <MultiStepper.h>
#include <AccelStepper.h>


////////////////////

Tasker gTasker;
//
bool gReporting = false;
static unsigned int gSampleInterval = 1000; // default 1 sec
static bool itWorksLED = false;
//alarmRepeat
typedef struct {
  const char *sensor;
  double low;
  double high;
  bool checking;
  int currentState;
} sensorLimit;


////////////////////


int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2

const uint8_t LEDTestPin = 7;
const uint8_t PumpDirectionPin = 8;
const uint8_t PumpRatePin = 9;  // analog write

const uint8_t AirOnOffPin = 4;
const uint8_t LightOnOffPin = 6;
const uint8_t LightIntensityPin = 10; // analog write
const bool AirOnOffPinInit = true;
const bool LightOnOffPinInit = true;
//
const uint8_t HeatOnOffPin = 5;
const uint8_t HeatIntensityPin = 11; // analog write
const bool HeatOnOffPinInit = true;
//
const uint8_t StepperPin1 = 12;   // what are these pins?
const uint8_t StepperPin2 = 16;
const uint8_t StepperPin3 = 14;
const uint8_t StepperPin4 = 15;

//
const uint8_t LID_STEPPER_MAX_SPEED = 100;
const uint8_t LID_STEPPER_SPEED = 0;
const uint8_t LID_STEPPER_ACCELERATION = 50;


//AccelStepper lidStepper(AccelStepper::FULL2WIRE,StepperPin1,StepperPin2,StepperPin3,StepperPin4,true);

/*
 stepper.moveTo(500);
  while (stepper.currentPosition() != 300) // Full speed up to 300
    stepper.run();
  stepper.runToNewPosition(0); // Cause an overshoot then back to 0
*/
#define NUMCMDS 10
#define BUFSIZE 128
//
CmdString commands(BUFSIZE,NUMCMDS);

char emptyCMDTable[50];

const static char *sreqCmd = emptyCMDTable + 0;
const static char *limtCmd = emptyCMDTable + 5;
const static char *stmeCmd = emptyCMDTable + 10;
const static char *srofCmd = emptyCMDTable + 15;
const static char *ltstCmd = emptyCMDTable + 20;
const static char *lampCmd = emptyCMDTable + 25;
const static char *pumpCmd = emptyCMDTable + 30;
const static char *airsCmd = emptyCMDTable + 35;
const static char *heatCmd = emptyCMDTable + 40;
const static char *stepCmd = emptyCMDTable + 45;

char emptyParamsTable[66];

const static char *pqry = emptyParamsTable + 0;
const static char *pstate = emptyParamsTable + 4;
const static char *pdelay = emptyParamsTable + 10;
const static char *plevel = emptyParamsTable + 16;
const static char *pstrt = emptyParamsTable + 22;
const static char *pstp = emptyParamsTable + 27;
const static char *psens = emptyParamsTable + 31;
const static char *phigh = emptyParamsTable + 36;
const static char *plow = emptyParamsTable + 41;
const static char *pyear = emptyParamsTable + 45;
const static char *pmo = emptyParamsTable + 50;
const static char *pday = emptyParamsTable + 53;
const static char *pmins = emptyParamsTable + 57;
const static char *pdir = emptyParamsTable + 62;

char emptyKeysTable[40];

const static char *ky_cmd = emptyKeysTable + 0;
const static char *ky_OD = emptyKeysTable + 5;
const static char *ky_temperature = emptyKeysTable + 8;
const static char *ky_ALL = emptyKeysTable + 20;
const static char *ky_OFF = emptyKeysTable + 24;
const static char *ky_ON = emptyKeysTable + 28;
const static char *ky_HIGH = emptyKeysTable + 31;
const static char *ky_LOW = emptyKeysTable + 36;


//
static const char *empty;
static const char *sreqParams[4] = {pqry,empty,empty,empty};
const uint8_t sreqPCount = 1;

static const char *limtParams[4] = {psens,phigh,plow,empty};
const uint8_t limtPCount = 3;

static const char *stmeParams[4] = {pmins,pday,pmo,pyear};
const uint8_t stmePCount = 4;

static const char *srofParams[4] = {pstate,pdelay,empty,empty};
const uint8_t srofPCount = 2;

static const char *ltstParams[4] = {pstate,empty,empty,empty};
const uint8_t ltstPCount = 1;

static const char *lampParams[4] = {pstate,plevel,pstrt,pstp};
const uint8_t lampPCount = 4;

static const char *pumpParams[4] = {pdir,plevel,pstrt,pstp};
const uint8_t pumpPCount = 4;

static const char *airsParams[4] = {pstate,empty,empty,empty};
const uint8_t airsPCount = 1;

static const char *heatParams[4] = {pstate,plevel,empty,empty};
const uint8_t heatPCount = 2;

static const char *stepParams[4] = {pstate,plevel,empty,empty};
const uint8_t stepPCount = 2;


void addCommandList(void) {
  commands.addCmd(sreqCmd,sreqParams,sreqPCount);
  commands.addCmd(limtCmd,limtParams,limtPCount);
  commands.addCmd(stmeCmd,stmeParams,stmePCount);
  commands.addCmd(srofCmd,srofParams,srofPCount);
  commands.addCmd(ltstCmd,ltstParams,ltstPCount);
  commands.addCmd(lampCmd,lampParams,lampPCount);
  commands.addCmd(pumpCmd,pumpParams,pumpPCount);
  commands.addCmd(airsCmd,airsParams,airsPCount);
  commands.addCmd(heatCmd,heatParams,heatPCount);
  commands.addCmd(stepCmd,stepParams,stepPCount);
  commands.setCmdMaker(ky_cmd);
}


////////////////////
  
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
//

////////////////////

bool gCheckLimits = false;
uint8_t gSensorCount = 2;
static sensorLimit appSensors[2] = {
  { ky_OD, -100.0,100.0, false, 0 },       // set beyond possible measurments
  { ky_temperature, -10000.0, 10000.0, false, 0 }
};




void setLimitOnSensor(char *sensor,float upperLimit,float lowerLimit) {
  for ( uint8_t i = 0; i < gSensorCount; i++ ) {
    if ( strcmp(appSensors[i].sensor,sensor) == 0 ) {
      appSensors[i].checking = !(appSensors[i].checking);
      appSensors[i].low = lowerLimit;
      appSensors[i].high = upperLimit;
      return;
    }
  }
}

#define LIMITS_LOWER_BOUND_REPORT_COUNT -4
#define LIMITS_UPPER_BOUND_REPORT_COUNT 4
#define LIMITS_LOWER_BOUND_REPORT_RESTART 10
#define LIMITS_UPPER_BOUND_REPORT_RESTART -10

int checkLimitOnSensor(char *sensor,float value) {
  for ( uint8_t i = 0; i < gSensorCount; i++ ) {
    if ( ( strcmp(appSensors[i].sensor,sensor) == 0 ) && appSensors[i].checking ) {
      //
      if ( value < appSensors[i].low ) {
        appSensors[i].currentState--;
        if ( appSensors[i].currentState < LIMITS_LOWER_BOUND_REPORT_COUNT ) {
          appSensors[i].currentState = LIMITS_LOWER_BOUND_REPORT_RESTART;
          return(0);
        }
        return -1;
      } else if (  value > appSensors[i].high ) {
        appSensors[i].currentState++;
        if ( appSensors[i].currentState > LIMITS_UPPER_BOUND_REPORT_COUNT ) {
          appSensors[i].currentState = LIMITS_UPPER_BOUND_REPORT_RESTART;
          return(0);
        }
        appSensors[i].currentState++;
        return 1;
      }
      //  Where it is supposed to be so make it responsive
      appSensors[i].currentState = 0;
      return(0);
    }
  }
  return(0);
}

// ---- ----


  
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  
  byte data[12];
  byte addr[8];
  
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      Serial.println(F("ERR: no more sensors on chain, reset search!"));
      ds.reset_search();
      return -1000;
  }
  
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println(F("ERR: CRC is not valid!"));
      return -1000;
  }
  
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print(F("ERR: Device is not recognized"));
      return -1000;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  
    
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
    
  ds.reset_search();
    
  byte MSB = data[1];
  byte LSB = data[0];
  
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
    
  return TemperatureSum;
}


static bool gCMD = 0;
void sensorReport(void) {
  itWorksLEDIndicator(gCMD);
  if ( gReporting ) {
    //
    int sensorValue = analogRead(A0);// read the input on analog pin 0:
    float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float temperature = getTemp();
    printData(voltage,temperature);
    //
  }
  itWorksLEDIndicator(!gCMD);
}



//   -------- ------------   -------- ------------

void itWorksLEDIndicator(bool cmd) {
  if ( itWorksLED ) {
    if ( cmd ) {
      digitalWrite(LEDTestPin, HIGH);       // sets the digital pin 7 on
    } else {
      digitalWrite(LEDTestPin, LOW);       // sets the digital pin 7 off
    }
  } else {
    digitalWrite(LEDTestPin, LOW);       // sets the digital pin 7 off
  }
}

//  writePump
static bool gPumpDirection = false;  // state info
static uint8_t gPumpRate = 0;
// 
void writePump(bool pDirection, uint8_t rate) {
    gPumpDirection = pDirection;
    gPumpRate = rate;
    digitalWrite(PumpDirectionPin, pDirection ? HIGH : LOW );
    analogWrite(PumpRatePin,rate);
}

// should be called in sequence
void writePumpInvert() {
  writePump(gPumpDirection,gPumpRate);
}

void writePumpRevert() {
  writePump(gPumpDirection,0);  
}


//  writeLight
static bool gLightState = false;  // state info
static uint8_t gLightIntensity = 0;
// 
void writeLight(bool state, uint8_t intensity) {
    gLightState = state;
    gLightIntensity = intensity;
    //
    digitalWrite(LightOnOffPin, state ? HIGH : LOW );
    if ( state == false ) {
      analogWrite(LightIntensityPin,intensity);
    } else {
      analogWrite(LightIntensityPin,0);
    }
}

// should be called in sequence
void writeLightInvert() {
  writeLight(!gLightState,gLightIntensity);
}

void writeLightRevert() {
  writeLight(!gLightState,gLightIntensity);  
}


uint8_t gLightOnLevel = 0;
AlarmID_t gStartAlarmID;
AlarmID_t gStopAlarmID;
//
void lightON(void) {
  uint8_t intensity = (uint8_t)(gLightOnLevel);
  writeLight(false,intensity);
}

void lightOFF(void) {
  uint8_t intensity = (uint8_t)(0);
  writeLight(true,intensity);
}




void initLightAlarms(uint8_t level, uint8_t start_hour, uint8_t start_minute, uint8_t stop_hour, uint8_t stop_minute) {
  //
  gLightOnLevel = level;
  
  gStartAlarmID = Alarm.alarmRepeat(start_hour,start_minute,0, lightON);  //
  gStopAlarmID = Alarm.alarmRepeat(stop_hour,stop_minute,0,lightOFF);  //
  //
}

void cancelLightAlarms(void) {
  gLightOnLevel = 0;
  Alarm.disable(gStartAlarmID);
  Alarm.disable(gStopAlarmID);
  Alarm.free(gStartAlarmID);
  Alarm.free(gStopAlarmID);
  lightOFF();
}


//  writeAir
static bool gAirState = false;  // state info
void writeAir(bool state) {
    gAirState = state;
    digitalWrite(AirOnOffPin, state ? HIGH : LOW );
}


// should be called in sequence
void writeAirInvert() {
  writeAir(!gAirState);
}

void writeAirRevert() {
  writeAir(!gAirState);  
}


//  writeHeat
static bool gHeatState = false;  // state info
static uint8_t gHeatIntensity = 0;
//
void writeHeat(bool state, uint8_t intensity) {
    gHeatState = state;
    gHeatIntensity = intensity;
    //  
    digitalWrite(HeatOnOffPin, state ? HIGH : LOW );
    if ( state == false ) {
      analogWrite(HeatIntensityPin,intensity);
    } else {
      analogWrite(HeatIntensityPin,0);
    }
}

// should be called in sequence
void writeHeatInvert() {
  writeHeat(!gHeatState,gHeatIntensity);
}

void writeHeatRevert() {
  writeHeat(!gHeatState,gHeatIntensity);  
}


bool gLIDStepperRunning = false;
void writeStepper(bool state, uint16_t lidPosition) {

  /*
  gLIDStepperRunning = state;
  if ( lidPosition == 0 ) {
    lidStepper.moveTo(0);
  } else {
    lidStepper.moveTo(lidPosition);
  }
  */
}


//   -------- ------------   -------- ------------


void printLimit(char *sensor,float value) {
  int cc = checkLimitOnSensor(sensor,value);
  if ( cc != 0 ) {
    Serial.print(F("CRX"));
    if ( cc > 0 ) {
      Serial.print(commands.preamble());
      Serial.print(F(",LIMIT: "));
      Serial.print(sensor);
      Serial.println(F(",LEVEL: HIGH "));          
    } else {
      Serial.print(commands.preamble());
      Serial.print(F(",LIMIT: "));
      Serial.print(sensor);
      Serial.println(F(",LEVEL: LOW "));          
    }
  }
}



void printData(float OD,float temperature) {
    Serial.print(commands.preamble());
    //
    Serial.print(F(",OD: "));
    Serial.print(OD,3); // print out the value you read:
    Serial.print(F(",Temperature: "));
    //
    Serial.println(temperature,3);

    if ( gCheckLimits ) {
      printLimit(ky_OD,OD);
      printLimit(ky_temperature,temperature);
    }
}



void sendAllStateInfo(void) {
  //
  Serial.print("QRY");
  Serial.print(commands.preamble());
  Serial.print(F(",SOO: "));
  Serial.print(gReporting); // print out the value you read:
  //Serial.print(F(",LEDTest: "));
  //Serial.print(itWorksLED); // print out the value you read:
  Serial.print(F(",SOO-ctrlValue: "));
  Serial.print(gSampleInterval);
  //
  Serial.print(F(",PumpDir: "));
  Serial.print(gPumpDirection);
  Serial.print(F(",PumpRate: "));
  Serial.print(gPumpRate);

  Serial.print(F(",Lamp: "));
  Serial.print(gLightState);
  Serial.print(F(",Lamp-level: "));
  Serial.print(gLightIntensity);

  Serial.print(F(",Heater: "));
  Serial.print(gHeatState);
  Serial.print(F(",Heater-level: "));
  Serial.print(gHeatIntensity);
  
  Serial.print(F(",Air: "));
  Serial.println(gAirState);
  //
}

// // // // // // // // ---->


uint8_t hour_of(uint32_t minutesTime) {
  uint8_t hrs = minutesTime/60;
  return(hrs);
}

uint8_t  minutes_of(uint32_t minutesTime) {
   uint8_t minutes = minutesTime % 60;
   return(minutes);
}





///   COMMAND HANDLERS

bool handleStateRequest(void) {
  char * qry = commands.valueOf(sreqParams[0]);
  if ( strcmp(qry,ky_ALL) == 0 )  {
    sendAllStateInfo();
  } //  might support subsets in some cases
  return(true);
}






bool handleLimit(void) {
  char *sensor = commands.valueOf(limtParams[0]);
  if ( strcmp(sensor,ky_OFF) != 0 )  {
    gCheckLimits = true;
    float lowerLimit = commands.floatValueOf(limtParams[1]);
    float upperLimit = commands.floatValueOf(limtParams[2]);
    setLimitOnSensor(sensor,lowerLimit,upperLimit);
  } else {
    gCheckLimits = false;
  }
  return(true);
}

bool handleSetTime(void) {
  //
  uint16_t allminutes = commands.intValueOf(stmeParams[0]);
  uint8_t theDay = commands.intValueOf(stmeParams[1]);
  uint8_t theMonth = commands.intValueOf(stmeParams[2]);
  uint8_t theYear = commands.intValueOf(stmeParams[3]);
  //
  uint8_t theHour = hour_of(allminutes);
  uint8_t theMinutes = minutes_of(allminutes);
  //
  setTime(theHour,theMinutes,0,theDay,theMonth,theYear);
  return(true);
}




bool handleReportCmd(void) {
  //
  char *stateStr = commands.valueOf(srofParams[0]);
  bool state = ( strcmp(stateStr,ky_ON) == 0 );
  gReporting = state;
  
  float interVal = commands.floatValueOf(srofParams[1]);
  //
  gTasker.cancel(sensorReport);
  gSampleInterval =  (unsigned int)(interVal*1000.0);
  if ( gReporting ) {
    gTasker.setInterval(sensorReport, gSampleInterval);
  }
  //
  return(true);
}


bool handleLedTestCmd(void) {
  char *stateStr = commands.valueOf(ltstParams[0]);
  bool state = ( strcmp(stateStr,ky_HIGH) == 0 );
  itWorksLED = state;
  return(true);
}



bool handleLedsCmd(void) {
  //
  char *stateStr = commands.valueOf(lampParams[0]);
  bool state = ( strcmp(stateStr,ky_HIGH) == 0 );
  //
//  uint8_t level = commands.intValueOf(lampParams[1]);
  float level = commands.intValueOf(lampParams[1]);
  uint8_t intensity = (uint8_t)((level/100.0)*255);

//  uint8_t intensity = (uint8_t)(level * (float)2.55);
  uint32_t startT = commands.intValueOf(lampParams[2]);
  uint32_t stopT = commands.intValueOf(lampParams[3]);
  //
  if ( startT == stopT ) {
    writeLight(state,intensity);
  } else {
    if ( !state ) {

      uint8_t start_hour = hour_of(startT);
      uint8_t start_minute = minutes_of(startT);
      uint8_t stop_hour = hour_of(stopT);
      uint8_t stop_minute = minutes_of(stopT);

      initLightAlarms(intensity, start_hour, start_minute, stop_hour, stop_minute);
  
      uint8_t cHr = hour();
      uint8_t cMin = minute();
  
      if ( cHr >= start_hour && cMin >= start_minute ) {
        if ( cHr <= stop_hour && cMin < stop_minute ) {
          writeLight(true,intensity);
        }
      }
      //
   } else {
      cancelLightAlarms();
    }
  }
  return(true);
}



bool handlePumpCmd(void) {
  char *dirStr = commands.valueOf(pumpParams[0]);
  bool dir = ( strcmp(dirStr,ky_ON) == 0 );
  //
  float level = commands.floatValueOf(pumpParams[1]);
  uint8_t rate = (uint8_t)((level/100.0)*255);
  uint32_t startT = commands.intValueOf(pumpParams[2]);
  uint32_t stopT = commands.intValueOf(pumpParams[3]);

  if ( startT != stopT ) {
    if ( startT == 0 ) {
      startT = 1;
      stopT += 1;
    }
    writePump(dir,0);
    gPumpRate = rate;
    gTasker.setTimeout(writePumpInvert,startT);
    gTasker.setTimeout(writePumpRevert,stopT);
  } else {
    writePump(dir,rate);
  }
  return(true);
}


bool handleAirCmd(void) {
  char *stateStr = commands.valueOf(airsParams[0]);
  bool state = ( strcmp(stateStr,ky_HIGH) == 0 );
  writeAir(state);
  return(true);
}


bool handleHeatCmd(void) {
  char *stateStr = commands.valueOf(heatParams[0]);
  bool state = ( strcmp(stateStr,ky_HIGH) == 0 );
//  uint8_t level = commands.intValueOf(heatParams[1]);
  float level = commands.intValueOf(heatParams[1]);
  uint8_t intensity = (uint8_t)((level/100.0)*255);

  writeHeat(state,intensity);
  return(true);
}


bool handleStepperCmd(void) {
  char *stateStr = commands.valueOf(stepParams[0]);
  bool state = ( strcmp(stateStr,ky_ON) == 0 );
  uint16_t lidPosition = commands.intValueOf(stepParams[1]);
  writeStepper(state,lidPosition);
  return(true);
}



bool commandUpdates() {
  bool cmdOK = false;
  if ( commands.ready() ) {
    //
    if ( commands.commandIS(sreqCmd) ) {
      cmdOK = handleStateRequest();
    } else if ( commands.commandIS(limtCmd) ) {
      cmdOK = handleLimit();
    } else if ( commands.commandIS(stmeCmd) ) {
      cmdOK = handleSetTime();
    } else if ( commands.commandIS(srofCmd) ) {
      cmdOK = handleReportCmd();
    } else if ( commands.commandIS(ltstCmd) ) {
      cmdOK = handleLedTestCmd();
    } else if ( commands.commandIS(lampCmd) ) {
      cmdOK = handleLedsCmd();
//      cmdOK = true; 
    } else if ( commands.commandIS(pumpCmd) ) {
      cmdOK = handlePumpCmd();
    } else if ( commands.commandIS(airsCmd) ) {
      cmdOK = handleAirCmd();
    } else if ( commands.commandIS(heatCmd) ) {
      cmdOK = handleHeatCmd();
    } else if ( commands.commandIS(stepCmd) ) {
      cmdOK = handleStepperCmd();
    }
    commands.reset();
  }
  return(cmdOK);
}




void setup()
{

  pinMode(LEDTestPin, OUTPUT); 
  pinMode(PumpDirectionPin, OUTPUT);
  pinMode(PumpRatePin, OUTPUT);
  pinMode(AirOnOffPin, OUTPUT);
  pinMode(LightOnOffPin, OUTPUT); 
  pinMode(LightIntensityPin, OUTPUT);
  pinMode(HeatOnOffPin, OUTPUT);

  //
  Serial.begin(9600); //Baud rate: 9600
  while (!Serial) ; // wait for Arduino Serial Monitor

  //
  commands.addRefTable(emptyCMDTable,CMD_LOAD_COMMANDS,50);
  commands.addRefTable(emptyParamsTable,CMD_LOAD_PARAMETERS,66);
  commands.addRefTable(emptyKeysTable,CMD_LOAD_KEYS,40);
  
  commands.setCommandAdder(addCommandList);
 
  //
  setTime(8,0,0,1,1,19); // set time to Saturday 8:29:00am Jan 1 2019
 
  addCommandList();

  commands.reportReadState("setup");

  //
  /*
  lidStepper.setMaxSpeed(LID_STEPPER_MAX_SPEED);
  lidStepper.setSpeed(LID_STEPPER_SPEED);
  lidStepper.setAcceleration(LID_STEPPER_ACCELERATION);
  //  */
  //
  writeAir(AirOnOffPinInit);
  writeHeat(HeatOnOffPinInit, 0);
  writeLight(LightOnOffPinInit, 0);
//  digitalWrite(LightOnOffPin, LOW);
//  analogWrite(LightIntensityPin, 100);
  //

}

void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    commands.addChar(c); // get the new byte:
  }
}


// -----------------------------


boolean testToDelete = true;

void testCRX(void) {
  printLimit(ky_temperature,80.00);
}


#ifdef TESTING
void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print("DBG=");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println(); 
}

void printDigits(int digits)
{
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
#endif



void loop()
{

  gCMD = commandUpdates();
  if ( gCMD ) {
    Serial.print("ACK");
    Serial.println(commands.preamble());
  }
  /*
  if ( testToDelete ) {     // FOR TESTING ONLY
    testToDelete = false;
    gTasker.setInterval(digitalClockDisplay,1000);
  }
  */
  //
  /*
  if ( gLIDStepperRunning ) {
    lidStepper.run();
  }
  //*/
  // /*
  gTasker.loop();
  Alarm.delay(0);
  //*/
  
}
