/* K-ev Power-Module script v2.2

   Written by Henri Pyoria (henri.pyoria@outlook.com)

   Temperature-IC addresses must be manually defined
   to temperatureAddress table. This skips teh lengthy search for 'em.
   IC's must support resolution configuration! (10bit conversion is used)

   2s watchdog implemented after the setups ready, in case of odd failures.
*/

#define F_CPU 16000000UL
#define BAUD 2400
#define BYTE 8

#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include <EEPROM.h>
#include <OneWire.h>

// DO NOT USE DEBUG ON LIVE SYSTEM!!! IMPERATIVE!!!
#define DEBUG 1
#define HOST_QUERY_ON 1
#define DEMO 1

//! const or #define?
// #Define it should and shall be
#define CURRENT_SENSE_PIN  0 // PORTC
#define VOLTAGE_SENSE_PIN  1 // PORTC
#define CURRENT_SENSE_REFERENCE_PIN  3 // PORTC
#define MOSFET_SWITCH_PIN  4 // PORTC
#define FAN_SWITCH_PIN 3 // PORTB
#define ONEWIRE_PIN 10 // Arduino Nano board pin

#define WRITE_BUFFER_SIZE 11
#define READ_BUFFER_SIZE 100
// Module receives only specified length commands ending in '\n'. Count excludes the '\0' char.
#define CMD_CHAR_COUNT 3 
#define HOST_QUERY_TIME 5000 // ms, time to wait host to answer back before shutting gates

#define GUI_MAX_FLOAT_VALUE 999.9 // GUI supports numeric values from -99.9 to 999.9 atm
#define GUI_MIN_FLOAT_VALUE -99.9

#define TEMPERATURE_SENSOR_COUNT 40
#define TEMPERATURE_READ_TIME 200 // ms, time where the temperature sensors sample the temperatures

#define FAN_MAX_DUTY_CYCLE 255

#define OVERCURRENT 50 //A
#define OVERVOLTAGE 85 //V
#define LOW_VOLTAGE 10 //V - 60V for the final product
#define CURRENT_THRESHOLD 0.5 // Max inaccuracy of current measurement when power is off
#define BATTERY_OVER_HEAT 60 //'C
#define MODULE_OVER_HEAT 80 //'C

#define EEPROM_SAVE_INTERVAL 3600000 // ms, interval to save battery data to EEPROM

OneWire ds(ONEWIRE_PIN); // Set OneWire communication on pin 10
// For some reason, OneWire library makes #define ONEWIRE_PIN cause "expected identifier 
// before numeric constant" error message if this is not instanced in global scope

typedef struct TemperatureSensors
{
  float temperature[TEMPERATURE_SENSOR_COUNT];
  byte data[TEMPERATURE_SENSOR_COUNT][9];
  int availableSensors = 0;
};

typedef struct PowerMonitor 
{
  volatile float current;
  volatile float voltage;
  volatile float board5v;

  volatile float coulombCounter = 0; //Ah
  volatile float lastCurrents[100] = {0}; //A
  
  bool criticalCFailure = false;
  bool criticalVFailure = false;
  bool criticalTFailure = false;
  bool lowVoltage = false;
  const float currentThreshold = 0.5;
} PowerMonitor;

PowerMonitor power; // This struct is accessed from an ISR and is therefore global

typedef struct Battery
{ // Not used atm
  float stateOfHealth = 1.0; // x100%
  float stateOfCharge = 1.0; // x100%
  float capacity = 0;
  const int parallelCellCount = 32;
  const int serialCellCount = 20;
  const float defaultCapacity = 80; //Ah

  // Cell manufacturer has given discharge curve for the cells, we use that for estimating the SOC

  const float voltageFull = 84.0; //V
  const float voltageEmpty = 66.0; //V
  const float current0_5C = 41.5; //A
  const float current0_3C = 24.9; //A
  const float current0_1C = 8.3; //A

  // at 0.5C current, and 1V voltage drop equals roughly 0.35Ah for a cell.
  const float dischargeVChange1Ah = 0.35; //V
  // Constant current charging stops around 71Ah of packs default capacity
  const float ccChargeEndCapacity = 0.8875; //x100% of capacity
} Battery;

typedef struct Serialport
{
  unsigned char command[CMD_CHAR_COUNT] = {'\0', '\0', '\0'};

  unsigned char inputBuffer[READ_BUFFER_SIZE] = {0}; // Ring buffer
  byte in_s = 0;
  byte in_e = 0;
  bool bufferFull = false;
  
  bool commandAvailable = false;
  bool hostUp = false;
  bool hostShut = false;
  bool hostReplied = false;
} Serialport;

Serialport serial; // This struct is accessed from an ISR and is therefore global

typedef struct ModuleStates
{
  enum state
  {
    ERROR_LOCK_DOWN,
    INIT,
    EEPROM_READ,
    TEMPERATURE_MEASUREMENT_COMMAND,
    BATTERY_CHARGE_CHECK,
    BATTERY_HEALTH_CHECK,
    TEMPERATURE_READ,
    HOST_CHECK,
    HOST_QUERY,
    DATA_SEND,
    EEPROM_WRITE
  };
  
  state nextState = INIT; // Start form initialization
    
  // These two floats are timestamps used for making sure there is minimum time 
  // between some of the states:
  
   // This is set when temperature measurement is commanded
  unsigned long msTemperatures;
   // This is set when system first starts and with every write in the EEPROM 
  unsigned long msEEPROM = millis();
};

// State machine's repeating functions
int checkPowerStatus();
int checkSerialInput();

// State functions
// returns 0 if successful, -1 for error or positive integer for alternative outcomes
int initialize(); // Set initial config
int startTemperatureMeasurement(); // Sends command to start measurement
int checkHost(); // Check serial input buffer if host is alive
int queryHost(); // If host isn't present, try ask it if it's still up
int readTemperatures(TemperatureSensors*); // Read the sensors measurement data
int sendData(TemperatureSensors*); // Send all measurement data to host, includin power status
int trapLoop(); // If critical conditions have been reached, lock up an inform host about it

//! Could these global variables be atleast inside of some struct?
//! Probably all of them doesn't have to be global
/* You're right, fine combing with critical eye I'm moving innecessarily global variables 
 * to more local scope + create structs
 */

ISR(TIMER1_COMPA_vect); // 10 Hz interrupt for power measures
ISR(USART_RX_vect);   // Serial interrupt to handle input

//! It's good to briefly tell about the function in the declaration
//! It would be nice to know what inputs the function reads and what outputs the function writes

unsigned int adConversion(byte channel); // Measures value from given MUX channel and returns it

int powerOn(); // Sets high power MOSFET bridge open, return 0 on success, -1 on failure
void powerOff(); // Sets high power MOSFET bridge closed
bool powerIsOn(); // Returns true if MOSFET bridge open is on

int temperatureSetup(); // Send sensor config to sensors
void serialSetup(); // Initializes serial port registers
void interruptSetup(); // Sets up 10 Hz measurement interrupt to interrupt register
int fanSetup(); // Initializes FAN_SWITCH_PIN as PWM output and sets FAN_MAX_DUTY_CYCLE to its compare register
int setFanSpeed(int speed);
// Takes speed value of 0 - 255 and writes it to compare register
// Return new speed on success, -1 if speed was invalid
// Invalid values set speed to FAN_MAX_DUTY_CYCLE to prevent overheating on failure

//! Even if buffer is global it would be nice to have common function
//! that takes pointer to buffer begin as argument and size of the buffer for second argument
//! It also makes sense to return some success information
/* This is now fixed in new serialWrite-functions, 
 * as they handle the buffer an take only c-string or a float value as an argument
 */
char* serialRead(); // Reads serial input buffer for any commands and return command in a c-string
                    // Returned string could be empty if no proper command was found
int handleCommand(char* cmd); // Acts upon received command, 0 if succesful, -1 if unknown/garbage
int serialWrite(char* data); // Sends string through serial
int serialWrite(float value); // Sends float number as a string through serial
int serialWriteTemperatureData(int i, float temperature); // Sends specially formatted temperature data


unsigned int adConversion(byte);

//! Function name doesn't tell what it does. There are hardly any case where you should use incomplete words
/* I recognized the cause for this to be imitating some example codes I have seen.
 * With a second thought, I understand why that is bad.
 */
byte* getTemperatureAddress(byte n);
float rawTemperatureToCelsius(byte* datapoint);
bool hadGoodCRC(byte* datapoint);

//! Should this be moved to own file? 
/* I tried to divide the whole projects code to multiple files, but Arduino IDE doesn't allow including 
 * headers from project folder. With lack of reasonably accessible documentation about splitting code 
 * to multiple files in IDE in question, I gave up. This code also started quite dependent of 
 * Arduino libraries. As problems accumulated, the Arduino specific codebase was mostly gotten rid of, 
 * but development remained within Arduino IDE due to the easy uploading.
 * 
 * Honestly, what I should have done already, would be to just create code with a proper IDE, 
 * avr-gcc and avrdude respectively. Or Atmelstudio.
 */
const byte temperatureAddress[TEMPERATURE_SENSOR_COUNT][8] PROGMEM =
{ 
  {0x28, 0xF4, 0x2E, 0x41, 0x09, 0x00, 0x00, 0x0B}, 
  
  {0x28, 0xBA, 0x4B, 0x41, 0x09, 0x00, 0x00, 0xAE},
  {0x28, 0xEE, 0x46, 0x41, 0x09, 0x00, 0x00, 0xB0},
  {0x28, 0x4C, 0x2B, 0x41, 0x09, 0x00, 0x00, 0x7F},
  {0x28, 0x68, 0x43, 0x41, 0x09, 0x00, 0x00, 0x3A},
  {0x28, 0x8E, 0x3F, 0x41, 0x09, 0x00, 0x00, 0xED},

  {0x28, 0x59, 0x49, 0x41, 0x09, 0x00, 0x00, 0x5D},
  {0x28, 0x2E, 0x4D, 0x41, 0x09, 0x00, 0x00, 0x5F},
  {0x28, 0x5C, 0x3F, 0x41, 0x09, 0x00, 0x00, 0x47},
  {0x28, 0x34, 0x45, 0x41, 0x09, 0x00, 0x00, 0xF5},
  {0x28, 0x53, 0x34, 0x41, 0x09, 0x00, 0x00, 0x13},

  {0x28, 0x84, 0x48, 0x41, 0x09, 0x00, 0x00, 0x1E},
  {0x28, 0x6F, 0x2D, 0x41, 0x09, 0x00, 0x00, 0x0C},
  {0x28, 0xA4, 0x3D, 0x41, 0x09, 0x00, 0x00, 0x17},
  {0x28, 0x6D, 0x3B, 0x41, 0x09, 0x00, 0x00, 0x82},
  {0x28, 0x58, 0x43, 0x41, 0x09, 0x00, 0x00, 0xD7},

  {0x28, 0xF6, 0x28, 0x41, 0x09, 0x00, 0x00, 0xF9},
  {0x28, 0x04, 0x49, 0x41, 0x09, 0x00, 0x00, 0x39},
  {0x28, 0x2B, 0x4D, 0x41, 0x09, 0x00, 0x00, 0xB4},
  {0x28, 0xD2, 0x42, 0x41, 0x09, 0x00, 0x00, 0x3F},
  {0x28, 0x08, 0x2E, 0x41, 0x09, 0x00, 0x00, 0x04},

  {0x28, 0x82, 0x2F, 0x41, 0x09, 0x00, 0x00, 0xEC},
  {0x28, 0x89, 0x29, 0x41, 0x09, 0x00, 0x00, 0x88},
  {0x28, 0xD6, 0x3D, 0x41, 0x09, 0x00, 0x00, 0xE1},
  {0x28, 0x8E, 0x42, 0x41, 0x09, 0x00, 0x00, 0x6C},
  {0x28, 0xC4, 0x40, 0x41, 0x09, 0x00, 0x00, 0x55},

  {0x28, 0xFD, 0x46, 0x41, 0x09, 0x00, 0x00, 0xB2},
  {0x28, 0xC4, 0x28, 0x41, 0x09, 0x00, 0x00, 0x7A},
  {0x28, 0x3F, 0x32, 0x41, 0x09, 0x00, 0x00, 0x31},
  {0x28, 0xFD, 0x2F, 0x41, 0x09, 0x00, 0x00, 0x50},
  {0x28, 0x8F, 0x2E, 0x41, 0x09, 0x00, 0x00, 0x6B},

  {0x28, 0x74, 0x4A, 0x41, 0x09, 0x00, 0x00, 0xEF},
  {0x28, 0x1F, 0x35, 0x41, 0x09, 0x00, 0x00, 0xD6},
  {0x28, 0x4F, 0x25, 0x41, 0x09, 0x00, 0x00, 0x84},
  {0x28, 0xF9, 0x41, 0x41, 0x09, 0x00, 0x00, 0x3F},
  {0x28, 0x36, 0x2A, 0x41, 0x09, 0x00, 0x00, 0xE5},

  {0x28, 0x67, 0x2F, 0x41, 0x09, 0x00, 0x00, 0x2E},
  {0x28, 0x18, 0x4E, 0x41, 0x09, 0x00, 0x00, 0x4E},
  {0x28, 0x4F, 0x27, 0x41, 0x09, 0x00, 0x00, 0x07},
  {0x28, 0x8D, 0x3B, 0x41, 0x09, 0x00, 0x00, 0xAB}/*,
  {0x28, 0x7B, 0x38, 0x41, 0x09, 0x00, 0x00, 0x25}*/
  
};

//! Quite big function
/* This is another caveat of using Arduino IDE to start project in, it strongly suggests to use setup()
 * for one time execution code and loop() for all else. As two seperate functions, they have also 
 * different scopes which forces quite many things to be global, that could be more local otherwise.
 *
 * With state machine I now use setup() as like a main(). 
 * (State machine is a new concept for me, though I found it's basic idea quite intuitive)
 */
void setup()
{
  // As an absolute first, set mosfet switch pin to output and set its output low
  DDRC |= (1<<MOSFET_SWITCH_PIN);
  powerOff();
  
  wdt_enable(WDTO_2S); // Start watchdog
  
  ModuleStates moduleStates; // Create the state machine

  TemperatureSensors sensors; // Create struct to hold temperature variables

  while(1)
  {
    // Reset watchdog after every iteration of the loop
    wdt_reset();
      
    switch(moduleStates.nextState)
    {
      case ModuleStates::INIT: // Runs once on startup
      if(initialize() == 0) // If everything initialized like supposed to
      {
        moduleStates.nextState = ModuleStates::TEMPERATURE_MEASUREMENT_COMMAND;
      }
      else // If something essential doesn't work
        moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
      break;

      //  First state to repeat in sequence:
      case ModuleStates::TEMPERATURE_MEASUREMENT_COMMAND:
      switch(startTemperatureMeasurement())
      {
        case 0: // Command send succeeds
        moduleStates.msTemperatures = millis();
        moduleStates.nextState = ModuleStates::TEMPERATURE_READ;
        break;
        
        case -1: // OneWire bus dysfunctional, skip reading
        moduleStates.nextState = ModuleStates::HOST_CHECK;
        break;

        default: // Something catastrophic happened
        moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
      }
      break;
  
      case ModuleStates::TEMPERATURE_READ:
      if(TEMPERATURE_READ_TIME <= (unsigned long)(millis() - moduleStates.msTemperatures))
      {
        switch(readTemperatures(&sensors))
        {
          case 0: // Read succeeds
          moduleStates.nextState = ModuleStates::HOST_CHECK;
          break;
  
          case -1: // Critical overtemperature
          moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
          break;

          default: // Something catastrophic happened
          moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
        }
      }
      break;
      
      case ModuleStates::HOST_CHECK:
      switch(checkHost())
      {
        case 0: // Host had sent "ok" signal earlier
        moduleStates.nextState = ModuleStates::DATA_SEND;
        break;
        
        case 1: // Host hadn't send signal or it was garbled on the way
        moduleStates.nextState = ModuleStates::HOST_QUERY;
        break;

        default: // Something catastrophic happened
        moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
      }
      break;
      
      case ModuleStates::HOST_QUERY:
      switch(queryHost())
      {
        case 0: // Host responds after asking if it's alive
        moduleStates.nextState = ModuleStates::DATA_SEND;
        break;

        case 1: // Host purposedly shut mosfet gate and stays quiet
        moduleStates.nextState = ModuleStates::DATA_SEND;
        break;
        
        case -1: // Host didn't respond
        powerOff();
        moduleStates.nextState = ModuleStates::DATA_SEND;
        break;

        default: // Something catastrophic happened
        moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
        
      }
      break;
  
      case ModuleStates::DATA_SEND:
      switch(sendData(&sensors))
      {
        case 0: // Data send succeeds
        moduleStates.nextState = ModuleStates::TEMPERATURE_MEASUREMENT_COMMAND; 
        break;

        case -1: // Something catastrophic happened
        moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;
        break;

        default: // There is still lines of data to send
        break;
      }
      break;

      // Something bad happened, shut off mosfet gate and try inform host about that
      case ModuleStates::ERROR_LOCK_DOWN: 
      trapLoop();
      break;

      default: // Same as case ModuleStates::ERROR_LOCK_DOWN
      trapLoop();
      break;
    }
        
    // Power status and serial input are to be checked also after every iteration of the loop
    if(checkPowerStatus() != 0) // Check if everything is under control electrically
      moduleStates.nextState = ModuleStates::ERROR_LOCK_DOWN;

    while(serial.commandAvailable)
    {
      handleCommand(serialRead());
    }
  }

//! These magic numbers (literals) are all over the code without any further information
/* These were a dirty way to check if value is too big to send with previous
 * implementation of serial transfer, better handling now in serialWrite() -function
 * if(!(power.voltage >= 999.9 || power.current >= 999.9 || power.current <= -99.9)) 
 */
}

//! Some of the setup should be here?
//! Could you use some kind of state machine here, this would help with the watchdog?
//! Is there a reason why there is separate timer interupt? Is this called to rarely or do you have some other priorization?
/* Voltage and current is good to have determined and constant interval for safety purposes, ie. battery short circuit
 * Also it makes it possible to integrate current flow over time to measure used energy when measuring battery charge. 
 * 
 * State machine is now implemented in setup(), which I use as a main() now
 */

int powerOn()
{  
  if(checkPowerStatus == 0 && !powerIsOn())
  {
    PORTC |= (1<<MOSFET_SWITCH_PIN);
    return 0;
  }
  else
    return -1;
}

void powerOff()
{
  PORTC &= ~(1<<MOSFET_SWITCH_PIN);
}

bool powerIsOn()
{
  if(PORTC & (1<<MOSFET_SWITCH_PIN))
    return true;
  else
    return false;
}

int initialize()
{
  // Disable digital buffers on analog input pins to enable them
  DIDR0 = 0b00101111; // PORTC IOs 0, 1 and 3
  
  fanSetup(); // Start PWM output to cooling fan
  serialSetup(); // Initialize serial registers
  //! Could you have function that you just call writeSomething("FooBar\n"); ?
  //! There is probably some reason to write 'st' to serial. That could be a function 
  /* Yes, I could and I did just that. Though, I moved this particular line to more 
   * logical place here.
   * I suppose I didn't see how to do that function so clearly when I wrote it last time.
   */
  serialWrite("st\n"); // Inform host that board is starting
  _delay_ms(200); // Let board stabilize before starting AD-conversion with interrupts
  interruptSetup(); // Set interrupts in place
  temperatureSetup(); // Write config to temperature measuring IC:s
  
  //! All after this should not probably be in setup because you have to wait something (Interrupt?) to happen 
  //! until you can continue
  /* _delay_ms(250);
   * That was to wait for the AD-converter results to stabilize, the thought behind that was originally to    
   * stop startup there right after measurement, where bare minimum to be able to inform host controller 
   * about a failure, was done.
   * 
   * Somehow the current and/or voltage would be first read too high, which caused the code to halt the startup 
   * process as supposed. I have yet to solve why that happens, possibilities are AD-conversion inaccuracy right  
   * after powering the board on, or something about the chip that measures the voltage difference over the
   * shunt resistor. Without my own oscilloscope this has been hard to pin down.
   */
  
  return 0;
}

void serialSetup()
{
    // UBRR values are calculated in setbaud.h header to set baud rate
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    // Don't use double speed
    UCSR0A &= ~(1<<U2X0);
    // 8 bit message
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
    // Enable tx, rx and rx interrupt
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
}

int handleCommand(char* cmd)
{
  if(cmd[0] == '!' && cmd[1] == 'S')
  {
    // Shut if needed
    powerOff();
    serial.hostShut = true;
    serial.hostUp = false;
    // Inform host
    serialWrite("sh\n"); 
    // Gate is now shut by host
  }
  else if(cmd[0] == 'o' && cmd[1] == 'k')
  {
    // Host wants power on
    serial.hostShut = false;
    serial.hostUp = true;
    serial.hostReplied = true;
    if(powerOn() == 0)
      serialWrite("up\n"); // Inform host about success
    else
      return -2;
  }
  else // Otherwise command was garbage/unknown
    return -1;

  return 0;
}

//! Quite long function and hard to say what it does
//! Dividing this to smaller and properly named function would help
/* That is true and now reimplemented*/
/*void checkSystemStatus()*/
int checkPowerStatus()
{
  if(power.criticalCFailure // Check if ISR has flagged any conditions
  || power.criticalVFailure 
  || power.criticalTFailure
  || power.criticalTFailure)
  {
    powerOff();
    return -1;
  }
  // If mosfet gate is shut, there shouldn't be any current measured
  // As powerIsOn() reads register directly, it's no use trying to write it again,
  // but there is possibility to open and shut the gate again. For now it is 
  // assumed though that it's more safe to just hold the current state.
  else if(!powerIsOn() && abs(power.current) > CURRENT_THRESHOLD) 
  {
    power.criticalCFailure = true;
    return -1;
  }
  else
    return 0;
}

int temperatureSetup()
{
  if(!ds.reset()) // If no presence response wasn't found try again
  { 
    if(!ds.reset()) // If response is still not found, return -1
      return -1;
  }
  else
  {
    // Store config values to a byte
    // scratchpad byte, TH&TL bytes, configuration byte
    byte setCnf[4] = {0x4E, 0x7C, 0x0A, 0b00111111};
    
    ds.write(0xCC); // Skip ROM address select
    ds.write_bytes(setCnf, 4); // Write config
  }
  return 0;
}

int startTemperatureMeasurement()
{
    if(!ds.reset())
    {
      if(!ds.reset())
        return -1; // No presence signal was detected
    }
    ds.write(0xCC); // Skip ROM select
    ds.write(0x44); // Command to start measurement
    return 0;
}

//! Temps? Temperature, temporary values? 
int readTemperatures(TemperatureSensors *sensors)
{ 
  if(sensors->availableSensors) // If there is a count for sensors
  {
    for(int i = 0; i < sensors->availableSensors; ++i) // For each of them
    {
      ds.reset(); // Start command by resetting line
      ds.select(getTemperatureAddress(i)); // Select device
      ds.write(0xBE); // Request sensor to read out the scratchpad
      for (byte j = 0; j < 9; ++j) // Read all 9 bytes
      {
        sensors->data[i][j] = ds.read();
      }
      // Data read, next check if data matches it's CRC byte

      if(OneWire::crc8(sensors->data[i], 8) != sensors->data[i][8])
      { // If crc is wrong, set all bits to 1
        for (byte k = 0; k < 9; ++k)
          sensors->data[i][k] = 0xFF;
      }
    }
  }  
  else // If there isn't count of addresses yet, count them
  {
    for (int i = 0; i < TEMPERATURE_SENSOR_COUNT; ++i)
    {
      if(pgm_read_byte(temperatureAddress[i][0]) != 0x00) // This checks if sensors' address table entry has any address in it
      {
        ++sensors->availableSensors;
      }
      else
        break;
    }
  }

  for (byte i = 0; i < sensors->availableSensors; ++i) // For each sensor's data
  {
    sensors->temperature[i] = rawTemperatureToCelsius(sensors->data[i]); // Convert it to float value

    if(hadGoodCRC(sensors->data[i])) // If sensor data was valid, check that no overheating is occurring
    {
      // First sensor will be in power module itself
      if(i == 0 && sensors->temperature[i] >= MODULE_OVER_HEAT) // If overheated, power off and flag that
      {
        powerOff(); //! Could these be in a function with a good name? - done, they're now powerOff() and powerOn().
        power.criticalTFailure = true;
      }
      // Every other sensor is in battery compartment
      else if(i != 0 && sensors->temperature[i] >= BATTERY_OVER_HEAT)
      {
        powerOff();
        power.criticalTFailure = true;
      }
    }
  }
}

float rawTemperatureToCelsius(byte *datapoint)
{
  //! if I'm correct this is first local variable. Probalby there could be more of these.
  /* Point taken, there is now much less global variables */
  int16_t rawTemperature = ((datapoint[1] << BYTE) | datapoint[0]); 
  float realTemperature;
  
  rawTemperature &= ~0b11;
  // ^ Sets last 2 bits to zero, as 10-bit resolution doesn't use them
  // At 85'C the raw integer value is 1360, so therefore:
  realTemperature = 85.0 * (float(rawTemperature) / 1360.0);
  return realTemperature;
}

//! This is Ok. This function  does only one thing
unsigned int adConversion(byte channel)
{
  ADMUX = ((1<<REFS0)| channel); // Select pin and voltage reference
  
  ADCSRA = 0b11000111; // Start conversion (first 2 bits from left) with prescaler 128 (last 3 bits)
  while ((ADCSRA & (1<<ADSC)) != 0) {}; // Wait for status bit to flip
  // First readings are garbage if MUX was changed, so lets just forget few results

  for(int i = 0; i < 3; ++i)
  {
    ADCSRA = (1<<ADSC); // Start conversion again
    while ((ADCSRA & (1<<ADSC)) != 0) {};
  }
  
  return ADC;
}

byte* getTemperatureAddress(byte n)
{
  static byte tmp[8];
  for (int i = 0; i < 8; ++i)
    tmp[i] = pgm_read_byte_near(temperatureAddress[n]+i);
    
  return tmp;
}

bool hadGoodCRC(byte* datapoint)
{
  if(datapoint[0] == 0xFF)
    return false;
  else
    return true;
}

int sendData(TemperatureSensors *sensors)
{
  static int i = -3;

  switch(i)
  {
    case -3:
    // First line of communication is if power is up or shut
    if(powerIsOn()) //! Function woul tell what we read
      serialWrite("up\n");
    else
      serialWrite("sh\n");
    break;

    case -2: 
    serialWrite(":c");
    serialWrite(power.current);
    serialWrite("\n");
    break;

    case -1:  
    serialWrite(":v");
    serialWrite(power.voltage);
    serialWrite("\n");
    break;

    case 0:
    if(sensors->temperature[i] < MODULE_OVER_HEAT)
      serialWriteTemperatureData(sensors->temperature[i], i);
    break;
    
    default:   
    if(sensors->temperature[i] < BATTERY_OVER_HEAT)
      serialWriteTemperatureData(sensors->temperature[i], i);
    break;
  }
  ++i;

  if(i < sensors->availableSensors)
  {
    // Return integer of how many times function is yet to run
    return (sensors->availableSensors - i); 
  }
  else if (i == sensors->availableSensors) // All data sent
  {
    serialWrite("e~\n"); // Send transmission end marker
    i = -3; // Restore variable's starting state
    serial.hostReplied = false; // Reset flag to wait for the next "ok" signal
    return 0;
  }
  else // Something unpredictable happened
    return -1;

  //! It's probably not a good idea wait here, this could be handled in a other way
  /* You're right, this was left after testing here by mistake
  _delay_ms(1000);*/
}

int checkHost()
{
  // If host isn't up
  if(!serial.hostUp || !serial.hostReplied)
  {
    serial.hostUp = false;
    return -1;
  }
  else
    return 0;
}

int queryHost()
{
  static bool queryInProgress = false;
  static unsigned long startTime;
  
  if(serial.hostShut) // If host shut gates on purpose, no need to ask if it's still there
  {
    queryInProgress = false;
    return 1;
  }
  else if(serial.hostUp) // Host has answered
  {
    queryInProgress = false;
    return 0;
  }
  if(!queryInProgress) // For first query, save a timestamp
  {
    startTime = millis();
    queryInProgress = true;
  }
  if(HOST_QUERY_TIME > (unsigned long)(millis() - startTime))
  {
    serialWrite("?h\n"); // Ask host if it's alive
  }
  _delay_ms(HOST_QUERY_TIME/10); // Give host time to answer

  return -1; // Host hadn't answered yet
}

void interruptSetup()
{  
  cli(); //stop interrupts
  
  //set timer1 interrupt at 10Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 10hz increments
  OCR1A = 1562; // = (16*10^6) / (10*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS10) | (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei(); //allow interrupts
}

ISR(TIMER1_COMPA_vect) // timer1 interrupt
{
  power.voltage = 895.0 * (float)adConversion(VOLTAGE_SENSE_PIN) / 9207.0; 
  // Simplified Function of (vsense/1023)*board5v*(537/27), 
  // where 27/537 is ratio of a voltage divider
  
  //! There should be other ways than puttin preprocessor macros inside the code. This just makes the code unreadable
  /* Duly noted */

  if(power.voltage > OVERVOLTAGE && !DEBUG)
  {
    powerOff();
    power.criticalVFailure = true;
  }
  
  // Max accuracy ~0.5A, theoretically ~0.24 A
  power.current = 250.0 * ((float)adConversion(CURRENT_SENSE_PIN) 
                  - (float)adConversion(CURRENT_SENSE_REFERENCE_PIN)) / 1023.0;

  if(power.current > OVERCURRENT && !DEBUG)
  {
    powerOff();
    power.criticalCFailure = true;
  }

  if(power.voltage < LOW_VOLTAGE)
    power.lowVoltage = true;
  else if(power.voltage >= LOW_VOLTAGE)
    power.lowVoltage = false;

}

ISR(USART_RX_vect)
{
  // First check if ring buffer is full
  if((serial.in_e + 1) % READ_BUFFER_SIZE == serial.in_s) 
    serial.bufferFull = true;
  else
  {
    serial.bufferFull = false; // Clear flag if it was up

    serial.inputBuffer[serial.in_e] = UDR0; // Copy character to buffer
    if(serial.inputBuffer[serial.in_e] == '!') // If it's alert ('!'), power off
      powerOff();
    // If it's newline, buffer has a new command available to read
    if(serial.inputBuffer[serial.in_e] == '\n') 
      serial.commandAvailable = true;

    serial.in_e = (serial.in_e + 1) % READ_BUFFER_SIZE; // Advance end pointer
  }
}

int serialWrite(char* data)
{
  if(strlen(data) < WRITE_BUFFER_SIZE) // Confirm that string is short enough for the buffer
  {
    static unsigned char outputBuffer[WRITE_BUFFER_SIZE] = {0};
    strcpy(outputBuffer, data); // Copy string to buffer
    // Send characters to register until '\0' is hit
    for(int i = 0; i < WRITE_BUFFER_SIZE && outputBuffer[i] != 0; ++i)
    {
      // Wait buffer to empty
      while (!(UCSR0A & (1<<UDRE0)));
        UDR0 = outputBuffer[i];
    }
    return 0;
  }
  else
    return -1;
}

int serialWrite(float value)
{
  // Check if host's GUI can display number 
  if(value < GUI_MIN_FLOAT_VALUE || value > GUI_MAX_FLOAT_VALUE)
    return -1;
  
  static unsigned char modBuffer[WRITE_BUFFER_SIZE] = {0};
  // Translate float to string with one decimal and minimum of one number left of decimal point
  dtostrf(value, 1, 1, modBuffer);
  
  // Send string pointer to other serialWrite-function and use it's
  // return value as this functions return value
  return serialWrite(modBuffer);
}

int serialWriteTemperatureData(int i, float temperature)
{
  static unsigned char temperatureData[WRITE_BUFFER_SIZE] = {0};
  if(temperature > GUI_MIN_FLOAT_VALUE && temperature < GUI_MAX_FLOAT_VALUE)
  {
    if(temperature < MODULE_OVER_HEAT)
    {
      // Modify data to wanted form, ie. ":13--13.0\n"
      sprintf(temperatureData, ":%i-%s\n", i, temperature);
      serialWrite(temperatureData); // Write it out
    }
    else  
    {
      // If system is not locking up of actually overheating, 
      // it was a CRC error in transmission
      sprintf(temperatureData, ":%i-CRC\n", i);
      serialWrite(temperatureData); // Write it out
    }
  }
  else // Temperature is not valid
    return -1;
}

char* serialRead()
{
  if(serial.commandAvailable)
  {
    // Find the first '\n'
    byte end = serial.in_e;
    for(byte start = serial.in_s;  start != end; start = (start + 1) % READ_BUFFER_SIZE)
    {
      if(serial.inputBuffer[start] == '\n')
      {
        start = (start + 1) % READ_BUFFER_SIZE; // Set locator to next starting point; after '\n' char
        
        for(byte pos = 0; pos < CMD_CHAR_COUNT; ++pos) // Write command to it's array
        {
          serial.command[pos] = serial.inputBuffer[(start - CMD_CHAR_COUNT + pos) % READ_BUFFER_SIZE];
        }
        // Traverse array until command ending newline or null character
        while(start != end && (serial.inputBuffer[start] == '\n' || serial.inputBuffer[start] == '\0'))
        {
          start = (start + 1) % READ_BUFFER_SIZE; // Mark next character as a starting point
        }
            
        serial.in_s = start; // Save new starting point
          
        if(serial.in_s == serial.in_e) // If buffer length is 0
          serial.commandAvailable = false; // No new command available
  
        return serial.command;
      }
    }
  }
  // If loop didn't find full command or no command was even available, 
  // return null command here and declare no commands available
  serial.commandAvailable = false;
  sprintf(serial.command, "\0\0"); // sprintf appends last '\0' automatically
  return serial.command;
}

int trapLoop()
{
  // This loops until full power reset if critical conditions occurr,
  if(power.criticalCFailure)
  {
    serialWrite("!C\n");
  }
  if(power.criticalVFailure)
  {
    serialWrite("!V\n");
  }
  if(power.criticalTFailure)
  {
    serialWrite("!T\n");
  }
  if(power.lowVoltage)
  {
    serialWrite("!v\n");
  } 

  _delay_ms(1000);

  return 0;
}

int fanSetup()
{
  DDRB |= (1<<FAN_SWITCH_PIN); // fan switch pin to output
  OCR2A = FAN_MAX_DUTY_CYCLE; // write PWM value to register
  TCCR2A |= ((1 << COM2A1)); // Non-inverted mode
  TCCR2A |= ((1 << WGM21) | (1 << WGM20)); // Fast PWM mode
  TCCR2B |= ((1 << CS22) | (1 << CS00)); // Set prescaler to 1024

  return 0;
}

int setFanSpeed(int speed)
{
  if(speed < 256 && speed >= 0)
  {
    OCR2A = speed; // write PWM value to compare register
    return 0;
  }
  else
    OCR2A = FAN_MAX_DUTY_CYCLE;
    
  return speed;
}
