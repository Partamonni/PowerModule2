/* K-ev Power-Module script v2.0

   Written by Henri Pyoria (henri.pyoria@outlook.com)

   Temperature-IC addresses must be manually defined
   to tempAdr table. This skips teh lengthy search for 'em.
   IC's must support resolution configuration! (10bit conversion is used)

   2s watchdog implemented after the setups ready, in case of odd failures.
*/


#define F_CPU 16000000UL
#define BAUD 2400
#define BYTE 8

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

#define WRITE_BUFFER_SIZE 10
#define READ_BUFFER_SIZE 100
#define TMP_COUNT 40
#define FAN_MAX_DUTY_CYCLE 255
#define CMD_CHAR_COUNT 3 // Module receives only specified length commands ending in '\n'. Count excludes the '\0' char.
#define OVERCURRENT 50
#define OVERVOLTAGE 85
#define LOW_VOLTAGE 10 //60
#define OVER_HEAT 60

#define HOST_QUERY_TIME 5000
// ms, time to wait host to answer back before shutting gates
#define EEPROM_SAVE_INTERVAL 3600000
#define READ_TIME 200
// ms, for temperature sensors to sample readings and while monitoring is on


OneWire ds(10);

const byte tempAdr[TMP_COUNT][8] PROGMEM =
{ 
  //{0x28, 0xBA, 0x4B, 0x41, 0x09, 0x00, 0x00, 0xAE},
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
  {0x28, 0x8D, 0x3B, 0x41, 0x09, 0x00, 0x00, 0xAB},
  {0x28, 0x7B, 0x38, 0x41, 0x09, 0x00, 0x00, 0x25},

  {0x28, 0xF4, 0x2E, 0x41, 0x09, 0x00, 0x00, 0x0B}
};
byte data[TMP_COUNT][9];

struct Command
{
  unsigned char ch[CMD_CHAR_COUNT] = {'\0', '\0', '\0'};
};

byte curSense = 0; // PORTC
byte vsense = 1; // PORTC
byte curSenseRef = 3; // PORTC
byte mosfetSwitch = 4; // PORTC
byte fanSwitch = 3; // PORTB

volatile float current;
volatile float voltage;
volatile float board5v;
unsigned int raw;
float ms;
float msEEPROM;
float temperature[TMP_COUNT];
byte availableSensors = 0;

volatile float coulombCounter = 0; //Ah
volatile float lastCurrents[100] = {0}; //A
const float defaultCapacity = 80; //Ah
float stateOfHealth = 1; // x100%
float soc = 1; // x100%
const float voltageFull = 84; //V
const float voltageEmpty = 66; //V
const float current0_5C = 41.5; //A
const float current0_3C = 24.9; //A
const float current0_1C = 8.3; //A
const float dischargeVChange1Ah = 0.35; //V
const float ccChargeEndCapacity = 71; //Ah

unsigned char serialInput[READ_BUFFER_SIZE] = {0};
byte in_s = 0;
byte in_e = 0;
bool bufferFull = false;
bool commandAvailable = false;

unsigned char outputBuffer[WRITE_BUFFER_SIZE] = {0};
unsigned char modBuffer[WRITE_BUFFER_SIZE] = {0};

bool success = false;
bool setupSuccess = false;
bool criticalCFailure = false;
bool criticalVFailure = false;
bool criticalTFailure = false;
bool lowVoltage = false;
bool hostUp = false;
bool hostShut = false;
bool hostReplied = false;

void serialSetup();
void interruptSetup();

void systemMonitoring();
void readTemps();
int calcSOC();
void sendData();
bool checkIfShut();
void checkHost();

unsigned int adConversion(byte);
byte* getTempAdr(byte n);

void setup()
{
  DDRC |= (1<<mosfetSwitch); // mosfetSwitch pin to ouput
  PORTC &= ~(1<<mosfetSwitch); // write 0 to that pin
  
  DDRC |= (1<<mosfetSwitch); // mosfetSwitch pin to ouput
  PORTC &= ~(1<<mosfetSwitch); // write 0 to that pin

  wdt_disable();
  wdt_reset();

  // PWM output on pin 11 (PortB:3)
  DDRB |= (1<<3); // fanSwitch pin to output
  OCR2A = FAN_MAX_DUTY_CYCLE; // write PWM value to register
  TCCR2A |= ((1 << COM2A1)); // Non-inverted mode
  TCCR2A |= ((1 << WGM21) | (1 << WGM20)); // Fast PWM mode
  TCCR2B |= ((1 << CS22) | (1 << CS00)); // Set prescaler to 1024

  DIDR0 = B00101111; // Disable digital buffers to enable analog input and conserve power 

  serialSetup();
  
  sprintf(outputBuffer, "st\n");
  serialWrite();
  
  interruptSetup();

  _delay_ms(250);
  if(current > 0.5 || current < -0.5)
  {
    do
    {
      sprintf(outputBuffer, "!C\n");
      serialWrite();  // Board current failure
      _delay_ms(1000);
    }
    while (!DEBUG);
  } // These trap loops are to send alert to user interface
  // They can be cleared only by full power cycle (for now, atleast)
    
  if(voltage > OVERVOLTAGE)
  {
    do
    {
      sprintf(outputBuffer, "!V\n");
      serialWrite();  // Board voltage failure
      _delay_ms(1000);
    }
    while(!DEBUG);
  }
  else if(voltage < LOW_VOLTAGE)
  {
    do
    {
      sprintf(outputBuffer, "!v\n");
      serialWrite();  // Board voltage failure
      _delay_ms(1000);
    }
    while(!DEBUG);
  }
  
  #if !DEMO
  msEEPROM = millis();
  byte tmp[4];
  for (int i = 0; i < 4; ++i)
  {
    tmp[i] = EEPROM.read(i);
  }
  memcpy(&coulombCounter, tmp, 4);
  
  for (int i = 0; i < 4; ++i)
  {
    tmp[i] = EEPROM.read(i+4);
  }
  memcpy(&stateOfHealth, tmp, 4);
  
  // If no EEPROM value ->
  if(coulombCounter == 0)
  { // Initialize SOC to an approximate with voltage
    coulombCounter = (defaultCapacity*stateOfHealth)*((voltage-voltageEmpty)/(voltageFull-voltageEmpty));
  }
  
  if(stateOfHealth == 0)
    stateOfHealth = 1;
    
  #endif
  
  if(!ds.reset())
  {
    do
    {
      sprintf(outputBuffer, "!b\n");
      serialWrite();  // Sensor bus failed
      _delay_ms(1000);
    }
    while (!DEBUG);
  }

  // Write config command and config values to all devices
  byte setCnf[4] = {0x4E, 0x7C, 0x0A, B00111111}; 
  // Write to scratchpad byte, TH&TL bytes, configuration byte
  ds.reset(); // reset 1-Wire
  ds.write(0xCC); // Skip ROM select
  ds.write_bytes(setCnf, 4);
  ds.reset(); // reset 1-Wire

  if(!(voltage >= 999.9 || current >= 999.9 || current <= -99.9)) 
  {
    sendToModBuffer(current);
    sprintf(outputBuffer, ":c%s\n", modBuffer);
    serialWrite(); 
    
    sendToModBuffer(voltage);
    sprintf(outputBuffer, ":v%s\n", modBuffer);
    serialWrite();
  }

  setupSuccess = true;
  wdt_reset();
  wdt_enable(WDTO_2S);
}

void loop()
{
  wdt_reset();

  if(!ds.reset())  // Sensor bus failed
  {
    sprintf(outputBuffer, "!b\n");
    serialWrite(); 
  }
  else
  {
    ds.reset();
    ds.write(0xCC); // Skip ROM select
    ds.write(0x44); // Command to start measurement  
    ms = millis(); // Store current time
    while (READ_TIME >= (unsigned long)(millis() - ms)) // Secondary loop
      systemMonitoring();     
    // Do over 200ms if no extra timing is given for temp sensors to do conversion
  
    wdt_reset();
  }

// Secondary loop completed

  readTemps(); // Read and store temperatures
  wdt_reset();

  //calcSOC(); // To be finalized
  //wdt_reset();

  checkHost();  // Check if host is up and set gate status accordingly
  systemMonitoring();     

  sendData(); // Send gathered data
  
  #if !DEMO
  // Save coulombCounter to EEPROM once an hour
  // EEPROM is calculated to quaranteedly withstand 10y with this interval
  if(EEPROM_SAVE_INTERVAL <= (unsigned long)(millis() - msEEPROM))
  {
    byte tmp[4];
    memcpy(tmp, &coulombCounter, 4);
    for (int i = 0; i < 4; ++i)
      EEPROM.update(i, tmp[i]);
      
    memcpy(tmp, &stateOfHealth, 4);
    for (int i = 0; i < 4; ++i)
      EEPROM.update(i+4, tmp[i]);

    msEEPROM = millis(); // Store current time
  }
  #endif
}

void systemMonitoring()
{
  if(criticalCFailure || criticalVFailure || criticalTFailure)
  {
    do
    {
      if(criticalCFailure)
      {
        sprintf(outputBuffer, "!C\n");
        serialWrite(); 
      }
      if(criticalVFailure)
      {
        sprintf(outputBuffer, "!V\n");
        serialWrite(); 
      }
      if(criticalTFailure)
      {
        sprintf(outputBuffer, "!T\n");
        serialWrite(); 
      }
      if(lowVoltage)
      {
        sprintf(outputBuffer, "!v\n");
        serialWrite(); 
      }
      #if !DEBUG
      _delay_ms(1000);
      #endif
      wdt_reset();
    }
    while(!DEBUG);
  }  
  else if(lowVoltage)
  {
    do
    {
      sprintf(outputBuffer, "!v\n");
      serialWrite();
      #if !DEBUG
      _delay_ms(1000);
      #endif
      wdt_reset();
    }
    while(!DEBUG && lowVoltage);
    if(!hostShut && !DEBUG)
    {
      sprintf(outputBuffer, "up\n");
      serialWrite();
    }
  }
  else if(!checkIfShut() && hostUp && !(PORTC & (1<<mosfetSwitch)))
  {
    // This is the only place where gate is opened
    PORTC |= (1<<mosfetSwitch); // digitalWrite(mosfetSwitch, HIGH);
    sprintf(outputBuffer, "up\n");
    serialWrite();
  }
    
}

void readTemps()
{
  if(!availableSensors)
  {
    for (int i = 0; i < TMP_COUNT; ++i)
    {
      if(pgm_read_byte(tempAdr[i][0]) != 0x00) // This checks if sensors' address table entry has any address in it
      {
        ++availableSensors;
      }
      else
        break;
    }
  }
  else
  {
    for(int i = 0; i < availableSensors; ++i)
    {
      ds.reset();
      ds.select(getTempAdr(i));
      // Device selected, commanding to read out the scratchpad");
      
      ds.write(0xBE); // Request sensor to read out the scratchpad
      for (byte j = 0; j < 9; ++j) // Read all 9 bytes
      {
        data[i][j] = ds.read();
      }
      // Data read
    
      if(OneWire::crc8(data[i], 8) != data[i][8])
      { // If crc is wrong, set all bits to 1
        for (byte k = 0; k < 9; ++k)
          data[i][k] = 0xFF;
      }
    }
  }

  int16_t rawTemp;
  for (byte i = 0; i < availableSensors; ++i)
  {
    rawTemp = 0;
    rawTemp = (data[i][1] << BYTE) | data[i][0];
    // ^ Reads data to temporary container
    if(rawTemp != 0xFFFF)
    {
      rawTemp &= ~3;
      // ^ Sets last 2 bits to zero,
      // as 10-bit resolution doesn't use them
  
      temperature[i] = 85.0 * (float(rawTemp) / 1360.0);
  
      if(i == 0 && temperature[i] >= 80) // 80'C is max for most chips
      { // This sensor will be in the mosfet heatsink
        PORTC &= ~(1<<mosfetSwitch);
        criticalTFailure = true;
      }
      else if(i != 0 && temperature[i] >= OVER_HEAT)
      {
        PORTC &= ~(1<<mosfetSwitch);
        criticalTFailure = true;
      }
    }
    else
      temperature[i] = 999.9;
  }
}

unsigned int adConversion(byte ch)
{
  ADMUX = ((1<<REFS0)| ch); // Select pin and voltage reference
  
  ADCSRA = B11000111; // Start conversion (first 2 bits from left) with prescaler 128 (last 3 bits)
  while ((ADCSRA & (1<<ADSC)) != 0) {}; // Wait for status bit to flip
  // First reading is garbage if MUX was changed, so lets just forget this value
  
  ADCSRA = (1<<ADSC); // Start conversion again
  while ((ADCSRA & (1<<ADSC)) != 0) {};

  return ADC;
}

byte* getTempAdr(byte n)
{
  static byte tmp[8];
  for (int i = 0; i < 8; ++i)
    tmp[i] = pgm_read_byte_near(tempAdr[n]+i);
    
  return tmp;
}

void sendData()
{
  systemMonitoring();  
  if(PORTC & (1<<mosfetSwitch))
  {
    sprintf(outputBuffer, "up\n");
    serialWrite(); 
  }
  else
  {
    sprintf(outputBuffer, "sh\n");
    serialWrite(); 
  }
  
  if(!(voltage >= 999.9 || current >= 999.9 || current <= -99.9)) 
  {
    sendToModBuffer(current);
    sprintf(outputBuffer, ":c%s\n", modBuffer);
    serialWrite(); 
    sendToModBuffer(voltage);
    sprintf(outputBuffer, ":v%s\n", modBuffer);
    serialWrite();
  }
  
  systemMonitoring();
  for (byte i = 0; i < availableSensors; ++i)
  {
    if(!(temperature[i] > 125.0))
    {
      sendToModBuffer(temperature[i]);
      sprintf(outputBuffer, ":%i-%s\n", i, modBuffer);
      serialWrite(); 
      systemMonitoring();
      wdt_reset();
    }
    else
    {
      sprintf(outputBuffer, ":%i-CRC\n", i);
      serialWrite(); 
      systemMonitoring();
      wdt_reset();
    }
      
  }
    sprintf(outputBuffer, "e~\n");
    serialWrite();
    _delay_ms(1000);
}

bool checkIfShut()
{
// Check all received data

  bool rtrn = false;
  while(commandAvailable)
  {
    static Command cmdCheck;
    cmdCheck = serialRead();
    
    if(cmdCheck.ch[0] == '!' && cmdCheck.ch[1] == 'S')
    {
      // Shut if needed
      PORTC &= ~(1<<mosfetSwitch); // digitalWrite(mosfetSwitch, LOW)
      hostShut = true;
      hostUp = false;
      rtrn = true;
      // Inform host
      sprintf(outputBuffer, "sh\n");
      serialWrite(); 
      
      // Gate is now shut by host
    }
    else if(cmdCheck.ch[0] == 'o' && cmdCheck.ch[1] == 'k')
    {
      // Host wants power on
      hostShut = false;
      hostUp = true;
      hostReplied = true;
      rtrn = false;
    }
    // Otherwise command was garbage/unknown, skip it and read next
  }
  return rtrn;
}

void checkHost()
{
#if HOST_QUERY_ON

  if(!hostReplied)
  {
    hostUp = false;
    checkIfShut();
  }
  else
    hostReplied = false;
  
  // Check if host answered "ok", otherwise checkHost() is now finished.
  if(!hostUp)
  { 
    // If not, try to reach it
    ms = millis();
    while (!(hostUp || hostShut) && HOST_QUERY_TIME > (unsigned long)(millis() - ms))
    {
      sprintf(outputBuffer, "?h\n");
      serialWrite(); 
      systemMonitoring();
      _delay_ms(HOST_QUERY_TIME / 10);
      
      wdt_reset();
    } 
    // If not reachable (or host just now shut the power), shut the gates
    if(!hostUp)
    {
      PORTC &= ~(1<<mosfetSwitch); // digitalWrite(mosfetSwitch, LOW)
      
      // Host not reached, gates are now shut
    }
  }
  
#else
    checkIfShut();
#endif
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
  // Selecting internal 1.1V voltage reference to MUX gets us the basepoint calculating the real voltage
  board5v = 1125.3 / (unsigned int)adConversion(0b1110) ; // Simplified Function of 5V*((1.1/5)*1023)/vref1v1
  voltage = 179.0 * (unsigned int)adConversion(vsense) * board5v / 9207; // Simplified Function of (vsense/1023)*board5v*(537/27)

  #if !DEBUG
  if(voltage > OVERVOLTAGE)
  {
    PORTC &= ~(1<<mosfetSwitch);
    criticalVFailure = true;
  }
  #endif
  
  // Can measure accuracy to ~0.25A
  current = board5v * (((unsigned int)adConversion(curSense) - (unsigned int)adConversion(curSenseRef)) / 1023) / 0.02;

  #if !DEBUG
  if(current > OVERCURRENT)
  {
    PORTC &= ~(1<<mosfetSwitch);
    criticalCFailure = true;
  }

  if(voltage < LOW_VOLTAGE)
    lowVoltage = true;
  else if(voltage >= LOW_VOLTAGE)
    lowVoltage = false;
  #endif


  static byte i = 0;
  static float currentHelper = 0;
  coulombCounter += current / 360000; // Simplified from: current*0.1/60/60
                                      //                      100ms^  ^m  ^h
  ++i;                              
  if(i < 10) // get average current on 1000ms period
    currentHelper += current;
  else
  { // This adds new current entry after every 1000ms
    currentHelper /= 10;
    memmove(&lastCurrents[1], &lastCurrents[0], sizeof(float) * 99);
    lastCurrents[0] = currentHelper;
    currentHelper = 0;
    i = 0;
  }

}

ISR(USART_RX_vect)
{
  static byte tmp = UDR0;
  if(tmp == '!')
    PORTC &= ~(1<<mosfetSwitch);
    
  if((in_e+1) % READ_BUFFER_SIZE == in_s)
    bufferFull = true;
  else
  {
    bufferFull = false;
    serialInput[in_e] = tmp;
    if(serialInput[in_e] == '\n')
      commandAvailable = true;
    in_e = (in_e+1) % READ_BUFFER_SIZE;
  }
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

void serialWrite()
{
  for(int i = 0; i < WRITE_BUFFER_SIZE && outputBuffer[i] != 0; ++i)
  {
    // Wait buffer to empty
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = outputBuffer[i];
  }
  memset(outputBuffer, 0, sizeof(outputBuffer));
  wdt_reset();
}

void sendToModBuffer(float value)
{
  // AVR version of sprintf can't handle float numbers, so this function modifies it to a modBuffer char array
  memset(modBuffer, 0, sizeof(modBuffer));
  dtostrf(value, 1, 1, modBuffer);
}

Command serialRead()
{
  static Command cmd;
  if(commandAvailable)
  {
    // Find the first '\n'
    byte e = in_e;
    for(byte s = in_s;  s != e; s = (s+1) % READ_BUFFER_SIZE)
    {
      if(serialInput[s] == '\n')
      {
        s = (s+1) % READ_BUFFER_SIZE; // Set locator to next starting point; after '\n' char
      
        for(byte pos = 0; pos < CMD_CHAR_COUNT; ++pos)
        {
          cmd.ch[pos] = serialInput[(s-(CMD_CHAR_COUNT)+pos) % READ_BUFFER_SIZE];
        }
        
        while(s != e && (serialInput[s] == '\n' || serialInput[s] == '\0'))
        {
          s = (s+1) % READ_BUFFER_SIZE;
        }
          
        in_s = s;
        
        if(in_s == in_e) // If buffer length is 0
          commandAvailable = false;
        /*  
        sprintf(outputBuffer, "cmd: %i ", in_s);
        serialWrite();
        sprintf(outputBuffer, "%i\n\n", in_e);
        serialWrite();
        _delay_ms(10);
        */
        return cmd;
      }
    }
  }
  // If loop didn't find full command or no command was even available, 
  // return null command here and declare no commands available
  commandAvailable = false;
  sprintf(cmd.ch, "\0\0"); // sprintf appends last '\0' automatically
  return cmd;
}

int calcSOC()
{
  static float capacity = stateOfHealth * defaultCapacity;
  soc = (capacity - coulombCounter) / capacity;

  static bool cvCharging = false;

  // Cell manufacturer has given discharge curve for the cells at 0.5C current,
  // and 1V voltage drop equals roughly 0.35Ah for a cell.
  // We use that for estimating the SOC.

  bool nominalUsage = true;
  bool lightUsage = true;
  cli(); // disable interrupts for a brief moment
  for (byte i = 0; i < 100; ++i)
  {
    if(lastCurrents[i] > current0_5C)
    {
      nominalUsage = false;
      lightUsage = false;
    }
    else if(lastCurrents[i] < current0_3C)
    {
      nominalUsage = false;
      if(lastCurrents[i] > current0_1C && lastCurrents[i] < 0)
      {
        lightUsage = false;
      }
    }
    if(!lightUsage && !nominalUsage)
      break;
  }
  sei();

  if(nominalUsage)
  {
    float socHelper =
      (capacity - (32 * (82 - voltage) / dischargeVChange1Ah)) / capacity;
    // 32 is number of parallel cells, thus used capacity is 32 times that of a single cell

    socHelper += soc * 99;
    socHelper /= 100;
    soc = socHelper;
    // Function reaches 90% of end value in about 45 seconds when execution happens every 200ms
  }
  else if(lightUsage)
  {
    if(voltage < voltageFull)
    {
      float socHelper = capacity*((voltage-voltageEmpty)/(voltageFull-voltageEmpty));
      socHelper += soc * 9999;
      socHelper /= 10000;
      soc = socHelper;
    // Function reaches 90% of end value in about 76 minutes when execution happens every 200ms
    }
  }

  if(voltage <= voltageEmpty && current < current0_1C)
  {
    soc = 0;
    if(coulombCounter < defaultCapacity)
    {
      stateOfHealth = coulombCounter / defaultCapacity;
    }
  }
  else if(  !cvCharging 
            && voltage >= voltageFull
            && current < 0 
            && current >= -current0_5C  )
  {
    soc = 2220 / defaultCapacity;
    cvCharging = true;
  }
  else if(cvCharging && current >= 0)
    cvCharging = false;
}
