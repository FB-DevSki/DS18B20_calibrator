/*
 * ARDUINO_multiple_DS18x20_Temperature.cpp
 *
 * Created: 23-8-2017 19:55:11
 *  Author: SJK
 */ 
#include <OneWire.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// OneWire Temperature Calibrator for DS18B20 (open source version)
// Uses OneWire library
// max 12 sensors on one port
// S.J. Koster, Wageningen Instruments (2017)

//== variables and instances

OneWire  ds(11);  // on pin 11 (a 2k2 resistor for 12 sensors in parallel)
enum mode_type {DISCOVER, LIST, TIME_SERIES};
enum mode_type mode;

#define MAX_NUMBER_OF_SENSORS        14
uint8_t number_of_sensors			= 0;		//number of sensors detected
uint8_t counter						= 0;		//used for counting number of sensors
bool list_end_reached = false;
byte type_s;									//type depending on device, influences temperature format
byte data[12];									//contains most recent data incl measurement
byte send_data[4];								//send packet
byte addr[8];									//container for sensor address
byte sensor_address_list[MAX_NUMBER_OF_SENSORS][8];       //container for list of addresses
float celsius;									//most recent measurement data (any sensor)
char user_input[20];							//contains user input as a string
int8_t offset						= 0;		//offset correction as signed 8-bit
uint8_t sensor;									//sensor list position
uint8_t id;										//sensor id/location
#define RX_BUFFER_LENGTH			40
char Rx_buffer[RX_BUFFER_LENGTH];               //reception buffer for user interaction
uint8_t read_index = 0;							//read buffer write position
uint8_t message_waiting = 0;					//gives length of (complete) message waiting
volatile uint8_t itr;							//iterator general use, set to 0
volatile char sensorstring[6];                  //sensor list position as a string
volatile char offsetstring [6];                 //sensor offset as a string
volatile char idstring[6];						//sensor id as a string
uint8_t loops_to_series_switch =	10;


//===prototypes====
void discover(void);							//find and print info on all sensors = start mode
void start_conversion(byte* sensor_addr);       //start conversion on specified device
void read_scratchpad (byte* sensor_addr);       //read scratchpad from specified device
void print_data_array(void);					//print data from most recent sensor read
void print_sensor(byte* sensor_addr);           //print contents array data, must be read in first
void print_temperature_corrected(void);         //temperature corrected for offset and parabolic error
void set_sensor(byte list_number, byte* send_data);       //select sensor and send settings
void print_address(byte* sensor_addr);          //print sensor address
void set_id(byte list_number);                  //print list with id, temperature, offset and set id by list position
void print_list(void);							//print list with corrected temperatures and reference temp

void print_time_series(void);					//repeatedly print temperatures by id
void do_menu(void);								//prints options and processes commands
void parse (void);								//parser for human interaction
void read_serial(void);							//reads user input from serial (USB, command line)

//===setup===
void setup(void) {
  Serial.begin(9600);							//start console (USB-serial)
  mode = DISCOVER;
  Serial.print("One wire temperature sensors calibrator");
  Serial.println();
}

//==main loop===
void loop(void) {

  //check for messages
  read_serial();
  
  //process messages
  if(message_waiting){
    parse();
    message_waiting = 0;
  }
  
  // ==DISCOVER mode is cycled to discover all sensors, one-by-one
  if(mode == DISCOVER){
    if ( !ds.search(addr)) {
      Serial.println();
      Serial.println("--Ends--");
      list_end_reached = true;
      ds.reset_search();
      delay(250);
      return;
    }
    for(uint8_t i=0; i<8;i++){
      sensor_address_list[counter][i] = addr[i];
    }
    counter++;													//each time this line is passed, 
																//adjust number of sensors (will be 1 too many)
    if(counter > MAX_NUMBER_OF_SENSORS) counter = MAX_NUMBER_OF_SENSORS;

    Serial.print("ROM =");
    print_address(addr);
    Serial.println();
    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
      Serial.print("  Chip = DS18S20");							// or old DS1820
      type_s = 1;
      break;
      case 0x28:
      Serial.print("  Chip = DS18B20");
      type_s = 0;
      break;
      case 0x22:
      Serial.print("  Chip = DS1822");
      type_s = 0;
      break;
      default:
      Serial.print("Device is not a DS18x20 family device.");
      return;
    }
    Serial.println();
    start_conversion (addr);									//start conversion on this sensor 
																//(save some time by making broadcast command)
    read_scratchpad (addr);										//read scratchpad from specified device
    print_data_array();
  }

  if(list_end_reached) {
    mode = LIST;												//after all sensors have been discovered, go to next menu item
    list_end_reached = false;									//prepare for new discover, if selected by user
    Serial.println();
    Serial.print("Switch to list view");
    Serial.println();
    number_of_sensors = counter-1;								//correction for 1 too many of this variable
    counter = 0;												//prepare for next time
  }
  
  if (mode == LIST){
    // ==Lists all devices and user dialog, cyclic

																// broadcast 'convert'command to all sensors (saves time)
    ds.reset();													// reset all sensors
    ds.skip();													// set broadcast
    ds.write(0x44, 1);											// start conversion, with parasite power on at the end
    delay(1000);												// depends on conversion resolution

    //cycle through all addresses and get data

    for(uint8_t i=0; i< number_of_sensors; i++){
      read_scratchpad (sensor_address_list[i]);
      offset = (int8_t)data[2];
      Serial.print("select ");
      Serial.print(i);
      Serial.print(": ");
      print_sensor(sensor_address_list[i]);
    }															//set resolution to 12 bits
//    send_data[0] = 0;											//offset = 0 
//    send_data[1] = 0;											//id = 0
//    send_data[2] = 0x7F;										//12 bit resolution
//    set_sensor(0, send_data);									//do not send all the time and wear out sensor flash !
  }
  
  if (mode == TIME_SERIES){
    // broadcast 'convert'command to all sensors (saves time)
    ds.reset();													// reset all sensors
    ds.skip();													// set broadcast
    ds.write(0x44, 1);											// start conversion, with parasite power on at the end
    delay(1000);												// depends on conversion resolution

    for(uint8_t i=0; i< number_of_sensors; i++){
      read_scratchpad (sensor_address_list[i]);
      print_temperature_corrected();
      if ((i+1)< number_of_sensors) Serial.print(", ");
    }
    Serial.println();
  }
}

//========start conversion =======
//
void start_conversion (byte* sensor_addr) {
  ds.reset();
  ds.select(sensor_addr);
  ds.write(0x44, 1);											// start conversion, with parasite power on at the end
  delay(1000);													// depends on conversion resolution
}

//=======read scratchpad ========
//
void read_scratchpad (byte* sensor_addr){
  ds.reset();
  ds.select(sensor_addr);
  ds.write(0xBE);												// Slave prepare to send scratchpad
  for (uint8_t i = 0; i < 9; i++) {								// we need 9 bytes, put in data array
    data[i] = ds.read();
  }
}

//=======print_data ========
//
void print_data_array(void){
  Serial.print("  Data = ");
  for (uint8_t i = 0; i < 9; i++) {								// 9 bytes of data 0,1 is temp, 
																// 2.3 = user/alarms, 4 = settings, 5..9 CRC
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
}

//=======print_sensor ========
//
void print_sensor(byte* sensor_addr){
  print_address(sensor_addr);
  Serial.print(", id: ");
  Serial.print(data[3]);
  Serial.print(", offset: ");									//int8_t value (signed decimal)
  Serial.print(offset);
  Serial.print(", sens.set: ");
  Serial.print(data[4],HEX);
  Serial.print(", T: ");
  print_temperature_corrected();
  Serial.println();
}

//=======print_address ========
//
void print_address(byte* sensor_addr){
  for(uint8_t i = 0; i < 8; i++) {								//8 bytes address
    Serial.write(' ');
    Serial.print(sensor_addr[i], HEX);
    Serial.print(" ");
  }
}

//=======print_temperature_corrected ========
//temperature corrected for offset and parabolic error
//
void print_temperature_corrected(void){

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
    } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  // get offset from latest read
  offset = (int8_t)data[2];
  celsius = (float)raw / 16.0;
  celsius -= (float)offset/100;									//temperature correction, from sensor calibration
  celsius += 0.2 - (abs(celsius - 20) * .005);					//parabolic correction
  Serial.print(celsius);
}

//=======set sensor ========
//write user settings to sensor flash
//write values to all 3 user bytes  2,3,4
//must be in send_data*
void set_sensor(byte list_number, byte* send_data){
  //should check if sensor[list_number] exists
  ds.reset();
  ds.select(sensor_address_list[list_number]);
  ds.write(0x4E);												// device prepare for read of byte 2, 3, 4 to Scratchpad
  delay(10);
  ds.write_bytes(send_data, 3, 0);								// master write Scratchpad, 
																// data, no bytes, parasitic power

  ds.reset();
  ds.select(sensor_address_list[list_number]);
  ds.write(0x48);												//copy scratchpad to register
  delay(20);													//time for sensor to process (datasheet)
}

//=======read_serial ========
//reads user input from serial (USB, command line)
//ATmega 328 has 20 Byte buffer (?)
//NB could read while still receiving data will lead to error
void read_serial(void){
  char inChar;
  while (Serial.available() > 0)								// Don't read unless you know there is data
  {
    if(read_index < 19)											// One less than the size of the array
    {
      inChar = Serial.read();									// Read a character
      Rx_buffer[read_index] = inChar;							// Store it
      read_index++;												// Increment where to write next
    }
  }
  //see if complete message has been received
  if(read_index > 0) {
    if ((Rx_buffer[read_index-1] == '\n')||(Rx_buffer[read_index-1] == '\r')){
      message_waiting = read_index;
      read_index = 0;
    }
  }
}

//=======parse ========
//interprets message and initiates actions
//commands: t (time series mode), l (list), o (set offset), i (set id)
//o and i followed by list position and value, separated by ','
//
void parse (void){
  uint8_t length = 0;
  uint8_t command = 0;
  uint8_t period_position;
  //find CR (end)
  for (uint8_t i = 1; i< RX_BUFFER_LENGTH; i++){
    if((Rx_buffer[i] == '\n') || (Rx_buffer[i] == '\r')) length = i;
  }
  if(length > 0){
    command = Rx_buffer[0];
    switch (command) {
      
      case 't': case 'T':
      mode = TIME_SERIES;
      break;
      
      case 'l': case 'L':
      mode = LIST;
      break;
      
      case 'o': case 'O':
      itr = 2;													//an 'o' has been received, next will be ',' sensor id starts at 2
      while ((Rx_buffer[itr] != ',') && (itr < length)) {		//data will be written to sensorstring, starting at pos 0
        sensorstring[itr-2] = Rx_buffer[itr];					//until next ',' or end of string
        itr++;
      }
      sensorstring[itr] = '\0';									//close with eos (\0) so string functions can be used
      sensor = (uint8_t) atoi(sensorstring);					//convert to integer (0:255)	
	    
      period_position = itr;
      itr++;
      while (itr < length) {									//data starts after second ',' = sensor listpos
        offsetstring[itr-period_position-1] = Rx_buffer[itr];
        itr++;
      }
      offsetstring[itr] = '\0';

 
      offset = (int8_t) atoi(offsetstring);						//convert to signed integer(-218:128)
 	  
      // fill in send_data with settings of this sensor
      start_conversion (sensor_address_list[sensor]);			//start conversion on this sensor 
																// (save some time by making broadcast command)
      read_scratchpad (sensor_address_list[sensor]);			//read scratchpad from specified device
																//this will fill 'data' array
      send_data[0] = offset;									//3 bytes must be sent, first is offset
      send_data[1] = data[3];									//second is id, same as before
      send_data[2] = data[4];									//third is conversion settings, same as before
      //fill in new value
      //set sensor
      set_sensor(sensor, send_data);							//write to device
      break;
      
      case 'i': case 'I':
      mode = LIST;
      itr = 2;													//an 'i' has been received, next will be ',' sensor id starts at 2
      while ((Rx_buffer[itr] != ',') && (itr < length)) {
        sensorstring[itr-2] = Rx_buffer[itr];
        itr++;
      }
      sensorstring[itr] = '\0';
      sensor = (uint8_t)atoi(sensorstring);						//do this here, when you do it later, the data is lost (optimizer ??)
	  
      period_position = itr;
      itr++;
      while (itr < length) {
        idstring[itr-period_position-1] = Rx_buffer[itr];
        itr++;
      }
      idstring[itr] = '\0';
      id = (uint8_t)atoi(idstring);
      
      // fill in send_data with settings of this sensor
      start_conversion (sensor_address_list[sensor]);			//start conversion on this sensor 
																//(save some time by making broadcast command)
      read_scratchpad (sensor_address_list[sensor]);			//read scratchpad from specified device
																//this will fill data array
      send_data[0] = data[2];									//3 bytes must be sent, first is offset, same as before
      send_data[1] = id;										//second is id
      send_data[2] = data[4];									//third is conversions settings, same as before
      //set sensor
      set_sensor(sensor, send_data);							//send to device
      break;
      
      default:
      mode = LIST;
      Serial.println();
      Serial.print("Invalid command use: t (time series), l (list view), o(set offset), i(set id) ");
      Serial.println();
      break;
    }
  }
  else{
    // no \n or \n found, start over
    read_index = 0;
    message_waiting = false;
  }
}

