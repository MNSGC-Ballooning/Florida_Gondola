#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#include <Adafruit_LSM9DS1.h>

#include <Adafruit_LSM9DS1.h>

#include <SPI.h>

#include <SD.h>

/**********************************************************************
   NAME: HAB_Transmitter.ino

   PURPOSE: AFOSR-MURI HIGH ALTITUDE BALLOON - Transmitter.

   DEVELOPMENT HISTORY:
     Date    Author  Version            Description Of Change
   --------  ------  -------  ---------------------------------------
   02/23/2018   NMG     1.1    Scientific packets included and sync.
   03/02/2018   NMG     1.2    GPS sensors included.
   03/20/2018   NMG     1.3    Cutting System Included.
   06/04/2018   NMG     1.4    SD Card System Included.
   06/08/2018   NMG     1.5    Watchdog Timer Included.
   07/20/2018   NMG     2.1    Data Packet and SD File Changes.
   08/09/2018   NMG     2.2    Scientific Packets With GPS Info.
   10/10/2018   NMG     3      Packets Structure Changes.
   01/12/2018   NMG     3.1    Cutting System I2C-SPI Changes.
   01/15/2018   NMG     4      Code Adapted to Teensy 3.5.
   01/15/2018   NMG     4.1    Code Restructured.
   03/09/2018   NMG     4.2    Re-send data implementation.
   03/13/2018   NMG     4.3    External Watchdog Timer Included.
   05/10/2019   NMG     4.4    GPS packets not considered.
   07/02/2019   NMG     4.5    Teensy ADC Reference - Flow Control.
   10/11/2019   NMG     5      Bluetooth Communication with CDU.
   01/23/2020   PMD     5.1    Additional comments and improved documentation
   02/25/2020   REC     5.2    Change to valve release commands
   02/26/2020   PMD     5.3    Documentation and congruency with CDU code

   05/07/2020   REC     SWE1.0  Mod flight code for sweeden use, stripping out radio
                                removed cdu control from main system (no valve)
                                see multi_ballon_ref folder for potential cdu code
                                tbd: reconfigure cdu data to match  2 balloon system
                                     modify science packet to fit sample data
                                     interface with arduino making sample data (or sensors)
                                     
   05/27/2020   JWM     SWE1.1  Replaced send_sci_packet with write_xxx_data. Modified
                                science packet to fit sample data. Interfaces with
                                two external boards that send sample data. Updated Serial 
                                definitions. Added periodic saving of SD card.
                                tbd: reconfigure cdu system and data to match 2 balloon system
                                     remove unnecessary variables 

   05/28/2020   JWM     SWE1.2  Removed some unnecessary variables. Slightly modified 
                                packet formats - added ERAU time stamp to every one. Moved
                                ERAU time stamp recording next to GPS time stamp recording.
                                Reduced time between SD card saves to 1.4 sec.

   06/11/2020   JWM     SWE1.3  Separated GPS measurements from ERAU measurements. Declared array
                                sizes to be the minimum necessary. Initialized arrays with zeros.
                                Organized new variable declarations. Added pre-launch serial interface.

   07/03/2020   JWM     SWE1.4  Added CDU control. Moved global variable declarations and macros 
                                to separate header file. Removed CDU temps from ERAU packet.
   07/06/2020   REC     SWE1.5  Fixes:  Changed BT communication to match UMN (9600).  
                                Changed cdu identifier to match correct hex interpretation of flags.  
                                Changed cdu parsing to wait for full cdu sentence before parsing data (failure to do so results in string filled with 0xFF).
                                Changed definations.h to reflect updated UMN packets.
                                Removed hard coded values for packet lenghts and replaced with CDU_RX_SIZE from definations.h
                                
 *********************************************************************/
/******SERIAL DEFINITIONS*******
  Startup Monitor     Serial
  Turbulence UART     Serial1
  SPS30 UART          Serial2  
  GPS UBlox           Serial3
  Bluetooth 1         Serial4
  Bluetooth 2         Serial5
********************************/

/*
*      Note: This program requires use of Teensyduino version 1.48
*      (newer versions edit the ADC.h file and make it unusable): https://www.pjrc.com/teensy/td_148/TeensyduinoInstall.exe
*      The version of Arduino which is compatible with Teensyduino 1.48 is Arduino 1.8.10: https://www.arduino.cc/download_handler.php?f=/arduino-1.8.10-windows.exe
*      Do not use the NMEAGPS.h file from Arduino's library manager as this is the wrong version.
*      Use the one on the project OneDrive within the "NeoGPS-master" folder
*/

#include <NMEAGPS.h>          // National Marine Electronics Association. Needed for Ublox GPS
#include <GPSport.h>          // This file declares the serial port toe be used for the GPS device
#include <EEPROM.h>           // Electrically-Erasable Programmable Road-Only Memeory
#include <SPI.h>              // Serial Peripheral Interface - communication protocol
#include <SD.h>               // Secure Digital. The SD library allows for reading from and writing to SD cards.
#include <avr/wdt.h>          // AVR watchdog timer
#include <Adafruit_LSM9DS1.h> // 9 Degrees of freedom sensor (accelerometer)
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <ADC.h>              // analog to digital conversion
#include "definitions.h"      // Program variable and macro definitions

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*********************** MAIN PROGRAM SETUP *****************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); //Serial Monitor
  Serial1.begin(115200);
  Serial2.begin(115200);

  /***********INITIALIZE ARRAYS***********/
  for(write_counter = 0; write_counter < GPS_SIZE; write_counter++) {
    erauPacket[write_counter] = 0;
  }
  for(write_counter = 0; write_counter < SPS_SIZE; write_counter++) {
    umnPacket[write_counter] = 0;
  }
  for(write_counter = 0; write_counter < RAW_SIZE; write_counter++) {
    cuPacket[write_counter] = 0;
  }
  for(write_counter = 0; write_counter < CDU_RX_SIZE; write_counter++) {
    cdu1Packet_rx[write_counter] = 0;
    cdu2Packet_rx[write_counter] = 0;
  }
  for(write_counter = 0; write_counter < SIZE; write_counter++) {
    latitude[write_counter] = 0;
    longitude[write_counter] = 0;
    Altitude[write_counter] = 0;
    timeStamp[write_counter] = 0;
    fixStatus[write_counter] = 0;
  }

  /************IMU SENSOR SETUP**********/
  lsm.begin();
  setupSensor();

  analogReadResolution(10);
  //adc->setAveraging(8);
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  //adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);

  /************SD CARD**************/
  sdCount = 0;
  if (!SD.begin(chipSelect)) { // CHANGE BACK TO WHILE
    delay(200);
  }
  //Read and set file number and name
  address = 0;
  fileNum = EEPROM.read(address);
  fileNum = fileNum + 1;
  EEPROM.write(address, fileNum);
  fileN =  "data";
  String m1 = fileN + fileNum;
  fileName1 = m1 + ext;

  //Open file
  flightData = SD.open(fileName1.c_str(), FILE_WRITE);
  timeSD = millis();

  /************GPS SERIALS**********/
  
  gpsuBloxExt.begin(9600);
  packet_number = 0;
  time_packet = millis();
  cdu_timer = millis();

  pinMode(FIX_3D_LED, OUTPUT);
  digitalWrite(FIX_3D_LED, LOW);

  /************BLUETOOTH SETUP**********/
  Serial4.begin(9600);
  cdu1_watchdog = millis();

  Serial5.begin(9600);
  cdu2_watchdog = millis();

  /*********WATCHDOG TIMER*********/
  noInterrupts();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);

  WDOG_TOVALH = 0x006d;
  WDOG_TOVALL = 0xdd00;
  WDOG_PRESC  = 0x400;

  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////**************** MAIN PROGRAM FUNCTION DEFINITIONS *********************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void write_gps_data() {
  
  erauPacket[8] = lat1;
  erauPacket[9] = lat1 >> 8;
  erauPacket[10] = lat1 >> 16;
  erauPacket[11] = lat1 >> 24;

  erauPacket[12] = lon;
  erauPacket[13] = lon >> 8;
  erauPacket[14] = lon >> 16;
  erauPacket[15] = lon >> 24;

  erauPacket[16] = alt;
  erauPacket[17] = alt >> 8;
  erauPacket[18] = alt >> 16;
  erauPacket[19] = alt >> 24;

  erauPacket[20] = stat;

  erauPacket[21] = numSats;

  erauPacket[22] = utcHour;
  erauPacket[23] = utcMin;
  erauPacket[24] = utcSec;
}

void write_cdu1_data() {

  cdu1Packet_rx[0] = cdu1_start >> 8;
  cdu1Packet_rx[1] = cdu1_start;
  
  for(read_counter = 2; read_counter < CDU_RX_SIZE; read_counter++) {
    cdu1Packet_rx[read_counter] = Serial4.read();
  }
  Serial4.read();   // Read STOP byte

  flightData.write(cdu1Packet_rx, CDU_RX_SIZE);

  if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
    Serial.println("CDU A Stuff");
    Serial.print(millis(), DEC);
    Serial.write(" / ");
    Serial.print(STARTUP_TIME_LIMIT, DEC);
    Serial.write(" ms. CDU 1 Packet: ");
    for (read_counter = 0; read_counter < CDU_RX_SIZE; read_counter++) {
      Serial.print(cdu1Packet_rx[read_counter],HEX);
      Serial.write("  ");
    }
    Serial.write("\n");
  }
}

void write_cdu2_data() {
  
  cdu2Packet_rx[0] = cdu2_start >> 8;
  cdu2Packet_rx[1] = cdu2_start;
  
  for(read_counter = 2; read_counter < CDU_RX_SIZE; read_counter++) {
    cdu2Packet_rx[read_counter] = Serial5.read();
  }
  Serial5.read();   // Read STOP byte

  flightData.write(cdu2Packet_rx, CDU_RX_SIZE);

  if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
    Serial.print(millis(), DEC);
    Serial.write(" / ");
    Serial.print(STARTUP_TIME_LIMIT, DEC);
    Serial.write(" ms. CDU 2 Packet: ");
    for (read_counter = 0; read_counter < CDU_RX_SIZE; read_counter++) {
      Serial.print(cdu2Packet_rx[read_counter]);
      Serial.write("  ");
    }
    Serial.write("\n");
  }
}

void write_erau_data() {
  
  erauPacket[0] = id_erau[0];
  erauPacket[1] = id_erau[1];

  erauPacket[2] = packet_number;
  erauPacket[3] = packet_number >> 8;
  packet_number += 1;

  /****PACKET TIMESTAMP****/
  erauPacket[4] = time_packet;
  erauPacket[5] = time_packet >> 8;
  erauPacket[6] = time_packet >> 16;
  erauPacket[7] = time_packet >> 24;

  /*******GPS DATA*******/
  // See write_gps_data()

  /*****SCIENCE DATA*****/

  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);   // Or MED_SPEED ?
  tempExt_h = analogRead(TEMP_EXT_H);
  erauPacket[25] = tempExt_h;
  erauPacket[26] = tempExt_h >> 8;

  tempInt = analogRead(TEMP_INT);
  erauPacket[27] = tempInt;
  erauPacket[28] = tempInt  >> 8;

  tempExt_l = analogRead(TEMP_EXT_L);
  erauPacket[29] = tempExt_l;
  erauPacket[30] = tempExt_l >> 8;

  voltage = analogRead(VOLTAGE);
  erauPacket[31] = voltage;
  erauPacket[32] = voltage >> 8;

  lsm.getEvent(&a, &m, &g, &temp);

  accel_x = a.acceleration.x;
  erauPacket[33] = accel_x;
  erauPacket[34] = accel_x >> 8;

  accel_y = a.acceleration.y;
  erauPacket[35] = accel_y;
  erauPacket[36] = accel_y >> 8;

  accel_z = a.acceleration.z;
  erauPacket[37] = accel_z;
  erauPacket[38] = accel_z >> 8;

  gyro_x = g.gyro.x;
  erauPacket[39] = gyro_x;
  erauPacket[40] = gyro_x >> 8;

  gyro_y = g.gyro.y;
  erauPacket[41] = gyro_y;
  erauPacket[42] = gyro_y >> 8;

  gyro_z = g.gyro.z;
  erauPacket[43] = gyro_z;
  erauPacket[44] = gyro_z >> 8;

  erau_checksum = 0;
  for(read_counter = 0; read_counter < GPS_SIZE - 2; read_counter++) {
    erau_checksum += erauPacket[read_counter];
  }
  erauPacket[45] = erau_checksum;
  erauPacket[46] = erau_checksum >> 8;

  flightData.write(erauPacket, GPS_SIZE);

  if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
    Serial.print(millis(), DEC);
    Serial.write(" / ");
    Serial.print(STARTUP_TIME_LIMIT, DEC);
    Serial.write(" ms. GPS+ERAU Packet: ");
    for (read_counter = 0; read_counter < GPS_SIZE; read_counter++) {
      Serial.print(erauPacket[read_counter]);
      Serial.write("  ");
    }
    Serial.write("\n");
    if (stat - 7 > 0) { digitalWrite(FIX_3D_LED, HIGH); }
    else { digitalWrite(FIX_3D_LED, LOW); }
  }
}

void write_sps_data() {

  umnPacket[0] = sps_start >> 8;
  umnPacket[1] = sps_start;

  write_counter = 2;
  for(read_counter = 2; read_counter < 8; read_counter++) {         // Get first part of data
    umnPacket[write_counter++] = Serial2.read();
  }
  for(read_counter = 8; read_counter < 23; read_counter++) {        // Discard GPS + time data
    Serial2.read();
  }
  for(read_counter = 23; read_counter < 86; read_counter++) {        // Get rest of data
    umnPacket[write_counter++] = Serial2.read();
  }
  Serial2.read();       // STOP byte

  umnPacket[write_counter++] = umn_time_packet;
  umnPacket[write_counter++] = umn_time_packet >> 8;
  umnPacket[write_counter++] = umn_time_packet >> 16;
  umnPacket[write_counter] = umn_time_packet >> 24;

  //umnPacket[2] = sps_packet_number >> 8;              // For testing purposes, may be removed later
  //umnPacket[3] = sps_packet_number;
  //sps_packet_number = sps_packet_number + 1;

  flightData.write(umnPacket, SPS_SIZE);

  if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
    Serial.print(millis(), DEC);
    Serial.write(" / ");
    Serial.print(STARTUP_TIME_LIMIT, DEC);
    Serial.write(" ms. UMN Packet: ");
    for (read_counter = 0; read_counter < SPS_SIZE; read_counter++) {
      Serial.print(umnPacket[read_counter]);
      Serial.write("  ");
    }
    Serial.write("\n");
    if (stat - 7 > 0) { digitalWrite(FIX_3D_LED, HIGH); }
    else { digitalWrite(FIX_3D_LED, LOW); }
  }
}

void write_turbulence_data() {
  switch (type) {
    case 0xC109:                        // Instrument packet
      cuPacket[0] = type >> 8;
      cuPacket[1] = type;
      cuPacket[2] = Serial1.read();  // packet num
      cuPacket[3] = Serial1.read();  // packet num
      cuPacket[4] = Serial1.read();  // epoch index
      cuPacket[5] = Serial1.read();  // epoch index
      cuPacket[6] = Serial1.read();  // interval index

      write_counter = 7;
      for(read_counter = 7; read_counter < 25; read_counter++) {
        Serial1.read();                 // Throw away GPS data
      }
      for(read_counter = 25; read_counter < 81; read_counter++) {  // Get rest of data
        cuPacket[write_counter++] = Serial1.read();
      }

      cuPacket[write_counter++] = cu_time_packet;
      cuPacket[write_counter++] = cu_time_packet >> 8;
      cuPacket[write_counter++] = cu_time_packet >> 16;
      cuPacket[write_counter++] = cu_time_packet >> 24;

      //cuPacket[2] = cu_packet_number >> 8;        // For testing purposes, may be removed later
      //cuPacket[3] = cu_packet_number;
      //cu_packet_number = cu_packet_number + 1;

      flightData.write(cuPacket, INSTRUMENT_SIZE);

      if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
        Serial.print(millis(), DEC);
        Serial.write(" / ");
        Serial.print(STARTUP_TIME_LIMIT, DEC);
        Serial.write(" ms. Instrument Packet: ");
        for (read_counter = 0; read_counter < INSTRUMENT_SIZE; read_counter++) {
          Serial.print(cuPacket[read_counter]);
          Serial.write("  ");
        }
        Serial.write("\n");
        if (stat - 7 > 0) { digitalWrite(FIX_3D_LED, HIGH); }
        else { digitalWrite(FIX_3D_LED, LOW); }
      }
      
      break;
      
    case 0xD2A8:                        // Gondola packet
      cuPacket[0] = type >> 8;
      cuPacket[1] = type;
      cuPacket[2] = Serial1.read();  // packet num
      cuPacket[3] = Serial1.read();  // packet num
      cuPacket[4] = Serial1.read();  // epoch index
      cuPacket[5] = Serial1.read();  // epoch index

      write_counter = 6;
      for(read_counter = 6; read_counter < 24; read_counter++) { // Throw away GPS data
        Serial1.read();                 
      }
      for(read_counter = 24; read_counter < 52; read_counter++) { // Get more data
        cuPacket[write_counter++] = Serial1.read();
      }
      for(read_counter = 52; read_counter < 58; read_counter++) { // Throw away GPS data
        Serial1.read();                 
      }
      for(read_counter = 58; read_counter < 70; read_counter++) { // Get rest of data
        cuPacket[write_counter++] = Serial1.read();
      }

      cuPacket[write_counter++] = cu_time_packet;
      cuPacket[write_counter++] = cu_time_packet >> 8;
      cuPacket[write_counter++] = cu_time_packet >> 16;
      cuPacket[write_counter++] = cu_time_packet >> 24;

      //cuPacket[2] = cu_packet_number >> 8;          // For testing purposes, may be removed later
      //cuPacket[3] = cu_packet_number;
      //cu_packet_number = cu_packet_number + 1;

      flightData.write(cuPacket, GONDOLA_SIZE);

      if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
        Serial.print(millis(), DEC);
        Serial.write(" / ");
        Serial.print(STARTUP_TIME_LIMIT, DEC);
        Serial.write(" ms. Gondola Packet: ");
        for (read_counter = 0; read_counter < GONDOLA_SIZE; read_counter++) {
          Serial.print(cuPacket[read_counter]);
          Serial.write("  ");
        }
        Serial.write("\n");
        if (stat - 7 > 0) { digitalWrite(FIX_3D_LED, HIGH); }
        else { digitalWrite(FIX_3D_LED, LOW); }
      }
      
      break;
      
    case 0xA123:                        // Raw data
      cuPacket[0] = type >> 8;
      cuPacket[1] = type;

      for(read_counter = 2; read_counter < RAW_SIZE; read_counter++) {  // Get all data
        cuPacket[read_counter] = Serial1.read();
      }

      flightData.write(cuPacket, RAW_SIZE);

      if (millis() < STARTUP_TIME_LIMIT) {          // For pre-launch data check
        Serial.print(millis(), DEC);
        Serial.write(" / ");
        Serial.print(STARTUP_TIME_LIMIT, DEC);
        Serial.write(" ms. Raw Packet: ");
        for (read_counter = 0; read_counter < RAW_SIZE; read_counter++) {
          Serial.print(cuPacket[read_counter]);
          Serial.write("  ");
        }
        Serial.write("\n");
        if (stat - 7 > 0) { digitalWrite(FIX_3D_LED, HIGH); }
        else { digitalWrite(FIX_3D_LED, LOW); }
      }
      
      break;
      
    default:
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////************* BEGINNING OF MAIN PROGRAM LOOP ***************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  if(millis() - time_packet > 999) {
    time_packet = millis();               // relative time stamp - close to GPS time stamp
    write_erau_data();
  }
  
  while (uBloxEX.available( gpsuBloxExt )) {
    
    fix_timestamp = millis();
    uBloxEXFix = uBloxEX.read();
    lat1 = uBloxEXFix.latitudeL();         // Scaled by 10,000,000
    lon = uBloxEXFix.longitudeL();         // Scaled by 10,000,000
    alt = uBloxEXFix.altitude_cm();
    stat = uBloxEXFix.status;              // Enum type member of gps_fix class declared in GPSfix.h, used in NMEAGPS.cpp. stat = 8 means 3D fix
    numSats = uBloxEX.sat_count;
    utcHour = uBloxEXFix.dateTime.hours;
    utcMin = uBloxEXFix.dateTime.minutes;
    utcSec = uBloxEXFix.dateTime.seconds;

    latC = uBloxEXFix.latitude();         // Same value as lat1, but represented as a float (degrees)
    lonC = uBloxEXFix.longitude();        // Same value as lon, but represented as a float (degrees)
    altC = uBloxEXFix.altitude();         // Same value as alt, but represented as a float (meters)

    write_gps_data();

 /*   Serial.write("\nLatitude: ");         // Print GPS info to serial monitor for debugging
    Serial.print(latC, DEC);
    Serial.write("\nLongitude: ");
    Serial.print(lonC, DEC);
    Serial.write("\nAltitude: ");
    Serial.print(altC, DEC);
    Serial.write("\nStatus: ");
    Serial.print(stat, DEC);

    Serial.write("\n"); */
  }
  
  while(Serial2.available()) {                        // UMN science measurements
    umn_time_packet = millis();
    sps_start = uint16_t(Serial2.read()) << 8;
    sps_start = sps_start + uint16_t(Serial2.read()); // Grab packet ID
    if(sps_start == 0x4201) {                         // Proceed only if system ID was received
      delay(10);                                      // Allow time for RX buffer to fill up
      write_sps_data();
    }
  }

  while(Serial1.available()) {                        // CU-B science measurements
    cu_time_packet = millis();
    type = uint16_t(Serial1.read() << 8);
    type = type + uint16_t(Serial1.read());                           // Grab packet ID
    if((type == 0xC109) || (type == 0xD2A8) || (type == 0xA123)) {    // Write only if a valid packet ID is read
      delay(10);
      write_turbulence_data(); 
    }
  }
      
  /********CDU CHECKS-ACTIONS********/

  while(Serial4.available() > CDU_RX_SIZE-1) {
    
    cdu1_start = uint16_t(Serial4.read()) << 8;
    cdu1_start += uint16_t(Serial4.read());
//    if (cdu1_start == 0x4261) {             // Cutter A
      if (cdu1_start == 0x4241) { 
      Serial.println("WRITING CDU1 DATA");
      write_cdu1_data();
      }

   cdu1_watchdog = millis();
  }

 if ((millis() - cdu1_watchdog) > 3000) {
    // BLUETOOTH WATCHDOG: RE-ESTABLISH BLUETOOTH CONNECTION
  }

  while(Serial5.available() > CDU_RX_SIZE-1) {

    cdu2_start = uint16_t(Serial5.read()) << 8;
    cdu2_start += uint16_t(Serial5.read());
//    if (cdu2_start == 0x4262) {               // Cutter B
    if (cdu2_start == 0x4242) {               // Cutter B
      write_cdu2_data();
    }

    cdu2_watchdog = millis();
  }

  if ((millis() - cdu2_watchdog) > 3000) {
    // BLUETOOTH WATCHDOG: RE-ESTABLISH BLUETOOTH CONNECTION
  }

  if (millis() - cdu_timer > 1000) {
    cdu_timer = millis();
    stateMachine();    // Use gondola GPS to determine what command, if any, should be sent to a cutting system
  }

/*******SAVE AND CLOSE SD CARD FILE ? ************/
    if (Serial.available()) {                       // Waits for a byte from serial monitor
      Serial.read();
      flightData.close();
      Serial.write("\nSD Card file closed\n");
    }

/***********PERIODICALLY SAVE SD CARD***************/
      if ((millis() - timeSD) > 1400) {            // Save once every 1.4 sec
        appendPos = flightData.position();
        flightData.close();
        flightData = SD.open(fileName1.c_str(), FILE_WRITE);
        timeSD = millis();
        flightData.seek(appendPos);
        Serial.write("\nSD Card saved\n");
      }

  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();

  // sendConfirmation();

  /*if(millis()-thisStamp == 1000) 
  {
    thisStamp = millis();
    Serial4.println();
    Serial4.print(0x42);
    Serial4.println();
  }*/

  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* END OF MAIN PROGRAM LOOP ***************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
