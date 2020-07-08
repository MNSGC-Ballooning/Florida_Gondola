/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*************************** MAIN DEFINITIONS ****************************/////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/**************DATA BACKUP*******************/
File flightData;
String ext = ".bin";
String fileN;
String fileName1;
unsigned long timeSD;     // Time stamp - used for determining when to save SD card
int appendPos;            // File position marker - used for saving and re-opening SD file
unsigned int fileNum;
unsigned int address;
int sdCount;
const int chipSelect = BUILTIN_SDCARD;

/**************GPS SENSORS*******************/
static NMEAGPS  uBloxEX;   //uBlox GPS instantiate NMEAGPS object
static gps_fix  uBloxEXFix; //gps_fix object

int32_t lat1 = 0;     //Latitude
int32_t lon = 0;      //Longitude
int32_t alt = 0;      //Altitude
byte stat = 0;        //Status
uint8_t numSats = 0;  //Number of Satellites in View
uint8_t utcHour = 0;  //UTC Time - Hour
uint8_t utcMin = 0;   //UTC Time - Minutes
uint8_t utcSec = 0;   //UTC Time - Seconds

/******************DATA PACKETS AND SENSORS VARIABLES*********************/
ADC *adc = new ADC();

byte id_erau[2] = {0xA0, 0xB1};//Identifier for GPS + ERAU packet.
uint16_t type = 0;             //Identifier for type of turbulence packet.
uint16_t sps_start = 0;        //Identifier for SPS30 data packet.
uint16_t erau_checksum = 0;

uint16_t sps_packet_number = 0;// Used to test whether packets are being skipped - may be removed later
uint16_t cu_packet_number = 0; // Used to test whether packets are being skipped - may be removed later

//ARRAY SIZE CONSTANTS
#define GPS_SIZE 47             // Number of bytes in GPS + ERAU packet
#define SPS_SIZE 74             // Number of bytes in SPS30 packet
#define INSTRUMENT_SIZE 67      // Number of bytes in instrument packet
#define GONDOLA_SIZE 50         // Number of bytes in gondola packet
#define RAW_SIZE 73             // Number of bytes in raw packet
#define CDU_RX_SIZE 23          // Number of bytes in CDU packet
#define CDU_TX_SIZE 6           // Number of bytes to send to CDU

//CODES FOR BLUETOOTH
byte CUT_A_command = 0x15;      // Gondola to cutter A - CUT command
byte CUT_B_command = 0x25;      // Gondola to cutter B - CUT command

byte cdu1Packet_tx[CDU_TX_SIZE] = {0x42, 0x61, CUT_A_command, 0, 0, 0x53};
byte cdu2Packet_tx[CDU_TX_SIZE] = {0x42, 0x62, CUT_B_command, 0, 0, 0x53};

byte erauPacket[GPS_SIZE];      //Scientific data packet byte array.
byte umnPacket[SPS_SIZE];       //UMN - SPS30 packet
byte cuPacket[RAW_SIZE];        //CU - turbulence packet
byte cdu1Packet_rx[CDU_RX_SIZE];//Cutting Descent Unit 1 - Cutter A 
byte cdu2Packet_rx[CDU_RX_SIZE];//Cutting Descent Unit 2 - Cutter B
unsigned int packet_number;     //Packet number/counter.
unsigned long time_packet;      //Time stamp used to mark ERAU packets - close to UTC time stamp
unsigned long umn_time_packet;  //Time stamp used to relate SPS measurement to absolute time
unsigned long cu_time_packet;   //Time stamp used to relate CU measurement to absolute time
unsigned char write_counter;    //Array indexer for writing data.
unsigned char read_counter;     //Array indexer for reading data.

unsigned long fix_timestamp = 0;
unsigned long cdu_timer;

//CDU VARIABLES DEFINITON
byte cdu1_status = 0;
byte cdu2_status = 0; // Status of CDUs
int16_t cdu1_temp_uno = 0;
int16_t cdu1_temp_bat = 0;
int16_t cdu2_temp_uno = 0;
int16_t cdu2_temp_bat = 0; //CDU temperatures

unsigned long cdu1_watchdog;    // timer used to reset bluetooth 1
unsigned long cdu2_watchdog;    // timer used to reset bluetooth 2
uint16_t cdu1_start;            //Identifier for CDU 1 packet
uint16_t cdu2_start;            //Identifier for CDU 2 packet

//ANALOG PINS DEFINITION
#define TEMP_EXT_H A9           //External Temperature - High Range.
#define TEMP_EXT_L A8           //External Temperature - Low Range.
#define TEMP_INT A7             //Internal Temperature.
#define VOLTAGE A6              //Voltage Monitor (VBat).

int tempExt_l = 0;              //External Temperature - Low Range.
int tempExt_h = 0;              //External Temperature - High Range.
int tempInt = 0;                //Internal Temperature.
int voltage = 0;                //Voltage Monitor (VBat).

//9DoF PINS DEFINITION
#define LSM9DS1_XGCS 15         //set pin for accelerometer/gyro chip select (XGCS)
#define LSM9DS1_MCS 14          //set pin for magnetometer chip select(MCS)

//9DoF VARIABLES DEFINITION
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
sensors_event_t a, m, g, temp;
int accel_x = 0;
int accel_y = 0;
int accel_z = 0;

int gyro_x = 0;
int gyro_y = 0;
int gyro_z = 0;

#define STARTUP_TIME_LIMIT 180000  // How much time passes before data stops coming through UART 6
#define FIX_3D_LED         6      // GPIO pin for pre-launch 3D fix verification


/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*************************** CDU DEFINITIONS ****************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_LAT_CHANGE 0.05 // In degrees (because the gps function returns this in degrees)
#define MAX_LON_CHANGE 0.02 // In degrees this is smaller because the latitude in sweeden is smaller per degree
#define MAX_ALT_CHANGE 300 // I think this is in meters, I believe thats what the gps outputs but can easily be changed to feet
#define SIZE 10 // size of arrays that store values
#define FIX 0x01
#define NOFIX 0x00
#define D2R PI/180
#define SECONDS_PER_HOUR 3600
#define FPM_PER_MPH 88
uint8_t fixStatus[SIZE]; // gps fix status
float latitude[SIZE], longitude[SIZE], Altitude[SIZE];
bool AWorking, BWorking, CWorking;
unsigned long timeStamp[SIZE];
float dt = 0;
uint8_t sats;
float ascentRate;
float groundSpeed;
float heading;

float latA, latB, latC, latAPrev, latBPrev, latCPrev;
float lonA, lonB, lonC, lonAPrev, lonBPrev, lonCPrev;
float altA, altB, altC, altAPrev, altBPrev, altCPrev;

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////********************** STATE MACHINE DEFINITIONS ***********************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

float tempInt_float = 0;  // For main gondola Teensy, temperature failure will not be checked

// Make sure to update Boundaries!!!


// States
#define INITIALIZATION 0x00   // corresponding hex values are chosen so as to avoid bit-flipping in the stratosphere
#define ASCENT 0x01
#define SLOW_ASCENT 0x02
#define SLOW_DESCENT 0x04
#define DESCENT 0x08
#define FLOAT 0x10
#define OUT_OF_BOUNDARY 0x20
#define TEMPERATURE_FAILURE 0x40
#define RECOVERY 0x80

// Boundaries
///////CHANGE BEFORE EACH FLIGHT////////
#define EASTERN_BOUNDARY 40.0            // longitudes
#define WESTERN_BOUNDARY -2.0
#define NORTHERN_BOUNDARY 77.0            // latitudes
#define SOUTHERN_BOUNDARY 50.0
#define SLOW_DESCENT_CEILING 110000.0     // max altitude stack can reach before balloon is cut and stack enters slow descent state
#define SLOW_DESCENT_FLOOR 80000.0        // min altitude for the slow descent state
#define INIT_ALTITUDE 5000.0              // altitude at which the state machine begins
#define RECOVERY_ALTITUDE 7000.0          // altitude at which the recovery state intializes on descent
#define MIN_TEMP -60.0                    // minimum acceptable internal temperature
#define MAX_TEMP 90.0                     // maximum acceptable interal temperature

// Velocity Boundaries
#define MAX_SA_RATE 375                 // maximum velocity (ft/min) that corresponds to a slow ascent state
#define MAX_FLOAT_RATE 100              // maximum velocity that corresponds to a float state, or minimum for a slow ascent state
#define MIN_FLOAT_RATE -100             // minimum velocity that corresponds to a float state, or maximum for a slow descent state
#define MIN_SD_RATE -600                // minimum velocity that corresponds to a slow desent state

// Intervals
#define FIX_INTERVAL 5000               // GPS with a fix�will flash for 5 seconds
#define NOFIX_INTERVAL 2000             // GPS with no fix�will flash for 2 seconds
#define GPS_LED_INTERVAL 10000          // GPS LED runs on a 10 second loop
#define UPDATE_INTERVAL 2000            // update all data and the state machine every 4 seconds
#define CUT_INTERVAL 30000              // ensure the cutting mechanism is on for 30 seconds
#define MASTER_INTERVAL 135             // master timer that cuts balloon after 2hr, 15min
#define PRESSURE_TIMER_INTERVAL 50      // timer that'll cut the balloon 50 minutes after pressure reads 70k feet
#define ASCENT_INTERVAL 135             // timer that cuts balloon A 2 hours and 15 minutes after ASCENT state initializes
#define SLOW_DESCENT_INTERVAL 60        // timer that cuts both balloons (as a backup) an hour after SLOW_DESCENT state initializes

// State Machine
uint8_t state;
bool stateSwitched;
bool maxAltReached = false;
bool cutStatusA = false, cutStatusB = false;
bool cutterOnA = false,  cutterOnB = false;
String cutReasonA,  cutReasonB;
String stateString;
