/** 
 * Champagne Datalogger
 * Surficial Tilt IMU sensor added

Features:
* Sends sensor data via LoRa or GSM
* Built-in rtc with configurable wake interrupt
* Low power mode ~10uA
* ~0.5mA with IMU sensor

The circuit:
* Champagne Board
* Adafruit Feather M0
* RFM95 433MHz
* Real time clock
* GSM

Created: January 2020
By : MAD, TEP
Modified: November 11, 2022
By : DRM
*/

#include <Wire.h>
#include <LowPower.h>
#include "Sodaq_DS3231.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <avr/dtostrf.h>  // dtostrf missing in Arduino Zero/Due
#include <EnableInterrupt.h>
#include <FlashStorage.h>
#include <Arduino.h>         // required before wiring_private.h
#include "wiring_private.h"  // pinPeripheral() function
#include <string.h>
#include <Adafruit_SleepyDog.h>

#define BAUDRATE 115200
#define DUEBAUD 9600
#define DUESerial Serial1
#define RTCINTPIN 6
#define RTCSCLPIN 21
#define DUETRIG 5
#define DEBUG 1
#define VBATPIN A7
#define VBATEXT A5
#define GSMRST 12
#define GSMPWR A2  // not connected in rev3 (purple) boards
#define GSMDTR A1
#define GSMINT A0    // gsm ring interrupt
#define IMU_POWER 9  // A3-17

// gsm related
#define GSMBAUDRATE 115200
#define GSMSerial Serial2
#define MAXSMS 168

// for m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 433.0            // Change to 434.0 or other frequency, must match RX's freq!
#define DATALEN 200                // max size of dummy length
#define LORATIMEOUT 500000         // 8.33 minutes delay  (temporarily unused)
#define LORATIMEOUTMODE2 900000    // 15.0 mnutes  (temporarily unused)
#define LORATIMEOUTMODE3 1200000   // 20.0 mnutes  (temporarily unused)
#define DUETIMEOUT 300000          // 180 second delay
#define DUEDELAY 60000             // 1.0 minute delay
#define RAININT A4                 // rainfall interrupt pin A4
#define DEBUGTIMEOUT 300000        // debug timeout in case no data recieved; 60K~1minute
#define ACKWAIT 2000               // wait time for gateway acknowledgement
#define LORATIMEOUTWITHACK 300000  // 5.0 min timeout for gateways

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// initialize LoRa global variables
char dataToSend[DATALEN];  // lora
uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t ack_payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(payload);
char ack_key[8] = "^REC'D_";
// Function: Used for determining sender reciever address
const uint8_t LOGGER_COUNT = 114;
char logger_names[LOGGER_COUNT][6] = { "phita", "agbsb", "agbta", "bakg", "bakta", "baktb", "baktc", "banta", "bantb", "barsc",
                                       "bartb", "bayg", "baysb", "bayta", "baytc", "bcnta", "bcntb", "blcpz", "blcsb", "blcta",
                                       "bolra", "bolta", "btog", "btota", "btotb", "cartc", "cartd", "cudra", "cudtb", "dadra",
                                       "dadta", "gaasa", "gaatc", "gamra", "hinsa", "hinsb", "humb", "imera", "imesb", "imeta",
                                       "imug", "imuta", "imutd", "imute", "inag", "inata", "inatc", "inaxa", "jorta", "knrta",
                                       "knrtb", "labb", "labt", "laysa", "laysb", "lipra", "loota", "lootb", "lpasa", "lpasb",
                                       "lteg", "ltesa", "ltetb", "lung", "luntc", "luntd", "magg", "magte", "mamta", "mamtb",
                                       "marg", "marta", "martb", "mcata", "mngsa", "mngtb", "mslra", "mslta", "msura", "nagra",
                                       "nurta", "nurtb", "oslra", "parta", "partb", "pepg", "pepsb", "peptc", "pinra", "plara",
                                       "pngta", "pngtb", "pugra", "sagta", "sagtb", "sibta", "sinsa", "sintb", "sumta", "sumtc",
                                       "talra", "tgata", "tgatb", "tueta", "tuetb", "umig", "testa", "testb", "testc", "teste",
                                       "testf", "tesua", "sinua", "nagua" };
// When adding new datalogger names:   Increment the variabe "logger_count" subscript for every datalogger name added.
//                                     Limit datalogger names to five (5) characters with the 6th as terminating character

// LoRa received
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len2 = sizeof(buf);
char received[260];
char ack_msg[13];
int lora_TX_end = 0;

char streamBuffer[250];  // store message
char rssiString[250];
char voltMessage[200];
int customDueFlag = 0;  // for data gathering
int sendToLoRa = 0;
String tx_RSSI;    // tx rssi of sensor A
String tx_RSSI_B;  // tx rssi of sensor B
String tx_RSSI_C;  // tx rssi of sensor C
// bool valid_LoRa_tx = false;

// rain gauge
static unsigned long last_interrupt_time = 0;
const unsigned int DEBOUNCE_TIME = 75;  // 40
volatile float rainTips = 0.00;
char sendRainTip[7] = "0.00";

volatile bool gsmRingFlag = false;  // gsm interrupt
volatile bool OperationFlag = false;
volatile unsigned long ringNow = 0;
volatile unsigned long ringPrev = 0;
bool getSensorDataFlag = false;
bool debug_flag_exit = false;

char firmwareVersion[9] = "2402.22";  // year . monthdate
char station_name[6] = "MADTA";
char Ctimestamp[13] = "";
char command[26];
char txVoltage[100] = "0";
char txVoltageB[100] = "0";
char txVoltageC[100] = "0";
char get_sensor_cmd[9];

unsigned long timestart = 0;
uint8_t serial_flag = 0;
uint8_t debug_flag = 0;
uint8_t rcv_LoRa_flag = 0;
uint16_t store_rtc = 00;  // store rtc alarm
// uint8_t gsm_power = 0; //gsm power (sleep or hardware ON/OFF)

// GSM
char default_serverNumber[] = "09175972526";
bool gsmPwrStat = true;
bool runGSMInit = true;
String tempServer, regServer;
char _csq[10];
char response[150];
bool registerNumber = false;
char sending_stack[4000];
char stack_temp[4000];
char prev_gsm_line[500];
char ota_sender[20];

//some default values for config parameters
char default_dataloggerNameA[6] = "TESTA";
char default_dataloggerNameB[6] = "TESTB";
char default_dataloggerNameC[6] = "TESTC";
char default_dataloggerNameD[6] = "TESTD";
char default_sensCommand[9] = "ARQCMD6T";
char default_MCUpassword[9] = "SENSLOPE";
char loggerName_buffer[6];

//RAM check
extern char _end;
extern "C" char *sbrk(int i);
char *ramstart = (char *)0x20070000;
char *ramend = (char *)0x20088000;

//WDT sleep time
int WDTsleepDuration;
bool bootMsg = true;

//  Reset flag triggered by alarm. Used by the function setResetFlag(hh,mm)
//  FALSE by default: If TRUE - resets the datalogger instead of sleeping
bool alarmResetFlag = false;

/* Pin 11-Rx ; 10-Tx (GSM comms) */
Uart Serial2(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

typedef struct
{
  boolean valid;
  char senslopeCommand[9];
  char stationName[10];
} Senslope;
Senslope sensCommand;

typedef struct
{
  boolean valid;
  char sensorA[6];
  char sensorB[6];
  char sensorC[6];
  char sensorD[6];
  char sensorE[6];
  char sensorF[6];
} SensorName;
SensorName loggerName;

typedef struct
{
  boolean valid;
  char inputNumber[13];
  char OTAserver[13];
} serNumber;
serNumber flashServerNumber;

typedef struct
{
  boolean valid;
  char keyword[50];
} smsPassword;
smsPassword flashPassword;

typedef struct
{
  boolean valid;
  char accel_param[100];
  char magneto_param[100];
  char gyro_param[15];
} imu_calib;

/**
 * Reserve a portion of flash memory to store an "int" variable
 * and call it "alarmStorage".
 */
FlashStorage(alarmStorage, int);
FlashStorage(loggerMode, int);
FlashStorage(imuRawCalib, int);
FlashStorage(gsmPower, int);
FlashStorage(rainCollectorType, int);
FlashStorage(OTAserverFlag, int);
FlashStorage(passCommand, Senslope);
FlashStorage(newServerNum, serNumber);
FlashStorage(flashLoggerName, SensorName);
FlashStorage(flashPasswordIn, smsPassword);
FlashStorage(flash_imu_calib, imu_calib);

//enables/disable LoRa acknowledgement function
//disabled[0] by default
FlashStorage(allow_unlisted_flag, int);
//loggger_filtering_enable
//enabled[0] by default: only data units from listed logges are added to sending stack.
//When disabled[0]: adds all valid received data units to sending stack
FlashStorage(ack_filter, int);

void setup() {
  Serial.begin(BAUDRATE);
  DUESerial.begin(DUEBAUD);
  GSMSerial.begin(GSMBAUDRATE);

  /* Assign pins 10 & 11 UART SERCOM functionality */
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  Wire.begin();
  rtc.begin();
  init_lora();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DUETRIG, OUTPUT);
  pinMode(GSMPWR, OUTPUT);
  pinMode(GSMRST, OUTPUT);
  pinMode(IMU_POWER, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(DUETRIG, LOW);
  digitalWrite(GSMPWR, LOW);
  digitalWrite(GSMRST, HIGH);
  digitalWrite(IMU_POWER, LOW);

  /* rain gauge interrupt */
  attachInterrupt(digitalPinToInterrupt(RAININT), rainISR, FALLING);
  /* rtc interrupt */
  attachInterrupt(digitalPinToInterrupt(RTCINTPIN), wakeISR, FALLING);
  /* ring interrupt */
  attachInterrupt(digitalPinToInterrupt(GSMINT), ringISR, FALLING);
  /* sda ticker interrupt test*/
  // attachInterrupt(digitalPinToInterrupt(RTCSCLPIN), rtcTicker, FALLING);

  init_Sleep();  // initialize MCU sleep state

  setNextAlarm(alarmFromFlashMem());  // rtc alarm settings
  rf95.sleep();

  delay_millis(3000);
  enable_watchdog();
  if ((get_logger_mode() == 2) || (get_logger_mode() == 8)) {
    Serial.println(F("****************************************"));
    Serial.print("Logger Version: ");
    Serial.println(get_logger_mode());
    Serial.println("Default to LoRa communication.");
    Serial.println(F("****************************************"));
    bootMsg = false;  //skip sending logger powerup msg

  } else {
    // GSM power related
    Serial.println(F("****************************************"));
    Serial.print("Logger Version: ");
    Serial.println(get_logger_mode());
    Serial.println("Default to GSM.");
    Serial.println(F("****************************************"));
    flashLed(LED_BUILTIN, 10, 100);
    Watchdog.reset();
  }

  /*Automatically enters DEBUG mode when USB cable is connected to PC*/
  digitalWrite(LED_BUILTIN, HIGH);
  unsigned long serStart = millis();
  while (serial_flag == 0) {
    if (Serial) {
      debug_flag = 1;
      Serial.println(F("-------------- DEBUG MODE! -------------"));
      Serial.println(F("****************************************"));
      // printMenu();
      digitalWrite(LED_BUILTIN, LOW);
      serial_flag = 1;
      bootMsg = false;  //skip sending logger powerup msg
      disable_watchdog();
    }
    // timeOut in case walang serial na makuha in ~10 seconds
    if ((millis() - serStart) > 10000) {
      digitalWrite(LED_BUILTIN, LOW);
      serStart = millis();
      serial_flag = 1;
      delay_millis(1000);
      enable_watchdog();
      // turn_OFF_GSM(get_gsm_power_mode());
    }
  }
  flashLed(LED_BUILTIN, 5, 100);
}

void loop() {

  if (runGSMInit && (get_logger_mode() != 2)) {  // single instance run to initiate gsm module
    runGSMInit = false;
    delay_millis(500);
    resetGSM();
  }

  if (debug_flag == 1) printMenu();

  // if (bootMsg) {  //for testing only
  //   send_thru_gsm("LOGGER POWER UP", get_serverNum_from_flashMem());
  //   // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
  //   //   Serial.print("Sending to alternate number: ");
  //   //   send_thru_gsm("LOGGER POWER UP", serverALT(get_serverNum_from_flashMem()));
  //   // }
  //   bootMsg = false;
  // }

  while (debug_flag == 1) {
    getAtcommand();
    if (debug_flag_exit) {
      Serial.println(F("****************************************"));
      Serial.println(F("Exiting DEBUG MENU..."));
      Serial.println(F("****************************************"));
      resetGSM();
      turn_OFF_GSM(get_gsm_power_mode());
      debug_flag = 0;
    }
  }

  if (gsmRingFlag) {
    // check sms commands
    // flashLed(LED_BUILTIN, 2, 100);
    // send_thru_gsm("OTA CALL RESET", get_serverNum_from_flashMem());        //for testing only
    gsmRingFlag = false;
    Watchdog.reset();
    delay_millis(1000);
    turn_ON_GSM(get_gsm_power_mode());
    Watchdog.reset();
    digitalWrite(LED_BUILTIN, HIGH);
    GSMSerial.write("AT+CMGL=\"ALL\"\r");
    delay_millis(300);
    while (GSMSerial.available() > 0) {
      processIncomingByte(GSMSerial.read(), 0);
    }
    Watchdog.reset();
    turn_OFF_GSM(get_gsm_power_mode());
    Watchdog.reset();
    gsmDeleteReadSmsInbox();
    Watchdog.reset();
    attachInterrupt(digitalPinToInterrupt(GSMINT), ringISR, FALLING);
    digitalWrite(LED_BUILTIN, LOW);
    Watchdog.reset();
  }

  if (OperationFlag) {  // main operation
    Watchdog.reset();
    flashLed(LED_BUILTIN, 5, 100);
    detachInterrupt(digitalPinToInterrupt(RTCINTPIN));

    // set RESET 'ALARM TIME' here.
    // Use 24hr format; but '0' insetead of '00')
    // 23,30 by default
    setResetFlag(23, 30);

    sending_stack[0] = '\0';
    if (get_logger_mode() == 1) {
      // Gateway with sensor with 1 LoRa transmitter
      receive_lora_data(1);
      Watchdog.reset();
      turn_ON_GSM(get_gsm_power_mode());
      get_Due_Data(1, get_serverNum_from_flashMem());
      Watchdog.reset();
      // Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 2) {
      // LoRa transmitter of version 5 datalogger
      get_Due_Data(2, get_serverNum_from_flashMem());
      Watchdog.reset();
    } else if (get_logger_mode() == 3) {
      // Gateway only with 1 LoRa transmitter
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      receive_lora_data(3);
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 4) {
      // Gateway only with 2 LoRa transmitter
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      receive_lora_data(4);
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 5) {
      // Gateway only with 3 LoRa transmitter
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      receive_lora_data(5);
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 6) {
      // Rain gauge ONLY datalogger - GSM
      debug_println("Begin: logger mode 6");
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 7) {
      // GNSS sensor only - GSM
      debug_println("Begin: logger mode 7");
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      getGNSSData(dataToSend, sizeof(dataToSend));  //read gnss data
      send_thru_gsm(dataToSend, get_serverNum_from_flashMem());
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else if (get_logger_mode() == 8) {
      // GNSS sensor Tx
      debug_println("Begin: logger mode 8");
      getGNSSData(dataToSend, sizeof(dataToSend));  //read gnss data
      send_thru_lora(dataToSend);
      delay(100);
      send_thru_lora(read_batt_vol(get_calib_param()));
      Watchdog.reset();
    } else if (get_logger_mode() == 9) {
      // Gateway with Subsurface Sensor, Rain Gauge and GNSS Sensor
      debug_println("Begin: logger mode 9");
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0); //send rain
      Watchdog.reset();
      getGNSSData(dataToSend, sizeof(dataToSend));  //read gnss data
      Watchdog.reset();
      get_Due_Data(1, get_serverNum_from_flashMem());
      Watchdog.reset();
      send_thru_gsm(dataToSend, get_serverNum_from_flashMem()); 
      Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    } else {
      // default arQ like sending
      turn_ON_GSM(get_gsm_power_mode());
      Watchdog.reset();
      send_rain_data(0);
      Watchdog.reset();
      get_Due_Data(1, get_serverNum_from_flashMem());
      Watchdog.reset();
      // send_rain_data(0);
      // Watchdog.reset();
      turn_OFF_GSM(get_gsm_power_mode());
      Watchdog.reset();
    }
    // attachInterrupt(digitalPinToInterrupt(RTCINTPIN), wakeISR, FALLING);
    rf95.sleep();
    getSensorDataFlag = false;
    OperationFlag = false;
    // sending_stack[0] = '\0';
    flashLed(LED_BUILTIN, 5, 100);
  }

  Watchdog.reset();
  setNextAlarm(alarmFromFlashMem());
  delay_millis(75);

  attachInterrupt(digitalPinToInterrupt(GSMINT), ringISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAININT), rainISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RTCINTPIN), wakeISR, FALLING);

  rtc.clearINTStatus();

  if (alarmResetFlag) {
    if (get_logger_mode() != 2) send_thru_gsm("Resetting data logger with alarm..", get_serverNum_from_flashMem());
    delay_millis(2000);
    NVIC_SystemReset();
  } else {
    sleepNow();
  }
}


void enable_watchdog() {
  Serial.println("Watchdog Enabled!");
  int countDownMS = Watchdog.enable(16000);  // max of 16 seconds
}

void disable_watchdog() {
  Serial.println("Watchdog Disabled!");
  Watchdog.disable();
}

/*Enable sleep-standby*/
void sleepNow() {

  if ((get_logger_mode() == 2) || (get_logger_mode() == 8)) {
    Watchdog.reset();
  } else {
    gsmDeleteReadSmsInbox();
  }
  // Watchdog.reset();
  delay_millis(1000);
  Serial.println("MCU is going to sleep . . .");
  Serial.println("");
  Serial.end();
  flashLed(LED_BUILTIN, 5, 300);
  disable_watchdog();
  // digitalWrite(LED_BUILTIN, HIGH);
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;  // disable systick interrupt
  LowPower.standby();                          // enters sleep mode
  // __DSB();
  // __WFI();
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // Enable systick interrupt
  // WDTsleepDuration = Watchdog.sleep();
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(2000);
}

/**RTC Pin interrupt
 * hardware interrupt from RTC
 * microcontroller will wake from sleep
 * execute the process
 */
void wakeISR() {
  if (debug_flag == 0) {
    enable_watchdog();
    // Watchdog.reset();
  }

  OperationFlag = true;
  //insert function here to set reset flag for daialy reset
  // compute daily hh mm to match desired reset time



  // detach the interrupt in the ISR so that multiple ISRs are not called
  Serial.println("RTC interrupt");
}

void ringISR() {
  if (debug_flag == 0) {
    enable_watchdog();
  }
  gsmRingFlag = true;

  // ringNow = millis();
  // if ((ringNow - ringPrev > 5000) && (ringNow - ringPrev < 10000))  {
  // NVIC_SystemReset();     // immediate reset
  // }
  // ringPrev = ringNow;
}

/**GATEWAY*RSSI,MAD,MADTA,rssi,voltage,MADTB,,,*200212141406
 *main logger name, MADTA, MADTB, . . .
 *send data to gsm
 */
void get_rssi(uint8_t mode) {
  char convertRssi[20];
  char convertRssiB[20];
  char convertRssiC[20];
  char logger_name[200];
  tx_RSSI.toCharArray(convertRssi, sizeof(convertRssi));
  tx_RSSI_B.toCharArray(convertRssiB, sizeof(convertRssiB));
  tx_RSSI_C.toCharArray(convertRssiC, sizeof(convertRssiC));
  readTimeStamp();

  String loggerName = String(get_logger_A_from_flashMem());
  loggerName.replace("\r", "");
  loggerName.remove(3);
  loggerName.toCharArray(logger_name, 200);

  for (int i = 0; i < 250; i++)
    rssiString[i] = 0;
  strncpy(rssiString, "GATEWAY*RSSI,", 13);
  strncat(rssiString, logger_name, sizeof(logger_name));
  strcat(rssiString, ",");
  strncat(rssiString, get_logger_B_from_flashMem(), 20);
  strcat(rssiString, ",");
  strncat(rssiString, convertRssi, 100);
  strcat(rssiString, ",");
  strncat(rssiString, txVoltage, sizeof(txVoltage));  // voltage working 02-17-2020
  // strncat(dataToSend, parse_voltage(received), sizeof(parse_voltage(received)));
  if (mode == 4) {
    strcat(rssiString, ",");
    strncat(rssiString, get_logger_C_from_flashMem(), 20);
    strcat(rssiString, ",");
    strncat(rssiString, convertRssiB, 100);
    strcat(rssiString, ",");
    strncat(rssiString, txVoltageB, sizeof(txVoltageB));  // voltage working 02-17-2020
    // strncat(dataToSend, parse_voltage_B(received), sizeof(parse_voltage_B(received)));
  } else if (mode == 5) {
    strcat(rssiString, ",");
    strncat(rssiString, get_logger_C_from_flashMem(), 20);
    strcat(rssiString, ",");
    strncat(rssiString, convertRssiB, 100);
    strcat(rssiString, ",");
    strncat(rssiString, txVoltageB, sizeof(txVoltageB));  // voltage working 02-17-2020
    strcat(rssiString, ",");
    strncat(rssiString, get_logger_D_from_flashMem(), 20);  // sensorD
    strcat(rssiString, ",");
    strncat(rssiString, convertRssiC, 100);
    strcat(rssiString, ",");
    strncat(rssiString, txVoltageC, sizeof(txVoltageC));
  } else if (mode == 12) {
    strcat(rssiString, ",");
    strncat(rssiString, get_logger_C_from_flashMem(), 20);
    strcat(rssiString, ",");
    strncat(rssiString, convertRssiB, 100);
    strcat(rssiString, ",");
    strncat(rssiString, txVoltageB, sizeof(txVoltageB));  // voltage working 02-17-2020
    strcat(rssiString, ",");
    strncat(rssiString, get_logger_D_from_flashMem(), 20);  // sensorD
    strcat(rssiString, ",");
    strncat(rssiString, convertRssiC, 100);
    strcat(rssiString, ",");
    strncat(rssiString, txVoltageC, sizeof(txVoltageC));
  }
  strcat(rssiString, ",*");
  strncat(rssiString, Ctimestamp, 12);
  delay_millis(500);
  aggregate_received_data(rssiString);
  // send_thru_gsm(rssiString, get_serverNum_from_flashMem());

  // clear RSSI values
  tx_RSSI = "";
  tx_RSSI_B = "";
  tx_RSSI_C = "";
}

char *get_logger_A_from_flashMem() {
  char *lgr_name;
  loggerName = flashLoggerName.read();
  if (strlen(loggerName.sensorA) == 0) {
    lgr_name = default_dataloggerNameA;
  } else {
    lgr_name = loggerName.sensorA;
  }
  return lgr_name;
}

char *get_logger_B_from_flashMem() {
  char *lgr_name;
  loggerName = flashLoggerName.read();
  if (strlen(loggerName.sensorB) == 0) {
    lgr_name = default_dataloggerNameB;
  } else {
    lgr_name = loggerName.sensorB;
  }
  return lgr_name;
}

char *get_logger_C_from_flashMem() {
  char *lgr_name;
  loggerName = flashLoggerName.read();
  if (strlen(loggerName.sensorC) == 0) {
    lgr_name = default_dataloggerNameC;
  } else {
    lgr_name = loggerName.sensorC;
  }
  return lgr_name;
}

char *get_logger_D_from_flashMem() {
  char *lgr_name;
  loggerName = flashLoggerName.read();
  if (strlen(loggerName.sensorD) == 0) {
    lgr_name = default_dataloggerNameD;
  } else {
    lgr_name = loggerName.sensorD;
  }
  return lgr_name;
}

char *get_logger_E_from_flashMem() {
  loggerName = flashLoggerName.read();
  return loggerName.sensorE;
}

char *get_logger_F_from_flashMem() {
  loggerName = flashLoggerName.read();
  return loggerName.sensorF;
}

// String get_serverNum_from_flashMem() {
//   String flashNum;
//   flashServerNumber = newServerNum.read();
//   flashNum = flashServerNumber.inputNumber;
//   flashNum.replace("\r", "");
//   flashNum.replace(" ", "");
//   if (flashNum == "") {
//     flashNum = default_serverNumber;
//   }

//   return flashNum;
// }

char *get_serverNum_from_flashMem() {
  static char serverNumber[15];
  flashServerNumber = newServerNum.read();
  strcpy(serverNumber, flashServerNumber.inputNumber);
  // flashNum.replace("\r", "");
  // flashNum.replace(" ", "");
  if (strlen(serverNumber) == 0) {
    strcpy(serverNumber, default_serverNumber);
  }
  serverNumber[strlen(serverNumber) + 1] = 0x00;

  return serverNumber;
}

char *get_sensCommand_from_flashMem() {
  get_sensor_cmd[0] = '\0';
  sensCommand = passCommand.read();
  strncpy(get_sensor_cmd, sensCommand.senslopeCommand, 8);
  get_sensor_cmd[strlen(get_sensor_cmd) + 1] = '\0';

  if (strlen(get_sensor_cmd) != 8) {
    strncpy(get_sensor_cmd, default_sensCommand, 8);
    get_sensor_cmd[strlen(get_sensor_cmd) + 1] = '\0';
  }
  return get_sensor_cmd;
}

char *get_password_from_flashMem() {
  char *mcu_pass;
  flashPassword = flashPasswordIn.read();
  if (strlen(flashPassword.keyword) == 0) {
    mcu_pass = default_MCUpassword;
  } else {
    mcu_pass = flashPassword.keyword;
  }
  return mcu_pass;
}

/**sendTo
 * 0 - default for GSM sending
 * 1 - send to LoRa with >> and << added to data
 */
void send_rain_data(uint8_t sendTo) {
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  char temp[10];
  char volt[10];
  readTimeStamp();

  for (int i = 0; i < DATALEN; i++)
    dataToSend[i] = 0;
  /* MADTAW,32.00,0.00,0.82,99,200415171303
      station name, temperature, raintips, input voltage, gsm csq */
  if (sendTo == 1) {
    strncpy(dataToSend, ">>", 2);
    strncat((dataToSend), (get_logger_A_from_flashMem()), (20));
  } else {
    strncpy((dataToSend), (get_logger_A_from_flashMem()), (20));
  }
  // strncpy((dataToSend), (get_logger_A_from_flashMem()), (20));
  strcat(dataToSend, "W");
  strcat(dataToSend, ",");

  snprintf(temp, sizeof temp, "%.2f", readTemp());
  strncat(dataToSend, temp, sizeof(temp));
  strcat(dataToSend, ",");

  snprintf(sendRainTip, sizeof sendRainTip, "%.2f", rainTips);
  strncat(dataToSend, sendRainTip, sizeof(sendRainTip));
  strcat(dataToSend, ",");

  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(get_calib_param()));
  strncat(dataToSend, volt, sizeof(volt));

  strcat(dataToSend, ",");
  strncat(dataToSend, readCSQ(), sizeof(readCSQ()));
  // strncat(dataToSend, _csq, sizeof(_csq));
  strcat(dataToSend, ",");
  strncat(dataToSend, Ctimestamp, 12);
  // if (get_logger_mode() == 6)
  // {
  //   strncat(dataToSend, "<<", 2);
  // }
  delay_millis(500);
  if (sendTo == 1) {
    send_thru_lora(dataToSend);
  } else {
    send_thru_gsm(dataToSend, get_serverNum_from_flashMem());
    // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
    //     Serial.print("Sending to alternate number: ");
    //     send_thru_gsm(dataToSend, serverALT(get_serverNum_from_flashMem()));
    //   }
    delay_millis(500);
    resetRainTips();
  }
  if (debug_flag == 0) {
    Watchdog.reset();
  }
}

void flashLed(int pin, int flashCount, int flashDuration) {
  for (int i = 0; i < flashCount; i++) {
    digitalWrite(pin, HIGH);
    delay_millis(flashDuration);
    digitalWrite(pin, LOW);
    if (i + 1 < flashCount) {
      delay_millis(flashDuration);
    }
  }
}

char *read_batt_vol(uint8_t ver) {
  char volt[10];
  for (int i = 0; i < 200; i++)
    voltMessage[i] = 0;
  // dtostrf(readBatteryVoltage(ver),-5, 2, volt);
  // volt[strlen(volt)+1] = '\0';
  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(ver));
  readTimeStamp();

  if (ver == 6) {
    strncpy(voltMessage, ">>", 2);
  } else {
    strncpy(voltMessage, get_logger_A_from_flashMem(), 20);
  }
  strcat(voltMessage, "*VOLT:");
  strncat(voltMessage, volt, sizeof(volt));
  strcat(voltMessage, "*");
  if (ver == 6) {
    strncat(voltMessage, Ctimestamp, 12);
    strcat(voltMessage, "<<");
  } else {
    strncat(voltMessage, Ctimestamp, 12);
  }
  return voltMessage;
}

// Measure battery voltage using divider on Feather M0
float readBatteryVoltage(uint8_t ver) {
  float measuredvbat;
  // if ((ver == 3) || (ver == 9) || (ver == 10) || (ver == 11))
  if (ver == 1) {
    // 4.2 volts
    measuredvbat = analogRead(VBATPIN);  // Measure the battery voltage at pin A7
    measuredvbat *= 2;                   // we divided by 2, so multiply back
    measuredvbat *= 3.3;                 // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024;                // convert to voltage
    measuredvbat += 0.15;                // between 0.15 and 0.4 drop in schottky diode
  } else {
    /* Voltage Divider 1M and  100k */
    // 12 volts
    measuredvbat = analogRead(VBATEXT);
    measuredvbat *= 3.3;     // reference voltage
    measuredvbat /= 1024.0;  // adc max count
    measuredvbat *= 11.0;    // (100k+1M)/100k
  }
  return measuredvbat;
}

/*
  ** interrupts EIC
  EXTERNAL_INT_2: A0, A5, 10
  EXTERNAL_INT_4: A3, 6
  EXTERNAL_INT_5: A4, 7
  EXTERNAL_INT_6: 8, SDA
  EXTERNAL_INT_7: 9, SCL
  EXTERNAL_INT_9: A2, 3
  EXTERNAL_INT_10: TX, MOSI
  EXTERNAL_INT_11: RX, SCK
*/
void init_Sleep() {

  // SYSCTRL->VREG.bit.RUNSTDBY = 1;
  SYSCTRL->XOSC32K.reg |= (SYSCTRL_XOSC32K_RUNSTDBY | SYSCTRL_XOSC32K_ONDEMAND);  // set external 32k oscillator to run when idle or sleep mode is chosen

  REG_GCLK_CLKCTRL |= GCLK_CLKCTRL_ID(GCM_EIC) |  // generic clock multiplexer id for the external interrupt controller
                      GCLK_CLKCTRL_GEN_GCLK1 |    // generic clock 1 which is xosc32k
                      GCLK_CLKCTRL_CLKEN;         // enable it
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // write protected, wait for sync

  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN4;  // Set External Interrupt Controller to use channel 4 (pin 6)
  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN5;  // Set External Interrupt Controller to use channel 5 (pin A4)
  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN2;  // Set External Interrupt Controller to use channel 2 (pin A0)

  PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;  // Enable Idle0 mode - sleep CPU clock only
  // PM->SLEEP.reg |= PM_SLEEP_IDLE_AHB; // Idle1 - sleep CPU and AHB clocks
  // PM->SLEEP.reg |= PM_SLEEP_IDLE_APB; // Idle2 - sleep CPU, AHB, and APB clocks

  // It is either Idle mode or Standby mode, not both.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // Enable Standby or "deep sleep" mode
  // __DSB();
}

/**
 * Get custom due sensor data.
 * ~5 minutes timeout if no data read.
 * mode in sending data: 1-gsm ; 0 - LoRa(defualt) ; 2 - V5 logger
 */
void get_Due_Data(uint8_t mode, String serverNum) {

  if (debug_flag == 0) {
    Watchdog.reset();
  }

  Serial.println(">>> changing baud rate to duebaud");
  DUESerial.begin(DUEBAUD);
  Serial.println(">>> duebaud changed");
  unsigned long start = millis();

  readTimeStamp();
  turn_ON_due(get_logger_mode());
  delay_millis(500);

  command[0] = '\0';
  // if (mode != 1) {
  //   sending_stack[0] = '\0';
  // }
  strcpy(command, get_sensCommand_from_flashMem());
  // Serial.println(command);
  strcat(command, "/");
  strncat(command, Ctimestamp, 12);
  Serial.println(command);
  DUESerial.write(command);
  // command[0] = '\0';
  Serial.println("Waiting for sensor data. . .");

  while (customDueFlag == 0) {
    if (debug_flag == 0) {
      Watchdog.reset();
    }
    // timeOut in case walang makuhang data sa due
    if ((millis() - start) > DUETIMEOUT) {
      start = millis();
      no_data_from_senslope(mode);
      customDueFlag = 1;
    }

    for (int i = 0; i < 250; ++i)
      streamBuffer[i] = 0x00;
    DUESerial.readBytesUntil('\n', streamBuffer, 250);

    // delay_millis(200);

    if (strlen(streamBuffer) > 1) {
      if (strstr(streamBuffer, ">>")) {
      } else {
        Serial.println(streamBuffer);
      }
    }

    if (strstr(streamBuffer, ">>")) {
      if (strstr(streamBuffer, "*")) {
        Serial.println("Getting sensor data. . .");
        if (debug_flag == 0) {
          Watchdog.reset();
        }
        if (mode == 0 || mode == 1) {
          /**
           * Remove 1st and 2nd character data in string
           * Not needed in GSM mode
           */
          for (byte i = 0; i < strlen(streamBuffer); i++) {
            streamBuffer[i] = streamBuffer[i + 2];
          }

          // send_thru_gsm(streamBuffer, get_serverNum_from_flashMem());
          // send_thru_gsm(streamBuffer, serverNum);
          aggregate_received_data(streamBuffer);
          if (debug_flag == 0) {  //reset watchdog before resuming
            Watchdog.reset();
          }
          flashLed(LED_BUILTIN, 2, 100);
          Serial.println("Data received..");
          DUESerial.write("OK");

        } else if (mode == 6 || mode == 7) {
          strcat(streamBuffer, "<<");
          delay(10);
          send_thru_lora(streamBuffer);
          if (debug_flag == 0) {  //reset watchdog before resuming
            Watchdog.reset();
          }
          flashLed(LED_BUILTIN, 2, 100);
          DUESerial.write("OK");
          // send_thru_lora(streamBuffer);
          // flashLed(LED_BUILTIN, 2, 100);
          // DUESerial.write("OK");
        } else {

          aggregate_received_data(streamBuffer);
          // send_thru_lora(streamBuffer);
          flashLed(LED_BUILTIN, 2, 100);
          Serial.println("Data received mode 2..");
          DUESerial.write("OK");
        }
        // Serial.print(sending_stack);
      } else {
        // maglagay ng counter max 5 then exit
        Serial.println("Message incomplete");
        DUESerial.write("NO");
      }
      if (debug_flag == 0) {
        Watchdog.reset();
      }
    } else if (strstr(streamBuffer, "STOPLORA")) {
      /*if (mode == 0 || mode == 2)
      {
        delay(1000);
        send_thru_lora(read_batt_vol(mode));
        delay(1500); //needed for the gsm to wait until sending
        send_thru_lora("STOPLORA");
      }*/
      DUESerial.write("OK");
      delay_millis(500);
      Serial.println("Done getting DUE data!");
      streamBuffer[0] = '\0';
      customDueFlag = 1;
    }
  }
  turn_OFF_due(get_logger_mode());
  DUESerial.end();
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  send_message_segments(sending_stack);
  if (debug_flag == 0) {
    Watchdog.reset();
  }

  if (mode == 2 || mode == 7) {
    delay_millis(2000);
    send_thru_lora(read_batt_vol(get_calib_param()));
  }

  flashLed(LED_BUILTIN, 4, 90);
  customDueFlag = 0;
  getSensorDataFlag = true;
  // display_freeram();

  if (debug_flag == 0) {
    Watchdog.reset();
  }
}

/**
 * Sends no data from senslope if no data available
 * mode :     1 - gsm
 * default:   0 - LoRa
 *Serial.println("[0] Sendng data using GSM only");
  Serial.println("[1] Version 5 datalogger LoRa with GSM");
  Serial.println("[2] LoRa transmitter for version 5 datalogger");
  Serial.println("[3] Gateway Mode with only ONE LoRa transmitter");
  Serial.println("[4] Gateway Mode with TWO LoRa transmitter");
  Serial.println("[5] Gateway Mode with THREE LoRa transmitter");
  Serial.println("[6] LoRa transmitter for Raspberry Pi");
  Serial.println("[7] Sends rain gauge data via LoRa");
*/
void no_data_from_senslope(uint8_t mode) {
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  readTimeStamp();
  sensCommand = passCommand.read();  // read from flash memory
  Serial.println("No data from senslope");
  streamBuffer[0] = '\0';

  if (mode == 1 || mode == 0) {
    strncpy((streamBuffer), (get_logger_A_from_flashMem()), (20));
  } else {
    strncpy(streamBuffer, ">>", 2);
    strncat((streamBuffer), (get_logger_A_from_flashMem()), (20));
  }

  strcat(streamBuffer, "*NODATAFROMSENSLOPE*");
  strncat(streamBuffer, Ctimestamp, strlen(Ctimestamp));

  if (mode == 1 || mode == 0) {
    if (debug_flag == 0) {
      Watchdog.reset();
    }
    send_thru_gsm(streamBuffer, get_serverNum_from_flashMem());
    if (debug_flag == 0) {
      Watchdog.reset();
    }
    // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
    //     Serial.print("Sending to alternate number: ");
    //     send_thru_gsm(streamBuffer, serverALT(get_serverNum_from_flashMem()));
    //   }
  } else if (mode == 2) {
    send_thru_lora(streamBuffer);
  } else {
    strcat(streamBuffer, "<<");
    send_thru_lora(streamBuffer);
  }
  customDueFlag = 1;
  if (debug_flag == 0) {
    Watchdog.reset();
  }
}

void turn_ON_due(uint8_t mode) {
  Serial.println("Turning ON Custom Due. . .");
  digitalWrite(DUETRIG, HIGH);
  delay_millis(100);
}

void turn_OFF_due(uint8_t mode) {
  Serial.println("Turning OFF Custom Due. . .");
  digitalWrite(DUETRIG, LOW);
  delay_millis(100);
}

void rainISR() {
  // detachInterrupt(digitalPinToInterrupt(RAININT));
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
    if (get_rainGauge_type() == 0) {
      rainTips += 0.5;
    } else if (get_rainGauge_type() == 1) {
      rainTips += 0.2;
    } else {
      rainTips += 1;
    }
    if (debug_flag == 1) {
      Serial.print("Rain tips: ");
      if (get_rainGauge_type() == 0) {
        Serial.println(rainTips * 2.0);
      } else if (get_rainGauge_type() == 1) {
        Serial.println(rainTips * 5.0);
      } else {
        Serial.println(rainTips);
      }
    }
  }
  last_interrupt_time = interrupt_time;
}

void resetRainTips() {
  rainTips = 0.00;
  delay_millis(75);
  dtostrf(rainTips, 3, 2, sendRainTip);  // convert rainTip to char
  // Serial.print("Rain tips: ");
  // Serial.println(rainTips);
}

void send_rain_tips() {
  send_rain_data(0);
}

String parse_voltage(char *toParse) {
  int i = 0;
  String parse_volt;

  // MADTB*VOLT:12.33*200214111000
  char *buff = strtok(toParse, ":");
  while (buff != 0) {
    char *separator = strchr(buff, '*');


    if (separator != 0) {
      *separator = 0;
      if (i == 1)  // 2nd appearance
      {
        parse_volt = buff;
      }
      i++;
    }
    buff = strtok(NULL, ":");
  }
  return parse_volt;
}

void aggregate_received_data(char *data_chunk) {
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  // Serial.println(strlen(data_chunk));
  strncat(sending_stack, data_chunk, strlen(data_chunk));
  strcat(sending_stack, "~");
  sending_stack[strlen(sending_stack) + 1] = '\0';
  Serial.print("Added to sending stack: ");
  Serial.println(data_chunk);

  // display_freeram();
  // delay(500);
}

void display_freeram() {
  Serial.print("SRAM left: ");
  Serial.println(freeRam());
}

int freeRam() {
  char top;
  return &top - reinterpret_cast<char *>(sbrk(0));
}

void send_message_segments(char *msg_dump) {

  char freeram_buf[6];
  char ram_buffer[30];
  ram_buffer[0] = '\0';

  if (debug_flag == 0) {
    Watchdog.reset();
  }

  char *msg_token = strtok(msg_dump, "~");
  while (msg_token != NULL) {
    Serial.println("Sending message segment..");
    // Serial.println(msg_token);
    if (get_logger_mode() == 2) {
      send_thru_lora(msg_token);
    } else {
      send_thru_gsm(msg_token, get_serverNum_from_flashMem());
      if (debug_flag == 0) {
        Watchdog.reset();
      }
      // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
      //   Serial.print("Sending to alternate number: ");
      //   send_thru_gsm(msg_token, serverALT(get_serverNum_from_flashMem()));
      //   if (debug_flag == 0) {
      //     Watchdog.reset();
      //   }
      // }
    }

    msg_token = strtok(NULL, "~");
    if (debug_flag == 0) {
      Watchdog.reset();
    }
  }

  itoa(freeRam(), freeram_buf, 10);
  strncat(ram_buffer, get_logger_A_from_flashMem(), 5);
  strcat(ram_buffer, " SRAM Remaining: ");
  strncat(ram_buffer, freeram_buf, strlen(freeram_buf));
  ram_buffer[strlen(ram_buffer) + 1] = '\0';

  Serial.println(ram_buffer);

  if (debug_flag == 0) {
    Watchdog.reset();
  }
}

void delay_millis(int _delay) {
  uint8_t delay_turn_on_flag = 0;
  unsigned long _delayStart = millis();
  // Serial.println("starting delay . . .");
  do {
    if ((millis() - _delayStart) > _delay) {
      _delayStart = millis();
      delay_turn_on_flag = 1;
      // Serial.println("delay timeout!");
    }
  } while (delay_turn_on_flag == 0);
}

void rtcTicker() {
  Serial.println("Tick");
}

//  Resets the datalogger once a day.
//  Sets a flag [alarmResetFlag] to reset(instead of sleep) the datalogger before the main operaion loop ends.
//  Executed within operation loop (for now), minuta alarm should be equivalent to set RTC alam minute.
//  [0 or 30 for 30 minute interval]
//  [0, 15, 30, 45 for 15 minute interval]
//  etc..
void setResetFlag(uint8_t hourAlarm, uint8_t minuteAlarm) {
  DateTime checkTime = rtc.now();
  char sendNotif[100];
  if (((checkTime.hour() == hourAlarm) || (checkTime.hour() == hourAlarm-6) || (checkTime.hour() == hourAlarm-12) || (checkTime.hour() == hourAlarm-18)) && (checkTime.minute() == minuteAlarm)) {
    if (get_logger_mode() != 2) {
      sprintf(sendNotif, "Current time [%d:%d] Datalogger will reset after data collection.", checkTime.hour(), checkTime.minute());
      send_thru_gsm(sendNotif, get_serverNum_from_flashMem());
    }
    alarmResetFlag = true;
    return;
  } else {
    return;
  }
}