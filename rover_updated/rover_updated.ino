#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LowPower.h>
#include <EnableInterrupt.h>
#include <FlashStorage.h>
#include "Sodaq_DS3231.h"
#include <Adafruit_SleepyDog.h>
SFE_UBLOX_GNSS myGNSS;

#define BUFLEN (5*RH_RF95_MAX_MESSAGE_LEN) //max size of data burst we can handle - (5 full RF buffers) - just arbitrarily large
#define RFWAITTIME 500 //maximum milliseconds to wait for next LoRa packet - used to be 600 - may have been too long
#define rtcm_timeout 180000 //3 minutes

char sitecode[6] = "TESUA"; //logger name - sensor site code
int min_sat = 30;
// int loop_counter = 15;
int ave_count = 12;

char dataToSend[200];
char voltMessage[200];
char Ctimestamp[13] = "";

uint16_t store_rtc = 00; //store rtc alarm
volatile bool OperationFlag = false;
bool read_flag = false;
uint8_t rx_lora_flag = 0;

// initialize LoRa global variables
uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t ack_payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(payload);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len2 = sizeof(buf);
char ack_key[8] = "^REC'D_";
char ack_msg[13];
int lora_TX_end = 0;
#define ACKWAIT 2000    

#define DEBUG 1
#define RTCINTPIN 6
#define VBATPIN A7    //new copy
#define VBATEXT A5
#define UBXPWR 5     //ublox power pin

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// We will use Serial2 - Rx on pin 11, Tx on pin 10
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

#define LED 13
unsigned long start;

// FlashStorage(ack_filter, int);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  delay(100);
  
  rtc.begin();
  attachInterrupt(RTCINTPIN, wake, FALLING);
  init_Sleep(); //initialize MCU sleep state
  setAlarmEvery30(8); //rtc alarm settings -- 10 minutes interval

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(UBXPWR, OUTPUT);
  digitalWrite(UBXPWR, HIGH);

  // digitalWrite(RFM95_RST, LOW);
  // delay_millis(10);
  // digitalWrite(RFM95_RST, HIGH);
  // delay_millis(10);

  Serial.println("Feather LoRa RX");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  for (int y = 0; y < 20; y++) {
    if (!rf95.init()) {
      Serial.println("LoRa radio init failed");
      delay(1000);
    } else {
      Serial.println("LoRa radio init OK!");
      break;
    }
  }

  for (int z = 0; z < 20; z++) {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      delay(1000);
    } else {
      Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
      break;
    }
  }

  rf95.setTxPower(23, false);
  init_ublox();
  Serial.println("done init ubx + lora");
  // readTimeStamp();
}

//10.26.23 - averaging data with filter
void loop() {
  start = millis();
  rx_lora_flag = 0;

  do {
    Serial.println("Enter do loop...");
    get_rtcm();
  } while (((RTK() != 2) || SIV() < min_sat) && ((millis() - start) < rtcm_timeout)); 

  if (RTK() == 2 && SIV() >= min_sat) {
    if (rx_lora_flag == 0) {
      read_ublox_data();
      rx_lora_flag == 1;
      read_flag = true;
    }
  } else if (((RTK() != 2) || (SIV() < min_sat)) && ((millis() - start) >= rtcm_timeout)) {
    Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
    no_ublox_data();     
    rx_lora_flag == 1;
    read_flag = true;
  } 

  if (read_flag = true) {
    read_flag = false;
    rx_lora_flag == 0;

    readTimeStamp();
    strncat(dataToSend, "*", 2);
    strncat(dataToSend, Ctimestamp, 13);

    send_thru_lora(dataToSend);
    delay(1000);
    send_thru_lora(voltMessage); //EOS
  }

  attachInterrupt(RTCINTPIN, wake, FALLING);
  setAlarmEvery30(8); //10 minutes interval
  rtc.clearINTStatus();
  sleepNow();

  // start = millis();
}


// //08.21.23
// void loop() {
//   start = millis();
//   rx_lora_flag = 0;
    
//     do {
//       get_rtcm();
//     } while (((RTK() != 2) || SIV() < min_sat) && ((millis() - start) < rtcm_timeout)); 
  
//     if (RTK() == 2 && SIV() >= min_sat) {
//       if (rx_lora_flag == 0) {
//         if (HACC() == 141 && VACC() == 100) {
//           read_ublox_data();
//           rx_lora_flag == 1;
//           read_flag = true;
//         }
//         else if (HACC() != 141 || VACC() != 100) {
//           for (int c = 0; c <= loop_counter; c++) {
//             get_rtcm();
//             VACC();

//             if (HACC() == 141 && VACC() == 100) {
//               read_ublox_data();
//               rx_lora_flag == 1;
//               read_flag = true;
//               break;
//             }

//             else if (c == loop_counter) {
//               read_ublox_data();
//               rx_lora_flag == 1;
//               read_flag = true;
//               break;
//             }
//           }
//         }
//       }

//     } else if ((RTK() != 2) || (SIV() < min_sat) || ((millis() - start) >= rtcm_timeout)) {
//         Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
//         no_ublox_data();     
//         rx_lora_flag == 1;
//         read_flag = true;
//     }  

//     if (read_flag = true) {
//       read_flag = false;
//       rx_lora_flag == 0;

//       readTimeStamp();
//       strncat(dataToSend, "*", 2);
//       strncat(dataToSend, Ctimestamp, 13);

//       send_thru_lora(dataToSend);
//       delay(1000);
//       send_thru_lora(voltMessage); //EOS
//     }

//   attachInterrupt(RTCINTPIN, wake, FALLING);
//   setAlarmEvery30(8);
//   rtc.clearINTStatus();
//   sleepNow();
// }


// //09.19.23 - with ubxpwr
// void loop() {
//   digitalWrite(UBXPWR, HIGH);
//   start = millis();
//   rx_lora_flag = 0;

//     do {
//       get_rtcm();
//     } while (((RTK() != 2) || SIV() < min_sat) && ((millis() - start) < rtcm_timeout)); 
  
//     if (RTK() == 2 && SIV() >= min_sat) {
//       if (rx_lora_flag == 0) {
//         if (HACC() == 141 && VACC() == 100) {
//           read_ublox_data();
//           rx_lora_flag == 1;
//           read_flag = true;
//         }
//         else if (HACC() != 141 || VACC() != 100) {
//           for (int c = 0; c <= loop_counter; c++) {
//             get_rtcm();
//             VACC();

//             if (HACC() == 141 && VACC() == 100) {
//               read_ublox_data();
//               rx_lora_flag == 1;
//               read_flag = true;
//               break;
//             }

//             else if (c == loop_counter) {
//               read_ublox_data();
//               rx_lora_flag == 1;
//               read_flag = true;
//               break;
//             }
//           }
//         }
//       }

//     } else if ((RTK() != 2) || (SIV() < min_sat) || ((millis() - start) >= rtcm_timeout)) {
//         Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
//         no_ublox_data();     
//         rx_lora_flag == 1;
//         read_flag = true;
//     }  

//     if (read_flag = true) {
//       read_flag = false;
//       rx_lora_flag == 0;

//       readTimeStamp();
//       strncat(dataToSend, "*", 2);
//       strncat(dataToSend, Ctimestamp, 13);

//       send_thru_lora(dataToSend);
//       delay(1000);
//       send_thru_lora(voltMessage); //EOS

//       digitalWrite(UBXPWR, LOW);
//     }

//   attachInterrupt(RTCINTPIN, wake, FALLING);
//   setAlarmEvery30(8);
//   rtc.clearINTStatus();
//   sleepNow();
// }