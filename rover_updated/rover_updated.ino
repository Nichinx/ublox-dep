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

#define rtcm_timeout 90000 //1.5min

char sitecode[6] = "SINUA"; //logger name - sensor site code
int min_sat = 30;
int loop_counter = 15;
char dataToSend[200];
char Ctimestamp[13] = "";
uint16_t store_rtc = 00; //store rtc alarm
volatile bool OperationFlag = false;
bool read_flag = false;
uint8_t rx_lora_flag = 0;

#define DEBUG 1
#define RTCINTPIN 6
#define VBATPIN A7    //new copy
#define VBATEXT A5

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
  setAlarmEvery30(7); //rtc alarm settings 

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RFM95_RST, LOW);
  delay_millis(10);
  digitalWrite(RFM95_RST, HIGH);
  delay_millis(10);

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
}

//////lora always receiving rtcm
//void loop() {
////  int intent_delay = random(0, 1000); //set delay
//  get_rtcm();
//  Serial.print("getting "); VACC();
//  Serial.print("fix type "); RTK();
//  Serial.print("print "); SIV();
//  rx_lora_flag = 0;
//
//  if (samplingTrial() && samplingSec()) {
//    if (rx_lora_flag == 0) {
//      if (RTK() == 2 && SIV() >= min_sat) {
//        if (HACC() == 141 && VACC() == 100) {
//          read_ublox_data();
//          rx_lora_flag == 1;
//          read_flag = true;
//        }
//        else if (HACC() != 141 || VACC() != 100) {
//          for (int c = 0; c <= loop_counter; c++) {
//            get_rtcm();
//            VACC();
//
//            if (HACC() == 141 && VACC() == 100) {
//              read_ublox_data();
//              rx_lora_flag == 1;
//              read_flag = true;
//              break;
//            }
//
//            else if (c == loop_counter) {
//              read_ublox_data();
//              rx_lora_flag == 1;
//              read_flag = true;
//              break;
//            }
//          }
//        }
//
//      } else if (RTK() != 2 || SIV() < min_sat) {
//        delay(23000);
//        Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
//        no_ublox_data();     
//        rx_lora_flag == 1;
//        read_flag = true;
//      }
//    }
//
//    if (read_flag = true) {
//      read_flag = false;
//      rx_lora_flag == 0;
//      
////      readTimeStamp();
////      strncat(dataToSend, "*", 2);
////      strncat(dataToSend, Ctimestamp, 13);
//
////      Serial.print("Delaying at "); Serial.println(intent_delay);
////     delay(intent_delay);
//      send_thru_lora(dataToSend);
//    }
//  }
//}

/////lora sleep
//void loop() {
//  start = millis();
//  rx_lora_flag = 0;
//  
//  do {
//    get_rtcm();
//    Serial.print("getting "); VACC();
//    Serial.print("fix type "); RTK();
//    Serial.print("print "); SIV();
//  } while (((RTK() != 2) || SIV() < min_sat) && ((millis() - start) < 90000)); 
//                                                                    //1.5 minutes - 90
//                                                                    //2 minutes - 120
//                                                                    //3 minutes - 180
//                                                                    //5 minutes - 300
//  
//  if (RTK() == 2 && SIV() >= min_sat) {
//    if (rx_lora_flag == 0) {
//        if (HACC() == 141 && VACC() == 100) {
//          read_ublox_data();
//          rx_lora_flag == 1;
//          read_flag = true;
//        }
//        else if (HACC() != 141 || VACC() != 100) {
//          for (int c = 0; c <= loop_counter; c++) {
//            get_rtcm();
//            VACC();
//
//            if (HACC() == 141 && VACC() == 100) {
//              read_ublox_data();
//              rx_lora_flag == 1;
//              read_flag = true;
//              break;
//            }
//
//            else if (c == loop_counter) {
//              read_ublox_data();
//              rx_lora_flag == 1;
//              read_flag = true;
//              break;
//            }
//          }
//        }
//
//      } else if (RTK() != 2 || SIV() < min_sat) {
//        Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
//        no_ublox_data();     
//        rx_lora_flag == 1;
//        read_flag = true;
//      }
//    }
//
//    if (read_flag = true) {
//      read_flag = false;
//      rx_lora_flag == 0;
//      send_thru_lora(dataToSend);
//    }
//
//  attachInterrupt(RTCINTPIN, wake, FALLING);
//  setAlarmEvery30(7);
//  rtc.clearINTStatus();
//  sleepNow();
//}

/////lora sleep
void loop() {
  start = millis();
  rx_lora_flag = 0;
  
  do {
    get_rtcm();
  } while (((RTK() != 2) || SIV() < min_sat) && ((millis() - start) < rtcm_timeout)); 
  
  if (rx_lora_flag == 0) {
    if (RTK() == 2 && SIV() >= min_sat) {
        if (HACC() == 141 && VACC() == 100) {
          read_ublox_data();
          rx_lora_flag == 1;
          read_flag = true;
        }
        else if (HACC() != 141 || VACC() != 100) {
          for (int c = 0; c <= loop_counter; c++) {
            get_rtcm();
            VACC();

            if (HACC() == 141 && VACC() == 100) {
              read_ublox_data();
              rx_lora_flag == 1;
              read_flag = true;
              break;
            }

            else if (c == loop_counter) {
              read_ublox_data();
              rx_lora_flag == 1;
              read_flag = true;
              break;
            }
          }
        }

      } else if ((RTK() != 2) || (SIV() < min_sat) || ((millis() - start) == rtcm_timeout)) {
        Serial.println("Unable to obtain fix or no. of satellites reqd. not met");
        no_ublox_data();     
        rx_lora_flag == 1;
        read_flag = true;
      }
    }

    if (read_flag = true) {
      read_flag = false;
      rx_lora_flag == 0;
      send_thru_lora(dataToSend);
    }

  setAlarmEvery30(7);
  rtc.clearINTStatus();
  attachInterrupt(RTCINTPIN, wake, FALLING);
  sleepNow();
  
  start = millis();
}
