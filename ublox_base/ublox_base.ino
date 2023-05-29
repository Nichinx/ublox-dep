// LoRa Transmit Program Version 3
// Serial_to_LoRa_TX_V3.ino

#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LowPower.h>
#include <EnableInterrupt.h>
#include <FlashStorage.h>
#include "Sodaq_DS3231.h"
#include <Adafruit_SleepyDog.h>
SFE_UBLOX_GNSS myGNSS;

#define RTCM_START 0xd3
#define BUFLEN 1000 //max size of data burst we can handle
#define SER_TIMEOUT 500 //Timeout in millisecs for reads into buffer from serial - needs to be longer than bit time at our baudrate and any other delay between packets from GPS

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char Ctimestamp[13] = "";
uint16_t store_rtc = 00; //store rtc alarm
#define DEBUG 1
#define RTCINTPIN 6

// We will use Serial2 - Rx on pin 11, Tx on pin 10
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

#define U_SERIAL Serial1
#define LED 13 // The LED is turned on when 1st byte is received from the serial port. It is turned off after the last byte is transmitted over LoRa.
unsigned long start;

void setup()
{
  rtc.begin();
  /* rtc interrupt */
  // attachInterrupt(RTCINTPIN, wake, FALLING);
  init_Sleep(); //initialize MCU sleep state
  // setAlarmEvery30(0); //rtc alarm settings 0 [00 & 30] 1 [05 & 35]

  // setAlarm2(); // every 10 minutes

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  U_SERIAL.begin(115200); 
  // while (!Serial) { //Waits for the Serial Monitor
  //   delay(1);
  // }

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  delay(100);

  Serial.println("Feather LoRa TX");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1);
  }
  
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  rf95.setTxPower(23, false);
  U_SERIAL.setTimeout(SER_TIMEOUT);
  
  readTimeStamp();
  Serial.print("Current timestamp: ");
  Serial.println(Ctimestamp);
  start = millis();
  // setupTime(); //set RTC time
}

void loop()
{
    // if (samplingTime()){
        // do {
            send_rtcm();
        // } while ((millis() - start) < 300000); // 5minutes
    // }
}
