#include "Arduino.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "wiring_private.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Sodaq_DS3231.h"
#include <LowPower.h>
#include <EnableInterrupt.h>
#include <Adafruit_SleepyDog.h>

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint16_t store_rtc = 00; //store rtc alarm
#define DEBUG 1
#define RTCINTPIN 6

char Ctimestamp[13] = "";
// initialize LoRa global variables
uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t ack_payload[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(payload);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];


// We will use Serial2 - Rx on pin 11, Tx on pin 10
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

#define LED 13


class UbloxModule {
  public:
    // UbloxModule() {} //default constructor

    // UbloxModule(byte ubx_power_pin) {
    //   this->ubx_power_pin = ubx_power_pin
    // }

    UbloxModule() : rtk_fixtype(0), sat_num(0) {
      rtk_fixtype = checkRTKFixType();
      sat_num = checkSatelliteCount();
    }

    void init() {
      // pinMode(ubx_power_pin, OUTPUT);

      Wire.begin();
      if (myGNSS.begin(Wire) == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
      }
      
      myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
      myGNSS.setNavigationFrequency(5); //Set output to 20 times a second
      myGNSS.setHighPrecisionMode(true);  
      myGNSS.powerSaveMode(true);
    }

    // void ON() {
    //   digitalWrite(ubx_power_pin, HIGH);
    // }

    // void OFF() {
    //   digitalWrite(ubx_power_pin, LOW);
    // }

    void getGNSSData() {
      do {
        getRTCM();
      } while (((checkRTKFixType() != 2) || checkSatelliteCount() < min_sat) && ((millis() - start) < rtcm_timeout));

      if (checkRTKFixType() == 2 && checkSatelliteCount() >= min_sat) {
        if (!rx_lora_flag) {
          processGNSSData();
          rx_lora_flag = true;
          read_flag = true;
        }
      } else if (((checkRTKFixType() != 2) || (checkSatelliteCount() < min_sat)) && ((millis() - start) >= rtcm_timeout)) {
        Serial.println("Unable to obtain fix or number of satellites required not met");
        noGNSSDataAcquired();
        rx_lora_flag = true;
        read_flag = true;
      }

      if (read_flag == true) {
        read_flag = false;
        rx_lora_flag = false;

        readTimeStamp();
        strncat(dataToSend, "*", 2);
        strncat(dataToSend, Ctimestamp, 13);

        send_thru_lora(dataToSend);
        delay(1000);
        send_thru_lora(voltMessage); // End-of-String
      }
    }

    void getRTCM() {
      rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);   //lora config for send/receive rtcm
      uint8_t buf[BUFLEN];
      unsigned buflen;

      uint8_t rfbuflen;
      uint8_t *bufptr;
      unsigned long lastTime;

      bufptr = buf;
      if (rf95.available()) {
        digitalWrite(LED, HIGH);
        rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
        if (rf95.recv(bufptr, &rfbuflen)) {
          bufptr += rfbuflen;
          lastTime = millis();
          while (((millis() - lastTime) < RFWAITTIME) && ((bufptr - buf) < (BUFLEN - RH_RF95_MAX_MESSAGE_LEN))) { //Time out or buffer can't hold anymore
            if (rf95.available()) {
              rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
              if (rf95.recv(bufptr, &rfbuflen)) {
                Serial.println((unsigned char) *bufptr, HEX);
                bufptr += rfbuflen;
                lastTime = millis();
              } else {
                Serial.println("Receive failed");
              }
            }
          }
        } else {
          Serial.println("Receive failed");
        }
        buflen = (bufptr - buf);     //Total bytes received in all packets
        Serial2.write(buf, buflen); //Send data to the GPS
        digitalWrite(LED, LOW);
      }
    }

    void processGNSSData() {
      initializeBuffers();

      if ((millis() - start) < rtcm_timeout) {
        for (int i = 1; i <= ave_count; i++) {
          getRTCM();
          getPositionData();

          if ((checkHorizontalAccuracy() == 141 && checkVerticalAccuracy() <= 141)) {
            accumulatePositionData();
          } else {
            i--;
            getRTCM();
          }
        }
      } else {
        Serial.println("Timeout reached!");
      }
      
      averagePositionData();

      if ((d_lat > 0) || (d_lon > 0)){
        prepareGNSSDataString();
      } else {
        noGNSSDataAcquired();
      }

      prepareVoltMessage();
    }

    void initializeBuffers() {
      memset(dataToSend, '\0', sizeof(dataToSend));
      memset(voltMessage, '\0', sizeof(voltMessage));
    }

    void getPositionData() {
      int32_t latitude = myGNSS.getHighResLatitude();
      int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
      int32_t longitude = myGNSS.getHighResLongitude();
      int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
      int32_t msl = myGNSS.getMeanSeaLevel();
      int8_t mslHp = myGNSS.getMeanSeaLevelHp();
      uint32_t hor_acc = myGNSS.getHorizontalAccuracy();
      uint32_t ver_acc = myGNSS.getVerticalAccuracy();

      // Assemble the high precision latitude and longitude
      d_lat = ((double)latitude) / 10000000.0 + ((double)latitudeHp) / 1000000000.0;
      d_lon = ((double)longitude) / 10000000.0 + ((double)longitudeHp) / 1000000000.0;

      // Calculate the height above mean sea level in meters
      f_msl = (msl * 10 + mslHp) / 10000.0;

      // Convert the accuracy (mm * 10^-1) to a float
      f_accuracy_hor = hor_acc / 10000.0;
      f_accuracy_ver = ver_acc / 10000.0;
    }

    void accumulatePositionData() {
      accu_lat += d_lat;
      accu_lon += d_lon;
      accu_msl += f_msl;
      accu_accuracy_hor += f_accuracy_hor;
      accu_accuracy_ver += f_accuracy_ver;
      accu_count++;

      Serial.print("accu_count: ");
      Serial.println(accu_count);
    }

    void averagePositionData() {
      d_lat = accu_lat / accu_count; 
      d_lon = accu_lon / accu_count;
      f_msl = accu_msl / accu_count; 
      f_accuracy_hor = accu_accuracy_hor / accu_count;
      f_accuracy_ver = accu_accuracy_ver / accu_count;
    }

    void prepareGNSSDataString() {
      sprintf(tempstr, ">>%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, d_lat, d_lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
      strncpy(dataToSend, tempstr, strlen(tempstr) + 1);
      strncat(dataToSend, ",", 2);
      strncat(dataToSend, RTCtemp, sizeof(RTCtemp));
      strncat(dataToSend, ",", 2);
      strncat(dataToSend, RTCvolt, sizeof(RTCvolt)); 
      Serial.print("data to send: "); 
      Serial.println(dataToSend);
    }

    void prepareVoltMessage() {
      memset(voltMessage, '\0', sizeof(voltMessage));

      snprintf(RTCvolt, sizeof(RTCvolt), "%.2f", readBatteryVoltage(10));
      sprintf(voltMessage, "%s*VOLT:", sitecode);
      strncat(voltMessage, RTCvolt, sizeof(RTCvolt));
      Serial.print("voltage data message: "); 
      Serial.println(voltMessage);
    }


    void noGNSSDataAcquired() {
      memset(dataToSend, '\0', sizeof(dataToSend));
  
      snprintf(ndstr, sizeof(ndstr), ">>%s:No Ublox data", sitecode);
      strncat(dataToSend, ndstr, sizeof(ndstr));

      Serial.print("data to send: "); 
      Serial.println(dataToSend);
    }

    byte checkRTKFixType() {
      byte RTK = myGNSS.getCarrierSolutionType();
      Serial.print("RTK: "); Serial.print(RTK);
      if (RTK == 0) Serial.println(F(" (No solution)"));
      else if (RTK == 1) Serial.println(F(" (High precision floating fix)"));
      else if (RTK == 2) Serial.println(F(" (High precision fix)"));
      return RTK;
    }

    byte checkSatelliteCount() {
      byte SIV = myGNSS.getSIV();
      Serial.print("Sat #: "); 
      Serial.println(SIV);
      return SIV;
    }

    float checkHorizontalAccuracy() {
      float HACC = myGNSS.getHorizontalAccuracy();
      Serial.print("Horizontal Accuracy: "); 
      Serial.println(HACC);
      return HACC;
    }

    float checkVerticalAccuracy(){
      float VACC = myGNSS.getVerticalAccuracy();
      Serial.print("Vertical Accuracy: "); 
      Serial.println(VACC);
      return VACC;
    }

    float readBatteryVoltage(int ver) {
      float measuredvbat;
      if ((ver == 3) || (ver == 9) || (ver == 10) || (ver == 11)) {
        measuredvbat = analogRead(VBATPIN); //Measure the battery voltage at pin A7
        measuredvbat *= 2;                  // we divided by 2, so multiply back
        measuredvbat *= 3.3;                // Multiply by 3.3V, our reference voltage
        measuredvbat /= 1024;               // convert to voltage
        measuredvbat += 0.28;               // add 0.7V drop in schottky diode
      } else {
        /* Voltage Divider 1M and  100k */
        measuredvbat = analogRead(VBATEXT);
        measuredvbat *= 3.3;                // reference voltage
        measuredvbat /= 1024.0;             // adc max count
        measuredvbat *= 11.0;               // (100k+1M)/100k
      }
      return measuredvbat;
    }

    float readRTCTemp() {
      float RTCtemp;
      rtc.convertTemperature();
      RTCtemp = rtc.getTemperature();
      return RTCtemp;
    }


  private:
    SFE_UBLOX_GNSS myGNSS;
    // RH_RF95 rf95;

    unsigned long start;

    // static const int ubx_power_pin = 5;
    static const int BUFLEN = 5 * RH_RF95_MAX_MESSAGE_LEN;
    static const int RFWAITTIME = 500;
    static const int rtcm_timeout = 180000; //3 minutes

    double d_lat;
    double d_lon;
    double f_msl;
    float f_accuracy_hor;
    float f_accuracy_ver;
    double accu_lat = 0.0;
    double accu_lon = 0.0;
    double accu_msl = 0.0;
    float accu_accuracy_hor = 0.0;
    float accu_accuracy_ver = 0.0;
    int accu_count = 0;

    char dataToSend[200];
    char voltMessage[200];
    char ndstr[100]; 
    char tempstr[100];
    char RTCvolt[10];
    char RTCtemp[10];
    char Ctimestamp[13] = "";

    byte rtk_fixtype;
    int sat_num;
    char sitecode[6] = "TESUA"; //do func : get_sitecode
    const int min_sat = 30;
    const int ave_count = 12;

    bool read_flag = false;
    uint8_t rx_lora_flag = 0;

    static const int VBATPIN = A7;
    static const int VBATEXT = A5;

};

UbloxModule UbloxModule;

void readTimeStamp() {
  DateTime now = rtc.now(); //get the current date-time
  String ts = String(now.year());

  if (now.month() <= 9) {
    ts += "0" + String(now.month());
  } else {
    ts += String(now.month());
  }

  if (now.date() <= 9) {
    ts += "0" + String(now.date());
  } else {
    ts += String(now.date());
  }

  if (now.hour() <= 9) {
    ts += "0" + String(now.hour());
  } else {
    ts += String(now.hour());
  }

  if (now.minute() <= 9) {
    ts += "0" + String(now.minute());
  } else {
    ts += String(now.minute());
  }

  if (now.second() <= 9) {
    ts += "0" + String(now.second());
  } else {
    ts += String(now.second());
  }

  ts.remove(0, 2); //remove 1st 2 data in ts
  ts.toCharArray(Ctimestamp, 13);
}

void setAlarmEvery30(int alarmSET) {
  DateTime now = rtc.now(); //get the current date-time
  switch (alarmSET)
  {
    case 0: //5 mins interval
      {
        if ((now.minute() >= 0) && (now.minute() <= 4)) {
          store_rtc = 5;
        } else if ((now.minute() >= 5) && (now.minute() <= 9)) {
          store_rtc = 10;
        } else if ((now.minute() >= 10) && (now.minute() <= 14)) {
          store_rtc = 15;
        } else if ((now.minute() >= 15) && (now.minute() <= 19)) {
          store_rtc = 20;
        } else if ((now.minute() >= 20) && (now.minute() <= 24)) {
          store_rtc = 25;
        } else if ((now.minute() >= 25) && (now.minute() <= 29)) {
          store_rtc = 30;
        } else if ((now.minute() >= 30) && (now.minute() <= 34)) {
          store_rtc = 35;
        } else if ((now.minute() >= 35) && (now.minute() <= 39)) {
          store_rtc = 40;
        } else if ((now.minute() >= 40) && (now.minute() <= 44)) {
          store_rtc = 45;
        } else if ((now.minute() >= 45) && (now.minute() <= 49)) {
          store_rtc = 50;
        } else if ((now.minute() >= 50) && (now.minute() <= 54)) {
          store_rtc = 55;
        } else if ((now.minute() >= 55) && (now.minute() <= 59)) {
          store_rtc = 0;
        }
        enable_rtc_interrupt();
        break;
      }
    case 1: //3 mins interval
      {
        if ((now.minute() >= 0) && (now.minute() <= 2)) {
          store_rtc = 3;
        } else if ((now.minute() >= 3) && (now.minute() <= 5)) {
          store_rtc = 6;
        } else if ((now.minute() >= 6) && (now.minute() <= 8)) {
          store_rtc = 9;
        } else if ((now.minute() >= 9) && (now.minute() <= 11)) {
          store_rtc = 12;
        } else if ((now.minute() >= 12) && (now.minute() <= 14)) {
          store_rtc = 15;
        } else if ((now.minute() >= 15) && (now.minute() <= 17)) {
          store_rtc = 18;
        } else if ((now.minute() >= 18) && (now.minute() <= 20)) {
          store_rtc = 21;
        } else if ((now.minute() >= 21) && (now.minute() <= 23)) {
          store_rtc = 24;
        } else if ((now.minute() >= 24) && (now.minute() <= 26)) {
          store_rtc = 27;
        } else if ((now.minute() >= 27) && (now.minute() <= 29)) {
          store_rtc = 30;
        } else if ((now.minute() >= 30) && (now.minute() <= 32)) {
          store_rtc = 33;
        } else if ((now.minute() >= 33) && (now.minute() <= 35)) {
          store_rtc = 36;
        } else if ((now.minute() >= 36) && (now.minute() <= 38)) {
          store_rtc = 39;
        } else if ((now.minute() >= 39) && (now.minute() <= 41)) {
          store_rtc = 42;
        } else if ((now.minute() >= 42) && (now.minute() <= 44)) {
          store_rtc = 45;
        } else if ((now.minute() >= 45) && (now.minute() <= 47)) {
          store_rtc = 48;
        } else if ((now.minute() >= 48) && (now.minute() <= 50)) {
          store_rtc = 51;
        } else if ((now.minute() >= 51) && (now.minute() <= 53)) {
          store_rtc = 54;
        } else if ((now.minute() >= 54) && (now.minute() <= 56)) {
          store_rtc = 57;
        } else if ((now.minute() >= 57) && (now.minute() <= 59)) {
          store_rtc = 0;
        }
        enable_rtc_interrupt();
        break;
      }
    case 2:  //set every 10 minutes interval
      {
        if ((now.minute() >= 0) && (now.minute() <= 9)) {
          store_rtc = 10;
        } else if ((now.minute() >= 10) && (now.minute() <= 19)) {
          store_rtc = 20;
        } else if ((now.minute() >= 20) && (now.minute() <= 29)) {
          store_rtc = 30;
        } else if ((now.minute() >= 30) && (now.minute() <= 39)) {
          store_rtc = 40;
        } else if ((now.minute() >= 40) && (now.minute() <= 49)) {
          store_rtc = 50;
        } else if ((now.minute() >= 50) && (now.minute() <= 59)) {
          store_rtc = 0;
        }
        enable_rtc_interrupt();
        break;
      }  
  }
}

void enable_rtc_interrupt() {
  rtc.enableInterrupts(store_rtc, 00); // interrupt at (minutes, seconds)
  if (DEBUG == 1) {
    Serial.print("Next alarm: "); 
    Serial.println(store_rtc);
  }

  readTimeStamp();
}

void wake() {
  detachInterrupt(RTCINTPIN);
}

void init_Sleep() {
  //working to as of 05-17-2019
  SYSCTRL->XOSC32K.reg |= (SYSCTRL_XOSC32K_RUNSTDBY | SYSCTRL_XOSC32K_ONDEMAND); // set external 32k oscillator to run when idle or sleep mode is chosen
  REG_GCLK_CLKCTRL |= GCLK_CLKCTRL_ID(GCM_EIC) |                                 // generic clock multiplexer id for the external interrupt controller
                      GCLK_CLKCTRL_GEN_GCLK1 |                                   // generic clock 1 which is xosc32k
                      GCLK_CLKCTRL_CLKEN;                                        // enable it
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // write protected, wait for sync

  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN4; // Set External Interrupt Controller to use channel 4 (pin 6)
  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN5; // Set External Interrupt Controller to use channel 2 (pin A4)
  // EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN2; // channel 2 (pin A0)

  PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU; // Enable Idle0 mode - sleep CPU clock only
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_AHB; // Idle1 - sleep CPU and AHB clocks
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_APB; // Idle2 - sleep CPU, AHB, and APB clocks

  // It is either Idle mode or Standby mode, not both.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Enable Standby or "deep sleep" mode
}

void sleepNow() {
  Serial.println("MCU is going to sleep . . .");
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; //disable systick interrupt
  LowPower.standby();                         //enters sleep mode
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  //Enable systick interrupt
}

void send_thru_lora(char* radiopacket){
    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
    uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
    int len = sizeof(payload);
    int i = 0, j = 0;
    memset(payload,'\0',sizeof(payload));

    Serial.println("Sending to rf95_server");
    // Send a message to rf95_server

    //do not stack
    for(i=0; i<len; i++){
      payload[i] = (uint8_t)'0';
    }
    
    for(i=0; i<len; i++){
      payload[i] = (uint8_t)radiopacket[i];
    }
    payload[i] = (uint8_t)'\0';
    
    Serial.println((char*)payload);
    Serial.println("sending payload!");
    rf95.send(payload, sizeof(payload));
    rf95.waitPacketSent();
    delay(100);  
}

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
  setAlarmEvery30(0); //rtc alarm settings -- 10 minutes interval

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  } Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);

  UbloxModule.init();
}

void loop() {
  
  UbloxModule.getGNSSData();

  attachInterrupt(RTCINTPIN, wake, FALLING);
  setAlarmEvery30(0); //10 minutes interval
  rtc.clearINTStatus();
  sleepNow();
}
