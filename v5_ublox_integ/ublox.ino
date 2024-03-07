#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

#define BUFLEN (5*RH_RF95_MAX_MESSAGE_LEN) //max size of data burst we can handle - (5 full RF buffers) - just arbitrarily large
#define RFWAITTIME 500 //maximum milliseconds to wait for next LoRa packet - used to be 600 - may have been too long
#define rtcm_timeout 180000 //3 minutes

char sitecode[6]; //logger name - sensor site code
int min_sat = 30;
int ave_count = 12;

bool read_flag = false;
uint8_t rx_lora_flag = 0;

// Defines storage for the lat and lon as double
double d_lat; // latitude
double d_lon; // longitude

double accu_lat = 0.0; // latitude accumulator
double accu_lon = 0.0; // longitude accumulator
int accu_count = 0;

// Now define float storage for the heights and accuracy
float f_msl;
float f_accuracy_hor;
float f_accuracy_ver;

float accu_msl = 0.0;           //msl accumulator
float accu_accuracy_hor = 0.0;  //hacc acuumulator
float accu_accuracy_ver = 0.0;  //vacc accumulator

char tempstr[100];
char volt[10];
char temp[10];

unsigned long start;

// /* Pin 23-Rx ; 22-Tx (UBLOX serial) */
// Uart Serial3(&sercom3, 23, 22, SERCOM_RX_PAD_0, UART_TX_PAD_2);
// void SERCOM2_Handler() {
//   Serial3.IrqHandler();
// }

// Define the pin numbers for MISO and MOSI
// #define PIN_MISO        12 // MISO: SERCOM4/PAD[0]
// #define PIN_MOSI        10 // MOSI: SERCOM4/PAD[2]

// Create a new UART instance (e.g., Serial3) using SERCOM4 -- 12 for Rx, 10 for Tx -- 22 for MISO(TX), 23 for MOSI(RX)
// Uart Serial3(&sercom4, PIN_SPI_MOSI, PIN_SPI_MISO, SERCOM_RX_PAD_2, UART_TX_PAD_0);
// Uart Serial3(&sercom4, PIN_SPI_MOSI, PIN_SPI_MISO, SERCOM_RX_PAD_3, UART_TX_PAD_2);

// // Interrupt handler for SERCOM4
// void SERCOM4_Handler() {
//   Serial3.IrqHandler();
// }

// void init_sercom3() {
//   Serial3.begin(BAUDRATE);
  
//   // Assign pins 10 & 12 SERCOM_ALT functionality -- 22 & 23
//   pinPeripheral(PIN_SPI_MOSI, PIO_SERCOM_ALT);
//   pinPeripheral(PIN_SPI_MISO, PIO_SERCOM_ALT);
// }

void init_ublox() {
  DUESerial.begin(BAUDRATE);
  Wire.begin();
  if (myGNSS.begin(Wire) == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5); //Set output to 20 times a second
  myGNSS.setHighPrecisionMode(true);  
  myGNSS.powerSaveMode(true);
  initialize_sitecode();
  // disableNMEAMessages();
}

void disableNMEAMessages() {
  // Serial.println("disabling....");
  // Disable specific NMEA messages
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, PIO_SERCOM); //Several of these are on by default on ublox board so let's disable them
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, PIO_SERCOM);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, PIO_SERCOM);
  myGNSS.disableNMEAMessage(UBX_NMEA_RMC, PIO_SERCOM);
  myGNSS.disableNMEAMessage(UBX_NMEA_GGA, PIO_SERCOM);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, PIO_SERCOM);
}

byte checkRTKFixType() {
  byte RTK = myGNSS.getCarrierSolutionType();
  Serial.print("RTK: ");
  Serial.print(RTK);
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

float checkVerticalAccuracy() {
  float VACC = myGNSS.getVerticalAccuracy();
  Serial.print("Vertical Accuracy: ");
  Serial.println(VACC);
  return VACC;
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
    digitalWrite(LED_BUILTIN, HIGH);
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
    // Serial2.write(buf, buflen); //Send data to the GPS
    DUESerial.write(buf, buflen); //Send data to the GPS
    // Serial3.write(buf, buflen); //Send data to the GPS
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void getGNSSData(char *dataToSend, unsigned int bufsize) {
  init_ublox(); 
  start = millis();

  do {
    getRTCM();
  } while (((checkRTKFixType() != 2) || checkSatelliteCount() < min_sat) && ((millis() - start) < rtcm_timeout));

  if (checkRTKFixType() == 2 && checkSatelliteCount() >= min_sat) {
    if (!rx_lora_flag) {
      processGNSSData(dataToSend);
      rx_lora_flag = true;
      read_flag = true;
    }
  } else if (((checkRTKFixType() != 2) || (checkSatelliteCount() < min_sat)) && ((millis() - start) >= rtcm_timeout)) {
    Serial.println("Unable to obtain fix or number of satellites required not met");
    noGNSSDataAcquired();
    prepareVoltMessage();
    rx_lora_flag = true;
    read_flag = true;
  }

  if (read_flag == true) {
    read_flag = false;
    rx_lora_flag = false;

    readTimeStamp();
    strncat(dataToSend, "*", 2);
    strncat(dataToSend, Ctimestamp, 13);

    // if (get_logger_mode() == 7) { 
    //   send_message_segments(dataToSend);
    //   delay(500);
    //   send_message_segments(voltMessage); // End-of-String
    // } else if (get_logger_mode() == 8){
    //   send_thru_lora(dataToSend);
    //   delay(1000);
    //   send_thru_lora(voltMessage); // End-of-String
    // }

    if (get_logger_mode() == 7) {
    /**
    * Remove 1st and 2nd character data in string
    * Not needed in GSM mode
    */
      for (byte i = 0; i < strlen(dataToSend); i++) {
        dataToSend[i] = dataToSend[i + 2];
      }

      GSMSerial.begin(GSMBAUDRATE);
      resetGSM();
      gsmNetworkAutoConnect();

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

      turn_ON_GSM(get_gsm_power_mode());
    }
  }
}

void processGNSSData(char *dataToSend) {
  memset(dataToSend, '\0', sizeof(dataToSend));
  memset(voltMessage, '\0', sizeof(voltMessage));

  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
  snprintf(temp, sizeof temp, "%.2f", readTemp());

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
  byte rtk_fixtype = checkRTKFixType();
  int sat_num = checkSatelliteCount();

  sprintf(tempstr, ">>%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, d_lat, d_lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
  strncpy(dataToSend, tempstr, strlen(tempstr) + 1);
  strncat(dataToSend, ",", 2);
  strncat(dataToSend, temp, sizeof(temp));
  strncat(dataToSend, ",", 2);
  strncat(dataToSend, volt, sizeof(volt)); 
  Serial.print("data to send: "); 
  Serial.println(dataToSend);
}

void prepareVoltMessage() {
  memset(voltMessage, '\0', sizeof(voltMessage));

  snprintf(volt, sizeof(volt), "%.2f", readBatteryVoltage(10));
  sprintf(voltMessage, "%s*VOLT:", sitecode);
  strncat(voltMessage, volt, sizeof(volt));
  Serial.print("voltage data message: "); 
  Serial.println(voltMessage);
}

void noGNSSDataAcquired() {
  memset(dataToSend, '\0', sizeof(dataToSend));

  char ndstr[50]; 
  snprintf(ndstr, sizeof(ndstr), ">>%s:No Ublox data", sitecode);
  strncat(dataToSend, ndstr, sizeof(ndstr));

  Serial.print("data to send: "); 
  Serial.println(dataToSend);
}

void initialize_sitecode() {
  if (get_logger_mode() == 9) {
    char *logger_B_data = get_logger_B_from_flashMem();
    strncpy(sitecode, logger_B_data, 5); // Copy up to 5 characters to avoid buffer overflow
    sitecode[5] = '\0'; // Null-terminate the string
  } else {
    char *logger_A_data = get_logger_A_from_flashMem();
    strncpy(sitecode, logger_A_data, 5); // Copy up to 5 characters to avoid buffer overflow
    sitecode[5] = '\0'; // Null-terminate the string
  } 
}
