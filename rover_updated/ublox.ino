void init_ublox() {
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

byte RTK() {
  byte RTK = myGNSS.getCarrierSolutionType();
  Serial.print("RTK: ");
  Serial.print(RTK);
  if (RTK == 0) Serial.println(F(" (No solution)"));
  else if (RTK == 1) Serial.println(F(" (High precision floating fix)"));
  else if (RTK == 2) Serial.println(F(" (High precision fix)"));
  return RTK;
}

byte SIV() {
  byte SIV = myGNSS.getSIV();
  Serial.print("Sat #: ");
  Serial.println(SIV);
  return SIV;
}

float HACC() {
  float HACC = myGNSS.getHorizontalAccuracy();
  Serial.print("Horizontal Accuracy: ");
  Serial.println(HACC);
  return HACC;
}

float VACC() {
  float VACC = myGNSS.getVerticalAccuracy();
  Serial.print("Vertical Accuracy: ");
  Serial.println(VACC);
  return VACC;
}

float readBatteryVoltage(uint8_t ver) {
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

void get_rtcm() {
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);   //lora config for send/receive rtcm
  uint8_t buf[BUFLEN];
  unsigned buflen;

  uint8_t rfbuflen;
  uint8_t *bufptr;
  unsigned long lastTime, curTime;

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

// void readVolt() {
//  char volt[10];
 
//  for (int i = 0; i < 200; i++){
//    voltMessage[i] = 0x00;  
//  }
 
//  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
//  sprintf(voltMessage, "%s*VOLT:", sitecode);
//  strncat(voltMessage, volt, sizeof(volt));
//  Serial.print("voltage data message: "); Serial.println(voltMessage);
// }

////v1
// void read_ublox_data() {
//   memset(dataToSend,'\0',200);
//   for (int i = 0; i < 200; i++) {
//     dataToSend[i] = 0x00;
//   }

//   memset(voltMessage,'\0',200);
//   for (int i = 0; i < 200; i++){
//    voltMessage[i] = 0x00;  
//   }

//   byte rtk_fixtype = RTK();
//   int sat_num = SIV();
//   int accu_count = 0;

//   // Defines storage for the lat and lon as double
//   double d_lat; // latitude
//   double d_lon; // longitude

//   double accu_lat = 0.0; // latitude accumulator
//   double accu_lon = 0.0; // longitude accumulator

//   // Now define float storage for the heights and accuracy
//   float f_msl;
//   float f_accuracy_hor;
//   float f_accuracy_ver;

//   float accu_msl = 0.0;           //msl accumulator
//   float accu_accuracy_hor = 0.0;  //hacc acuumulator
//   float accu_accuracy_ver = 0.0;  //vacc accumulator

//   char tempstr[100];
//   char volt[10];
//   char temp[10];

//   snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
//   snprintf(temp, sizeof temp, "%.2f", readTemp());

//   // if (RTK() == 2 && SIV() >= min_sat) { //this condition is already set on main loop
//   // for (int i = 0; i <= (ave_count - 1); i++) {
//   for (int i = 1; i <= ave_count; i++) {
//     get_rtcm();

//     // First, let's collect the position data
//     int32_t latitude = myGNSS.getHighResLatitude();
//     int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
//     int32_t longitude = myGNSS.getHighResLongitude();
//     int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
//     int32_t msl = myGNSS.getMeanSeaLevel();
//     int8_t mslHp = myGNSS.getMeanSeaLevelHp();
//     uint32_t hor_acc = myGNSS.getHorizontalAccuracy();
//     uint32_t ver_acc = myGNSS.getVerticalAccuracy();

//     // Assemble the high precision latitude and longitude
//     d_lat = ((double)latitude) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
//     d_lat += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
//     d_lon = ((double)longitude) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
//     d_lon += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

//     // Calculate the height above mean sea level in mm * 10^-1
//     f_msl = (msl * 10) + mslHp;  // Now convert to m
//     f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

//     // Convert the accuracy (mm * 10^-1) to a float
//     f_accuracy_hor = hor_acc / 10000.0; // Convert from mm * 10^-1 to m
//     f_accuracy_ver = ver_acc / 10000.0; // Convert from mm * 10^-1 to m

//     // sprintf(tempstr, ">>%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, d_lat, d_lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
//     // strncpy(dataToSend, tempstr, String(tempstr).length() + 1);
//     // strncat(dataToSend, ",", 2);
//     // strncat(dataToSend, temp, sizeof(temp));
//     // strncat(dataToSend, ",", 2);
//     // strncat(dataToSend, volt, sizeof(volt)); 
//     // Serial.print("data to send: "); Serial.println(dataToSend);

//     // snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
//     // sprintf(voltMessage, "%s*VOLT:", sitecode);
//     // strncat(voltMessage, volt, sizeof(volt));
//     // Serial.print("voltage data message: "); Serial.println(voltMessage);

//     if ((HACC() == 141 && VACC() == 100) || (HACC() == 141 && VACC() <= 141)) {
//       // Accumulation
//       accu_lat = accu_lat + d_lat;
//       accu_lon = accu_lon + d_lon;
//       accu_msl = accu_msl + f_msl;
//       accu_accuracy_hor = accu_accuracy_hor + f_accuracy_hor;
//       accu_accuracy_ver = accu_accuracy_ver + f_accuracy_ver;
//       accu_count = accu_count + 1;
//       Serial.print("accu_count: "); Serial.println(accu_count);
//       Serial.print("iter_count: "); Serial.println(i);
//     } else {
//       // i--; //loop until hacc&vacc conditions are satisfied
//       i++; //wont wait for conditions to be satistied, loop continues to iterate regardless
//       Serial.print("iter_count: "); Serial.println(i);
//       get_rtcm();
//     }
//   }

//   // Averaging
//   d_lat = accu_lat / accu_count; 
//   d_lon = accu_lon / accu_count;
//   f_msl = accu_msl / accu_count; 
//   f_accuracy_hor = accu_accuracy_hor / accu_count;
//   f_accuracy_ver = accu_accuracy_ver / accu_count;

//   if (d_lat != 0) { //try next na >1 kase ayaw pumasok sa else loop
//     sprintf(tempstr, "double_%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, d_lat, d_lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
//     strncpy(dataToSend, tempstr, String(tempstr).length() + 1);
//     strncat(dataToSend, ",", 2);
//     strncat(dataToSend, temp, sizeof(temp));
//     strncat(dataToSend, ",", 2);
//     strncat(dataToSend, volt, sizeof(volt)); 

//     // readTimeStamp();
//     // strncat(dataToSend, "*", 2);
//     // strncat(dataToSend, Ctimestamp, 13);
//     Serial.print("data to send: "); Serial.println(dataToSend);
//     // get_rtcm();

//   } else if (isnan(d_lat)) {
//     no_ublox_data();

//     // readTimeStamp();
//     // strncat(dataToSend, "*", 2);
//     // strncat(dataToSend, Ctimestamp, 13);
//     Serial.print("data to send: "); Serial.println(dataToSend);
//     // get_rtcm();
//   }
// }


////v2
void read_ublox_data() {
  memset(dataToSend,'\0',200);
  for (int i = 0; i < 200; i++) {
    dataToSend[i] = 0x00;
  }

  memset(voltMessage,'\0',200);
  for (int i = 0; i < 200; i++){
   voltMessage[i] = 0x00;  
  }

  byte rtk_fixtype = RTK();
  int sat_num = SIV();
  int accu_count = 0;

  // Defines storage for the lat and lon as double
  double d_lat; // latitude
  double d_lon; // longitude

  double accu_lat = 0.0; // latitude accumulator
  double accu_lon = 0.0; // longitude accumulator

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

  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
  snprintf(temp, sizeof temp, "%.2f", readTemp());

  for (int i = 1; i <= ave_count; i++) {
    if ((millis() - start) < rtcm_timeout) {
      get_rtcm();

      // First, let's collect the position data
      int32_t latitude = myGNSS.getHighResLatitude();
      int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
      int32_t longitude = myGNSS.getHighResLongitude();
      int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
      int32_t msl = myGNSS.getMeanSeaLevel();
      int8_t mslHp = myGNSS.getMeanSeaLevelHp();
      uint32_t hor_acc = myGNSS.getHorizontalAccuracy();
      uint32_t ver_acc = myGNSS.getVerticalAccuracy();

      // Assemble the high precision latitude and longitude
      d_lat = ((double)latitude) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
      d_lat += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
      d_lon = ((double)longitude) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
      d_lon += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

      // Calculate the height above mean sea level in mm * 10^-1
      f_msl = (msl * 10) + mslHp;  // Now convert to m
      f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

      // Convert the accuracy (mm * 10^-1) to a float
      f_accuracy_hor = hor_acc / 10000.0; // Convert from mm * 10^-1 to m
      f_accuracy_ver = ver_acc / 10000.0; // Convert from mm * 10^-1 to m

      if ((HACC() == 141 && VACC() <= 141)) {
        // Accumulation
        accu_lat = accu_lat + d_lat;
        accu_lon = accu_lon + d_lon;
        accu_msl = accu_msl + f_msl;
        accu_accuracy_hor = accu_accuracy_hor + f_accuracy_hor;
        accu_accuracy_ver = accu_accuracy_ver + f_accuracy_ver;
        accu_count = accu_count + 1;
        Serial.print("accu_count: "); Serial.println(accu_count);
        Serial.print("iter_count: "); Serial.println(i);
      } else {
        i--; //loop until hacc&vacc conditions are satisfied or until timeout reached
        Serial.print("iter_count: "); Serial.println(i);
        get_rtcm();
      }
    } else if ((millis() - start) >= rtcm_timeout) {
      Serial.println("Timeout reached!");
      break;
    }
  }

  // Averaging
  d_lat = accu_lat / accu_count; 
  d_lon = accu_lon / accu_count;
  f_msl = accu_msl / accu_count; 
  f_accuracy_hor = accu_accuracy_hor / accu_count;
  f_accuracy_ver = accu_accuracy_ver / accu_count;

  if ((d_lat > 0) || (d_lon > 0)) { //try next na >0 (instead of !=0) kase ayaw pumasok sa else loop
    sprintf(tempstr, ">>%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, d_lat, d_lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
    strncpy(dataToSend, tempstr, String(tempstr).length() + 1);
    strncat(dataToSend, ",", 2);
    strncat(dataToSend, temp, sizeof(temp));
    strncat(dataToSend, ",", 2);
    strncat(dataToSend, volt, sizeof(volt)); 

    // readTimeStamp();
    // strncat(dataToSend, "*", 2);
    // strncat(dataToSend, Ctimestamp, 13);
    Serial.print("data to send: "); Serial.println(dataToSend);

  } else {
    no_ublox_data();

    // readTimeStamp();
    // strncat(dataToSend, "*", 2);
    // strncat(dataToSend, Ctimestamp, 13);
    // Serial.print("data to send: "); Serial.println(dataToSend);
    // get_rtcm();
  }

  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
  sprintf(voltMessage, "%s*VOLT:", sitecode);
  strncat(voltMessage, volt, sizeof(volt));
  Serial.print("voltage data message: "); Serial.println(voltMessage);
}

void no_ublox_data() {
  for (int i = 0; i < 200; i++){
    dataToSend[i] = 0x00;  
  } 
  memset(dataToSend,'\0',200);
  
  char ndstr[100];
  char ND[14] = "No Ublox data";

  sprintf(ndstr, ">>%s:%s", sitecode, ND);
  strncat(dataToSend, ndstr, sizeof(ndstr));
  Serial.print("data to send: "); Serial.println(dataToSend);
}
