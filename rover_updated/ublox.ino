void init_ublox() {
  Wire.begin();
  if (myGNSS.begin(Wire) == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5); //Set output to 20 times a second
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

void readVolt() {
 char volt[10];
 
 for (int i = 0; i < 200; i++){
   voltMessage[i] = 0x00;  
 }
 
 snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
 sprintf(voltMessage, "%s*VOLT:", sitecode);
 strncat(voltMessage, volt, sizeof(volt));
 Serial.print("voltage data message: "); Serial.println(voltMessage);
}

void read_ublox_data() {
  for (int i = 0; i < 200; i++) {
    dataToSend[i] = 0x00;
  }
  memset(dataToSend,'\0',200);

  for (int i = 0; i < 200; i++){
   voltMessage[i] = 0x00;  
  }
  memset(voltMessage,'\0',200);

  byte rtk_fixtype = RTK();
  int sat_num = SIV();
  float lat = 0.0, lon = 0.0;

  // Now define float storage for the heights and accuracy
  float f_ellipsoid;
  float f_msl;
  float f_accuracy_hor;
  float f_accuracy_ver;

  char tempstr[100];
  char volt[10];
  char temp[10];

  snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
  snprintf(temp, sizeof temp, "%.2f", readTemp());

  if (RTK() == 2 && SIV() >= min_sat) {
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t hor_acc = myGNSS.getHorizontalAccuracy();
    uint32_t ver_acc = myGNSS.getVerticalAccuracy();

    int32_t lat_int; // Integer part of the latitude in degrees
    int32_t lat_frac; // Fractional part of the latitude
    int32_t lon_int; // Integer part of the longitude in degrees
    int32_t lon_frac; // Fractional part of the longitude

    // Calculate the latitude and longitude integer and fractional parts
    lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
    lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component

    if (lat_frac < 0) {
      lat_frac = 0 - lat_frac;  // If the fractional part is negative, remove the minus sign
    }

    lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
    lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component

    if (lon_frac < 0) {
      lon_frac = 0 - lon_frac;  // If the fractional part is negative, remove the minus sign
    }

    // Calculate lat-long in float
    lat = lat + (float)lat_int + (float)lat_frac / pow(10, 9);
    lon = lon + (float)lon_int + (float)lon_frac / pow(10, 9);

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;  // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;  // Now convert to m
    f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Now convert to m
    f_accuracy_hor = f_accuracy_hor + ((float)hor_acc / 10000.0); // Convert from mm * 10^-1 to m
    f_accuracy_ver = f_accuracy_ver + ((float)ver_acc / 10000.0); // Convert from mm * 10^-1 to m

    sprintf(tempstr, ">>%s:%d,%.9f,%.9f,%.4f,%.4f,%.4f,%d", sitecode, rtk_fixtype, lat, lon, f_accuracy_hor, f_accuracy_ver, f_msl, sat_num);
    strncpy(dataToSend, tempstr, String(tempstr).length() + 1);
    strncat(dataToSend, ",", 2);
    strncat(dataToSend, temp, sizeof(temp));
    strncat(dataToSend, ",", 2);
    strncat(dataToSend, volt, sizeof(volt)); 
    Serial.print("data to send: "); Serial.println(dataToSend);

    snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
    sprintf(voltMessage, "%s*VOLT:", sitecode);
    strncat(voltMessage, volt, sizeof(volt));
    Serial.print("voltage data message: "); Serial.println(voltMessage);
  }
}

void printFractional(int32_t fractional, uint8_t places) {
  char tempstr[64];
  if (places > 1) {
    for (uint8_t place = places - 1; place > 0; place--)  {
      if (fractional < pow(10, place))  {
        strncat(dataToSend, "0", 1);
      }
    }
  }
  sprintf(tempstr, "%d", fractional);
  strncat(dataToSend, tempstr, String(tempstr).length() + 1);
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
