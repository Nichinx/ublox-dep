#include "Arduino.h"
#include "UbloxModule.h"
SFE_UBLOX_GNSS myGNSS;


UbloxModule::UbloxModule() {
  init();
}

void UbloxModule::init() {
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

void UbloxModule::no_ublox_data() {
  for (int i = 0; i < 200; i++){
    dataToSend[i] = 0x00;  
  } 
  memset(dataToSend,'\0',200);
  
  char ndstr[100];
  sprintf(ndstr, ">>%s:No Ublox data", sitecode);
  strncat(dataToSend, ndstr, sizeof(ndstr));
  Serial.print("data to send: "); Serial.println(dataToSend);
}

byte UbloxModule::get_RTK() {
  byte RTK = myGNSS.getCarrierSolutionType();
  Serial.print("RTK: "); Serial.print(RTK);
  if (RTK == 0) Serial.println(F(" (No solution)"));
  else if (RTK == 1) Serial.println(F(" (High precision floating fix)"));
  else if (RTK == 2) Serial.println(F(" (High precision fix)"));
  return RTK;
}

byte UbloxModule::get_SIV() {
  byte SIV = myGNSS.getSIV();
  Serial.print("Sat #: "); Serial.println(SIV);
  return SIV;
}

float UbloxModule::get_HACC() {
  float HACC = myGNSS.getHorizontalAccuracy();
  Serial.print("Horizontal Accuracy: "); Serial.println(HACC);
  return HACC;
}

float UbloxModule::get_VACC() {
  float VACC = myGNSS.getVerticalAccuracy();
  Serial.print("Vertical Accuracy: "); Serial.println(VACC);
  return VACC;
}


// void UbloxModule::get_rtcm() {

//   rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);   //lora config for send/receive rtcm
//   uint8_t buf[BUFLEN];
//   unsigned buflen;

//   uint8_t rfbuflen;
//   uint8_t *bufptr;
//   unsigned long lastTime, curTime;

//   bufptr = buf;
//   if (rf95.available()) {
//     digitalWrite(LED, HIGH);
//     rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
//     if (rf95.recv(bufptr, &rfbuflen)) {
//       bufptr += rfbuflen;
//       lastTime = millis();
//       while (((millis() - lastTime) < RFWAITTIME) && ((bufptr - buf) < (BUFLEN - RH_RF95_MAX_MESSAGE_LEN))) { //Time out or buffer can't hold anymore
//         if (rf95.available()) {
//           rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
//           if (rf95.recv(bufptr, &rfbuflen)) {
//             Serial.println((unsigned char) *bufptr, HEX);
//             bufptr += rfbuflen;
//             lastTime = millis();
//           } else {
//             Serial.println("Receive failed");
//           }
//         }
//       }
//     } else {
//       Serial.println("Receive failed");
//     }
//     buflen = (bufptr - buf);     //Total bytes received in all packets
//     Serial2.write(buf, buflen); //Send data to the GPS
//     digitalWrite(LED, LOW);
//   }
// }
