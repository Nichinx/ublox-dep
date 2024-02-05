#ifndef UbloxModule_h
#define UbloxModule_h

#include "Arduino.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <RH_RF95.h>

#define BUFLEN (5*RH_RF95_MAX_MESSAGE_LEN) //max size of data burst we can handle - (5 full RF buffers) - just arbitrarily large
#define RFWAITTIME 500 //maximum milliseconds to wait for next LoRa packet - used to be 600 - may have been too long


class UbloxModule {
  public:
    UbloxModule();
    void init();
    // void ON();
    // void OFF();
    void get_rtcm();
    // void get_gnss_data();

    void no_ublox_data();
    byte get_RTK();
    byte get_SIV();
    float get_HACC();
    float get_VACC();


  private:
    SFE_UBLOX_GNSS myGNSS;
    RH_RF95 rf95;
    Uart& Serial2 = Serial2;

    const int LED = 13;

    char dataToSend[200];
    char voltMessage[200];
    char ndstr[100];


    char sitecode[6] = "TESUA";
    int min_sat = 30;
    int ave_count = 12;

};

#endif