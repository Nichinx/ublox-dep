#ifndef UbloxModule_h
#define UbloxModule_h

#include "Arduino.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


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

    // double data_accumulator();
    // double data_averaging();

  private:
    char dataToSend[200];
    char sitecode[6] = "TESUA";
    int min_sat = 30;
    int ave_count = 12;

};

#endif