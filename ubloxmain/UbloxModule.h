#ifndef UbloxModule_h
#define UbloxModule_h

#include "Arduino.h"

#endif


class UbloxModule {
  private:
    _moduleType = moduleType;


  public:
    UbloxModule (char* moduleType);
    void init();
    void set_lora_config(); //not sure pa dito
    void ublox_ON();
    void ublox_OFF();
    void get_rtcm();
    void get_gnss_data();
    void no_ublox_data();
    byte get_RTK();
    byte get_SIV();
    float get_HACC();
    float get_VACC();
    // double data_accumulator();
    // double data_averaging();


};