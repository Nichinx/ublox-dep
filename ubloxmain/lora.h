#ifndef lora_h
#define lora_h

#include "Arduino.h"
#include <SPI.h>
#include <RH_RF95.h>

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0


class LoRaModule {
public:
  LoRaModule();
  void init_lora();
  void send_thru_lora(char* radiopacket);

private:
  RH_RF95 rf95;

};



#endif