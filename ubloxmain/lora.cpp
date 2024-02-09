#include "lora.h"

LoRaModule::LoRaModule() : rf95(RFM95_CS, RFM95_INT) {
  init_lora();
}

void LoRaModule::init_lora() {
  pinMode(RFM95_RST, OUTPUT);
  // digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);
}

void LoRaModule::send_thru_lora(char* radiopacket) {
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
  int len = sizeof(payload);

  memset(payload, '\0', 251);

  Serial.println("Sending to rf95_server");

  for (int i = 0; i < 251; i++) {
    payload[i] = (uint8_t)'0';
  }

  for (int i = 0; i < len; i++) {
    payload[i] = (uint8_t)radiopacket[i];
  }
  payload[len] = (uint8_t)'\0';

  Serial.println((char*)payload);
  Serial.println("sending payload!");
  rf95.send(payload, 251);
  rf95.waitPacketSent();
  delay(100);
}