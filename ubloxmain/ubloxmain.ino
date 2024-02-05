#include <Arduino.h>
#include "wiring_private.h"
#include "UbloxModule.h"
#include "lora.h"

char Ctimestamp[13];


// We will use Serial2 - Rx on pin 11, Tx on pin 10
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
}
