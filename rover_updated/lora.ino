void send_thru_lora(char* radiopacket){
    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
    uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
    //int len = sizeof(payload);
    // int len = String(radiopacket).length();
    int i=0, j=0;
    memset(payload,'\0',251);

    Serial.println("Sending to rf95_server");
    // Send a message to rf95_server

    //do not stack
    for(i=0; i<251; i++){
      payload[i] = (uint8_t)'0';
    }
    
    for(i=0; i<251; i++){
      payload[i] = (uint8_t)radiopacket[i];
    }
    payload[i] = (uint8_t)'\0';
    
    Serial.println((char*)payload);
    Serial.println("sending payload!");
    rf95.send(payload, 251);
    rf95.waitPacketSent();
    delay(100);  
}

// //send w/ ack
// void send_thru_lora(char *radiopacket) {
//   rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
//   int length = sizeof(payload);
//   int i = 0, j = 0;
//   bool ack_wait = true;
//   bool exit_retry = false;
//   int LORA_SEND_RETRY_LIMIT = 3;

//   char ack_payload2[20];
//   memset(ack_payload2,'\0',20);

//   for (i = 0; i < 200; i++) {
//     payload[i] = (uint8_t)'0';
//   }
//   for (i = 0; i < length; i++) {
//     payload[i] = (uint8_t)radiopacket[i];
//   }
//   payload[i] = (uint8_t)'\0';

//   if (logger_ack_filter_enabled()) {
//     for (j = 0; j <= LORA_SEND_RETRY_LIMIT; j++) {
//       Serial.print("Sending to LoRa: ");
//       Serial.println((char *)payload);
//       // Serial.println("sending payload!");
//       rf95.send(payload, length);  // sending data to LoRa
//       rf95.waitPacketSent();       // Waiting for packet to complete...

//       unsigned long ack_wait_start = millis();
//       ack_payload[0] = '\0';
//       do {
//         if (rf95.waitAvailableTimeout(ACKWAIT)) {
//           if (rf95.recv(ack_payload, &len2)) {
//             int i = 0;
//             for (i =0; i<len2;++i){
//               ack_payload2[i] = (uint8_t)ack_payload[i];
//             }
//             ack_payload2[i] = (uint8_t)'\0';
//         // int i = 0;
//         // for (i = 0; i < len2; ++i) {
//         //   received[i] = (uint8_t)buf[i];
//         // }
//         // received[i] = (uint8_t)'\0';
//             // Serial.print("Ack_payload2:::");
//             // Serial.println(ack_payload2);
//             // Serial.println(ack_payload2);
             

//             // if (strstr((char *)ack_payload, ack_key)) {
//             if (strstr(ack_payload2, ack_key)) {
//               key_gen((char *)payload);  //generate own key [passed to the variable ack_msg] to be matched with gateway ack_msg

//               // if (check_loRa_ack((char*)ack_payload)) {
//               if (check_loRa_ack(ack_payload2)) {
//                 Serial.print("Received ack_key from gateway: ");
//                 Serial.println((char *)ack_payload);
//                 Serial.print("RSSI: ");
//                 Serial.println(rf95.lastRssi(), DEC);
//                 exit_retry = true;
//                 // delay_millis(random(1000, 2000));
//                 break;
//               }
//             } else {
//               // Serial.print("Received reply: ");
//               // Serial.println((char *)ack_payload);
//             }
//           }
//         }
//         // else {
//         //   Serial.println("No valid response...");
//         // }
//       } while ((millis() - ack_wait_start) < ACKWAIT);
//       if (exit_retry) {
//         exit_retry = false;
//         break;
//       }
//       Serial.println("Retrying...");
//     }
//   } else {
//     // do not stack
//     Serial.print("Sending to LoRa: ");
//     Serial.println((char *)payload);
//     // Serial.println("sending payload!");
//     rf95.send(payload, length);  // sending data to LoRa
//     delay_millis(100);
//   }
//   delay_millis(1000);
// }

// void key_gen(char *_payload_received) {
//  // char *key_ptr;

//  ack_msg[0] = '\0';
//  if (strstr(received, sitecode)) {
//    // key_ptr = strncat(ack_msg, get_logger_A_from_flashMem(), 5);
//    // key_ptr = strncat(ack_msg, ack_key, 7);
//    strncat(ack_msg, sitecode, 5);
//    strncat(ack_msg, ack_key, 7);
//  } else {
//    // key_ptr = strncat(ack_msg, _payload_received, 5);
//    // key_ptr = strncat(ack_msg, ack_key, 7);
//    strncat(ack_msg, _payload_received, 5);
//    strncat(ack_msg, ack_key, 7);
//  }
//  // Serial.print("ack key: ");
//  // Serial.println(ack_msg);
//  ack_msg[13] = '\0';
// }

// bool check_loRa_ack(char *_received) {
//   bool lora_tx_flag = false;  //will retry sending by default unless valid ack_msg is received
//   Serial.print("_received::::"); Serial.print(_received);
//   Serial.print("=========ack_msg::::"); Serial.println(ack_msg);
//   if (strstr(_received, ack_msg)) {
//     Serial.print("Ack valid");
//     // Serial.println((char *)_received);
//     lora_tx_flag = true;
//   }
//   return lora_tx_flag;
// }

// bool logger_ack_filter_enabled() {
//   int filter_toggle = ack_filter.read();
//   if (filter_toggle == 0) {  // using default value 0 - enabled by default
//     // Serial.println("logger ack fiter enabled");
//     return true;
//   } else {
//     // Serial.println("logger ack fiter diabled");
//     return false;
//   }
// }

// bool allow_unlisted()
// {
//    int ack_toggle = allow_unlisted_flag.read();
//    if (ack_toggle == 1) {    // using default value 0 - disabled by default
//      return true;
//    } else {
//      return false;
//    }
// }
