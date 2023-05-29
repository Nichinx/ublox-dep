void send_thru_lora(char* radiopacket){
    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
    uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
    //int len = sizeof(payload);
    int len = String(radiopacket).length();
    int i=0, j=0;

    Serial.println("Sending to rf95_server");
    // Send a message to rf95_server

    //do not stack
    for(i=0; i<200; i++){
      payload[i] = (uint8_t)'0';
    }
    
    for(i=0; i<len; i++){
      payload[i] = (uint8_t)radiopacket[i];
    }
    payload[i] = (uint8_t)'\0';
    
    Serial.println((char*)payload);
    Serial.println("sending payload!");
    rf95.send(payload, len);
    rf95.waitPacketSent();
    delay(100);  
}
