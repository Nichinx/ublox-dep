/**
 * @brief Send message to mobile number
 * 
 * @return success: sent, failed: not sent
 */
// void send_thru_gsm(const char* inputMessage, String serverNumber) {
//   if (debug_flag == 0) {
//     Watchdog.reset();
//   }
//   // turn_ON_GSM(get_gsm_power_mode());
//   if (serverNumber == "") {
//     serverNumber = default_serverNumber;
//   }
//   String smsCMD = ("AT+CMGS=");
//   String quote = ("\"");
//   String CR = ("\r");
//   char msgToSend[250];
//   char atCmgsNo[250];
//   atCmgsNo[0] = '\0';
//   msgToSend[0] = '\0';
//   String incomingData = String(inputMessage);
//   incomingData.replace("\r", "");
//   incomingData.toCharArray(msgToSend, 250);
//   String rawMsg = smsCMD + quote + serverNumber + quote + CR;
//   rawMsg.toCharArray(atCmgsNo, 250);
//   atCmgsNo[strlen(atCmgsNo)+1] = '\0';
//   msgToSend[strlen(msgToSend)+1] = '\0';

//   Serial.print("Sending to '");
//   Serial.print(serverNumber);
//   Serial.print("': ");
//   Serial.println(msgToSend);
//   int send_count = 0;

//   long start = millis();
//   long timeout = 10000;
//   bool timeout_flag = false;
//   bool error_flag = false;  //true if error occurred when sending message

//   gsmSerialFlush();
//   GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
//   delay_millis(100);
//   GSMSerial.write(msgToSend);
//   delay_millis(400);
//   GSMSerial.write(26);

//   while (!timeout_flag) {

//     delay_millis(200);
//     sprintf(response, readGSMResponse());

//     if (strstr(response, "+CMGS")) {
//       Serial.println(F("Sent!"));
//       flashLed(LED_BUILTIN, 2, 30);
//       break;

//     } else if (strstr(response, "ERROR")) {
//       error_flag = true;
//       Serial.println(F("Sending Failed!"));
//       GSMSerial.write(27);
//       delay_millis(200);
//       resetGSM();
//       gsmSerialFlush();
//       GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
//       delay_millis(100);
//       GSMSerial.write(msgToSend);
//       delay_millis(400);
//       GSMSerial.write(26);
//       break;
//     }

//     if (millis() - start >= timeout && !error_flag) {
//       GSMSerial.write(27);
//       Serial.println(F("No response from GSM!"));
//       resetGSM();
//       gsmSerialFlush();
//       GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
//       delay_millis(100);
//       GSMSerial.write(msgToSend);
//       delay_millis(400);
//       GSMSerial.write(26);
//       // timeout_flag = true;
//       break;
//     }
//   }
//   if (debug_flag == 0) {
//     Watchdog.reset();
//   }
//   delay_millis(random(5000, 10000));
//   // turn_OFF_GSM(get_gsm_power_mode());
// }

void send_thru_gsm(const char* messageToSend, const char* serverNumber) {
  if (!send_SMS_instance(messageToSend, serverNumber)) {
    if (debug_flag == 0) Watchdog.reset();
    delay_millis(random(5000,10000));
    resetGSM();
    Serial.println("Retrying..");
    if (debug_flag == 0) Watchdog.reset();
    send_SMS_instance(messageToSend, serverNumber);
  } 
}

bool send_SMS_instance(const char* messageToSend, const char* serverNumber) {
  
  bool sentFlag = false;
  Watchdog.reset();
  // turn_ON_GSM(get_gsm_power_mode());

  char messageContainer[250];
  char CMGSContainer[50];
  
  sprintf(messageContainer, "%s", messageToSend);
  for (int j = 0; j < sizeof(messageContainer); j++) {
    if (messageContainer[j] == '\n' || messageContainer[j] == '\r') {
      messageContainer[j] = 0x00;
    } 
  }

  Serial.print("Sending to ");
  sprintf(CMGSContainer,"%s", serverNumber);
  // Serial.print(strlen(CMGSContainer));
  if (strlen(CMGSContainer) == 13 || strlen(CMGSContainer) == 11) {
    sprintf(CMGSContainer, "AT+CMGS=\"%s\"\r",serverNumber);
    Serial.print("'");
    Serial.print(serverNumber);
  } else {
    sprintf(CMGSContainer, "AT+CMGS=\"%s\"\r",default_serverNumber);
    Serial.print("default server number '");
    Serial.print(default_serverNumber);
  }

  messageContainer[strlen(messageContainer)+1] = 0x00;
  CMGSContainer[strlen(CMGSContainer)+1] = 0x00;

  
  Serial.print("': ");
  Serial.println(messageContainer);

  GSMSerial.write("AT\r");                                                                     
  if (GSMWaitResponse("OK",1000, 0)) {                                                        // Checks if GSM serial is accessible
    GSMSerial.write(CMGSContainer);
    if (GSMWaitResponse(">",5000, 1)) {
      GSMSerial.write(messageContainer);
      delay_millis(500);
      GSMSerial.write(26);
    } else {
      Serial.println("Unable to send");
      GSMSerial.write(27);
      GSMSerial.write(27);
    }
    if (GSMWaitResponse("+CMGS",5000, 1)) {
      Serial.println("Message sent!");
      flashLed(LED_BUILTIN, 3, 50);
      sentFlag = true;
    } else {
      delay_millis(5000);
      Serial.println("Sending failed");
      GSMSerial.write(27);  //crude escape
      GSMSerial.write(27);
      GSMSerial.write(27);
      GSMSerial.flush();
      // responive but cannot send
    }
  } else {  //GSM serial is not available
    Serial.println("GSM module error.");
    //insert GSM reset function here
    // GSMSerial.write(messageContainer);
  }

  return sentFlag;
}

/**Commonly used AT commands
 * AT+CSQ - read current signal strenght
 * AT+COPS? - reads current network connected to
 * AT+COPS=?  - reads available networks in the area
 * AT+CMGF=1  - make gsm receive sms to text format (PDU is in binary - 0) 
 * AT+CMGR=1  - read 1st sms in inbox
 * AT+CMGL="ALL"  - read all current received sms
 * AT+CPMS?   - Gets the # of SMS messages stored in GSM modem.
 * AT+CNMI=1,2,0,0,0 - read incoming message from buffer
 * AT+CSCLK=2 - sleep GSM via AT command
 * AT+CSCLK=0 - wake gsm from sleep mode in AT command
*/
void manualGSMcmd() {
  char cmdAllchar[80];
  // String quote = ("\"");
  // String CR = ("\r");
  cmdAllchar[0] = '\0';

  Serial.setTimeout(15000);
  Serial.print("Insert GSM command: ");
  String manualCMD = Serial.readStringUntil('\n');
  Serial.println(manualCMD);
  manualCMD.toCharArray(cmdAllchar, 80);
  strcat(cmdAllchar, "\r");
  // String cmdAll = quote + manualCMD + quote + CR;
  // cmdAll.toCharArray(cmdAllchar, sizeof(cmdAllchar));

  GSMSerial.write(cmdAllchar);
  delay_millis(300);
  readGSMResponse();
  // while (GSMSerial.available() > 0) {
  //   processIncomingByte(GSMSerial.read(), 1);
  // }
}

bool isPassWordCorrect(char *_passW) {
  if (strstr(get_password_from_flashMem(), _passW)) {
    Serial.println("Valid password received!");
    return true;
  } else {
    Serial.println("Invalid password!");
    return false;
  }
}

/** Over the air commands
 * Parse sms if valid and execute command
 * REGISTER:SENSLOPE:639954645704         - not needed but will respond
 * SENSORPOLL:SENSLOPE:                   - force data sampling; sampled data will be sent to the server number (temporary) not to the OTA sender
 * SERVERNUMBER:SENSLOPE:639954645704     - change server number in flash memory  *12
 * ?SERVERNUMBER:SENSLOPE:                - check current server number *13
 * RESET:SENSLOPE:                        - reset MCU   *5
 * SETDATETIME:SENSLOPE:[YYYY,MM,DD,HH,MM,SS,dd[0-6/m-sun],] 2021,02,23,21,22,40,1, *11 - setting similar with debug menu "E: Set date and time manually"
 * SETSENDINGTIME:SENSLOPE:[0-4]:         - sending time; default is 0 [sending every 30 mins] (not yet fully tested)
 * CMD?:SENSLOPE:                         - read current DUE command [ARQCMD6T/S]
 * CMD:SENSLOPE:[SENSLOPE]:               - update DUE command
 * FETCHGPRSTIME:SENSLOPE:                - attempt (with good signal quality) timestamp update using GPRS 
 * CHECKTIMESTAMP:SENSLOPE:               - check stored timestamp
*/
void process_data(char *data) {
  Watchdog.reset();
  bool valid_OTA_command = false;
  char messageToSend[100];
  messageToSend[0] = '\0';

  // tokenize response to extract OTA sender mobile number
  // overwrites preceeding sender mobile number with no valid OTA command
  if (strstr(data, "+CMGL")) {
    prev_gsm_line[0] = 0x00;
    strcat(prev_gsm_line,data);
    prev_gsm_line[strlen(prev_gsm_line)+1] = 0x00;
    
    char *_ota_sender_detail = strtok(prev_gsm_line, "+,\"");
    int ota_detail_index = 0;
    while (_ota_sender_detail != NULL) {
      ota_detail_index++;
      if (ota_detail_index==3) {
        strcpy(ota_sender, "+");
        strcat(ota_sender, _ota_sender_detail);
        break;
      }
      _ota_sender_detail = strtok(NULL,"+,\"");
    }

    ota_sender[strlen(ota_sender)+1] = 0X00;
  }
  //REGISTER:SENSLOPE:639954645704
  if (strncmp(data, "REGISTER:SENSLOPE:", 18) == 0) {
    send_thru_gsm("Registration is no longer needed; proceed with normal OTA commands", ota_sender);
  }
  //SENSORPOLL:SENSLOPE:
  else if (strncmp(data, "SENSORPOLL:SENSLOPE:", 20) == 0) {
    sending_stack[0] = '\0';
    get_Due_Data(get_logger_mode(), ota_sender);
    send_thru_gsm("Data sampling finished - check received data", ota_sender);
  }
  //SERVERNUMBER:SENSLOPE:639954645704
  else if (strncmp(data, "SERVERNUMBER:", 13) == 0) {
    char newServer[50];
    newServer[0] = 0x00;
    Serial.println("change server number");
    char *_password = strtok(data + 13, ":");
    char *_newServerNum = strtok(NULL, ":");
    Serial.println(_newServerNum);

    // store new server number to flash memory
    strcpy(flashServerNumber.inputNumber, _newServerNum);
    newServerNum.write(flashServerNumber);  //save to flash memory

    // compose OTA command reply
    strcpy(newServer, get_serverNum_from_flashMem());
    newServer[sizeof(newServer)+1] = 0x00;
    strcpy(messageToSend, "New server number: ");
    messageToSend[strlen(messageToSend)+1] = 0x00;
    strncat(messageToSend, newServer, sizeof(newServer));
    Serial.println(messageToSend);

    send_thru_gsm(messageToSend, ota_sender);
  }
  //?SERVERNUMBER:SENSLOPE:
  else if (strncmp(data, "?SERVERNUMBER:", 14) == 0) {
    char currenServerNumber[30];
    Serial.println("Check current server number");
    char *_password = strtok(data + 14, ":");
    currenServerNumber[0] = '\0';

    strcpy(currenServerNumber, get_serverNum_from_flashMem());
    currenServerNumber[strlen(currenServerNumber)+1]=0x00;
    // get_serverNum_from_flashMem().toCharArray(currenServerNumber, sizeof(currenServerNumber));

    strcpy(messageToSend, "Current server number: ");
    messageToSend[strlen(messageToSend)+1] = 0x00;
    strncat(messageToSend, currenServerNumber, sizeof(currenServerNumber));
    Serial.println(messageToSend);

    send_thru_gsm(messageToSend, ota_sender);
  }
  //RESET:SENSLOPE:
  else if (strncmp(data, "RESET:", 6) == 0) {
    Serial.println("Resetting microcontroller!");
    send_thru_gsm("Resetting datalogger...", ota_sender);
    // delay_millis(1000);
    // send_thru_gsm(data, ota_sender);
    NVIC_SystemReset();
  }
  //SETDATETIME:SENSLOPE:[YYYY,MM,DD,HH,MM,SS,dd[0-6/m-sun],] 2021,02,23,21,22,40,1,
  else if (strncmp(data, "SETDATETIME", 11) == 0) {
    Serial.println("change timestamp!");
    Serial.println(data);
    char *_password = strtok(data + 11, ":");
    char *YY = strtok(NULL, ",");
    char *MM = strtok(NULL, ",");
    char *DD = strtok(NULL, ",");
    char *hh = strtok(NULL, ",");
    char *mm = strtok(NULL, ",");
    char *ss = strtok(NULL, ",");
    char *dd = strtok(NULL, ",");

    int _YY = atoi(YY);
    int _MM = atoi(MM);
    int _DD = atoi(DD);
    int _hh = atoi(hh);
    int _mm = atoi(mm);
    int _ss = atoi(ss);
    int _dd = atoi(dd);

    //set date and time
    Serial.println(_YY);
    Serial.println(_MM);
    Serial.println(_DD);
    Serial.println(_hh);
    Serial.println(_mm);
    Serial.println(_ss);
    Serial.println(_dd);
    adjustDate(_YY, _MM, _DD, _hh, _mm, _ss, _dd);
    readTimeStamp();

    strcpy(messageToSend, "Current timestamp: ");
    messageToSend[strlen(messageToSend)+1] = 0x00;
    strncat(messageToSend, Ctimestamp, 12);
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
  }
  //FETCHGPRSTIME:SENSLOPE:
  else if (strncmp(data, "FETCHGPRSTIME", 13) == 0) {
    update_time_with_GPRS();
    readTimeStamp();
    strcpy(messageToSend, "Current timestamp: ");
    messageToSend[strlen(messageToSend)+1] = 0x00;
    strncat(messageToSend, Ctimestamp, 12);
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
  }
  else if (strncmp(data, "CHECKTIMESTAMP", 14) == 0) {
    readTimeStamp();
    strcpy(messageToSend, "Current timestamp: ");
    messageToSend[strlen(messageToSend)+1] = 0x00;
    strncat(messageToSend, Ctimestamp, 12);
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
  }
  //SENDINGTIME:SENSLOPE:[0-4]:
  else if (strncmp(data, "SETSENDINGTIME", 14) == 0) {
    Serial.println("change sending time!");
    char sendStorage[10];
    sendStorage[0] = '\0';
    Serial.println(data);

    char *_password = strtok(data + 14, ":");
    char *inputSending = strtok(NULL, ":");
    int _inputSending = atoi(inputSending);


      //set sending time
    Serial.println(_inputSending);
    alarmStorage.write(_inputSending);
    sprintf(sendStorage, "%d", alarmFromFlashMem());

    strncpy(messageToSend, "Updated sending time: ", 22);
    strncat(messageToSend, sendStorage, sizeof(sendStorage));
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
  }
  //CMD?:SENSLOPE:
  else if (strncmp(data, "CMD?", 4) == 0) {
    Serial.println("change sending time!");
    char *_password = strtok(data + 4, ":");

    //read DUE command
    // sensCommand = passCommand.read();
    strncpy(messageToSend, "Current DUE command: ", 21);
    strncat(messageToSend, get_sensCommand_from_flashMem(), 8);
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
  }
  //CMD:SENSLOPE[ARQCM6T/6S]
  else if (strncmp(data, "CMD:", 4) == 0) {
    Serial.println("change sending time!");

    char *_password = strtok(data + 4, ":");
    char *dueCmd = strtok(NULL, ":");

    // if (isPassWordCorrect(_password) && (OTAserverFlag.read() != 0)) {
      //read DUE command
    strcpy((sensCommand.senslopeCommand), dueCmd);
    passCommand.write(sensCommand);  //save to flash memory

    //read command
    // sensCommand = passCommand.read();
    strncpy(messageToSend, "Current DUE command: ", 21);
    strncat(messageToSend, get_sensCommand_from_flashMem(), 8);
    Serial.println(messageToSend);
    send_thru_gsm(messageToSend, ota_sender);
    // }
  }


  // ADD OPTION FOR POWER SAVING MODE
  // ~ TURN ON/OFF GSM DURING NIGHTTIME
  // ~ INCREASE SAMPLING INTERVAL?
  // ADD OPTION CHECKING SAVED CONFIGURATION
  // ~ ALSO FETCH DUE CONFIG?
  // ~ FETCH SD CARD DATA IF PERMISSIBLE?
  

}

/* Read GSM reply; non read blocking process*/
void processIncomingByte(const byte inByte, int _mode) {
  const unsigned int MAX_SERIAL_INPUT = 256;  // how much serial data we expect before a newline
  static char input_line[255];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':                    // end of text
      input_line[input_pos] = 0;  // terminating null byte
      if (_mode == 0) {
        process_data(input_line);
        Serial.println(input_line);
      } else {
        Serial.println(input_line);
      }
      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':  // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_SERIAL_INPUT - 1))
        input_line[input_pos++] = inByte;
      break;
  }  // end of switch
}  // end of processIncomingByte

char *readGSMResponse() {
  // char response[100]; //200
  int length = sizeof(response);

  for (int j = 0; j < length; j++) {
    response[j] = '\0';
  }

  for (int i = 0; GSMSerial.available() > 0 && i < length; i++) {
    response[i] = GSMSerial.read();
  }
  Serial.print(response);
  return response;
}

bool gsmReadOK() {
  for (int i = 0; i < 100; i++)  // 50 - 500ms
  {
    if (strstr(readGSMResponse(), "OK")) {
      // Serial.println("GSM OK");
      return true;
      break;
    }
    // Serial.print(" .");
    delay_millis(10);
  }
  return false;
}

bool gsmReadRing() {
  for (int i = 0; i < 500; i++)  // 50 - 500ms
  {
    if (strstr(readGSMResponse(), "RING")) {
      // Serial.println("Read OK from GSM");
      return true;
      break;
    }
    // Serial.print(" .");
    // delay_millis(10);
  }
  return false;
}

void gsmSerialFlush() {
  while (GSMSerial.available() > 0) {
    char t = GSMSerial.read();
  }
}

int getCsqStrtok(char *buffer) {
  char *tmpBuf;
  tmpBuf = strtok(buffer, ": ");
  tmpBuf = strtok(NULL, ",");
  return (atoi(tmpBuf));
}

char *readCSQ() {
  // char c_csq[5] = "99";
  gsmSerialFlush();
  GSMSerial.write("AT+CSQ\r");
  delay_millis(500);
  snprintf(_csq, sizeof _csq, "%d", getCsqStrtok(readGSMResponse()));
  return _csq;
}

String getCSQ() {
  String cCSQval = ("99");
  gsmSerialFlush();
  GSMSerial.write("AT\r");
  delay(500);
  if (gsmReadOK()) {
    GSMSerial.write("AT+CSQ\r");
    delay(500);
    String nCSQ = String(getCsqStrtok(readGSMResponse()));
    return nCSQ;
  } else {
    return cCSQval;
  }
}

/*  RING pin
  - normal voltage: 2.7v
  - when SMS received 2.7v , LOW , 2.7v
  - when call pin is 0v, then back after to 2.7 after end call
  *i-connect sa intrrupt pin, mabubuhay if may received na sms
*/
void gsmHangup() {
  //when ring pin triggers, call this function
  delay_millis(2000);
  GSMSerial.write("ATH\r");
}

void gsmDeleteReadSmsInbox() {
  Serial.println("deleting sms read from inbox . . .");
  GSMSerial.write("AT+CMGD=1,2\r");
  delay_millis(1000);
  if (gsmReadOK()) {
    Serial.println("deleting done!");
  } else {
    Serial.println("deleting failed!");
  }
}

void gsmNetworkAutoConnect() {
  if (debug_flag == 0) Watchdog.reset();
  Serial.println("Checking GSM function");
  unsigned long initStart = millis();
  int initTimeout = 20000;
  bool gsmSerial = false;
  bool GSMconfig = false;
  bool signalCOPS = false;
  int serialFaultCount = 0;

  while (!gsmSerial || !GSMconfig || !signalCOPS ) { 
    if (debug_flag == 0) Watchdog.reset();
    if (!gsmSerial) {
      GSMSerial.flush();
      delay_millis(1000);
      Serial.println(F("Checking serial comms.."));
      GSMSerial.write("ATE0\r");
      delay_millis(500);
      if (gsmReadOK()) {
        gsmSerial = true;
      } else {
        serialFaultCount++;
      }
    }
    if (gsmSerial && !GSMconfig && !signalCOPS) {
      delay_millis(1000);
      GSMSerial.flush();
      Serial.println(F("Checking GSM config.."));
      GSMSerial.write("AT+COPS=0,1;+CMGF=1;+IPR=0\r");
      delay_millis(500);
      GSMSerial.write("AT+CNMI=0,0,0,0,0\r");
      delay_millis(500);
      if (gsmReadOK()) GSMconfig = true;
    } 
    
    if (gsmSerial && GSMconfig && !signalCOPS) {
      delay_millis(4000);
      gsmSerialFlush();
      Serial.println(F("Checking network status.."));
      Serial.println(readGSMResponse());
      GSMSerial.write("AT+COPS?;+CSQ\r");  
      delay_millis(1000);
      if (strstr(readGSMResponse(), ",\"")) {
        signalCOPS = true;
      } else delay_millis(2000);
    }
    if (gsmSerial && GSMconfig && signalCOPS) {
      GSMSerial.write("AT&W\r");
      Serial.println("");
      Serial.println(F("GSM READY"));
      Serial.println(F("****************************************"));
      break;
    }
    if (serialFaultCount == 3) {
      Serial.println("CHECK_GSM_MODULE_POWER_OR_CONNECTION");
      break;
    }
    if (millis() - initStart > initTimeout) {
      Serial.print("GSM MODULE ERROR: ");
      if (!gsmSerial) {
        Serial.println("POWER_OR_HARDWARE_SERIAL_CONNECTION_ERROR");
      } else if (!GSMconfig) {
        Serial.println("MODULE_ERROR_UNABLE_TO_SET_PARAMETERS");
      } else if (!signalCOPS) {
        Serial.println("NETWORK_OR_SIM_ERROR");
      }
      break;
    }
  }
  if (debug_flag == 0) Watchdog.reset();
}

void sleepGSM() {
  //the module enters sleep mode after 5 seconds of inactivity
  GSMSerial.write("AT+CSCLK=2\r");
  if (gsmReadOK()) {
    Serial.println("GSM going to sleep!");
  } else {
    Serial.println("GSM failed to sleep!");
  }
}

void wakeGSM() {
  /* to wake it up, you need to send any AT command, which will be ignored by 
  the module (so no response), followed (within 5 seconds) by "AT+CSCLK=0\r" */

  // To save more power +CFUN=0 before SLEEP and +CFUN=1 after WAKE UP.
  GSMSerial.write("AT\r");
  delay_millis(100);
  GSMSerial.write("AT\r");
  gsmReadOK();
  GSMSerial.write("AT+CSCLK=0\r");
  // gsmReadOK();
  // GSMSerial.write("AT\r");
  if (gsmReadOK()) {
    Serial.println("GSM is awake!");
  } else {
    Serial.println("GSM did NOT wake!");
  }
}

/**Reset and initialize GSM
 * turn ON and OFF GSM
 * set GSM to no echo mode
 * set GSM to text mode
*/
void resetGSM() {
  if (debug_flag == 0) Watchdog.reset();
  detachInterrupt(digitalPinToInterrupt(GSMINT));
  Serial.println("Initializing GSM...");
  digitalWrite(GSMPWR, LOW);
  delay_millis(1000);  //wait for at least 800ms before power on
  digitalWrite(GSMPWR, HIGH);
  delay_millis(3000);
  gsmNetworkAutoConnect();
  REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT2; //clear interrupt flag before enabling
  attachInterrupt(digitalPinToInterrupt(GSMINT), ringISR, FALLING);
  if (debug_flag == 0) Watchdog.reset();
}

void init_gsm() {
  GSMSerial.write("AT+CMGF=1\r");
  delay_millis(200);
  if (strstr(readGSMResponse(), "OK")) {
    Serial.println(" ");
    Serial.println("GSM module set to text mode.");
  }
  GSMSerial.write("ATE0\r");  //turn off echo
  if (gsmReadOK()) {
    Serial.println("GSM set to NO echo mode.");
  }
}

void turn_ON_GSM(int _gsmPowerMode) {
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  if (_gsmPowerMode == 2) {             // ON and OFF cycle
    Serial.println("Turning ON GSM ");
    digitalWrite(GSMPWR, HIGH);
    delay_millis(5000);
    // gsmNetworkAutoConnect();
  } else if (_gsmPowerMode == 1) {      // sleep wake cycle
    wakeGSM();
  } else {    // always OM

  }
  if (debug_flag == 0) {
    Watchdog.reset();
  }
}

void turn_OFF_GSM(int _gsmPowerMode) {
  if (debug_flag == 0) {
    Watchdog.reset();
  }
  delay_millis(1000);
  if (_gsmPowerMode == 2) {        // ON and OFF cycle
    Serial.println("Turning OFF GSM . . .");
    digitalWrite(GSMPWR, LOW);
    delay_millis(2000);
    // readGSMResponse();
  }
  if (_gsmPowerMode == 1) {       // sleep wake cycle
    sleepGSM();
    delay_millis(2000);
  } else {
  }
}

void gsm_network_connect() {
  int overflow_counter = 0;
  do {
    gsmNetworkAutoConnect();
    overflow_counter++;
  } while (readCSQ() == "0" || overflow_counter < 2);

  GSMSerial.write("AT\r");
  if (gsmReadOK()) {
    Serial.println("GSM module connected to network!");
  } else {
    Serial.println("");
    Serial.println("Check GSM if connected or powered ON!");
  }
  Serial.println(readCSQ());
}

void update_time_with_GPRS() {
  detachInterrupt(digitalPinToInterrupt(GSMINT));
  char timebuffer[13];
  int ts_buffer[7];

  GSMSerial.write("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r");  //AT+SAPBR=3,1,"Contype","GPRS"
  delay_millis(200);
  readGSMResponse();
  GSMSerial.write("AT+SAPBR=3,1,\"APN\",\"internet\"\r");  //AT+SAPBR=3,1,"APN","internet"
  delay_millis(200);
  readGSMResponse();
  GSMSerial.write("AT+SAPBR=1,1\r");  //Open bearer
  delay_millis(4000);
  readGSMResponse();
  GSMSerial.write("AT+CNTPCID=1\r"); //use bearer profile 1
  delay_millis(500);
  readGSMResponse();
  GSMSerial.write("AT+CNTP=\"time.upd.edu.ph\",32\r");  //AT+CNTP="time.upd.edu.ph",32
  delay_millis(200);
  readGSMResponse();
  GSMSerial.write("AT+CNTP\r");
  delay_millis(3000);
  readGSMResponse();
  GSMSerial.print("AT+CCLK?\r");
  delay(1000);

  if (strstr(readGSMResponse(), "+CCLK: \"2"))  //2 denotes 3rd number or year YYYY
  {
    response[27] = '\0';
    for (byte i = 0; i < strlen(response); i++) {
      response[i] = response[i + 10];
    }

    char *ts_token = strtok(response, ",/:+");  //22/09/23,18:38:19+08
    byte ts_counter = 0;
    timebuffer[0] = '\0';
    while (ts_token != NULL) {

      ts_buffer[ts_counter] = atoi(ts_token);
      ts_counter++;
      ts_token = strtok(NULL, ",/:+");
    }
    Serial.println("Time synced with GSM network time!");

    // Serial.println(timebuffer);
    adjustDate(ts_buffer[0], ts_buffer[1], ts_buffer[2], ts_buffer[3], ts_buffer[4], ts_buffer[5], ts_buffer[6]);
    readTimeStamp();
    Serial.print("Current timestamp: ");
    Serial.println(Ctimestamp);
    Serial.println(" ");

    GSMSerial.write("AT+SAPBR=0,1\r");
    delay_millis(1000);
    readGSMResponse();

  } else {
    Serial.println("Time sync failed!");
    resetGSM();
  }
  REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT2; //clear interrupt flag before enabling
  attachInterrupt(digitalPinToInterrupt(GSMINT), ringISR, FALLING);
}

// Hardcode: For changing server numbers
// void changeServerNumber() {
//   unsigned long startHere = millis();
//   Serial.print("Enter new server number: ");
//   while (!Serial.available()) {
//     if (timeOutExit(startHere, DEBUGTIMEOUT)) {
//       debug_flag_exit = true;
//       break;
//     }
//   }
//   if (Serial.available()) {
//     String ser_num = Serial.readStringUntil('\n');

//     ser_num.toUpperCase();
//     ser_num.trim();
//     Serial.println(ser_num);

//     if (ser_num.toInt() > 0) {
//       ser_num.toCharArray(flashServerNumber.inputNumber, 13);
//     } else if (ser_num == "GLOBE1") {
//       Serial.println("Server number set to: GLOBE1 - 639175972526");
//       strcpy(flashServerNumber.inputNumber, "639175972526");
//     } else if (ser_num == "GLOBE2") {
//       Serial.println("Server number set to: GLOBE2 - 639175388301");
//       strcpy(flashServerNumber.inputNumber, "639175388301");
//     } else if (ser_num == "SMART1") {
//       Serial.println("Server number set to: SMART1 - 639088125642");
//       strcpy(flashServerNumber.inputNumber, "639088125642");
//     } else if (ser_num == "SMART2") {
//       Serial.println("Server number set to: SMART2 - 639088125639");
//       strcpy(flashServerNumber.inputNumber, "639088125639");
//     } else if (ser_num == "DAN") {
//       Serial.println("Server number set to: 639762372823");
//       strcpy(flashServerNumber.inputNumber, "639762372823");
//     } else if (ser_num == "WEB") {
//       Serial.println("Server number set to: 639053648335");
//       strcpy(flashServerNumber.inputNumber, "639053648335");
//     } else if (ser_num == "KATE") {
//       Serial.println("Server number set to: 639476873967");
//       strcpy(flashServerNumber.inputNumber, "639476873967");
//     } else if (ser_num == "LOUIE") {
//       Serial.println("Server number set to: 639561586434");
//       strcpy(flashServerNumber.inputNumber, "639561586434");
//     } else if (ser_num == "CARLA") {
//       Serial.println("Server number set to: 639557483156");
//       strcpy(flashServerNumber.inputNumber, "639557483156");
//     } else if (ser_num == "REYN") {
//       Serial.println("Server number set to: 639669622726");
//       strcpy(flashServerNumber.inputNumber, "639669622726");
//     } else if (ser_num == "KENNEX") {
//       Serial.println("Server number set to: 639293175812");
//       strcpy(flashServerNumber.inputNumber, "639293175812");
//     } else if (ser_num == "DON") {
//       Serial.println("Server number set to: 639179995183");
//       strcpy(flashServerNumber.inputNumber, "639179995183");
//     } else if (ser_num == "JOSE") { 
//       Serial.println("Server number set to: 639451136212");
//       strcpy(flashServerNumber.inputNumber, "639451136212");
//     } else {
//       Serial.println("Server number defaulted to GLOBE1");
//       strcpy(flashServerNumber.inputNumber, "639175972526");
//     }
//     newServerNum.write(flashServerNumber);  // save to flash memory
//   }
// }

void changeServerNumber() {
  char serverNumberBuffer[15];
  unsigned long startHere = millis();
  Serial.print("Enter new server number: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    // String ser_num = Serial.readStringUntil('\n');
    strcpy(serverNumberBuffer, getSerialCommand(60000));
    

    // if (strlen(serverNumberBuffer) == 0) {
    //   strncpy(serverNumberBuffer, default_serverNumber, strlen(default_serverNumber));
    //   Serial.print("Default serial number: ");
    // }

    serverNumberBuffer[strlen(serverNumberBuffer)+1]=0x00; 
    Serial.println(serverNumberBuffer);
    
    if (inputIs(serverNumberBuffer,"GLOBE1")) {
      Serial.println("Server number set to: GLOBE1 - 09175972526");
      strcpy(flashServerNumber.inputNumber, "09175972526");
    } else if (inputIs(serverNumberBuffer,"GLOBE2")) {
      Serial.println("Server number set to: GLOBE2 - 09175388301");
      strcpy(flashServerNumber.inputNumber, "09175388301");
    } else if (inputIs(serverNumberBuffer,"SMART1")) {
      Serial.println("Server number set to: SMART1 - 09088125642");
      strcpy(flashServerNumber.inputNumber, "09088125642");
    } else if (inputIs(serverNumberBuffer,"SMART2")) {
      Serial.println("Server number set to: SMART2 - 09088125639");
      strcpy(flashServerNumber.inputNumber, "09088125639");
    } else if (inputIs(serverNumberBuffer,"DAN")) {
      Serial.println("Server number set to: 09762372823");
      strcpy(flashServerNumber.inputNumber, "09762372823");
    } else if (inputIs(serverNumberBuffer,"WEB")) {
      Serial.println("Server number set to: 09053648335");
      strcpy(flashServerNumber.inputNumber, "09053648335");
    } else if (inputIs(serverNumberBuffer,"KATE")) {
      Serial.println("Server number set to: 09476873967");
      strcpy(flashServerNumber.inputNumber, "09476873967");
    // } else if (serverNumberBuffer == "LOUIE") {
    //   Serial.println("Server number set to: 639561586434");
    //   strcpy(flashServerNumber.inputNumber, "639561586434");
    // } else if (serverNumberBuffer == "CARLA") {
    //   Serial.println("Server number set to: 639557483156");
    //   strcpy(flashServerNumber.inputNumber, "639557483156");
    } else if (inputIs(serverNumberBuffer,"REYN")) {
      Serial.println("Server number set to: 09669622726");
      strcpy(flashServerNumber.inputNumber, "09669622726");
    } else if (inputIs(serverNumberBuffer,"KENNEX")) {
      Serial.println("Server number set to: 09293175812");
      strcpy(flashServerNumber.inputNumber, "09293175812");
    } else if (inputIs(serverNumberBuffer,"DON")) {
      Serial.println("Server number set to: 09179995183");
      strcpy(flashServerNumber.inputNumber, "09179995183");
    } else if (inputIs(serverNumberBuffer,"JAY")) { 
      Serial.println("Server number set to: 09451136212");
      strcpy(flashServerNumber.inputNumber, "09451136212");
    } else if (inputIs(serverNumberBuffer,"CHI")) { 
      Serial.println("Server number set to: 09954127577");
      strcpy(flashServerNumber.inputNumber, "09954127577");
    } else if (atoi(serverNumberBuffer) == 0) {          //Server number should start with an integer or '-'/'+' sign, else it will default to GLOBE1
      Serial.println("Server number defaulted to GLOBE1");
      strcpy(flashServerNumber.inputNumber, default_serverNumber);
    } else {
      Serial.print("New server number: ");
      Serial.println(serverNumberBuffer);
      strcpy(flashServerNumber.inputNumber, serverNumberBuffer);
    } 
    newServerNum.write(flashServerNumber);  // save to flash memory
  }
}

char* getSerialCommand(int commandInputTimeout) {
  static char commandContainer[500];
  int containerIndex = 0;
  commandContainer[containerIndex] = 0x00;
  unsigned long commandStart = millis();                        // initial start for timeout count
  
  while (millis() - commandStart < commandInputTimeout) {   
    char buf; 
    buf = Serial.read();                                    // execute loop if timout is not reached
    if (buf == '\n' || buf == '\r'){
      break;
    } else {
      commandContainer[containerIndex++] = buf;
      commandContainer[containerIndex] = 0x00;
      commandStart = millis();
    }
    // if (read_serial_input(buf, serialInputbuf, 250) > 0) {                   // waits for valid serial input
    //   // programMenu(serial_input_buffer);                                                   // do something with the serial input here
    //   commandStart = millis();                                                              // resets timeout start for new command
    // }

  }
  return commandContainer;
}

/* Calculate day of week in proleptic Gregorian calendar. Sunday == 0. */
int weekday(int year, int month, int day) {
  int adjustment, mm, yy;
  year += 4000;
  adjustment = (14 - month) / 12;
  mm = month + 12 * adjustment - 2;
  yy = year - adjustment;
  return (day + (13 * mm - 1) / 5 + yy + yy / 4 - yy / 100 + yy / 400) % 7;
}

bool GSMWaitResponse(const char* responseRef, int waitDuration, int showResponse) {
  bool responseValid = false;
  char toCheck[50];
  char charBuffer;
  char responseBuffer[300];
  unsigned long waitStart = millis();
  strcpy(toCheck, responseRef);
  toCheck[strlen(toCheck)+1] = 0x00;

  do {
    delay_millis(100);
      for (int i = 0; i < sizeof(responseBuffer); i++){
        responseBuffer[i] = 0x00;
      }
      for (int j = 0; j < sizeof(responseBuffer) && GSMSerial.available() > 0 ; j++) {
        charBuffer = GSMSerial.read();
        if (charBuffer == '\n' || charBuffer == '\r') {
          break;
        } else {
          responseBuffer[j] = charBuffer;
        }
      }
      if (strlen(responseBuffer) >= sizeof(responseBuffer)) {
        responseBuffer[sizeof(responseBuffer)-1] = 0x00;
      } else { responseBuffer[strlen(responseBuffer)+1]=0x00; }

      if (strlen(responseBuffer) > 0 && responseBuffer != "/n") {
        if (showResponse > 0) Serial.println(responseBuffer);
        if (strstr(responseBuffer, toCheck)) {
          responseValid = true;
          // break;
        }
      }
  } while (!responseValid && millis() - waitStart < waitDuration);
  return responseValid;
}
