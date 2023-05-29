/**
 * @brief Send message to mobile number
 * 
 * @return success: sent, failed: not sent
 */
void send_thru_gsm(const char* inputMessage, String serverNumber) {
  // turn_ON_GSM(get_gsm_power_mode());
  if (serverNumber == "") {
    serverNumber = default_serverNumber;
  }
  String smsCMD = ("AT+CMGS=");
  String quote = ("\"");
  String CR = ("\r");
  char msgToSend[250];
  char atCmgsNo[250];
  String incomingData = String(inputMessage);
  incomingData.replace("\r", "");
  incomingData.toCharArray(msgToSend, 250);
  String rawMsg = smsCMD + quote + serverNumber + quote + CR;
  rawMsg.toCharArray(atCmgsNo, 250);

  Serial.print("Sending to '");
  Serial.print(serverNumber);
  Serial.print("': ");
  Serial.println(msgToSend);
  int send_count = 0;

  long start = millis();
  long timeout = 10000;
  bool timeout_flag = false;
  bool error_flag = false;  //true if error occurred when sending message

  gsmSerialFlush();
  GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
  delay_millis(100);
  GSMSerial.write(msgToSend);
  delay_millis(400);
  GSMSerial.write(26);

  while (!timeout_flag) {

    delay_millis(200);
    sprintf(response, readGSMResponse());

    if (strstr(response, "+CMGS")) {
      Serial.println(F("Sent!"));
      flashLed(LED_BUILTIN, 2, 30);
      break;

    } else if (strstr(response, "ERROR")) {
      error_flag = true;
      Serial.println(F("Sending Failed!"));
      GSMSerial.write(27);
      delay_millis(200);
      resetGSM();
      gsmNetworkAutoConnect();
      gsmSerialFlush();
      GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
      delay_millis(100);
      GSMSerial.write(msgToSend);
      delay_millis(400);
      GSMSerial.write(26);
    }

    if (millis() - start >= timeout && !error_flag) {
      GSMSerial.write(27);
      Serial.println(F("No response from GSM!"));
      resetGSM();
      gsmNetworkAutoConnect();
      gsmSerialFlush();
      GSMSerial.write(atCmgsNo);  //AT+CMGS="639XXXXXXXXX"\r
      delay_millis(100);
      GSMSerial.write(msgToSend);
      delay_millis(400);
      GSMSerial.write(26);
      timeout_flag = true;
    }
  }

  // turn_OFF_GSM(get_gsm_power_mode());
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
  String quote = ("\"");
  String CR = ("\r");
  cmdAllchar[0] = '\0';

  Serial.setTimeout(15000);
  Serial.print("Insert GSM command: ");
  String manualCMD = Serial.readStringUntil('\n');
  Serial.println(manualCMD);
  String cmdAll = quote + manualCMD + quote + CR;
  cmdAll.toCharArray(cmdAllchar, sizeof(cmdAllchar));

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
 * REGISTER:SENSLOPE:639954645704         - register mobile number to control MCU *9
 * SENSORPOLL:SENSLOPE:                   - force data sampling *10
 * SERVERNUMBER:SENSLOPE:639954645704     - change server number in flash memory  *12
 * ?SERVERNUMBER:SENSLOPE:                - check current server number *13
 * RESET:SENSLOPE:                        - reset MCU   *5
 * ?PASSW:SENSLOPE:                       - check current password  *6
 * PASSWORD:SENSLOPE:[updated password]:  - change password  *8
 * SETDATETIME:SENSLOPE:[YYYY,MM,DD,HH,MM,SS,dd[0-6/m-sun],] 2021,02,23,21,22,40,1, *11
 * SETSENDINGTIME:SENSLOPE:[0-4]:         - sending time *14
 * CMD?:SENSLOPE:                         - read current DUE command
 * CMD:SENSLOPE:[SENSLOPE]:               - update DUE command
*/
void process_data(char *data) {
  //REGISTER:SENSLOPE:639954645704
  if (strncmp(data, "REGISTER:", 9) == 0) {
    Serial.println("REGISTER is read");
    char *_password = strtok(data + 9, ":");
    char *_regThisNum = strtok(NULL, ":");

    // Serial.println(_password);
    tempServer = String(_regThisNum);
    tempServer.replace(" ", "");
    regServer = tempServer;
    Serial.println(regServer);

    if (isPassWordCorrect(_password)) {
      registerNumber = true;
      send_thru_gsm("Number Registered!", regServer);
    }
  }
  //SENSORPOLL:SENSLOPE:
  else if (strncmp(data, "SENSORPOLL", 10) == 0) {
    Serial.println("SENSORPOLL is read");
    char *_password = strtok(data + 10, ":");
    // Serial.println(_password);

    if (isPassWordCorrect(_password) && registerNumber) {
      get_Due_Data(get_logger_mode(), regServer);
    }
  }
  //SERVERNUMBER:SENSLOPE:639954645704
  else if (strncmp(data, "SERVERNUMBER", 12) == 0) {
    char messageToSend[100];
    char newServer[50];
    Serial.println("change server number");
    char *_password = strtok(data + 12, ":");
    char *_newServerNum = strtok(NULL, ":");
    // Serial.println(_password);
    Serial.println(_newServerNum);

    if (isPassWordCorrect(_password) && registerNumber) {
      //strore new server number to flash memory
      strcpy(flashServerNumber.inputNumber, _newServerNum);
      newServerNum.write(flashServerNumber);  //save to flash memory

      get_serverNum_from_flashMem().toCharArray(newServer, sizeof(newServer));
      strncpy(messageToSend, "New server number: ", 19);
      strncat(messageToSend, newServer, sizeof(newServer));
      Serial.println(messageToSend);

      send_thru_gsm(messageToSend, regServer);
    }
  }
  //?SERVERNUMBER:SENSLOPE:
  else if (strncmp(data, "?SERVERNUMBER", 13) == 0) {
    char currenServerNumber[30];
    char messageToSend[100];
    Serial.println("Check current server number");
    char *_password = strtok(data + 13, ":");
    currenServerNumber[0] = '\0';
    messageToSend[0] = '\0';

    if (isPassWordCorrect(_password) && registerNumber) {
      get_serverNum_from_flashMem().toCharArray(currenServerNumber, sizeof(currenServerNumber));

      strncpy(messageToSend, "Current server number: ", 23);
      strncat(messageToSend, currenServerNumber, sizeof(currenServerNumber));
      Serial.println(messageToSend);

      send_thru_gsm(messageToSend, regServer);
    }
  }
  //RESET:SENSLOPE:
  else if (strncmp(data, "RESET", 5) == 0) {
    Serial.println("Resetting microcontroller!");
    char *_password = strtok(data + 5, ":");

    if (isPassWordCorrect(_password) && registerNumber) {
      send_thru_gsm("Resetting datalogger, please register your number again to access OTA commands.", regServer);
      Serial.println("Resetting Watchdog in 2 seconds");
      int countDownMS = Watchdog.enable(2000);  //max of 16 seconds
    }
  }
  //?PASSW:SENSLOPE:
  else if (strncmp(data, "?PASSW", 6) == 0) {
    char messageToSend[100];
    Serial.println("Reading current password!");
    char *_password = strtok(data + 6, ":");

    if (isPassWordCorrect(_password) && registerNumber) {
      Serial.println("Sending current password!");
      strncpy(messageToSend, "Current password: ", 18);
      strncat(messageToSend, get_password_from_flashMem(), 50);
      send_thru_gsm(messageToSend, regServer);
    }
  }
  //PASSWORD:SENSLOPE:[updated password]:
  else if (strncmp(data, "PASSWORD", 8) == 0) {
    char messageToSend[100];
    char newPassword[50];
    char *_password = strtok(data + 8, ":");
    char *_newPassword = strtok(NULL, ":");
    Serial.println(_newPassword);
    Serial.println("change server number");

    if (isPassWordCorrect(_password) && registerNumber) {
      //strore new password to flash memory
      strcpy((flashPassword.keyword), _newPassword);
      flashPasswordIn.write(flashPassword);  //save to flash memory

      strncpy(messageToSend, "New password: ", 14);
      strncat(messageToSend, get_password_from_flashMem(), 50);
      Serial.println(messageToSend);

      send_thru_gsm(messageToSend, regServer);
    }
  }
  //SETDATETIME:SENSLOPE:[YYYY,MM,DD,HH,MM,SS,dd[0-6/m-sun],] 2021,02,23,21,22,40,1,
  else if (strncmp(data, "SETDATETIME", 11) == 0) {
    Serial.println("change timestamp!");
    char messageToSend[100];
    messageToSend[0] = '\0';
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

    if (isPassWordCorrect(_password) && registerNumber) {
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

      strncpy(messageToSend, "Current timestamp: ", 19);
      strncat(messageToSend, Ctimestamp, 12);
      Serial.println(messageToSend);
      send_thru_gsm(messageToSend, regServer);
    }
  }
  //SENDINGTIME:SENSLOPE:[0-4]:
  else if (strncmp(data, "SETSENDINGTIME", 14) == 0) {
    Serial.println("change sending time!");
    char messageToSend[100];
    char sendStorage[10];
    messageToSend[0] = '\0';
    sendStorage[0] = '\0';
    Serial.println(data);

    char *_password = strtok(data + 14, ":");
    char *inputSending = strtok(NULL, ":");
    int _inputSending = atoi(inputSending);

    if (isPassWordCorrect(_password) && registerNumber) {
      //set sending time
      Serial.println(_inputSending);
      alarmStorage.write(_inputSending);
      sprintf(sendStorage, "%d", alarmFromFlashMem());

      strncpy(messageToSend, "Updated sending time: ", 22);
      strncat(messageToSend, sendStorage, sizeof(sendStorage));
      Serial.println(messageToSend);
      send_thru_gsm(messageToSend, regServer);
    }
  }
  //CMD?:SENSLOPE:
  else if (strncmp(data, "CMD?", 4) == 0) {
    Serial.println("change sending time!");
    char messageToSend[100];
    ;
    messageToSend[0] = '\0';
    char *_password = strtok(data + 4, ":");

    if (isPassWordCorrect(_password) && registerNumber) {
      //read DUE command
      // sensCommand = passCommand.read();
      strncpy(messageToSend, "Current DUE command: ", 21);
      strncat(messageToSend, get_sensCommand_from_flashMem(), 8);
      Serial.println(messageToSend);
      send_thru_gsm(messageToSend, regServer);
    }
  }
  //CMD:SENSLOPE[ARQCM6T/6S]
  else if (strncmp(data, "CMD?", 4) == 0) {
    Serial.println("change sending time!");
    char messageToSend[100];
    messageToSend[0] = '\0';

    char *_password = strtok(data + 4, ":");
    char *dueCmd = strtok(NULL, ":");

    if (isPassWordCorrect(_password) && registerNumber) {
      //read DUE command
      strcpy((sensCommand.senslopeCommand), dueCmd);
      passCommand.write(sensCommand);  //save to flash memory

      //read command
      // sensCommand = passCommand.read();
      strncpy(messageToSend, "Current DUE command: ", 21);
      strncat(messageToSend, get_sensCommand_from_flashMem(), 8);
      Serial.println(messageToSend);
      send_thru_gsm(messageToSend, regServer);
    }
  }
  //LISTPHONENUM:SENSLOPE:
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
      // Serial.println("Read OK from GSM");
      return true;
      break;
    }
    // Serial.print(" .");
    delay_millis(10);
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

/**
 * @brief Check if mobile number network is GLOBE or SMART
 * 
 * @return : SMART: 63908, failed: not sent
 * @default : GLOBE
 */
String simNetwork() {
  char tNum[50];
  get_serverNum_from_flashMem().toCharArray(tNum, sizeof(tNum));
  if (strstr(tNum, "63908")) {
    return "SMART";
  } else {
    return "GLOBE";
  }
}

void gsmNetworkAutoConnect() {
  String response;
  Serial.println("Connecting GSM to network...");
  delay_millis(2000);
  for (int i = 0; i < 10; i++) {
    gsmSerialFlush();
    GSMSerial.write("AT+COPS=0,1;+CMGF=1;+IPR=115200;+COPS?;+CSQ\r");
    delay_millis(1000);
    if (strstr(readGSMResponse(), "OK")) {
      GSMSerial.write("ATE0&W\r");  //turn off echo
      if (gsmReadOK()) {
        Serial.println("GSM set to NO echo mode.");
        Serial.println(" ");
        Serial.println("GSM is now ready!");
      }
      break;
    } else {
      Serial.print(". ");
    }
    if (i == 9) {
      Serial.println("");
      Serial.println("Check GSM if connected or powered ON!");
    }
  }
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
  int overflow_counter = 0;
  Serial.println("GSM resetting...");
  digitalWrite(GSMRST, LOW);
  delay_millis(500);  //reset timing sequence min duation 105ms
  digitalWrite(GSMRST, HIGH);
  delay_millis(2700);  //reset timing sequence indicate 2.7s before status becomes active

  for (int i = 0; i <= 4; i++) {
    Serial.println("Sending AT cmd to GSM");
    GSMSerial.write("AT\r");  //gsm initialization
    Serial.print(". ");
    delay_millis(1000);
    if (gsmReadOK() == true) {
      Serial.println("GSM reset done");
      break;
    }
    if (i == 2) {
      Serial.println("2nd reset sequence");  // not working with rev.3 (violet) boards w/o IC switch
      digitalWrite(GSMPWR, LOW);
      delay_millis(1000);  //wait for at least 800ms before power on
      digitalWrite(GSMPWR, HIGH);
      delay_millis(1000);
    }
    if (i == 4) {
      Serial.println("GSM reset failed!");
    }
  }
  Serial.println(" ");
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
  // disable_watchdog();
  if (_gsmPowerMode == 2) {
    Serial.println("Turning ON GSM ");
    digitalWrite(GSMPWR, HIGH);
    delay_millis(2000);
    gsmNetworkAutoConnect();
  } else if (_gsmPowerMode == 1) {
    wakeGSM();
  } else {    // always OM

  }
  // enable_watchdog();
}

void turn_OFF_GSM(int _gsmPowerMode) {

  delay_millis(1000);
  if (_gsmPowerMode == 2) {
    gsmDeleteReadSmsInbox();
    delay_millis(2000);
    Serial.println("Turning OFF GSM . . .");
    digitalWrite(GSMPWR, LOW);
    delay_millis(1500);
    // GSMSerial.write("AT+CPOWD=1\r");
    delay_millis(300);
    readGSMResponse();
  }
  if (_gsmPowerMode == 1) {
    gsmDeleteReadSmsInbox();
    delay_millis(2000);
    sleepGSM();
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
  char timebuffer[13];
  int ts_buffer[7];

  GSMSerial.write("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r");  //AT+SAPBR=3,1"Contype","GPRS"
  delay_millis(200);
  readGSMResponse();
  GSMSerial.write("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r");  //AT+SAPBR=3,1,"APN","CMNET"
  delay_millis(1000);
  readGSMResponse();
  GSMSerial.write("AT+SAPBR=1,1\r");  //Open bearer
  delay_millis(4000);
  readGSMResponse();
  GSMSerial.write("AT+CNTPCID=1\r");
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
}

// Hardcode: For changing server numbers
void changeServerNumber() {
  unsigned long startHere = millis();
  Serial.print("Enter new server number: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    String ser_num = Serial.readStringUntil('\n');

    ser_num.toUpperCase();
    ser_num.trim();
    Serial.println(ser_num);

    if (ser_num.toInt() > 0) {
      ser_num.toCharArray(flashServerNumber.inputNumber, 13);
    } else if (ser_num == "GLOBE1") {
      Serial.println("Server number set to: GLOBE1 - 639175972526");
      strcpy(flashServerNumber.inputNumber, "639175972526");
    } else if (ser_num == "GLOBE2") {
      Serial.println("Server number set to: GLOBE2 - 639175388301");
      strcpy(flashServerNumber.inputNumber, "639175388301");
    } else if (ser_num == "SMART1") {
      Serial.println("Server number set to: SMART1 - 639088125642");
      strcpy(flashServerNumber.inputNumber, "639088125642");
    } else if (ser_num == "SMART2") {
      Serial.println("Server number set to: SMART2 - 639088125639");
      strcpy(flashServerNumber.inputNumber, "639088125639");
    } else if (ser_num == "DAN") {
      Serial.println("Server number set to: 639762372823");
      strcpy(flashServerNumber.inputNumber, "639762372823");
    } else if (ser_num == "WEB") {
      Serial.println("Server number set to: 639053648335");
      strcpy(flashServerNumber.inputNumber, "639053648335");
    } else if (ser_num == "KATE") {
      Serial.println("Server number set to: 639476873967");
      strcpy(flashServerNumber.inputNumber, "639476873967");
    } else if (ser_num == "LOUIE") {
      Serial.println("Server number set to: 639561586434");
      strcpy(flashServerNumber.inputNumber, "639561586434");
    } else if (ser_num == "CARLA") {
      Serial.println("Server number set to: 639557483156");
      strcpy(flashServerNumber.inputNumber, "639557483156");
    } else if (ser_num == "REYN") {
      Serial.println("Server number set to: 639669622726");
      strcpy(flashServerNumber.inputNumber, "639669622726");
    } else if (ser_num == "KENNEX") {
      Serial.println("Server number set to: 639293175812");
      strcpy(flashServerNumber.inputNumber, "639293175812");
    } else if (ser_num == "DON") {
      Serial.println("Server number set to: 639179995183");
      strcpy(flashServerNumber.inputNumber, "639179995183");
    } else if (ser_num == "JOSE") { 
      Serial.println("Server number set to: 639451136212");
      strcpy(flashServerNumber.inputNumber, "639451136212");
    } else if (ser_num == "CHI") { 
      Serial.println("Server number set to: 639954127577");
      strcpy(flashServerNumber.inputNumber, "639954127577");
    } else {
      Serial.println("Server number defaulted to GLOBE1");
      strcpy(flashServerNumber.inputNumber, "639175972526");
    }
    newServerNum.write(flashServerNumber);  // save to flash memory
  }
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
