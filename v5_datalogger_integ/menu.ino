#define ATCMD "AT"
#define ATECMDTRUE "ATE"
#define ATECMDFALSE "ATE0"
#define OKSTR "OK"
#define ERRORSTR "ERROR"

bool ate = false;

void getAtcommand() {
  // wakeGSM();
  String serial_line, command;
  int i_equals = 0;
  unsigned long startHere = millis();
  bool timeToExit;
  sending_stack[0] = '\0'; 
  
  do {
    timeToExit = timeOutExit(startHere, DEBUGTIMEOUT);
    serial_line = Serial.readStringUntil('\n');
  } while (serial_line == "" && timeToExit == false);
  serial_line.toUpperCase();
  serial_line.replace("\r", "");

  if (timeToExit) {
    Serial.println("Exit from debug menu");
    debug_flag_exit = true;
  }

  // echo command if ate is set, default true
  if (ate)
    Serial.println(serial_line);

  // get characters before '='
  i_equals = serial_line.indexOf('=');
  if (i_equals == -1)
    command = serial_line;
  else
    command = serial_line.substring(0, i_equals);

  if (command == ATCMD)
    Serial.println(OKSTR);
  else if (command == ATECMDTRUE) {
    ate = true;
    Serial.println(OKSTR);
  } else if (command == ATECMDFALSE) {
    ate = false;
    Serial.println(OKSTR);
  } else if (command == "A") {

    Serial.print("Datalogger mode: ");
    get_logger_mode_equivalent();

    if (get_logger_mode() == 0) {
      // default arQ like sending
      turn_ON_GSM(get_gsm_power_mode());
      send_rain_data(0);
      get_Due_Data(1, get_serverNum_from_flashMem());
      turn_OFF_GSM(get_gsm_power_mode());
    } else if (get_logger_mode() == 1) {
      // arQ + 1 LoRa receiver
      receive_lora_data(1);
      if (gsmPwrStat) {
        turn_ON_GSM(get_gsm_power_mode());
      }
      get_Due_Data(1, get_serverNum_from_flashMem());
      send_rain_data(0);
      if (gsmPwrStat) {
        turn_OFF_GSM(get_gsm_power_mode());
      }

    } else if (get_logger_mode() == 2) {
      // LoRa transmitter of version 5 datalogger
      get_Due_Data(2, get_serverNum_from_flashMem());
    } else if (get_logger_mode() == 3) {
      // only one trasmitter
      turn_ON_GSM(get_gsm_power_mode());
      send_rain_data(0);
      receive_lora_data(3);
      turn_OFF_GSM(get_gsm_power_mode());

    } else if (get_logger_mode() == 4) {
      // Two transmitter
      turn_ON_GSM(get_gsm_power_mode());
      send_rain_data(0);
      receive_lora_data(4);
      turn_OFF_GSM(get_gsm_power_mode());
    } else if (get_logger_mode() == 5) {
      // Three transmitter
      turn_ON_GSM(get_gsm_power_mode());
      send_rain_data(0);
      receive_lora_data(5);
      turn_OFF_GSM(get_gsm_power_mode());
    } else if (get_logger_mode() == 6) {
      // Sends rain gauge data ONLY
      turn_ON_GSM(get_gsm_power_mode());
      send_rain_data(0);
      delay_millis(1000);
      turn_OFF_GSM(get_gsm_power_mode());
    } else if (get_logger_mode() == 7) {
      // GNSS sensor only (GSM)
      turn_ON_GSM(get_gsm_power_mode());
      getGNSSData(dataToSend, sizeof(dataToSend)); //read gnss data
      send_thru_gsm(dataToSend, get_serverNum_from_flashMem());
      delay_millis(1000);
      turn_OFF_GSM(get_gsm_power_mode());
    } else if (get_logger_mode() == 8) {
      // GNSS sensor only (LORA)
      getGNSSData(dataToSend, sizeof(dataToSend)); //read gnss data
      send_thru_lora(dataToSend);
      delay(100);
      send_thru_lora(read_batt_vol(get_calib_param()));
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "B") {
    Serial.print("Rain tips: ");
    if (get_rainGauge_type() == 0) {
      Serial.println(rainTips * 2.0);
    } else if (get_rainGauge_type() == 1) {
      Serial.println(rainTips * 5.0);
    } else {
      Serial.println(rainTips);
    }
    delay_millis(20);
    Serial.print("Equivalent value: ");
    Serial.print(rainTips);
    Serial.println(" mm");
    resetRainTips();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "C") {
    printMenu();
  } else if (command == "D") {
    Serial.print("Logger mode: ");
    Serial.println(get_logger_mode());
    printLoggerMode();
    if (isChangeParam())
      setLoggerMode();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "E") {
    readTimeStamp();
    Serial.print("Timestamp: ");
    Serial.println(Ctimestamp);
    if (isChangeParam())
      setupTime();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "TIME_SYNC" || command == "F") {
    Serial.println("Results may vary depending on your network connection");
    update_time_with_GPRS();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "G") {
    if (get_logger_mode() == 1) {
      Serial.print("Gateway sensor name: ");
      Serial.println(get_logger_A_from_flashMem());
      Serial.print("Remote sensor name: ");
      Serial.println(get_logger_B_from_flashMem());
    } else if (get_logger_mode() == 3) {
      Serial.print("Gateway name: ");
      Serial.println(get_logger_A_from_flashMem());
      Serial.print("Remote Sensor Name A: ");
      Serial.println(get_logger_B_from_flashMem());
    } else if (get_logger_mode() == 4) {
      Serial.print("Gateway name: ");
      Serial.println(get_logger_A_from_flashMem());
      Serial.print("Remote Sensor Name A: ");
      Serial.println(get_logger_B_from_flashMem());
      Serial.print("Remote Sensor Name B: ");
      Serial.println(get_logger_C_from_flashMem());
    } else if (get_logger_mode() == 5) {
      Serial.print("Gateway name: ");
      Serial.println(get_logger_A_from_flashMem());
      Serial.print("Remote Sensor Name A: ");
      Serial.println(get_logger_B_from_flashMem());
      Serial.print("Remote Sensor Name B: ");
      Serial.println(get_logger_C_from_flashMem());
      Serial.print("Remote Sensor Name C: ");
      Serial.println(get_logger_D_from_flashMem());
    } else if (get_logger_mode() == 13) {
      Serial.print("Gateway name: ");
      Serial.println(get_logger_A_from_flashMem());
      Serial.print("Remote Sensor Name A: ");
      Serial.println(get_logger_B_from_flashMem());
      Serial.print("Remote Sensor Name B: ");
      Serial.println(get_logger_C_from_flashMem());
      Serial.print("Remote Sensor Name C: ");
      Serial.println(get_logger_D_from_flashMem());
      Serial.print("Remote Sensor Name D: ");
      Serial.println(get_logger_E_from_flashMem());
      Serial.print("Remote Sensor Name E: ");
      Serial.println(get_logger_F_from_flashMem());
    } else {  // 2; 6; 7; 8
      // Serial.print("Gateway sensor name: ");
      // Serial.println(get_logger_A_from_flashMem());
      Serial.print("Sensor name: ");
      Serial.println(get_logger_A_from_flashMem());
    }
    if (isChangeParam())
      inputLoggerNames();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "H") {
    Serial.print("Server Number: ");
    Serial.println(get_serverNum_from_flashMem());
    Serial.println("");
    Serial.println("Default server numbers:");
    Serial.println("GLOBE1 - 09175972526 ; GLOBE2 - 09175388301");
    Serial.println("SMART1 - 09088125642 ; SMART2 - 09088125639");
    if (isChangeParam())
      changeServerNumber();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "I") {
    resetGSM();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "J") {
    Serial.print("Rain collector type: ");
    Serial.println(get_rainGauge_type());
    Serial.println("[0] Pronamic Rain Collector (0.5mm/tip)");
    Serial.println("[1] DAVIS Rain Collector (0.2mm/tip)");
    Serial.println("[2] Generic Rain Collector (1.0/tip)");
    if (isChangeParam())
      setRainCollectorType();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "K") {
    Serial.print("Alarm interval stored: ");
    Serial.println(alarmFromFlashMem());
    print_rtcInterval();
    if (isChangeParam())
      setAlarmInterval();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "L") {
    // converted to battery voltage input either 12v or 4.2v
    Serial.print("Battery voltage reference: ");
    Serial.println(get_calib_param());
    Serial.println("[0] 12 volts battery");
    Serial.println("[1] 4.2 volts battery");
    if (isChangeParam())
      setIMUdataRawCalib();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "M") {
    unsigned long startHere = millis();
    char specialMsg[200];
    Serial.print("Enter message to send: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String gsmCmd = Serial.readStringUntil('\n');
      Serial.println(gsmCmd);
      gsmCmd.toCharArray(specialMsg, sizeof(specialMsg));
      GSMSerial.write("AT\r");
      delay_millis(300);
      send_thru_gsm(specialMsg, get_serverNum_from_flashMem());
      // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
      //   Serial.print("Sending to alternate number: ");
      //   send_thru_gsm(specialMsg, serverALT(get_serverNum_from_flashMem()));
      // }
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "N") {
    Serial.print("Current gsm power mode: ");
    Serial.println(get_gsm_power_mode());
    // Serial.println("[0] Hardware power ON or OFF");
    Serial.println("[0] GSM power always ON");
    Serial.println("[1] Sleep and wake via AT commands");
    Serial.println("[2] GSM power ON/OFF");
    if (isChangeParam())
      setGsmPowerMode();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "O") {
    manualGSMcmd();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "P") {
    Serial.print("Current command: ");
    // sensCommand = passCommand.read();

    Serial.print(get_sensCommand_from_flashMem());
    if (isChangeParam())
      changeSensCommand();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  }

  else if (command == "?") {
    Serial.println("*** Stored Parameters ***");
    Serial.print("");
    readTimeStamp();
    Serial.print("Real time clock: ");
    Serial.println(Ctimestamp);
    Serial.print("Send interval:   ");
    if (alarmFromFlashMem() == 0) {
      Serial.println("30 minutes (hh:00 & hh:30)");
    } else if (alarmFromFlashMem() == 1) {
      Serial.println("15 minutes (hh:00, hh:15, hh:30, hh:45)");
    } else if (alarmFromFlashMem() == 2) {
      Serial.println("10 minutes (hh:00, hh:10, hh:20, ... )");
    } else if (alarmFromFlashMem() == 3) {
      Serial.println("5 minutes (hh:00, hh:05, hh:10, hh:15, ... )");
    } else if (alarmFromFlashMem() == 4) {
      Serial.println("3 minutes (hh:00, hh:03, hh:06, hh:09, ... )");
    } else {
      Serial.println("Default 30 minutes (hh:00 & hh:30)");
    }
    Serial.print("Logger mode:     ");
    get_logger_mode_equivalent();
    Serial.print("Sensor command:  ");
    // sensCommand = passCommand.read();
    Serial.println(get_sensCommand_from_flashMem());
    Serial.print("Rain collector:  ");
    if (get_rainGauge_type() == 0) {
      Serial.println("Pronamic (0.5mm/tip)");
    } else if (get_rainGauge_type() == 1) {
      Serial.println("DAVIS (0.2mm/tip)");
    } else {  //option 2
      Serial.println("Generic (1.0/tip)");
    }
    Serial.print("Input Voltage:   ");
    Serial.print(readBatteryVoltage(get_calib_param()));
    Serial.println("v");
    Serial.print("Battery type:    ");
    if (get_calib_param() == 1) {
      Serial.println("Li-ion");
    } else {
      Serial.println("Lead acid");
    }
    Serial.print("RTC temperature: ");
    Serial.println(readTemp());
    Serial.print("");
    if (get_logger_mode() != 2) {
      Serial.print("Server number:   ");
      Serial.println(get_serverNum_from_flashMem());
      // if (serverALT(get_serverNum_from_flashMem()) != "NANEEEE") {
      //   Serial.print("Alternate number: ");
      //   Serial.println(serverALT(get_serverNum_from_flashMem()));
      // }
      Serial.print("Gsm power mode:  ");
      if (get_gsm_power_mode() == 1) {
        Serial.println("Sleep/wake via AT commands");
      } else if (get_gsm_power_mode() == 2) {
        Serial.println("ON/OFF via RTC alarm");
      } else {
        Serial.println("Always ON");
      }
      // Serial.println(get_gsm_power_mode());
      GSMSerial.write("AT+CSQ;+COPS?\r");
      delay_millis(200);
      readGSMResponse();

      // Serial.print("MCU password:    ");
      // Serial.println(get_password_from_flashMem());
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "TOGGLE_ACK") {
    if (logger_ack_filter_enabled()) {
      ack_filter.write(1);
      delay_millis(100);
      Serial.println("MODE: DISABLE logger filtering and acknowledgement");
    } else {
      ack_filter.write(0);
      delay_millis(100);
      Serial.println("MODE: ENABLE logger filtering and acknowledgement");
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "TOGGLE_UNLISTED") {
    if (allow_unlisted()) {
      allow_unlisted_flag.write(0);
      delay_millis(100);
      Serial.println("MODE: IGNORE all data from unlisted logger");

    } else {
      allow_unlisted_flag.write(1);
      delay_millis(100);
      Serial.println("MODE: ADD data of unlisted loggers to sending stack!");
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "CC") {
    printMenu2();
  } else if (command == "CHANGE_PASSWORD") {
    Serial.print("Password: ");
    Serial.println(get_password_from_flashMem());
    if (isChangeParam())
      changePassword();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    Serial.readStringUntil('\n');
  } else if (command == "EXIT" || command == "X") {

    Serial.println("Exiting debug mode!");
    resetGSM();
    resetRainTips();
    setNextAlarm(alarmFromFlashMem());
    delay_millis(75);
    rtc.clearINTStatus();  // needed to re-trigger rtc

    Serial.println("* * * * * * * * * * * * * * * * * * * *");
    debug_flag = 0;

  } else if (command == "GSM_RECEIVE_TEST") {
    Serial.println("GSM receive test!");
    flashLed(LED_BUILTIN, 3, 50);

    // Serial.println("AT + CNMI");
    // if (get_gsm_power_mode() == 0) {
    //   Serial.println("1st AT + CNMI");
    //   GSMSerial.write("AT+CNMI=1,2,0,0,0\r");
    //   delay_millis(100);
    // }
    GSMSerial.write("AT+CMGL=\"ALL\"\r");
    delay_millis(300);
    // Serial.println("after CNMI");
    while (GSMSerial.available() > 0) {
      processIncomingByte(GSMSerial.read(), 0);
    }

    // Serial.println("delete sms");
    // gsmDeleteReadSmsInbox();
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "INPUT_VOLTAGE") {
    // change battery input *12volts *4.2volts
    Serial.print("Voltage: ");
    Serial.println(readBatteryVoltage(get_calib_param()));
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "SEND_RAIN_VIA_LORA") {
    send_rain_data(1);
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "LORA_WAIT_TEST") {
    Serial.print("Datalogger Mode: ");
    get_logger_mode_equivalent();
    receive_lora_data(get_logger_mode());
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "LORA_SEND_TEST") {
    char send_buffer[150];
    send_buffer[0] = '\0';
    strcat(send_buffer, ">>");
    strncat(send_buffer, get_logger_A_from_flashMem(), 5);
    // strcat(send_buffer, "LABBDUE");
    strcat(send_buffer, "*SAMPLE_DATA_qwertyuiopasdfghjklzxcvbnm*");
    strncat(send_buffer, Ctimestamp, 12);
    send_buffer[strlen(send_buffer) + 1] = '\0';
    send_thru_lora(send_buffer);
    send_thru_lora(read_batt_vol(get_calib_param()));
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "CSQ") {
    Serial.print("CSQ: ");
    Serial.println(readCSQ());
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "RTC_TEMP") {
    Serial.print("RTC temperature: ");
    Serial.println(readTemp());
    // Serial.println(readTempRTC());
    // Serial.println(BatteryVoltage(get_logger_mode()));
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else if (command == "SD") {
    char sd_cmd[22] = "ARQCMD6C/121212121212";
    sd_cmd[22] = '\0';
    uint8_t sd_timeout = 0;
    unsigned long start = millis();
    Serial.println("Fetching config from customDue SD card: ");
    turn_ON_due(get_logger_mode());
    Serial.println(" ");
    delay_millis(2000);
    // Serial.println(sd_cmd);
    DUESerial.write(sd_cmd);
    while (sd_timeout == 0) {
      if ((millis() - start) > 10000) {
        // start = millis();
        Serial.print("Timeout reached");
        sd_timeout = 1;
      }
      for (int i = 0; i < 250; ++i) {
        streamBuffer[i] = 0x00;
      }
      DUESerial.readBytesUntil('\n', streamBuffer, 250);
      delay_millis(50);
      if (strlen(streamBuffer) <= 1) {
        Serial.print(".");
      } else {
        Serial.println(streamBuffer);
      }
      if (strstr(streamBuffer, "OK")) {
        sd_timeout = 1;
      }
    }
    turn_OFF_due(get_logger_mode());
    streamBuffer[0] = '\0';
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  }

  else if (command == "TRIGGER_TEST" || command == "T") {
    Serial.println("Trigger test:");
    turn_ON_due(get_logger_mode());
    delay_millis(5000);
    turn_OFF_due(get_logger_mode());
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  }

  // else if (command == "SEND_RAIN_TRU_LORA")
  // {
  //     Serial.println("sending rain gauge data . . .");
  //     // if (get_logger_mode() == 6)
  //     // {
  //     //     // LoRa
  //     //     send_rain_data(1);
  //     // }
  //     else
  //     {
  //         // GSM
  //         send_rain_data(0);
  //     }
  //     // wakeGSM();
  //     Serial.println("* * * * * * * * * * * * * * * * * * * *");
  // }
  else if (command == "XXX") {
    Serial.println("Coming soon...");
    Serial.println("* * * * * * * * * * * * * * * * * * * *");
  } else {
    Serial.println(" ");
    Serial.println("Command invalid!");
    Serial.println(" ");
  }
}

void printMenu() {
  Serial.println(F(" "));
  Serial.println(F("****************************************"));
  Serial.print(F("Firmware Version: "));
  Serial.println(F(firmwareVersion));
  Serial.println(F("****************************************"));
  Serial.println(F("[?] Print stored config parameters."));
  Serial.println(F("[A] Get data from customDue"));
  Serial.println(F("[B] Read rain gauge tips"));
  Serial.println(F("[C] Print this menu"));
  Serial.println(F("[D] Change LOGGER MODE"));
  Serial.println(F("[E] Set date and time manually"));
  Serial.println(F("[F] Set date and time using GPRS"));
  Serial.println(F("[G] Change DATALOGGER NAMES"));
  Serial.println(F("[H] Change SERVER NUMBER"));
  Serial.println(F("[I] Reset GSM"));
  Serial.println(F("[J] Set rain collector type."));
  Serial.println(F("[K] Change sending interval."));
  Serial.println(F("[L] Set battery type (4.2V Li-ion / 12V Lead Acid)"));
  Serial.println(F("[M] Send CUSTOM SMS to SERVER"));
  Serial.println(F("[N] Set GSM POWER MODE"));
  Serial.println(F("[O] Manual GSM commands"));
  Serial.println(F("[P] Change SENSLOPE command."));
  Serial.println(F("[X] Exit Debug mode."));
  Serial.println(F(" "));
  Serial.println(F("****************************************"));
}
void printMenu2() {
  Serial.println(F(" "));
  Serial.print(F("Extra options: "));
  Serial.println(F("****************************************"));
  Serial.println(F("[TOGGLE_UNLISTED] Toggle ADD or IGNORE unlised loggers to sending stack (set to IGNORE by default)."));
  Serial.println(F("[TOGGLE_ACK] ENABLE or DISABLE filtering and acknowledgement funtion (set to ENABLED by default)."));
  Serial.println(F("[CHANGE_PASSWORD] Set MCU password."));
  Serial.println(F("[CSQ] Reads CSQ."));
  Serial.println(F("[GSM_RECEIVE_TEST] Test GSM receive function."));
  Serial.println(F("[SEND_RAIN_VIA_LORA]"));
  Serial.println(F("[LORA_WAIT_TEST] Waits for LoRa data and sends it thru GSM."));
  Serial.println(F("[LORA_SEND_TEST] Sends one (1) instance of dummy data thru LoRa."));
  Serial.println(F("[RTC_TEMP] Displays RTC temperature."));
  Serial.println(F("[SD] Prints customDue SD card contents (can only be used safely after reset)."));
  Serial.println(F("[T] Turns on customDue/trigger for 5 sec."));
  Serial.println(F(" "));
  Serial.println(F("****************************************"));
}

void print_rtcInterval() {
  // Serial.println("------------------------------------------------");
  Serial.println("[0] 30-minute interval from 0th minute (0,30)");
  Serial.println("[1] 15-minute interval from 0th minute (0,15,30,45)");
  Serial.println("[2] 10-minute interval from 0th minute (0,10,20,30...)");
  Serial.println("[3] 5-minute interval from 0th minute (0,5,10,15...)");
  Serial.println("[4] 3-minute interval from 0th minute (0,3,6,9,12...)");
  // Serial.println("[5] Alarm for every 5,15,25. . .  minutes interval");
  // Serial.println("------------------------------------------------");
}

void setRainCollectorType() {
  unsigned long startHere = millis();
  int _rainCollector;
  Serial.print("Enter rain collector type: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    _rainCollector = Serial.parseInt();
    Serial.print("Rain Collector type = ");
    Serial.println(_rainCollector);
    delay_millis(50);
    rainCollectorType.write(_rainCollector);
  }
}

uint8_t get_rainGauge_type() {
  int _rainType = rainCollectorType.read();
  return _rainType;
}

void setIMUdataRawCalib() {
  unsigned long startHere = millis();
  int raw_calib;
  Serial.print("Enter battery mode: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    raw_calib = Serial.parseInt();
    Serial.print("Battery mode = ");
    Serial.println(raw_calib);
    delay_millis(50);
    imuRawCalib.write(raw_calib);
  }
}

uint8_t get_calib_param() {
  int param = imuRawCalib.read();
  return param;
}

void setLoggerMode() {
  int version;
  unsigned long startHere = millis();
  Serial.print("Enter datalogger mode: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    version = Serial.parseInt();
    Serial.println(version);
    delay_millis(50);
    loggerMode.write(version);
  }
}

void printLoggerMode() {
  Serial.println("[0] arQ mode: Sending sensor data using GSM");           // arQ like function only
  Serial.println("[1] Gateway mode with Subsurface Sensor and 1 Router");  // arQ + LoRa rx
  Serial.println("[2] Router mode");                                       // TX LoRa
  Serial.println("[3] Gateway mode with 1 Router");
  Serial.println("[4] Gateway mode with 2 Routers");
  Serial.println("[5] Gateway mode with 3 Routers");
  Serial.println("[6] Rain gauge sensor only - GSM");
  Serial.println("[7] GNSS sensor only - GSM");
  Serial.println("[8] GNSS sensor Tx");
}

uint8_t get_logger_mode() {
  int lversion = loggerMode.read();
  return lversion;
}

void get_logger_mode_equivalent() {
  if (get_logger_mode() == 0) {
    Serial.println("arQ mode");
    Serial.print("Logger name:     ");
    Serial.println(get_logger_A_from_flashMem());
  } else if (get_logger_mode() == 1) {
    Serial.println("Gateway mode Subsurface Sensor and 1 Router");
    Serial.print("Gateway sensor name: ");
    Serial.println(get_logger_A_from_flashMem());
    Serial.print("Remote sensor name: ");
    Serial.println(get_logger_B_from_flashMem());
  } else if (get_logger_mode() == 3) {
    Serial.println("Gateway mode with 1 Router");
    Serial.print("Gateway name:    ");
    Serial.println(get_logger_A_from_flashMem());
    Serial.print("Remote Sensor Name A: ");
    Serial.println(get_logger_B_from_flashMem());
  } else if (get_logger_mode() == 4) {
    Serial.println("Gateway mode with 2 Routers");
    Serial.print("Gateway name:    ");
    Serial.println(get_logger_A_from_flashMem());
    Serial.print("Remote Sensor Name A: ");
    Serial.println(get_logger_B_from_flashMem());
    Serial.print("Remote Sensor Name B: ");
    Serial.println(get_logger_C_from_flashMem());
  } else if (get_logger_mode() == 5) {
    Serial.println("Gateway mode with 3 Routers");
    Serial.print("Gateway name:    ");
    Serial.println(get_logger_A_from_flashMem());
    Serial.print("Remote Sensor Name A: ");
    Serial.println(get_logger_B_from_flashMem());
    Serial.print("Remote Sensor Name B: ");
    Serial.println(get_logger_C_from_flashMem());
    Serial.print("Remote Sensor Name C: ");
    Serial.println(get_logger_D_from_flashMem());
  } else if (get_logger_mode() == 6) {
    Serial.println("Rain gauge only (GSM)");
    Serial.print("Logger name:     ");
    Serial.println(get_logger_A_from_flashMem());
  } else if (get_logger_mode() == 7) {
    Serial.println("GNSS sensor only (GSM)");
    Serial.print("Logger name:     ");
    Serial.println(get_logger_A_from_flashMem());
  } else if (get_logger_mode() == 8) {
    Serial.println("GNSS sensor only (Tx)");
    Serial.print("Remote Sensor name:     ");
    Serial.println(get_logger_A_from_flashMem());
  } else {  //mode 2
    Serial.println("Router mode");
    Serial.print("Logger name:     ");
    Serial.println(get_logger_A_from_flashMem());
  }
}

void setGsmPowerMode() {
  int gsm_power;
  unsigned long startHere = millis();
  Serial.print("Enter GSM mode setting: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    Serial.setTimeout(8000);
    gsm_power = Serial.parseInt();
    Serial.println(gsm_power);
    delay_millis(50);
    gsmPower.write(gsm_power);
  }
}

uint8_t get_gsm_power_mode() {
  int gsm_mode = gsmPower.read();
  return gsm_mode;
}

bool allow_unlisted() {
  int ack_toggle = allow_unlisted_flag.read();
  if (ack_toggle == 1) {  // using default value 0 - disabled by default
    return true;
  } else {
    return false;
  }
}

bool logger_ack_filter_enabled() {
  int filter_toggle = ack_filter.read();
  if (filter_toggle == 0) {  // using default value 0 - enabled by default
    // Serial.println("logger ack fiter enabled");
    return true;
  } else {
    // Serial.println("logger ack fiter diabled");
    return false;
  }
}

void setAlarmInterval() {
  unsigned long startHere = millis();
  int alarmSelect;
  Serial.print("Enter alarm settings: ");

  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    alarmSelect = Serial.parseInt();
    Serial.println(alarmSelect);
    delay_millis(50);
    alarmStorage.write(alarmSelect);
  }
}

uint8_t alarmFromFlashMem() {
  int fromAlarmStorage;
  fromAlarmStorage = alarmStorage.read();
  return fromAlarmStorage;
}

void changeSensCommand() {
  unsigned long startHere = millis();
  Serial.print("Insert DUE command: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    String dynaCommand = Serial.readStringUntil('\n');
    Serial.println(dynaCommand);
    dynaCommand.toCharArray(sensCommand.senslopeCommand, 9);
    passCommand.write(sensCommand);  // save to flash memory
  }
}

void inputLoggerNames() {
  unsigned long startHere = millis();
  Serial.setTimeout(20000);
  if (get_logger_mode() == 1) {
    Serial.print("Input name of gateway SENSOR: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);
      Serial.print("Input name of remote SENSOR: ");
      String inputLoggerB = Serial.readStringUntil('\n');
      Serial.println(inputLoggerB);

      inputLoggerA.trim();
      inputLoggerB.trim();

      if (inputLoggerA.length() == 4) {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[5] = '\0';
      } else {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[6] = '\0';
      }
      if (inputLoggerB.length() == 4) {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[5] = '\0';
      } else {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[6] = '\0';
      }
      
      flashLoggerName.write(loggerName);
    }
  } else if (get_logger_mode() == 3) {
    Serial.print("Input name of GATEWAY: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);
      Serial.print("Input name of remote SENSOR: ");
      String inputLoggerB = Serial.readStringUntil('\n');
      Serial.println(inputLoggerB);

      inputLoggerA.trim();
      inputLoggerB.trim();

      if (inputLoggerA.length() == 4) {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[5] = '\0';
      } else {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[6] = '\0';
      }
      if (inputLoggerB.length() == 4) {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[5] = '\0';
      } else {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[6] = '\0';
      }

      // inputLoggerA.toCharArray(loggerName.sensorA, 6);
      // inputLoggerB.toCharArray(loggerName.sensorB, 6);
      flashLoggerName.write(loggerName);
    }
  } else if (get_logger_mode() == 4) {
    Serial.print("Input name of GATEWAY: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);
      Serial.print("Input name of remote SENSOR A: ");
      String inputLoggerB = Serial.readStringUntil('\n');
      Serial.println(inputLoggerB);
      Serial.print("Input name of remote SENSOR B: ");
      String inputLoggerC = Serial.readStringUntil('\n');
      Serial.println(inputLoggerC);

      inputLoggerA.trim();
      inputLoggerB.trim();
      inputLoggerC.trim();
      
      if (inputLoggerA.length() == 4) {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[5] = '\0';
      } else {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[6] = '\0';
      }
      if (inputLoggerB.length() == 4) {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[5] = '\0';
      } else {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[6] = '\0';
      }
      if (inputLoggerC.length() == 4) {
        inputLoggerC.toCharArray(loggerName.sensorC, 6);
        loggerName.sensorC[5] = '\0';
      } else {
        inputLoggerC.toCharArray(loggerName.sensorC, 6);
        loggerName.sensorC[6] = '\0';
      }

      // inputLoggerA.toCharArray(loggerName.sensorA, 6);
      // inputLoggerB.toCharArray(loggerName.sensorB, 6);
      // inputLoggerC.toCharArray(loggerName.sensorC, 6);
      flashLoggerName.write(loggerName);
    }
  } else if (get_logger_mode() == 5) {
    Serial.print("Input name of GATEWAY: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);
      Serial.print("Input name of remote SENSOR A: ");
      String inputLoggerB = Serial.readStringUntil('\n');
      Serial.println(inputLoggerB);
      Serial.print("Input name of remote SENSOR B: ");
      String inputLoggerC = Serial.readStringUntil('\n');
      Serial.println(inputLoggerC);
      Serial.print("Input name of remote SENSOR C: ");
      String inputLoggerD = Serial.readStringUntil('\n');
      Serial.println(inputLoggerD);

      inputLoggerA.trim();
      inputLoggerB.trim();
      inputLoggerC.trim();
      inputLoggerD.trim();

      if (inputLoggerA.length() == 4) {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[5] = '\0';
      } else {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[6] = '\0';
      }
      if (inputLoggerB.length() == 4) {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[5] = '\0';
      } else {
        inputLoggerB.toCharArray(loggerName.sensorB, 6);
        loggerName.sensorB[6] = '\0';
      }
      if (inputLoggerC.length() == 4) {
        inputLoggerC.toCharArray(loggerName.sensorC, 6);
        loggerName.sensorC[5] = '\0';
      } else {
        inputLoggerC.toCharArray(loggerName.sensorC, 6);
        loggerName.sensorC[6] = '\0';
      }
      if (inputLoggerD.length() == 4) {
        inputLoggerD.toCharArray(loggerName.sensorD, 6);
        loggerName.sensorD[5] = '\0';
      } else {
        inputLoggerD.toCharArray(loggerName.sensorD, 6);
        loggerName.sensorD[6] = '\0';
      }

      // inputLoggerA.toCharArray(loggerName.sensorA, 6);
      // inputLoggerB.toCharArray(loggerName.sensorB, 6);
      // inputLoggerC.toCharArray(loggerName.sensorC, 6);
      // inputLoggerD.toCharArray(loggerName.sensorD, 6);
      flashLoggerName.write(loggerName);
    }
  } else if (get_logger_mode() == 13) {
    Serial.print("Input name of GATEWAY: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);
      Serial.print("Input name of remote SENSOR A: ");
      String inputLoggerB = Serial.readStringUntil('\n');
      Serial.println(inputLoggerB);
      Serial.print("Input name of remote SENSOR B: ");
      String inputLoggerC = Serial.readStringUntil('\n');
      Serial.println(inputLoggerC);
      Serial.print("Input name of remote SENSOR C: ");
      String inputLoggerD = Serial.readStringUntil('\n');
      Serial.println(inputLoggerD);

      Serial.print("Input name of remote SENSOR D: ");
      String inputLoggerE = Serial.readStringUntil('\n');
      Serial.println(inputLoggerE);
      Serial.print("Input name of remote SENSOR E: ");
      String inputLoggerF = Serial.readStringUntil('\n');
      Serial.println(inputLoggerF);

      inputLoggerA.toCharArray(loggerName.sensorA, 6);
      inputLoggerB.toCharArray(loggerName.sensorB, 6);
      inputLoggerC.toCharArray(loggerName.sensorC, 6);
      inputLoggerD.toCharArray(loggerName.sensorD, 6);

      inputLoggerE.toCharArray(loggerName.sensorE, 6);
      inputLoggerF.toCharArray(loggerName.sensorF, 6);
      flashLoggerName.write(loggerName);
    }
  } else {
    Serial.print("Input name of SENSOR: ");
    while (!Serial.available()) {
      if (timeOutExit(startHere, DEBUGTIMEOUT)) {
        debug_flag_exit = true;
        break;
      }
    }
    if (Serial.available()) {
      String inputLoggerA = Serial.readStringUntil('\n');
      Serial.println(inputLoggerA);

      inputLoggerA.trim();
      
      if (inputLoggerA.length() == 4) {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[5] = '\0';
      } else {
        inputLoggerA.toCharArray(loggerName.sensorA, 6);
        loggerName.sensorA[6] = '\0';
      }
      // inputLoggerA.toCharArray(loggerName.sensorA, 6);
      flashLoggerName.write(loggerName);
    }
  }
}

void changePassword() {
  unsigned long startHere = millis();
  Serial.print("Enter MCU password: ");
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    String inPass = Serial.readStringUntil('\n');
    // inPass += ",";
    Serial.println(inPass);
    inPass.toCharArray(flashPassword.keyword, 50);
    flashPasswordIn.write(flashPassword);
  }
}

bool isChangeParam() {
  String serial_line;
  unsigned long serStart = millis();
  unsigned long startHere = millis();
  bool timeToExit;
  Serial.println(" ");
  Serial.println("Enter C to change:");

  do {
    timeToExit = timeOutExit(startHere, DEBUGTIMEOUT);
    serial_line = Serial.readStringUntil('\n');
  } while (serial_line == "" && timeToExit == false);
  // } while ((serial_line == "") && ((millis() - serStart) < DEBUGTIMEOUT));
  serial_line.toUpperCase();
  serial_line.replace("\r", "");

  if (timeToExit) {
    Serial.println("Exiting . . .");
    debug_flag_exit = true;
  }

  if (serial_line == "C") {
    return true;
  } else {
    // printMenu();
    Serial.println(" ");
    Serial.println("Change cancelled. Reselect.");
    Serial.println(" ");
    return false;
  }
}

bool timeOutExit(unsigned long _startTimeOut, int _timeOut) {
  if ((millis() - _startTimeOut) > _timeOut) {
    _startTimeOut = millis();
    Serial.println(" ");
    Serial.println("*****************************");
    Serial.print("Timeout reached: ");
    Serial.print(_timeOut / 1000);
    Serial.println(" seconds");
    Serial.println("*****************************");
    Serial.println(" ");
    return true;
  } else {
    return false;
  }
}

void debug_println(const char* debugText) {
  if (Serial.available()){
    Serial.println(debugText);
  }
}

void debug_print(const char* debugText) {
  if (Serial.available()){
    Serial.print(debugText);
  }
}

bool inputIs(const char *inputFromSerial, const char* expectedInput) {
  bool correctInput = false;
  if ((strstr(inputFromSerial,expectedInput)) && (strlen(expectedInput) == strlen (inputFromSerial))) {
    correctInput = true;
  }
  return correctInput;
}
