void setupTime() {
  unsigned long startHere = millis();
  int MM = 0, DD = 0, YY = 0, hh = 0, mm = 0, ss = 0, dd = 0;
  Serial.println(F("\nSet time and date in this format: YYYY,MM,DD,hh,mm,ss,dd[0-6]Mon-Sun"));
  // delay(10);
  while (!Serial.available()) {
    if (timeOutExit(startHere, DEBUGTIMEOUT)) {
      debug_flag_exit = true;
      break;
    }
  }
  if (Serial.available()) {
    YY = Serial.parseInt();
    MM = Serial.parseInt();
    DD = Serial.parseInt();
    hh = Serial.parseInt();
    mm = Serial.parseInt();
    ss = Serial.parseInt();
    dd = Serial.parseInt();
    delay_millis(10);
    adjustDate(YY, MM, DD, hh, mm, ss, dd);
    readTimeStamp();
    Serial.print("Current timestamp: ");
    Serial.println(Ctimestamp);
  }
}

void adjustDate(int year, int month, int date, int hour, int min, int sec, int weekday) {
  DateTime dt(year, month, date, hour, min, sec, weekday);
  rtc.setDateTime(dt);  // adjust date-time as defined by 'dt'
                        // Serial.println(rtc.now().getEpoch());	//debug info
                        //char weekDay[][4] = {"Sun"-0, "Mon"-1, "Tue"-2, "Wed"-3, "Thu"-4, "Fri"-5, "Sat"-6 };
}

float readTemp() {
  float temp;
  rtc.convertTemperature();
  temp = rtc.getTemperature();
  return temp;
}

char new_temp[15];
char *readTempRTC() {
  char tmp[10];
  rtc.convertTemperature();
  float temp = rtc.getTemperature();
  snprintf(tmp, sizeof tmp, "%.2f", temp);
  strncpy(new_temp, tmp, sizeof(tmp));
  return new_temp;
}

void readTimeStamp() {
  // add function for generating easy to read date time 
  DateTime now = rtc.now();  //get the current date-time
  String ts = String(now.year());

  if (now.month() <= 9) {
    ts += "0" + String(now.month());
  } else {
    ts += String(now.month());
  }

  if (now.date() <= 9) {
    ts += "0" + String(now.date());
  } else {
    ts += String(now.date());
  }

  if (now.hour() <= 9) {
    ts += "0" + String(now.hour());
  } else {
    ts += String(now.hour());
  }

  if (now.minute() <= 9) {
    ts += "0" + String(now.minute());
  } else {
    ts += String(now.minute());
  }

  if (now.second() <= 9) {
    ts += "0" + String(now.second());
  } else {
    ts += String(now.second());
  }

  ts.remove(0, 2);  //remove 1st 2 data in ts
  ts.toCharArray(Ctimestamp, 13);
}

// sets next alarm
void setNextAlarm(int intervalSET) {
  DateTime now = rtc.now();  //get the current date-time
  store_rtc = nextAlarm((int)(now.minute()),intervalSET);
  // enable rtc interrupt
  rtc.enableInterrupts(store_rtc, 00);  // interrupt at (minutes, seconds)
  if (DEBUG == 1) {
    Serial.print("Next alarm - hh:");
    Serial.println(store_rtc);  
  }
  readTimeStamp();
}

uint16_t nextAlarm(int currentMinute, int intervalSaved) {
  int intervalEquivalent = 0;
  uint16_t next_alarm = 0;
  if (intervalSaved == 1) {
    intervalEquivalent = 15;
  } else if (intervalSaved == 2) {
    intervalEquivalent = 10;
  } else if (intervalSaved == 3) {
    intervalEquivalent = 5;
  } else if (intervalSaved == 4) {
    intervalEquivalent = 3;
  } else {
    intervalEquivalent = 30;
  }
  //
  int computed_alarm = (((currentMinute/intervalEquivalent)+1)*intervalEquivalent);
  if (computed_alarm >= 60) {
    next_alarm = 0;
  } else {
    next_alarm = computed_alarm;
  }
  return next_alarm;
}

int dueSamplingTime() {
  DateTime now = rtc.now();
  if ((now.minute() >= 0) && (now.minute() <= 10)) {
    return 1;
  } else if ((now.minute() >= 30) && (now.minute() <= 40)) {
    return 1;
  } else {
    return 0;
  }
  enable_rtc_interrupt();
}

void enable_rtc_interrupt()
{
  rtc.enableInterrupts(store_rtc, 00); // interrupt at (minutes, seconds)
  if (DEBUG == 1) {
    Serial.print("Next alarm: ");
  }
  if (DEBUG == 1) {
    Serial.println(store_rtc);
  }
  readTimeStamp();
}
