void interrupt_tap()
{
    detachInterrupt(IMU_INT);
    IMUintFlag = true;
}

void resetTap()
{
    Serial.println("Reset taps");
    taps = 0;
}

char *read_IMU_data()
{
    disable_watchdog();
    on_IMU();

    // for (int i = 0; i < 200; i++)
    //     IMUdataToSend[i] = 0x00;
    memset(IMUdataToSend,'\0',200);
    /* MADTA*ST*accelerometer(x,y,z),magnetometer(x,y,z), gyro(x,y,z), 200901142120*/
    int *data;
    char str[140];
    char volt[10];
    char temp[10];

    data = get_raw_data();
    readTimeStamp();
    snprintf(volt, sizeof volt, "%.2f", readBatteryVoltage(10));
    snprintf(temp, sizeof temp, "%.2f", readTemp());

    strncpy(IMUdataToSend, ">>", 2);
    strncat((IMUdataToSend), (get_logger_A_from_flashMem()), (20));
    strncat(IMUdataToSend, "*0:", 4);

    for (int i = 0; i < 10; i++)
    {
        //if (i < 6)
        sprintf(str, "%04X", (uint16_t)data[i]);
        //else
        strncat(IMUdataToSend, str, String(str).length() + 1);
        if (i == 9) strncat(IMUdataToSend, ";", 2);
        else strncat(IMUdataToSend, ",", 2);
    }
    strncat(IMUdataToSend, ";", 2);

    String str2 = get_data_from_uart();
    strncat(IMUdataToSend, "1:", 3);
    str2.toCharArray(str, str2.length());
    strncat(IMUdataToSend, str, str2.length());
    strncat(IMUdataToSend, ";", 2);

    String(taps).toCharArray(str, String(taps).length() + 1);
    strncat(IMUdataToSend, str, String(taps).length());

    strncat(IMUdataToSend, ",", 2);
    strncat(IMUdataToSend, temp, sizeof(temp));
    strncat(IMUdataToSend, ",", 2);
    strncat(IMUdataToSend, volt, sizeof(volt));
    strncat(IMUdataToSend, "*", 2);
    strncat(IMUdataToSend, Ctimestamp, sizeof(Ctimestamp));
    delay(100);

    Serial.print("Data to Send: ");
    Serial.println(IMUdataToSend);

    off_IMU();
    resetTap();
    enable_watchdog();
    return IMUdataToSend;
}

int *get_raw_data()
{
    int *data = raw_data;
    /* Get a new sensor event */
    sensors_event_t accel, gyro, mag, temp;

    //  /* Get new normalized sensor events */
    lsm6ds.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    //accel data
    data[0] = lsm6ds.rawAccX;
    data[1] = lsm6ds.rawAccY;
    data[2] = lsm6ds.rawAccZ;

    //magnetometer data
    data[3] = lis3mdl.x;
    data[4] = lis3mdl.y;
    data[5] = lis3mdl.z;

    //gyro data
    data[6] = lsm6ds.rawGyroX;
    data[7] = lsm6ds.rawGyroY;
    data[8] = lsm6ds.rawGyroZ;

    //gyro temperature
    data[9] = lsm6ds.rawTemp;

    return data;
}

void init_IMU()
{
    bool lsm6ds_success, lis3mdl_success;

    // hardware I2C mode, can pass in address & alt Wire

    lsm6ds_success = lsm6ds.begin_I2C();
    //lsm6ds_success = lsm6ds.begin_SPI(AC_CS, SCK, MISO, MOSI);
    lis3mdl_success = lis3mdl.begin_I2C();
    //lis3mdl_success = lis3mdl.begin_SPI(MAG_CS, SCK, MISO, MOSI);

    if (!lsm6ds_success)
    {
        //    Serial.println("Failed to find LSM6DS chip");
    }
    if (!lis3mdl_success)
    {
        //    Serial.println("Failed to find LIS3MDL chip");
    }
    if (!(lsm6ds_success && lis3mdl_success))
    {
        // while (1) {
        //   delay(10);
        // }
    }

    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    attachInterrupt(IMU_INT, interrupt_tap, RISING);
}

void print_data()
{
    sensors_event_t accel, gyro, mag, temp;

    //  /* Get new normalized sensor events */
    lsm6ds.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    /* Display the results (acceleration is measured in m/s^2) */
    //Serial.print("\t\tAccel X: ");
    Serial.print(lsm6ds.rawAccX);
    Serial.print(",");
    Serial.print(lsm6ds.rawAccY);
    Serial.print(",");
    Serial.print(lsm6ds.rawAccZ);
    Serial.print(",");
    //Serial.println(" \tm/s^2 ");

    /* Display the results (magnetic field is measured in uTesla) */
    //Serial.print(" \t\tMag   X: ");
    Serial.print(lis3mdl.x);
    Serial.print(",");
    Serial.print(lis3mdl.y);
    Serial.print(",");
    Serial.print(lis3mdl.z);
    Serial.print(",");
    //Serial.println(" \tuTesla ");

    /* Display the results (rotation is measured in rad/s) */
    //Serial.print("\t\tGyro  X: ");
    Serial.print(lsm6ds.rawGyroX);
    Serial.print(",");
    Serial.print(lsm6ds.rawGyroY);
    Serial.print(",");
    Serial.print(lsm6ds.rawGyroZ);
    Serial.print(",");
    //Serial.println(" \tradians/s ");

    //Serial.print("\t\tTemp   :\t\t\t\t\t");
    Serial.print(lsm6ds.rawTemp);
    //Serial.println(" \tdeg C");
    Serial.println();
}

String get_data_from_uart()
{
    char str[100] = "";
    char read_char;
    String str2;

    delay(100);

    Serial1.write('r');
    //Serial.println("write r");

    delay(100);
    if (Serial1.available())
    {
        delay(100); //allows all serial sent to be received together

        //Serial1.readBytesUntil('\n',str,64);
        str2 = Serial1.readString();
        //Serial1.readBytes(str,64);
        //digitalWrite(6, LOW);
        delay(10);
        //Serial.println(str2.length());
        Serial.println(str2);
    }
    else
        Serial.println("not available");
    Serial1.write('x');
    //Serial.println("write x");
    //Serial.println("end");
    return str2;
}

void on_IMU()
{
    // Current consumption of bare IMU when sampling
    //  * 3.3V ~ 2.8mA
    Serial.println("Turning ON IMU sensor");
    digitalWrite(IMU_POWER, HIGH);
    digitalWrite(DUETRIG, HIGH);
    // digitalWrite(LED_BUILTIN, HIGH);
    flashLed(LED_BUILTIN, 2, 50);
    delay_millis(1000);
    init_IMU();
    delay_millis(1000);
}

void off_IMU()
{
    Serial.println("Turning OFF IMU sensor");
    delay_millis(200);
    digitalWrite(IMU_POWER, LOW);
    digitalWrite(DUETRIG, LOW);
    // digitalWrite(LED_BUILTIN, LOW);
    flashLed(LED_BUILTIN, 2, 50);
}