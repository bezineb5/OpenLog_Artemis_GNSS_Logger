void SerialPrintTimeString()
{
  char timeString[40];
  getTimeString(timeString);
  Serial.print(timeString);
}

//Returns the value of an NMEA digit character, or -1 if not a digit
static int nmeaDigit(char c)
{
  return (c >= '0' && c <= '9') ? (c - '0') : -1;
}

//Returns a pointer to the n-th comma-separated field of an NMEA sentence
//(field 0 is the address field, e.g. "GNRMC"), or NULL if there is no such field
static const char *nmeaField(const char *sentence, uint8_t index)
{
  while (index > 0)
  {
    sentence = strchr(sentence, ',');
    if (sentence == NULL)
      return NULL;
    sentence++; //Skip the comma
    index--;
  }
  return sentence;
}

//Parse an NMEA time field "hhmmss.ss" - returns false if malformed or out of range
static bool parseNmeaTime(const char *field, uint8_t &hour, uint8_t &minute, uint8_t &second, uint8_t &centis)
{
  if (strlen(field) < 6)
    return false;
  int h1 = nmeaDigit(field[0]), h2 = nmeaDigit(field[1]);
  int m1 = nmeaDigit(field[2]), m2 = nmeaDigit(field[3]);
  int s1 = nmeaDigit(field[4]), s2 = nmeaDigit(field[5]);
  if (h1 < 0 || h2 < 0 || m1 < 0 || m2 < 0 || s1 < 0 || s2 < 0)
    return false;
  hour = (uint8_t)(h1 * 10 + h2);
  minute = (uint8_t)(m1 * 10 + m2);
  second = (uint8_t)(s1 * 10 + s2);
  centis = 0;
  const char *dot = strchr(field, '.');
  if (dot != NULL)
  {
    int c1 = nmeaDigit(dot[1]);
    int c2 = nmeaDigit(dot[2]); //May be -1 if only one fractional digit
    if (c1 >= 0)
      centis = (uint8_t)(c1 * 10 + (c2 >= 0 ? c2 : 0));
  }
  return (hour <= 23) && (minute <= 59) && (second <= 59);
}

//Parse an NMEA date field "ddmmyy" (RMC) - returns false if malformed or out of range
static bool parseNmeaDateRmc(const char *field, uint8_t &day, uint8_t &month, uint16_t &year)
{
  if (strlen(field) < 6)
    return false;
  int d1 = nmeaDigit(field[0]), d2 = nmeaDigit(field[1]);
  int m1 = nmeaDigit(field[2]), m2 = nmeaDigit(field[3]);
  int y1 = nmeaDigit(field[4]), y2 = nmeaDigit(field[5]);
  if (d1 < 0 || d2 < 0 || m1 < 0 || m2 < 0 || y1 < 0 || y2 < 0)
    return false;
  day = (uint8_t)(d1 * 10 + d2);
  month = (uint8_t)(m1 * 10 + m2);
  year = (uint16_t)(2000 + y1 * 10 + y2);
  return (day >= 1 && day <= 31) && (month >= 1 && month <= 12);
}

//Sync the RTC from a checksum-validated NMEA sentence carrying time and date:
//  $xxRMC,hhmmss.ss,A,...,ddmmyy,...  (only when status is 'A' - valid fix)
//  $xxZDA,hhmmss.ss,dd,mm,yyyy,...   (no validity flag, ranges checked instead)
static void syncRtcFromNmeaSentence(const char *sentence)
{
  if (rtcNeedsSync != true) //Only sync once per boot / wake, same as callbackNAVPVT
    return;

  //sentence starts after '$': [0..1] = talker id, [2..4] = sentence type
  if (strncmp(sentence + 2, "RMC", 3) == 0)
  {
    const char *timeField = nmeaField(sentence, 1);
    const char *statusField = nmeaField(sentence, 2);
    const char *dateField = nmeaField(sentence, 9);
    if (timeField == NULL || statusField == NULL || dateField == NULL)
      return;
    if (statusField[0] != 'A') //No valid fix - do not sync the RTC
      return;

    uint8_t hour, minute, second, centis, day, month;
    uint16_t year;
    if (parseNmeaTime(timeField, hour, minute, second, centis) == false)
      return;
    if (parseNmeaDateRmc(dateField, day, month, year) == false)
      return;

    myRTC.setTime(centis, second, minute, hour, day, month, (year - 2000)); //Set the RTC
    rtcHasBeenSyncd = true;
    rtcNeedsSync = false;
    if (settings.printMinorDebugMessages == true)
    {
      Serial.printf("processNMEA: RTC sync'd from RMC to %04d/%02d/%02d %02d:%02d:%02d.%02d\r\n",
                    year, month, day, hour, minute, second, centis);
    }
  }
  else if (strncmp(sentence + 2, "ZDA", 3) == 0)
  {
    const char *timeField = nmeaField(sentence, 1);
    const char *dayField = nmeaField(sentence, 2);
    const char *monthField = nmeaField(sentence, 3);
    const char *yearField = nmeaField(sentence, 4);
    if (timeField == NULL || dayField == NULL || monthField == NULL || yearField == NULL)
      return;

    uint8_t hour, minute, second, centis, day, month;
    uint16_t year;
    if (parseNmeaTime(timeField, hour, minute, second, centis) == false)
      return;
    int d1 = nmeaDigit(dayField[0]), d2 = nmeaDigit(dayField[1]);
    int m1 = nmeaDigit(monthField[0]), m2 = nmeaDigit(monthField[1]);
    if (d1 < 0 || d2 < 0 || m1 < 0 || m2 < 0)
      return;
    day = (uint8_t)(d1 * 10 + d2);
    month = (uint8_t)(m1 * 10 + m2);
    if (strlen(yearField) < 4)
      return;
    int y1 = nmeaDigit(yearField[0]), y2 = nmeaDigit(yearField[1]);
    int y3 = nmeaDigit(yearField[2]), y4 = nmeaDigit(yearField[3]);
    if (y1 < 0 || y2 < 0 || y3 < 0 || y4 < 0)
      return;
    year = (uint16_t)(y1 * 1000 + y2 * 100 + y3 * 10 + y4);
    if (day < 1 || day > 31 || month < 1 || month > 12 || year < 2020 || year > 2099)
      return;

    myRTC.setTime(centis, second, minute, hour, day, month, (year - 2000)); //Set the RTC
    rtcHasBeenSyncd = true;
    rtcNeedsSync = false;
    if (settings.printMinorDebugMessages == true)
    {
      Serial.printf("processNMEA: RTC sync'd from ZDA to %04d/%02d/%02d %02d:%02d:%02d.%02d\r\n",
                    year, month, day, hour, minute, second, centis);
    }
  }
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  if (settings.enableTerminalOutput == true)
    Serial.write(incoming);

  //Assemble each NMEA sentence and verify its checksum. Valid RMC/ZDA
  //sentences are used to sync the RTC (see syncRtcFromNmeaSentence), so file
  //timestamps work even when UBX-NAV-PVT is not enabled. Bad checksums are
  //counted (healthNmeaBadChecksums) - if the SD log ever contains a corrupted
  //sentence while this counter stays zero, the corruption happened inside
  //the logger, not upstream of it.
  static char sentence[96]; //NMEA sentences are <= 82 chars including $ and *hh
  static uint8_t len = 0;
  static uint8_t checksum = 0;
  static bool inChecksum = false;
  static uint8_t checksumNibbles = 0;
  static uint8_t receivedChecksum = 0;

  if (incoming == '$')
  {
    len = 0;
    checksum = 0;
    inChecksum = false;
    checksumNibbles = 0;
    return;
  }

  if (incoming == '*')
  {
    inChecksum = true;
    sentence[len] = 0;
    return;
  }

  if (inChecksum == true)
  {
    if (incoming == '\r' || incoming == '\n')
      return; //End of sentence
    int v;
    if (incoming >= '0' && incoming <= '9') v = incoming - '0';
    else if (incoming >= 'A' && incoming <= 'F') v = incoming - 'A' + 10;
    else if (incoming >= 'a' && incoming <= 'f') v = incoming - 'a' + 10;
    else { len = 0; inChecksum = false; checksumNibbles = 0; return; } //Malformed
    if (checksumNibbles == 0)
    {
      receivedChecksum = (uint8_t)(v << 4);
      checksumNibbles = 1;
    }
    else
    {
      receivedChecksum |= (uint8_t)v;
      if (receivedChecksum == checksum)
        syncRtcFromNmeaSentence(sentence);
      else
        healthNmeaBadChecksums++;
      len = 0;
      inChecksum = false;
      checksumNibbles = 0;
    }
    return;
  }

  if (len < sizeof(sentence) - 1)
  {
    checksum ^= (uint8_t)incoming;
    sentence[len++] = incoming;
  }
  else
  {
    len = 0; //Overlong sentence - drop it
  }
}

void callbackNAVPOSECEF(UBX_NAV_POSECEF_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-POSECEF"));
  }
}

void callbackNAVSTATUS(UBX_NAV_STATUS_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-STATUS"));
  }
}

void callbackNAVDOP(UBX_NAV_DOP_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-DOP"));
  }
}

void callbackNAVATT(UBX_NAV_ATT_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-ATT"));
  }
}

void callbackNAVPVT(UBX_NAV_PVT_data_t ubxDataStruct)
{
  //Check if we need to update the RTC
  if (rtcNeedsSync == true) //Do we need to sync the RTC
  {
    if (ubxDataStruct.flags2.bits.confirmedDate && ubxDataStruct.flags2.bits.confirmedTime) //Are the date and time valid?
    {
      //If nanos is negative, set it to zero
      //The ZED-F9P Integration Manual says: "the nano value can range from -5000000 (i.e. -5 ms) to +994999999 (i.e. nearly 995 ms)."
      //"if a resolution of one hundredth of a second is adequate, negative nano values can simply be rounded up to 0 and effectively ignored."
      uint8_t centis;
      if (ubxDataStruct.nano < 0)
        centis = 0;
      else
        centis = (uint8_t)(ubxDataStruct.nano / 10000000); //Convert nanos to hundredths (centiseconds)
      myRTC.setTime(centis, ubxDataStruct.sec, ubxDataStruct.min, ubxDataStruct.hour, ubxDataStruct.day, ubxDataStruct.month, (ubxDataStruct.year - 2000)); //Set the RTC
      rtcHasBeenSyncd = true; //Set rtcHasBeenSyncd to show RTC has been sync'd
      rtcNeedsSync = false; //Clear rtcNeedsSync so we don't set the RTC multiple times
      if (settings.printMinorDebugMessages == true)
      {
        Serial.printf("callbackNAVPVT: RTC sync'd to %04d/%02d/%02d %02d:%02d:%02d.%02d\r\n", ubxDataStruct.year, ubxDataStruct.month, ubxDataStruct.day, ubxDataStruct.hour, ubxDataStruct.min, ubxDataStruct.sec, centis);
      }
    }
  }

  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();
  
    //Print the frame information
    Serial.println(F(" NAV-PVT"));
  }
}

void callbackNAVODO(UBX_NAV_ODO_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-ODO"));
  }
}

void callbackNAVVELECEF(UBX_NAV_VELECEF_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-VELECEF"));
  }
}

void callbackNAVVELNED(UBX_NAV_VELNED_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-VELNED"));
  }
}

void callbackNAVHPPOSECEF(UBX_NAV_HPPOSECEF_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-HPPOSECEF"));
  }
}

void callbackNAVHPPOSLLH(UBX_NAV_HPPOSLLH_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-HPPOSLLH"));
  }
}

void callbackNAVCLOCK(UBX_NAV_CLOCK_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-CLOCK"));
  }
}

void callbackNAVRELPOSNED(UBX_NAV_RELPOSNED_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" NAV-RELPOSNED"));
  }
}

void callbackRXMSFRBX(UBX_RXM_SFRBX_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" RXM-SFRBX"));
  }
}

void callbackRXMRAWX(UBX_RXM_RAWX_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" RXM-RAWX"));
  }
}

void callbackTIMTM2(UBX_TIM_TM2_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" TIM-TM2"));
  }
}

void callbackESFALG(UBX_ESF_ALG_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" ESF-ALG"));
  }
}

void callbackESFINS(UBX_ESF_INS_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" ESF-INS"));
  }
}

void callbackESFMEAS(UBX_ESF_MEAS_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" ESF-MEAS"));
  }
}

void callbackESFRAW(UBX_ESF_RAW_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" ESF-RAW"));
  }
}

void callbackESFSTATUS(UBX_ESF_STATUS_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" ESF-STATUS"));
  }
}

void callbackHNRPVT(UBX_HNR_PVT_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" HNR-PVT"));
  }
}

void callbackHNRATT(UBX_HNR_ATT_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" HNR-ATT"));
  }
}

void callbackHNRINS(UBX_HNR_INS_data_t ubxDataStruct)
{
  if (settings.enableTerminalOutput == true)
  {
    //Print some useful information. Let's keep this message short!
    SerialPrintTimeString();

    //Print the frame information
    Serial.println(F(" HNR-INS"));
  }
}
