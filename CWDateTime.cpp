//**********************************************************************************************
// CWDateTime.cpp
//
//
//**********************************************************************************************
// ***** TO SET TIMEZONE See below " myTZ.setLocation(F("America/New York")); "  **************
//**********************************************************************************************
//
//
//
//**********************************************************************************************
#include "CWDateTime.h"
//**********************************************************************************************
void CWDateTime::begin()
{
  myTZ.setCache(0);
  myTZ.setLocation(F("America/New York")); 
  myTZ.dateTime();
  waitForSync();
}
//**********************************************************************************************
void CWDateTime::setTimezone(const char *timeZone)
{
  myTZ.setCache(0);
  myTZ.setLocation(timeZone);
  waitForSync();
}
//**********************************************************************************************
String CWDateTime::getTimezone()
{
  return myTZ.getTimezoneName(0);
}
//**********************************************************************************************
void CWDateTime::update()
{
  
}
//**********************************************************************************************
String CWDateTime::getFormattedTime()
{
  return myTZ.dateTime();
}
//**********************************************************************************************
char *CWDateTime::getHour(const char *format)
{
  static char buffer[3] = {'\0'};
  snprintf(buffer, sizeof(buffer), format, myTZ.dateTime("H"));
  return buffer;
}
//**********************************************************************************************
char *CWDateTime::getMinute(const char *format)
{
  static char buffer[3] = {'\0'};
  strncpy(buffer, myTZ.dateTime("i").c_str(), sizeof(buffer));
  return buffer;
}
//**********************************************************************************************
char *CWDateTime::getSecond(const char *format)
{
  static char buffer[3] = {'\0'};
  strncpy(buffer, myTZ.dateTime("s").c_str(), sizeof(buffer));
  return buffer;
}
//**********************************************************************************************
int CWDateTime::getHour()
{
  return myTZ.dateTime("h").toInt();
}
//**********************************************************************************************
int CWDateTime::getMinute()
{
  return myTZ.dateTime("i").toInt();
}
//**********************************************************************************************
int CWDateTime::getSecond()
{
  return myTZ.dateTime("s").toInt();
}
//**********************************************************************************************
