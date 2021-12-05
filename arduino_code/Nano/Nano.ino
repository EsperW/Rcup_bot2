#include <Wire.h>
#include <TimeLib.h>
#include <stdio.h>
#include <string.h>
int i = 0;
int bytes_ = 0;
int value1 = 0;
int value2 = 0;
int value3 = 0;
int value4 = 0;
int value5 = 0;
float id1 = 0.0;
float id2 = 0.0;
float id3 = 0.0;
float id4 = 0.0;
float id5 = 0.0;
const byte numChars = 254;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;
void setup()
{
  Serial.begin(4800);
  Serial.setTimeout(0);
  pinMode(13, OUTPUT);
  Wire.begin(8);
  while(Serial.available() == 0)
  {
  }
  Wire.onRequest(requestEvent);
}

void loop() {
  get_data();
}

void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() != 0 && newData == false)
  {
    rc = Serial.read();
    //marker(2);
    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void parseData()
{
  char * strtokIndx;
  strtokIndx = strtok(tempChars, ",");
  strcpy(messageFromPC, strtokIndx);
  strtokIndx = strtok(NULL, ",");
  id1 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  id2 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  id3 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  id4 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  id5 = atof(strtokIndx);
  value1 = id1;
  value2 = id2;
  value3 = id3;
  value4 = id4;
  value5 = id5;
}
void get_data()
{
  recvWithStartEndMarkers();
  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
    if (id1 == 0 && id2 == 0 && id3 == 0 && id4 == 0)
    {
      digitalWrite(13, LOW);
    }
    else
    {
      digitalWrite(13, HIGH);
    }
  }
}
void requestEvent()
{
  Wire.write (highByte (value1));
  Wire.write (lowByte (value1));

  Wire.write (highByte (value2));
  Wire.write (lowByte (value2));

  Wire.write (highByte (value3));
  Wire.write (lowByte (value3));

  Wire.write (highByte (value4));
  Wire.write (lowByte (value4));
  
  Wire.write (highByte (value5));
  Wire.write (lowByte (value5));
}
