/* (
   )\ (    (           )
 (((_))\  ))\  (    ( /( (
 )\__((_)/((_) )\ ) )(_)))\
((/ __(_|_))( _(_/(((_)_((_)
 | (__| | || | ' \)) _` (_-<
  \___|_|\_,_|_||_|\__,_/__/ 2015
*/

#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);
LiquidCrystal lcd(13, 12, 11, 10, 9, 7);
void LCD(float, char);
void gpsDisplay();
void decimalCalculator();


const int SIZE = 70;
const int XRF = 8;
char GPS[SIZE] = {'$', 'G', 'P', 'R', 'M', 'C', ',', '1', '4', '2', '3', '0', '4', '.',
                  '0', '0', '0', ',', 'A', ',', '0', '0', '0', '0', '.', '0', '0', '0', '0',
                  ',', 'N', ',', '0', '0', '0', '0', '0', '.', '0', '0', '0', '0', ',', 'W',
                  ',', '0', '.', '1', '5', ',', '2', '9', '2', '.', '2', '3', ',', '1', '5',
                  '0', '4', '1', '5', ',', ',', ',', 'A', '*', '7', '6'};
char dis0, dis1, dis2, dis3, dis4, dis5, dis6;
char c1, c2, c3, c4;
char lat1, lon1;
char lat2, lon2;
char lat3, lon3;
char lat4, lon4;
char lat5, lon5;
char lat6, lon6;
char lat7, lon7;
char lat8, lon8;
char lon9;
boolean flag = true;
float distance = 0;
int i;

void setup() {
  Serial.begin(4800);
  mySerial.begin(4800);
  pinMode(XRF, OUTPUT);
  digitalWrite(XRF, HIGH);
  lcd.begin(16, 2);                          //Set the row and colume
  lcd.print("Starting Boat");
}

void loop() {
  if (Serial.available() > 0)
  {
    char charReceived = Serial.read();
    char transmitData = charReceived;
    mySerial.print (transmitData);
  }

  decimalCalculator();

  if (flag == true) {
    while (mySerial.available())
    {
      char c = mySerial.read();
      if (c == '@')
      {

        c1 = mySerial.read();
        c2 = mySerial.read();
        if ( c2 == '#' )
        {
          dis0 = mySerial.read();
          dis1 = mySerial.read();
          dis2 = mySerial.read();
          dis3 = mySerial.read();
          dis4 = mySerial.read();
          dis6 = mySerial.read();
          LCD(distance, c1);
          flag = false;

        }
      }
    }
  }
  else {
    while (mySerial.available())
    {
      char c = mySerial.read();
      if (c == '^')
      {
        lat1 = mySerial.read();
        lat2 = mySerial.read();
        lat3 = mySerial.read();
        lat4 = mySerial.read();
        lat5 = mySerial.read();
        lat6 = mySerial.read();
        lat7 = mySerial.read();
        lat8 = mySerial.read();
        lon1 = mySerial.read();
        lon2 = mySerial.read();
        lon3 = mySerial.read();
        lon4 = mySerial.read();
        lon5 = mySerial.read();
        lon6 = mySerial.read();
        lon7 = mySerial.read();
        lon8 = mySerial.read();
        lon9 = mySerial.read();
        GPS[20] = lat1;
        GPS[21] = lat2;
        GPS[22] = lat3;
        GPS[23] = lat4;
        GPS[25] = lat5;
        GPS[26] = lat6;
        GPS[27] = lat7;
        GPS[28] = lat8;
        GPS[32] = lon1;
        GPS[33] = lon2;
        GPS[34] = lon3;
        GPS[35] = lon4;
        GPS[36] = lon5;
        GPS[38] = lon6;
        GPS[39] = lon7;
        GPS[40] = lon8;
        GPS[41] = lon9;
        flag = true;
      }
    }

  }
  gpsDisplay();
  delay(500);
}

// Prints GPS Data to the serial.
void gpsDisplay()
{
  for (i = 0; i < SIZE; i++)
  {
    Serial.print(GPS[i]);
  }
  Serial.println();
}

// Displays data to the LCD
void LCD( float distance, char c1)
{
  char* manoeuvre[] = {"Plotting Course", "Tack Right", "Tack Left", "Downwind Run",
                       "Jibe Right", "Jibe Left", "Reach Right", "Reach Left", "Turn Right",
                       "Turn Left", "Straight"};
  int i;
  
  if (c1 == 'A')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[0]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'B')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[1]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");
  }
  else if (c1 == 'C')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[2]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'D')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[3]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'E')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[4]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'F')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[5]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");
  }
  else if (c1 == 'G')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[6]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'H')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[7]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'I')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[8]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'J')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[9]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }
  else if (c1 == 'K')
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(manoeuvre[10]);
    lcd.setCursor(0, 1);
    lcd.print(distance, 1);
    lcd.print("m 2 Target");

  }

}

// Calculate where the decimal point is for the distane and work actual distance
void decimalCalculator()
{
  distance = 0;

  if (dis1 == '.')
  {
    distance = distance + ((dis0 - '0'));
    distance = distance + ((float)(dis2 - '0') / 10);
  }
  else if (dis2 == '.')
  {
    distance = distance + ((dis0 - '0') * 10);
    distance = distance + ((dis1 - '0'));
    distance = distance + ((float)(dis3 - '0') / 10);
  }
  else if (dis3 == '.')
  {
    distance = distance + ((dis0 - '0') * 100);
    distance = distance + ((dis1 - '0') * 10);
    distance = distance + ((dis2 - '0'));
    distance = distance + ((float)(dis4 - '0') / 10);
  }
  else if (dis4 == '.')
  {
    distance = distance + ((dis0 - '0') * 1000);
    distance = distance + ((dis1 - '0') * 100);
    distance = distance + ((dis2 - '0') * 10);
    distance = distance + ((dis3 - '0'));
    distance = distance + ((float)(dis5 - '0') / 10);
  }
  else if (dis5 == '.')
  {
    distance = distance + ((dis0 - '0') * 10000);
    distance = distance + ((dis1 - '0') * 1000);
    distance = distance + ((dis2 - '0') * 100);
    distance = distance + ((dis3 - '0') * 10);
    distance = distance + ((dis4 - '0'));
    distance = distance + ((float)(dis6 - '0') / 10);
  }
}

