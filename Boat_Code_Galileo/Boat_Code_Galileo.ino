/*                                  
   )\  (     (             )      
 (((_) )\   ))\   (     ( /(  (   
 )\___((_) /((_)  )\ )  )(_)) )\  
((/ __|(_)(_))(  _(_/( ((_)_ ((_) 
 | (__ | || || || ' \))/ _` |(_-< 
  \___||_| \_,_||_||_| \__,_|/__/ 2015
 */

#include <TimerOne.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

// Functions
LSM303 compass;
void navigator();
void setupServos();
void setupCompass();
void readGPS();
double distanceToWaypoint();
double courseAngle();
double convertDegMinToDecDeg(float);
double radianConverter(float);
double degreeConverter(float);
int windDirection();
void driveServos(int , int, int);
int calcBearingError(float, float);
void tackLeft();
void tackRight();
float correctedAngle(float);
float  compassReturn();
void wayPoint();
void serialTransmitt();

// VARIABLES 
int counter = 0;
float angle;
int state;
unsigned long time1 = 0;
unsigned long lastTime = 0;
unsigned long times, stay;               // Station Keeping Contest
int update = 0;
boolean beating = false;  		 // go: true means boat is in autonomous mode, i.e. after starting pistol
boolean straight = false;
boolean autonomous = false;

float decLatatude = 0; 
float decLongatude = 0;                  // Current GPS decimal
float latatude1 = 0; 
float longatude1 = 0;                    // Current GPS Coordinates
float latatude2;
float longatude2;                        // Current GPS Target Coordinates
float homeLat = 0;                       
float homeLon = 0;                       //  Home coordinates
float wp_1_Lat =  53.266813;                        
float wp_1_Lon =   -9.046844;             // Current GPS Target Coordinates

boolean wp_1_Reached = false;
boolean wp_2_Reached = false;
boolean wp_3_Reached = false;
boolean flag1 = 0;
boolean flag2 = 0;
boolean flag3 = 0; // Station Kept
double dist = 0;
int i;
char lat1, lon1;
char lat2, lon2;
char lat3, lon3;
char lat4, lon4;
char lat5, lon5;
char lat6, lon6;
char lat7, lon7;
char lat8, lon8;
char lat9, lon9;
char lat10, lon10;
int turningMode = 0;
// COMPASS VARIABLE
float bearing = 0; 

// SERVO VARIABLES 
Servo mainSailServo;  // create servo objects to control: the main sail servo
Servo jibSailServo;  //  jib sail servo
Servo rudderServo;  //   rudder servo
int mainSailAngle, jibSailAngle, rudderAngle;


// Transeriver Variables

void setup() {
  Serial.begin(4800);  // Begin Serial at 4800 BAUD
  Serial1.begin(4800);
  mySerial.begin(4800);                                 // Begin mySerial at 4800 BAUD
  setupCompass();	                                // Compass Setup
  setupServos();	                                // Servo Setup
  Timer1.initialize(1000000); // set a timer of length 1000000 microseconds (or 1 sec)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
}


// Main loop
void loop(){
  
  if (Serial1.available() > 0)
  {
    char sentData = Serial1.read();
    char dataRecieved = sentData;
    if (dataRecieved == 's'|| dataRecieved == 'S')
    {
      autonomous = !autonomous ;
    }
    else if(dataRecieved == 'a'|| dataRecieved == 'A')
    {
      mainSailAngle = 90;
      jibSailAngle = 90;
      rudderAngle = 60;
      driveServos(mainSailAngle, jibSailAngle, rudderAngle);
    }
    else if(dataRecieved == 'd'|| dataRecieved == 'D')
    {
      mainSailAngle = 90;
      jibSailAngle = 90;
      rudderAngle = 120;
      driveServos(mainSailAngle, jibSailAngle, rudderAngle);
    }
    else if(dataRecieved == 'w'|| dataRecieved == 'W')
    {
      mainSailAngle = 90;
      jibSailAngle = 90;
      rudderAngle = 90;
      driveServos(mainSailAngle, jibSailAngle, rudderAngle);
    }  
  }
}

// SETCOURSE - Compare wind direction, course and heading to choose sailing mode.
void navigator()
{
  int windDirectionRealHeading = 0;
  double ruturnedCourseAngle = 0;
  readGPS();
  ruturnedCourseAngle = courseAngle();
  windDirectionRealHeading = windDirection();
  bearing = compassReturn();
  if (abs(calcBearingError(ruturnedCourseAngle, bearing)) > 10 )  // if Bearing Error is bigger the 10 degrees turn boat
  {
    if (calcBearingError(ruturnedCourseAngle, bearing) > 0)   // if error is positive turn right
    {
      turningMode = 8;
      rudderAngle = 120;
      straight = false;
    }
    else if (calcBearingError(courseAngle(), bearing) < 0)        // if error is negative turn left
    {
      turningMode = 9;
      rudderAngle = 60;
      straight = false;
    }
  }
  else
  {
    turningMode = 10;
    straight = true;
    rudderAngle = 90;
  }

  if (windDirectionRealHeading >= 270 && windDirectionRealHeading < 315)     //4th quadrant: Beam Reach to Close Hauled
  {
    turningMode = 6;
    mainSailAngle = 90 + (360 - windDirectionRealHeading) - 45;
    jibSailAngle = mainSailAngle;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }
  else if (windDirectionRealHeading > 45 && windDirectionRealHeading <= 90)     //1st quadrant: Beam Reach to Close Hauled
  {
    turningMode = 7;
    mainSailAngle = 90 - windDirectionRealHeading + 45;
    jibSailAngle = mainSailAngle;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }
  else if (windDirectionRealHeading > 90 && windDirectionRealHeading < 150)     //2nd quadrant: Beam Reach to Running
  {
    turningMode = 5;
    mainSailAngle = 270 - windDirectionRealHeading - 90;
    jibSailAngle = mainSailAngle;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }
  else if (windDirectionRealHeading > 210 && windDirectionRealHeading < 270)     //3rd quadrant
  {
    turningMode = 4;
    mainSailAngle = 270 - windDirectionRealHeading + 90;
    jibSailAngle = mainSailAngle;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);    //Beam Reach to Running
  }
  else if (windDirectionRealHeading >= 150 && windDirectionRealHeading <= 210)     // Go Running
  {
    turningMode = 3;
    mainSailAngle = 180;
    jibSailAngle = 0;
    
    if (straight)
    rudderAngle = 90;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }
  else if (windDirectionRealHeading >= 315 || windDirectionRealHeading <= 45)     // Can't go straight -> Tack
  {
    if (windDirectionRealHeading <= 45)
    {
      beating = false;
      tackLeft();
    }
    if (windDirectionRealHeading >= 330)
    {
      beating = false;
      tackRight();
    }
  }
}

//  Function to tack right 
void tackLeft()
{
  turningMode = 2;
  int windDirectionRealHeading;
  windDirectionRealHeading = windDirection();

  if (beating)
  {
    rudderAngle = 60;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
    delay(1000);
    mainSailAngle = 90 - windDirectionRealHeading + 45;
    jibSailAngle = mainSailAngle;
    rudderAngle = 90;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }

  if (windDirectionRealHeading <= 45)
  {
    windDirectionRealHeading = windDirection();

    mainSailAngle = 90 - windDirectionRealHeading + 45;
    jibSailAngle = mainSailAngle;
    rudderAngle = 90;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);

    readGPS();
    windDirectionRealHeading = windDirection();

    if (calcBearingError(courseAngle(), bearing) > 90)
    {
      beating = true;
      tackRight();
    }
  }

}

// Function to tack left 
void tackRight()
{
  turningMode = 1;
  int windDirectionRealHeading;
  windDirectionRealHeading = windDirection();

  if (beating)
  {
    rudderAngle = 120;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
    delay(1000);
    mainSailAngle = 90 + (360 - windDirectionRealHeading) - 45;
    jibSailAngle = mainSailAngle;
    rudderAngle = 90;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);
  }

  if (windDirectionRealHeading >= 330)
  {
    mainSailAngle = 90 + (360 - windDirectionRealHeading) - 45;
    jibSailAngle = mainSailAngle;
    rudderAngle = 90;
    driveServos(mainSailAngle, jibSailAngle, rudderAngle);

    readGPS();
    windDirectionRealHeading = windDirection();

    if (calcBearingError(courseAngle(), bearing) < -90)
    {
      beating = true;
      tackLeft();
    }
  }
}

// Waypoint - varifies if waypoint is reached and changes to next waypoint
void wayPoint()                                                                                                                                        //TO BE TESTED(!!!)
{
  double waypointDistance = 0;
  
  readGPS();
  waypointDistance = distanceToWaypoint();
  
  if (wp_1_Reached == false && flag1 == 0)
  {
    latatude2 = wp_1_Lat;      // choose first waypoint
    longatude2 = wp_1_Lon;
    flag1 = 1;
  }

  else if ( waypointDistance <= 5 && wp_1_Reached == false && flag1 == 1 ) // Waypoint 1 reached
  {
    wp_1_Reached = true;
    flag2 = 1;
  }
  else if (wp_1_Reached == true && flag2 == 1)    // Waypoint 3 reached sail home
  {
    wp_2_Reached = true;
    latatude2 = homeLat;
    longatude2 = homeLon;
  }
}

// Compass Function, returns averaged compass reading
float  compassReturn()
{
  
  float averagedHeading = 0;
  int headingCounter = 0;
  compass.read();
  while (headingCounter < 50)
  {
    float heading = compass.heading();
    averagedHeading = averagedHeading + heading;
    headingCounter++;
  } 
  return averagedHeading / 50;
}

// Calculate the bearing error
int calcBearingError(float course, float compHeading)
{
  int BearingError = int(course - compHeading);

  if (abs(BearingError) >= 180) 
  {  
    BearingError = BearingError + 360;
  }

  return BearingError;
}

// Returns direction wind
int windDirection()
{
  int analogPin = 0;
  int numAverage = 0;
  int headingCounter = 0;
  int angle = 0;         				      // Wind Sensor

  while(headingCounter < 50)
  {
    angle = analogRead(analogPin);    		              // read the input pin
    angle = map(angle, 0, 1023, 0, 359);                        // map range to 0-359 degree  
    numAverage = numAverage + angle;
    headingCounter++;
  }
  return  numAverage / 50;
}

// Servo Driving function
void driveServos(int mainAngle, int jibAngle, int ruddAngle)
{
  mainAngle = map(mainAngle, 0, 180 , 180, 0);
  mainAngle = constrain(mainAngle, 0, 180);
  jibSailAngle = constrain(jibAngle, 0, 180);
  ruddAngle = constrain(ruddAngle, 60, 120);

  mainSailServo.write(mainAngle);
  jibSailServo.write(jibAngle);
  rudderServo.write(ruddAngle);
}

// Gives the angle of course to nearest waypoint
double courseAngle()
{
  float decla = decLatatude;
  float declo = decLongatude;
  float wpLat = wp_1_Lat;
  float wpLon = wp_1_Lon;
  double dlon = (wp_1_Lon - declo);
  dlon = radianConverter(dlon);
  decla = radianConverter(decla);
  wpLat = radianConverter(wpLat);
  double a1 = sin(dlon) * cos(wpLat);
  double a2 = sin(decla) * cos(wpLat) * cos(dlon);
  a2 = cos(decla) * sin(wpLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degreeConverter(a2);
}

// Converts degree-minute to decimal-degrees     
double convertDegMinToDecDeg(float degMin)                                       
{
  int tempDeg = 0;
  float remainder = 0;
  float decResult = 0;
  tempDeg = int(degMin) / 100;
  remainder = degMin - (tempDeg * 100);
  remainder = (remainder / 60);
  decResult = tempDeg + remainder; 
  return decResult;
}

double distanceToWaypoint()
{
  decLatatude = convertDegMinToDecDeg(latatude1);
  decLongatude = convertDegMinToDecDeg(longatude1);
  float decla = decLatatude;
  float declo = -decLongatude;
  float wpLat = wp_1_Lat;
  float wpLon = wp_1_Lon;
  double delta = 0;
  delta = (declo - wpLon);
  delta = radianConverter(delta);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  decla = radianConverter(decla);
  wpLat = radianConverter(wpLat);
  double slat1 = sin(decla);
  double clat1 = cos(decla);
  double slat2 = sin(wpLat);
  double clat2 = cos(wpLat);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  
  return delta * 6372795;
}
 
// Convert decimal to radians
double radianConverter(float dec)
{
   return  dec * (PI/180);  
}

// Convert radians to degrees
double degreeConverter(float dec)
{
   return  dec * (180/PI);  
}

//Parses GPS NMEA string into longatude and latitude coordinates.
void readGPS()
{
 delay(250);
 while(mySerial.available())
  {
    char c = mySerial.read();
    if(c == ',')
    {
      char c1 = mySerial.read();
      if( c1 == 'A')
      {
        char c2 = mySerial.read();
        if(c2 == ',')
        {
          lat1 = mySerial.read();
          lat2 = mySerial.read();
          lat3 = mySerial.read();
          lat4 = mySerial.read();
          lat5 = mySerial.read();
          lat6 = mySerial.read();
          lat7 = mySerial.read();
          lat8 = mySerial.read();
          lat9 = mySerial.read();
          char c3 = mySerial.read();
          char c4 = mySerial.read();
          char c5 = mySerial.read();
          lon1 = mySerial.read();
          lon2 = mySerial.read();
          lon3 = mySerial.read();
          lon4 = mySerial.read();
          lon5 = mySerial.read();
          lon6 = mySerial.read();
          lon7 = mySerial.read();
          lon8 = mySerial.read();
          lon9 = mySerial.read();
          lon10 = mySerial.read();
          latatude1 = 0;
          latatude1 = latatude1 + ((lat1 - '0')*1000);
          latatude1 = latatude1 + ((lat2 - '0')*100);
          latatude1 = latatude1 + ((lat3 - '0')*10);
          latatude1 = latatude1 + ((lat4 - '0'));
          latatude1 = latatude1 + ((float)(lat6 - '0')/10);
          latatude1 = latatude1 + ((float)(lat7 - '0')/100);
          latatude1 = latatude1 + ((float)(lat8 - '0')/1000);
          latatude1 = latatude1 + ((float)(lat9 - '0')/10000);
          longatude1 = 0;
          longatude1 = longatude1 + ((lon1 - '0')*10000);
          longatude1 = longatude1 + ((lon2 - '0')*1000);
          longatude1 = longatude1 + ((lon3 - '0')*100);
          longatude1 = longatude1 + ((lon4 - '0')*10);
          longatude1 = longatude1 + ((lon5 - '0'));
          longatude1 = longatude1 + ((float)(lon7 - '0')/10);
          longatude1 = longatude1 + ((float)(lon8 - '0')/100);
          longatude1 = longatude1 + ((float)(lon9 - '0')/1000);
          longatude1 = longatude1 + ((float)(lon10 - '0')/10000);
        }
      }
    }
  }
}

// Compass Setup
void setupCompass()
{
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {-476, -428, -513};
  compass.m_max = (LSM303::vector<int16_t>) {+614, +615, +493};
}

// Servo Setup
void setupServos()
{
  mainSailServo.attach(9);  		// attaches pins 9, 10, 11 to the servo objects
  jibSailServo.attach(10);
  rudderServo.attach(11);
}

//  Returns corrected angel 
float correctedAngle(float angle)
{
  if (angle > 360)
  { 
    angle -= 360;
  }
  else if (angle < 0)
  {
    angle += 360;
  }
  
  return angle;
}

// Serial Data wireless tranmission
void serialTransmitt()
{

  char k = '#';
  char j = '^';
  double distance = 0;
  String sail[11] = { "@A", "@B", "@C", "@D", "@E", "@F", "@G", "@H", "@I", "@J", "@K" };
  
  readGPS();
  distance = distanceToWaypoint();
  
  Serial.println(distance);
  Serial1.print(sail[turningMode]);
  Serial1.print(k);
  Serial1.print(distance, 1);
  Serial1.print(j);
  Serial1.print(lat1);
  Serial1.print(lat2);
  Serial1.print(lat3);
  Serial1.print(lat4);
  Serial1.print(lat6);
  Serial1.print(lat7);
  Serial1.print(lat8);
  Serial1.print(lat9);
  Serial1.print(lon1);
  Serial1.print(lon2);
  Serial1.print(lon3);
  Serial1.print(lon4);
  Serial1.print(lon5);
  Serial1.print(lon7);
  Serial1.print(lon8);
  Serial1.print(lon9);
  Serial1.print(lon10);
 delay(500); 
}

// Timer interrupt happening every 1 second
void timerIsr()
{
  serialTransmitt();
  if(autonomous)
  {
    if (counter % 2 == 1)
    {
      navigator();
      wayPoint();
    }
    counter++;
  }
 
}
