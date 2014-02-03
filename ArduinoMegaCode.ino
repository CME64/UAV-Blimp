/*
################################
#
# Arduino Mega Code
# Project: UAV-Blimp
# Author: CME64
# date: 2013-6-20
#
#############################
*/


#include <Wire.h>
#include <Servo.h> 
#include <Wire.h>
#include <dht11.h>
#define DHT11_DIO 34
#define BMP085_ADDRESS 0x77
#define ABS(X) ((X)<0?(-(X)):(X))

/* Almanac:
INPUTS:
E:east brushless speed[45~150], W:west ~, D:both ~ ,, ex: D55

OUTPUTS:
E:east brushless, W:wst br, a:altitude (gps), o:longtitude, l:latitude,
U:altitude(pressure), T:tempreture C, F:weather forcast(S:sunny,C:cloudy,R:rainy),
d:direction degree(compass), M:tempreture(humidity) C, H:humidity(%),
C: UltraSonic Distance,
*/
////////////////////////////// vars
char distinct;
///////////////////////////// gps vars
char res;
String res2;
String wrong ="Wrong: ";
String lat = "";
String lon = "";
String alt = "";
String sub2= "";
String spd = "";
char state ;
int sats =0;
int delution = 0;
////////////////////////////// --gps
///////////////////////////// compass
int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];
int ci, headingValue;
////////////////////////// -- comp
////////////////////////// Brushless
Servo myservo,myservo2;  
int val=0;
int spd1 = 0;
int spd2 = 0;
int inc1,inc2;
char theNumberString[4];
////////////////////////// -- br
////////////////////////// pressure
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
const float p0 = 101325;     // Pressure at sea level (Pa)
float altitude;
long b5; 
short temperature2;
short temperature;
long pressure;

// get altitude from gps
//const float currentAltitude ; // GPS current altitude in METERS for 
float ePressure; //= p0 * pow((1-currentAltitude/44330), 5.255);  // expected pressure (in Pa) at altitude
float weatherDiff;
///////////////////////// --pr
///////////////////////// humidity
dht11 DHT11;
///////////////////////// -- hu
///////////////////////// UltraSonic
const int pingPin = 38;
///////////////////////// -- us
///////////////////////// end vars

void setup(){
  ///// gps
  Serial.begin(9600);
  Serial1.begin(38400);
  delay(100);
  Serial1.print("$PMTK251,38400*27\r\n"); // baud rate
  delay(100);
  Serial1.print("$PMTK220,200*2C\r\n");  //freq 200ms
  delay(100);
  Serial1.print("$PMTK101*32\r\n");// hot start
  delay(2000);
  Serial1.print("$PGCMD,16,1,0,0,0,1*6A\r\n");  //settings enabler
  delay(100);
  Serial1.print("$PMTK514,1,1,1,1,1,5,1,1,1,1,1,1,0,1,1,1,1*2A\r\n");  //settings
  delay(100);
  Serial1.print("$PMTK414*33\r\n");
  delay(100);
  //////// --gps
  //////// compass
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
  Wire.begin();
  //////// -- comp
  //////// brushless
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo2.attach(10);
  /////// -- br
  ////// pressure
  bmp085Calibration();
  ////// -pr
}

////////////////////// Functions
/////////////////////// gps
int toInt(String s){
  int n = 0 ;
  for(int i=0;i<s.length();i++){
      if (s[i] == '.') break;
      n *= 10;
      n += (int)s[i]-48;
    }
    return n;
}

boolean acceptable(char c){
  if(c > 31 && c <127 )
    return true;
  else
    return false;
  }
  
boolean isChecked(String s){
  String sub;
  String check;
  int ch = 0;
  int astr;
  astr = s.indexOf('*');
  if(astr){
    sub = s.substring(1,astr);
    check = s.substring(astr+1);
    check.toLowerCase();
    for(int i=0;i<sub.length();i++){
      ch ^= (int)sub[i];
    }
    if(String(ch, HEX) == check) return true; 
  }else return false;
  return false;
}

void GPSMain(){
  delay(100);
  res2="";
  while(res = Serial1.read()){
    if (res == '\r')
    if (Serial1.read()== '\n' )
     break;
    if (acceptable(res))
    res2 = res2+res;
  }
  res2 = res2.substring(res2.lastIndexOf('$'));
  if(res2.length()>5)
    if(res2.substring(0,6) == "$GPGGA" ||  res2.substring(0,6) == "$GPRMC"){
      if(res2.substring(0,6) == "$GPGGA"){
        if(isChecked(res2)){ /////////////////// when result confirmed for GGA
          sub2 = res2.substring(18);
          lat = sub2.substring(0,sub2.indexOf(','));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          lat += sub2.substring(0,sub2.indexOf(','));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          lon = sub2.substring(0,sub2.indexOf(','));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          lon += sub2.substring(0,sub2.indexOf(','));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          state = sub2[0];
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          sats = toInt(sub2.substring(0,sub2.indexOf(',')));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          delution = toInt(sub2.substring(0,sub2.indexOf(',')));
          sub2 = sub2.substring(sub2.indexOf(',')+1);
          alt = sub2.substring(0,sub2.indexOf(','));
          if(state != '0'){
            if(sats>3 && delution < 10){
              if(lat.length()>0 && lon.length()>0){
                delay(1);
                Serial.println("[l"+lat+"]");
                delay(1);
                Serial.println("[o"+lon+"]");
                delay(1);
                lat = "";
                lon = "";
              }
              if(alt.length()>0){
                Serial.println("[a"+alt+"]");
                delay(1);
              }
            }
            Serial1.print("$PMTK001,414,3*31\r\n"); // tell gps valid & accepted packet
          }else{
            Serial1.print("$PMTK001,414,2*30\r\n"); // tell gps valid & not accepted packet
          }
        }else{ /////////////// not correct GGA
          Serial1.print("$PMTK001,414,0*32\r\n"); // tell gps invalid packet
        }
        
      }else if(res2.substring(0,6) == "$GPRMC" && res2.length() > 30){ /////// RMC (cannot confirm currently because checksum is different *because of filtering noisy chars) -> excluded from check
      } else {                                    ///////// both fail
        Serial1.print("$PMTK001,414,0*32\r\n"); // tell gps invalid packet
      }
    } else {
      if(res2.substring(0,1)== "$")
        Serial1.print("$PMTK001,414,1*33\r\n"); // tell gps unwanted packet
      else
        Serial1.print("$PMTK001,414,0*32\r\n"); // tell gps invalid packet
    }
}
////////////////////// --gps
////////////////////// compass
void compMain(){
  delay(10);
  Wire.beginTransmission(slaveAddress);
  Wire.write("A");              // The "Get Data" command
  Wire.endTransmission();
  delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
  ci = 0;
  while(Wire.available() && ci < 2)
  { 
    headingData[ci] = Wire.read();
    ci++;
  }
  headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
  Serial.print("[d");
  Serial.print(int (headingValue / 10));     // The whole number part of the heading
  Serial.print(".");
  Serial.print(int (headingValue % 10));     // The fractional part of the heading
  Serial.println("]");
  delay(100);
}
////////////////////// -- comp
///////////////////// brushless
void setSpeed( int sp,int mot){
if (mot == 1){
  if (sp>spd1)
    for(;spd1<sp;spd1+=5){
      myservo.write(spd1);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      delay(100);
    }
  else if(sp<spd1)
    for(;spd1>sp;spd1-=5){
      myservo.write(spd1);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      delay(100);
    }
  } else if( mot == 2){
    if (sp>spd2)
    for(;spd2<sp;spd2+=5){
      myservo2.write(spd2);
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
  else if(sp<spd2)
    for(;spd2>sp;spd2-=5){
      myservo2.write(spd2);
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
  } else if ( mot == 3 ){
    inc1 = ABS(spd1-sp)/5;
    inc2 = ABS(spd2-sp)/5;
    if (sp>=spd2 && sp>=spd1){
    for(;spd2<sp || spd1<sp;spd2+=inc2,spd1+=inc1){
      myservo.write(spd1);
      myservo2.write(spd2);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
    myservo.write(sp);
    myservo2.write(sp);
    }else if(sp<=spd2 && sp<=spd1){
    for(;spd2>sp || spd1>sp;spd2-=inc2,spd1-=inc1){
      myservo.write(spd1);
      myservo2.write(spd2);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
    myservo.write(sp);
    myservo2.write(sp);
    } else if(sp<=spd2 && sp>=spd1){
    for(;sp<spd2 || sp>spd1;spd2-=inc2,spd1+=inc1){
      myservo.write(spd1);
      myservo2.write(spd2);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
    myservo.write(sp);
    myservo2.write(sp);
    }else if(sp>=spd2 && sp<=spd1){
      for(;sp>spd2 || sp<spd1;spd2+=inc2,spd1-=inc1){
      myservo.write(spd1);
      myservo2.write(spd2);
      Serial.print("[E");
      Serial.print(spd1);
      Serial.println("]");
      Serial.print("[W");
      Serial.print(spd2);
      Serial.println("]");
      delay(100);
    }
    myservo.write(sp);
    myservo2.write(sp);
    }
  }
}

void BrMain(char motor){
  if(motor == 'E')
    {
      for (int i = 0; i < 3; theNumberString[i++] = Serial.read());
      theNumberString[3] = 0x00;
      val = atoi(theNumberString);
      // Serial.print("[E");
      // Serial.print(val);
      // Serial.println("]");
      setSpeed(val,1);
    }else if (motor == 'W'){
      for (int i = 0; i < 3; theNumberString[i++] = Serial.read());
      theNumberString[3] = 0x00;
      val = atoi(theNumberString);
      // Serial.print("[W");
      // Serial.print(val);
      // Serial.println("]");
      setSpeed(val,2);
    } else if( motor == 'D'){
      for (int i = 0; i < 3; theNumberString[i++] = Serial.read());
      theNumberString[3] = 0x00;
      val = atoi(theNumberString);
      // Serial.print("[E");
      // Serial.print(val);
      // Serial.println("]");
      // Serial.print("[W");
      // Serial.print(val);
      // Serial.println("]");
      setSpeed(val,3);
    }
  if(val >149 ) val = 149;
  if(val <0 ) val=0;
  
}
///////////////////// --br
///////////////////// pressure
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3);
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  return up;
}

void pressMain(){
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  Serial.print("[T");
  temperature2=temperature*0.1;
  Serial.print(temperature2, DEC);
  Serial.println("]");
  Serial.print("[P");
  Serial.print(pressure, DEC);
  Serial.println("]");
  
  // "weather forcaster"  after you've calculated the pressure 
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
  Serial.print("[U");
  Serial.print(altitude, 2);
  Serial.println("]");

  if(toInt(alt)>0){
    ePressure = p0 * pow((1-toInt(alt)/44330), 5.255);
    weatherDiff = pressure - ePressure;
    if(weatherDiff > 250)
      Serial.println("[FS]");
    else if ((weatherDiff <= 250) || (weatherDiff >= -250))
      Serial.println("[FC]");
    else if (weatherDiff > -250)
      Serial.println("[FR]");
  }
  delay(100);
}
///////////////////// -- pr
///////////////////// humidity
void humidMain(){
  if (DHT11.read(DHT11_DIO) == DHTLIB_OK){
    Serial.print("[M");
    Serial.print((float)DHT11.temperature, 2);
    Serial.println("]"); //C
    Serial.print("[H");
    Serial.print((float)DHT11.humidity, 2);
    Serial.println("]"); // percent
  }
  delay(100);
}
///////////////////// -- hu
///////////////////// US
long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
}

void UltraMain(){
  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.print("[C");
  Serial.print(cm);
  Serial.println("]");//cm
  delay(100);
}
///////////////////// --us
///////////////////// --fn



void loop(){
  GPSMain();
  compMain();
  distinct = Serial.read();
  if (distinct == 'E' || distinct == 'W' || distinct == 'D') // East/West motor, D:both
    BrMain(distinct);
  Serial.print("[E");
  Serial.print(spd1);
  Serial.println("]");
  Serial.print("[W");
  Serial.print(spd2);
  Serial.println("]");
  delay(100);
  pressMain();
  humidMain();
  UltraMain();
}
