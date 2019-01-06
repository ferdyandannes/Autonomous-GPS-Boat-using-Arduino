#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

static const uint32_t GPSBaud = 9600;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPSPlus gps;
SoftwareSerial g(4, 3);

float lat2, long2; 
float headings;
int Haluan;
int tujuan = 1;
int motorkiri1Pin = 9;
int motorkanan1Pin = 10;
int lampusampaiPin = 13;

void displaySensor(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Test Program:   ");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  Serial.begin(9600);
  g.begin(GPSBaud);

  Serial.println(F("Simple Test with TinyGPS++ and attached NEO-6M GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  displaySensor();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded from the GPS Module.
  while (g.available() > 0)
    if (gps.encode(g.read()))
      compassNavigate();
}

void compassNavigate()
{
  // Prints the location if lat-lng information was recieved
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()){
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(","));
  }
  else{
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
  if(mag.begin()){ 
  sensors_event_t event; 
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */   
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print(" "); 
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print(" "); 
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print(" ");Serial.println("uT"); 
  
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  float declinationAngle = 0.010297442587 ;
  heading += declinationAngle;
  
  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
    
  float headingDegrees = heading * 180/M_PI; 
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  headings = headingDegrees;
  delay(500);

  mainNavigate();
  }
}

void mainNavigate(){
  switch (tujuan){
      case 1:
      {
        lat2 = -7.286394;
        long2 = 112.796125;
        break;
      }
      case 2:
      {
        lat2 = -7.286215; 
        long2 = 112.796000;
        break;
      }
    }

  double accuracy =
   TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    lat2,
    long2); 
    
   double jarak =
   TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    lat2, 
    long2);

   Haluan = (360 - (jarak - headings));

    if (accuracy <= 3){
      if (tujuan == 2)
        {
          digitalWrite (lampusampaiPin, HIGH);
          analogWrite(motorkiri1Pin, 0);
          analogWrite(motorkanan1Pin, 0);
          Serial.print("sudah sampai");
        }
       else {
        tujuan += 1;
        Serial.print("tambah");
       }
     }

    if (accuracy > 3){
     if (Haluan > 360)
     {
       Haluan -= 360;
     }
   //BELOK KANAN
    if (Haluan > 290 && Haluan < 359){
      Serial.print(" Kanan3");
      analogWrite(motorkiri1Pin, 0);
      analogWrite(motorkanan1Pin, 170);
    }
    if (Haluan > 220 && Haluan <= 290){
      Serial.print(" Kanan2");
      analogWrite(motorkiri1Pin, 15);
      analogWrite(motorkanan1Pin, 170);
    }
    if (Haluan > 185 && Haluan <= 220){
      Serial.print(" Kanan1");
      analogWrite(motorkiri1Pin, 25);
      analogWrite(motorkanan1Pin, 170);
    }
    if (Haluan > 175 && Haluan <= 185){
      Serial.print(" Gass");
      analogWrite(motorkiri1Pin, 170);
      analogWrite(motorkanan1Pin, 170);
    }

    //BELOK KIRI
    if (Haluan > 1 && Haluan < 70){
      Serial.print(" Kiri3");
      analogWrite(motorkiri1Pin, 170);
      analogWrite(motorkanan1Pin, 0);
    }
    if (Haluan >= 70 && Haluan < 140){
      Serial.print(" Kiri2");
      analogWrite(motorkiri1Pin, 170);
      analogWrite(motorkanan1Pin, 15);
    }
    if (Haluan >= 140 && Haluan <= 175){
      Serial.print(" Kiri1");
      analogWrite(motorkiri1Pin, 170);
      analogWrite(motorkanan1Pin, 25);
    }
   }
   Serial.print("Haluan : ");
   Serial.print(Haluan);
   Serial.print("jarak : ");
   Serial.print(accuracy); 
}

