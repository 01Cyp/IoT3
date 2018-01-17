#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#if ARDUINO >= 100
  SoftwareSerial mySerial(3, 2);
#else
  NewSoftSerial mySerial(3, 2);
#endif

#define GPSECHO  true
#define BNO055_SAMPLERATE_DELAY_MS (100)

boolean usingInterrupt = false;
void useInterrupt(boolean);

Adafruit_GPS GPS(&mySerial);
Adafruit_BNO055 bno = Adafruit_BNO055();


unsigned long t0;

void setup()  
{
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
  
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  GPS.sendCommand(PGCMD_ANTENNA);
  
  useInterrupt(true);
  
  delay(5000);

  t0 = millis();
  
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  
  if (GPSECHO)
    if (c) UDR0 = c;
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop()
{  
  /*if(millis()>(t0+2000)) {
    if(GPS.newNMEAreceived()){
      Serial.println(GPS.lastNMEA());
      /*Serial.print("longitude : ");
      Serial.println(GPS.longitude);
      Serial.print("latitude : ");
      Serial.println(GPS.latitude);
    }
  
    t0 = millis();
  }
  */
  useInterrupt(false);
  
  imu::Vector<3> lacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  uint8_t force = sqrt(pow(lacc.x(),2)+pow(lacc.y(),2)+pow(lacc.z(),2));
  
  Serial.println(force);
  
  useInterrupt(true);
  
}
