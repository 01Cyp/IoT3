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
  Serial.begin(9600);
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
  
  delay(1000);

  t0 = millis();
  
}


SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  
 /* if (GPSECHO)
    if (c) UDR0 = c;*/
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
  /*
  String msg = "";
  
  while(GPS.read()!='$') {}
  Serial.println("un $ sauvage est apparu");
  Serial.print("->"); Serial.println(GPS.read(),DEC);
  if(GPS.read()=='G' && GPS.read()=='P' && GPS.read()=='G' && GPS.read()=='G' && GPS.read()=='A'){
    Serial.println("ch'uis dedans");
    char c = GPS.read();
    while(c != '*'){
      Serial.println(c);
      msg += c;
      c = GPS.read();
    }
  }
  Serial.println(msg);
*/

  /*,160740.000,5036.4332,N,00308.1575,E,1,6,2.48,-46.6,M,47.2,M,,*/

  if(millis()>(t0+2000)) {
    if(GPS.newNMEAreceived()){
      GPS.parse(GPS.lastNMEA());
      Serial.println(GPS.longitude);
      Serial.println(GPS.latitude);
    }
  
    t0 = millis();
  }
  
  
  useInterrupt(false);
  
  imu::Vector<3> lacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  uint8_t force = sqrt(pow(lacc.x(),2)+pow(lacc.y(),2)+pow(lacc.z(),2));
  
  Serial.println(force);
  
  useInterrupt(true);
}
