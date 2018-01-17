#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> lacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  uint8_t force = sqrt(pow(lacc.x(),2)+pow(lacc.y(),2)+pow(lacc.z(),2));

  Serial.println(force);
}
