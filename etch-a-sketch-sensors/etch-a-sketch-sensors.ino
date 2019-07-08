#include <Wire.h>

#include "SparkFunBME280.h"
#include "SparkFun_VEML6075_Arduino_Library.h"
#include "SparkFun_AS7265X.h"
#include "RadiationWatch.h"

BME280 pthSensor;
VEML6075 uvSensor;
AS7265X specSensor;
RadiationWatch radiationWatch;

float radiation = 0.0;
float radiationError = 0.0;

void onRadiation() {

  radiation = radiationWatch.uSvh();
  radiationError = radiationWatch.uSvhError();
  
}

void setup() {

  Serial.begin(9600);
  Wire.begin();

  if (pthSensor.begin() == false) {
    Serial.println("Error, the PTH sensor did NOT respond!");
  }

  if (uvSensor.begin() == false) {
    Serial.println("Error, the UV sensor did NOT respond!");
  }

//  if (specSensor.begin() == false) {
//    Serial.println("Error, the spectral sensor did NOT respond!");
//  }

  radiationWatch.setup();
  radiationWatch.registerRadiationCallback(&onRadiation);

}

void loop() {

  float humidity = pthSensor.readFloatHumidity();
  float pressure = pthSensor.readFloatPressure();
  float altitude = pthSensor.readFloatAltitudeFeet();
  float temperature = pthSensor.readTempF();

  float uvaVal = uvSensor.uva();
  float uvbVal = uvSensor.uvb();
  float uvIndex = uvSensor.index();

//  specSensor.takeMeasurements();
//
//  float specCalibratedA = specSensor.getCalibratedA();
//  float specCalibratedB = specSensor.getCalibratedB();
//  float specCalibratedC = specSensor.getCalibratedC();
//  float specCalibratedD = specSensor.getCalibratedD();
//  float specCalibratedE = specSensor.getCalibratedE();
//  float specCalibratedF = specSensor.getCalibratedF();
//  float specCalibratedG = specSensor.getCalibratedG();
//  float specCalibratedH = specSensor.getCalibratedH();
//  float specCalibratedI = specSensor.getCalibratedI();
//  float specCalibratedJ = specSensor.getCalibratedJ();
//  float specCalibratedK = specSensor.getCalibratedK();
//  float specCalibratedL = specSensor.getCalibratedL();
//  float specCalibratedR = specSensor.getCalibratedR();
//  float specCalibratedS = specSensor.getCalibratedS();
//  float specCalibratedT = specSensor.getCalibratedT();
//  float specCalibratedU = specSensor.getCalibratedU();
//  float specCalibratedV = specSensor.getCalibratedV();

  radiationWatch.loop();

  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Altitude: ");
  Serial.println(altitude);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  Serial.print("UVA: ");
  Serial.println(uvaVal);
  Serial.print("UVB: ");
  Serial.println(uvbVal);
  Serial.print("UV Index: ");
  Serial.println(uvIndex);

//  Serial.print("specCalA: ");
//  Serial.println(specCalibratedA);
//  Serial.print("specCalB: ");
//  Serial.println(specCalibratedB);
//  Serial.print("specCalC: ");
//  Serial.println(specCalibratedC);
//  Serial.print("specCalD: ");
//  Serial.println(specCalibratedD);
//  Serial.print("specCalE: ");
//  Serial.println(specCalibratedE);
//  Serial.print("specCalF: ");
//  Serial.println(specCalibratedF);
//  Serial.print("specCalG: ");
//  Serial.println(specCalibratedG);
//  Serial.print("specCalH: ");
//  Serial.println(specCalibratedH);
//  Serial.print("specCalI: ");
//  Serial.println(specCalibratedI);
//  Serial.print("specCalJ: ");
//  Serial.println(specCalibratedJ);
//  Serial.print("specCalK: ");
//  Serial.println(specCalibratedK);
//  Serial.print("specCalL: ");
//  Serial.println(specCalibratedL);
//  Serial.print("specCalR: ");
//  Serial.println(specCalibratedR);
//  Serial.print("specCalS: ");
//  Serial.println(specCalibratedS);
//  Serial.print("specCalT: ");
//  Serial.println(specCalibratedT);
//  Serial.print("specCalU: ");
//  Serial.println(specCalibratedU);
//  Serial.print("specCalAV: ");
//  Serial.println(specCalibratedV);

  Serial.print("Radiation: ");
  Serial.println(radiation);
  Serial.print("Radiation Error: ");
  Serial.println(radiationError);

  delay(5000);

}
