#include <bbb.h>
#include <SparkFunBME280.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include <RadiationWatch.h>
#include "SparkFun_AS7265X.h" //Click here to get the library: http://librarymanager/All#SparkFun_AS7265X

// BBB Objects
#define BBB_SAVE_PIN 			(3)
#define BBB_LOW_BAT_PIN 		(4)
#define BBB_LAUNCH_DET_PIN 		(5)
#define BBB_DEPLOY_DET_PIN 		(6)
#define BBB_IMU_WAKE_DET_PIN 	(7)

BBB bbb;

float beacon_period_ms = 1000;

// Sensors
BME280 sensor_PTH;
VEML6075 sensor_uv;
RadiationWatch sensor_rad;
AS7265X sensor_spec;

struct struct_sensor {
   float temperature;
   float pressure;
   float humidity; 

   float uva;
   float uvb;
   float index;

   float ozone;

   float rad;
   float rad_err;

   float spec_a;
   float spec_b;
   float spec_c;
   float spec_d;
   float spec_e;
   float spec_f;
   float spec_g;
   float spec_h;
   float spec_i;
   float spec_j;
   float spec_k;
   float spec_l;
   float spec_r;
   float spec_s;
   float spec_t;
   float spec_u;
   float spec_v;
   float spec_w; 
};

struct struct_sensor data_sensor;

///////////////////
//// SETUP
//////////////////
void setup() {
   // Serial OUT
   Serial.begin(9600);

   Wire.begin();

   if (sensor_PTH.beginI2C() == false) //Begin communication over I2C
   {
      Serial.println("PTH Sensor did not respond. Please check wiring.");
      while(1); //Freeze
   }

   if (sensor_uv.begin() == false) {
      Serial.println("Unable to communicate with UV sensor (VEML6075).");
      while (1);
   }


   //if(sensor_spec.begin() == false)
   //{
   //   Serial.println("Spectrometry sensor does not appear to be connected. Please check wiring. Freezing...");
   //   while(1);
   //}
  
   //Initialize BBB interface, choose the serial interface you have connected and buad rate
   bbb.init(&Serial1,115200);
   //use software serial instead
   //bbb.init(&mySerial,4800);

   //set command handler function for BBB interface, gets called during bbb.refresh() if a command is received
   //bbb.setCommandCallback(&ground_command_handler);

   //other configuration
   bbb.requestBBBId(); //ask for BBB id, response is handled in loop()
  
   //set beacon rate to 1000 milliseconds
   //beacon_period_ms = 1000;
   //record current program counter
   //last_beacon_time = millis();
}

///////////////////
//// LOOP
//////////////////
void loop () {
   // put your main code here, to run repeatedly:
   // avoid using delays, the UART buffer may overflow from the BBB if too much delay is added
   // Send beacon of data once every second to transmit to ground

   getPTHSensorData();
   getUVSensorData();
   //getSpecSensorData();
   getO3SensorData();

   // ready to send packet, send to BBB
   if (bbb.sendRFPacket()) {
      Serial.println("Sent beacon!");
   } else {
      Serial.println("Failed to send beacon!");
   }
     
   // Set RF Packet
   bbb.sendRFPacketWrap((uint8_t * )(&data_sensor),sizeof(struct_sensor));

   // Save BBB Data to SD Card
   char data_to_save[256];
   uint32_t length = sprintf(data_to_save,
      "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      data_sensor.temperature,
      data_sensor.pressure,
      data_sensor.humidity, 
      data_sensor.uva,
      data_sensor.uvb,
      data_sensor.index,
      data_sensor.ozone,
      data_sensor.rad,
      data_sensor.rad_err,
      data_sensor.spec_a,
      data_sensor.spec_b,
      data_sensor.spec_c,
      data_sensor.spec_d,
      data_sensor.spec_e,
      data_sensor.spec_f,
      data_sensor.spec_g,
      data_sensor.spec_h,
      data_sensor.spec_i,
      data_sensor.spec_j,
      data_sensor.spec_k,
      data_sensor.spec_l,
      data_sensor.spec_r,
      data_sensor.spec_s,
      data_sensor.spec_t,
      data_sensor.spec_u,
      data_sensor.spec_v,
      data_sensor.spec_w
   );

   Serial.println(data_to_save);
      
   bbb.saveDataToBBB((uint8_t *)data_to_save,length);
}


void onRadiation() {
   data_sensor.rad = sensor_rad.uSvh();
   data_sensor.rad_err = sensor_rad.uSvhError();
}


void getPTHSensorData () {
   Serial.print("Humidity: ");
   Serial.print(sensor_PTH.readFloatHumidity(), 0);

   Serial.print(" Pressure: ");
   Serial.print(sensor_PTH.readFloatPressure(), 0);

   Serial.print(" Alt: ");
   //Serial.print(sensor_PTH.readFloatAltitudeMeters(), 1);
   Serial.print(sensor_PTH.readFloatAltitudeFeet(), 1);

   Serial.print(" Temp: ");
   //Serial.print(sensor_PTH.readTempC(), 2);
   Serial.print(sensor_PTH.readTempF(), 2);

   Serial.println();

   data_sensor.pressure = sensor_PTH.readFloatPressure();
   data_sensor.temperature = sensor_PTH.readTempC();
   data_sensor.humidity = sensor_PTH.readFloatHumidity();
}

void getUVSensorData () {
   Serial.println("UVA, UVB, UV Index");
   Serial.println(String(sensor_uv.uva()) + ", " + String(sensor_uv.uvb()) + ", " + String(sensor_uv.index()));

   data_sensor.uva = sensor_uv.uva();
   data_sensor.uvb = sensor_uv.uvb();
   data_sensor.index = sensor_uv.index();
}

void getO3SensorData () {
   data_sensor.ozone = analogRead(A0);
   Serial.print("O3: ");
   Serial.print(data_sensor.ozone);
   Serial.println();
}

void getSpecSensorData () {
   sensor_spec.takeMeasurements();
   
   Serial.print(sensor_spec.getCalibratedA());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedB());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedC());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedD());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedE());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedF());
   Serial.print(",");

   Serial.print(sensor_spec.getCalibratedG());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedH());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedI());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedJ());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedK());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedL());
   Serial.print(",");

   Serial.print(sensor_spec.getCalibratedR());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedS());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedT());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedU());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedV());
   Serial.print(",");
   Serial.print(sensor_spec.getCalibratedW());
   Serial.print("\n");

   data_sensor.spec_a = sensor_spec.getCalibratedA();
   data_sensor.spec_b = sensor_spec.getCalibratedB();
   data_sensor.spec_c = sensor_spec.getCalibratedC();
   data_sensor.spec_d = sensor_spec.getCalibratedD();
   data_sensor.spec_e = sensor_spec.getCalibratedE();
   data_sensor.spec_f = sensor_spec.getCalibratedF();
   data_sensor.spec_g = sensor_spec.getCalibratedG();
   data_sensor.spec_h = sensor_spec.getCalibratedH();
   data_sensor.spec_i = sensor_spec.getCalibratedI();
   data_sensor.spec_j = sensor_spec.getCalibratedJ();
   data_sensor.spec_k = sensor_spec.getCalibratedK();
   data_sensor.spec_l = sensor_spec.getCalibratedL();
   data_sensor.spec_r = sensor_spec.getCalibratedR();
   data_sensor.spec_s = sensor_spec.getCalibratedS();
   data_sensor.spec_t = sensor_spec.getCalibratedT();
   data_sensor.spec_u = sensor_spec.getCalibratedU();
   data_sensor.spec_v = sensor_spec.getCalibratedV();
   data_sensor.spec_w = sensor_spec.getCalibratedW();
}

