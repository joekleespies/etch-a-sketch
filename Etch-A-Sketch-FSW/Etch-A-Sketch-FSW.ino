#include <bbb.h>
#include <SparkFunBME280.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include <RadiationWatch.h>

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
VEML6075 uv;

struct struct_sensor {
  float temperature;
  float pressure;
  float humidity; 

  float uva;
  float uvb;
  float index;

  float o3_ppm;
  float o3_ppb;

  float rad;

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
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }

  if (uv.begin() == false) {
    Serial.println("Unable to communicate with VEML6075.");
    while (1);
  }
  
  //Initialize BBB interface, choose the serial interface you have connected and buad rate
  bbb.init(&Serial1,115200);
  //use software serial instead
  //bbb.init(&mySerial,4800);

  //set command handler function for BBB interface, gets called during bbb.refresh() if a command is received
  bbb.setCommandCallback(&ground_command_handler);

  //other configuration
  bbb.requestBBBId(); //ask for BBB id, response is handled in loop()
  
  //set beacon rate to 1000 milliseconds
  //beacon_period_ms = 1000;
  //record current program counter
//  last_beacon_time = millis();
}

void ground_command_handler(uint32_t command_id, uint32_t command_option){
	//do something with a command you get
	if(command_id == 0x4000){
		if(command_option == 0x2020){
			Serial.println("Got a special command!");
		}
	}
	else{
		Serial.println("Didn't get a special command :(");
	}
}

///////////////////
//// LOOP
//////////////////
void loop () {
  // put your main code here, to run repeatedly:
  //avoid using delays, the UART buffer may overflow from the BBB if too much delay is added
  //Send beacon of data once every second to transmit to ground
  //if(millis() - last_beacon_time > beacon_period_ms){
	// bbb.requestBBBId(); //request BBB id

   getPTHSensorData();
   getUVSensorData();

   Serial.print(data_sensor.temperature, 0);
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
    Serial.println(String(uv.uva()) + ", " + String(uv.uvb()) + ", " + String(uv.index()));

    data_sensor.uva = uv.uva();
    data_sensor.uvb = uv.uvb();
    data_sensor.index = uv.index();
}
