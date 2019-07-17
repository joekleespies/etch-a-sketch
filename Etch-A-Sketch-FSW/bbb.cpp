#include "bbb.h"

uint8_t BBB::init(HardwareSerial * bbb_port, int baud_rate){
	serial_type = 0;
	//init serial port, try to talk to BBB
	serial_port = (void *)bbb_port;
	
	((HardwareSerial *)serial_port)->begin(baud_rate);
	//initialially not doing anythign with I2C
	i2c_state = 0;
	//init buffer
	rf_packet_tail = 0;
	
	//cmd_cb = &(default_handler);
	
	//try to talk to BBB
	requestBBBId();
	if(BBB_id != 0){
		return 1;
	}
	else{
		return 0;
	}
}
uint8_t BBB::init(SoftwareSerial * bbb_port, int baud_rate){
	serial_type = 1;
	//init serial port, try to talk to BBB
	serial_port = (void *)bbb_port;
	
	((SoftwareSerial *)serial_port)->begin(baud_rate);
	//initialially not doing anythign with I2C
	i2c_state = 0;
	//init buffer
	rf_packet_tail = 0;
	
	//cmd_cb = &(default_handler);
	
	//try to talk to BBB
	requestBBBId();
	if(BBB_id != 0){
		return 1;
	}
	else{
		return 0;
	}
}
void BBB::setSavePin(uint8_t pin){
	save_pin = pin;
	pinMode(save_pin,OUTPUT);
	digitalWrite(save_pin,HIGH);
}
void BBB::setLowBatPin(uint8_t pin){
	low_bat_pin = pin;
	pinMode(low_bat_pin,INPUT);
}
void BBB::setLaunchPin(uint8_t pin){
	launch_pin = pin;
	pinMode(launch_pin,INPUT);
}
void BBB::setDepPin(uint8_t pin){
	deploy_pin = pin;
	pinMode(deploy_pin,INPUT);
}
void BBB::setImu_wakePin(uint8_t pin){
	imu_wake_pin = pin;
	pinMode(imu_wake_pin,INPUT);
}
void BBB::setI2cHijPin(uint8_t pin){
	i2c_hijack_pin = pin;
	digitalWrite(i2c_hijack_pin,LOW);
}

void BBB::startRFPacket(){
	rf_packet_tail = 0;
	//put in preamble information
	//add preamble
	addRFData((uint32_t)PAYLOAD_RF_PREAMBLE);
	//add payload ID
	addRFData(BBB_id);
}
void BBB::addRFData(uint8_t data){
	if(rf_packet_tail > 251){
		return;
	}
	rf_packet[rf_packet_tail] = data;
	rf_packet_tail++;
}
void BBB::addRFData(uint16_t data){
	if(rf_packet_tail > 250){
		return;
	}
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	#endif
}
void BBB::addRFData(uint32_t data){
	if(rf_packet_tail > 248){
		return;
	}
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	#endif
}
void BBB::addRFData(int32_t data){
	if(rf_packet_tail > 248){
		return;
	}
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	#endif
}
void BBB::addRFData(uint64_t data){
	if(rf_packet_tail > 244){
		return;
	}
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 32);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 40);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 48);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 56);
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 56);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 48);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 40);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 32);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 24);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 16);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)(data >> 8);
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = (uint8_t)data;
	rf_packet_tail++;
	#endif
}
void BBB::addRFData(float data){
	if(rf_packet_tail > 248){
		return;
	}
	union{
		uint32_t d;
		float f;
		uint8_t da[4];
	}c;
	
	c.d = data;
	
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = c.da[0];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[1];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[2];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[3];
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = c.da[3];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[2];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[1];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[0];
	rf_packet_tail++;
	#endif
}
void BBB::addRFData(double data){
	if(rf_packet_tail > 244){
		return;
	}
	union{
		uint64_t d;
		double f;
		uint8_t da[8];
	}c;
	
 	//convert c;
	c.d = data;
	#ifdef LITTLE_ENDIAN
	rf_packet[rf_packet_tail] = c.da[0];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[1];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[2];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[3];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[4];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[5];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[6];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[7];
	rf_packet_tail++;
	#else
	rf_packet[rf_packet_tail] = c.da[7];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[6];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[5];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[4];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[3];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[2];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[1];
	rf_packet_tail++;
	rf_packet[rf_packet_tail] = c.da[0];
	rf_packet_tail++;
	#endif
}
uint8_t BBB::sendRFPacket(){
	if(rf_packet_tail <= 4){
		return 0;
	}
	
	//add suffix
	addRFData((uint32_t)PAYLOAD_RF_SUFFIX);
	//write to BBB port
	/*for(uint8_t i = 0; i < rf_packet_tail; i++){
		Serial.print(*((uint8_t *)rf_packet+i), HEX);
	}*/ //used for testing
	if(!serial_type){
		((HardwareSerial *)serial_port)->write((uint8_t *)rf_packet, rf_packet_tail);
	}
	else{
		((SoftwareSerial *)serial_port)->write((uint8_t *)rf_packet, rf_packet_tail);
	}
	return 1;
}
uint8_t BBB::sendRFPacket(uint8_t * packet, size_t packet_length){
	if(packet == NULL || packet_length == 0 || packet_length > 256){
		return 0;
	}
	if(!serial_type){
		((HardwareSerial *)serial_port)->write(packet, packet_length);
	}
	else{
		((SoftwareSerial *)serial_port)->write(packet, packet_length);
	}
	return 1;
}
uint8_t BBB::sendRFPacketWrap(uint8_t * data, size_t data_length){
	if(data == NULL || data_length == 0 || data_length > 256){
		return 0;
	}
	uint32_t tmp = PAYLOAD_RF_PREAMBLE;
	if(!serial_type){
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((HardwareSerial *)serial_port)->write((uint8_t *)&BBB_id, 4);
		
		((HardwareSerial *)serial_port)->write(data, data_length);
		
		tmp = PAYLOAD_RF_SUFFIX;
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
	}
	else{
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((SoftwareSerial *)serial_port)->write((uint8_t *)&BBB_id, 4);
		
		((SoftwareSerial *)serial_port)->write(data, data_length);
		
		tmp = PAYLOAD_RF_SUFFIX;
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
	}
	return 1;
}

void BBB::sendBBBCommand(uint32_t command, uint32_t option){
	uint32_t tmp = CMD_PACKET_PREAMBLE;
	if(!serial_type){
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((HardwareSerial *)serial_port)->write(0);
		((HardwareSerial *)serial_port)->write(0);
		((HardwareSerial *)serial_port)->write(0);
		((HardwareSerial *)serial_port)->write(0);
		((HardwareSerial *)serial_port)->write((uint8_t *)&command, 4);
		((HardwareSerial *)serial_port)->write((uint8_t *)&option, 4);
		tmp = CMD_PACKET_SUFFIX;
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
	}
	else{
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((SoftwareSerial *)serial_port)->write((uint8_t)0);
		((SoftwareSerial *)serial_port)->write((uint8_t)0);
		((SoftwareSerial *)serial_port)->write((uint8_t)0);
		((SoftwareSerial *)serial_port)->write((uint8_t)0);
		((SoftwareSerial *)serial_port)->write((uint8_t *)&command, 4);
		((SoftwareSerial *)serial_port)->write((uint8_t *)&option, 4);
		tmp = CMD_PACKET_SUFFIX;
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
	}
}
void BBB::saveDataToBBB(uint8_t * data, uint8_t length){
	//digitalWrite(save_pin,LOW);
	uint32_t tmp = PAYLOAD_SD_PREAMBLE;
	if(!serial_type){
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((HardwareSerial *)serial_port)->write((uint8_t *)data, length);
		tmp = PAYLOAD_SD_SUFFIX;
		((HardwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((HardwareSerial *)serial_port)->flush();
	}
	else{
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((SoftwareSerial *)serial_port)->write((uint8_t *)data, length);
		tmp = PAYLOAD_SD_SUFFIX;
		((SoftwareSerial *)serial_port)->write((uint8_t *)&tmp, 4);
		((SoftwareSerial *)serial_port)->flush();
	}
	//digitalWrite(save_pin,HIGH);
}
uint8_t BBB::waitForCMDResponse(CMDPacket * packet){
	/*if(packet == NULL){
		return 0;
	}
	uint32_t time = millis();
	while(millis() - time > 300){
		while(serial_port->available()){
			if(packet_put(serial_port->read()) == 1){
				Serial.println("Got 1");
				return 1;
			}
		}
	}*/
	return 0;
}
uint32_t BBB::requestBBBId(){
	sendBBBCommand(SEND_BBB_ID,0);
	//sendBBBCommand(SEND_BBB_ID,0);
	/*uint32_t time = millis();
	while(BBB_id == 0 && millis()-time < 2000){
		//sendBBBCommand(SEND_BBB_ID,0);
		while(serial_port->available()){
			uint8_t ret = packet_put(serial_port->read());
			Serial.print("RET: ");
			Serial.println(ret);
			if(ret == 1){
				Serial.println("Got 1");
				return BBB_id;
			}
		}
	}
	Serial.println("GOT HERE:");
	return BBB_id;*/
	
	return 0;
}
void BBB::requestBBBIMUData(){
	sendBBBCommand(SEND_IMU_DATA,0);
	//Serial.println("test2");
	/*uint32_t current_time = millis();
	while(millis() - current_time < 600){
		while(serial_port->available()){
			uint8_t ret = packet_put(serial_port->read());
			Serial.print("RET: ");
			Serial.println(ret);
			if(packet_put(serial_port->read()) == 2){
				imu_data = &imu_packet_data;
				return;
			}
		}
	}*/
}
void BBB::enableI2COverride(){
	
}
void BBB::disableI2COverride(){
	
}
void BBB::getIMUDataDirect(IMUData * imu_data){
	
}

uint8_t BBB::setHeater1SetPoint(float set_point_temp){
	
	return 0;
}
uint8_t BBB::setHeater2SetPoint(float set_point_temp){
	
	return 0;
}
/* 
		Request the BBB to turn off the payload for X number of seconds.
	*/
uint8_t BBB::setPayloadTurnOffTime(uint32_t seconds){
	BBB::sendBBBCommand(0,0);
	
	return 1;
}

void BBB::setCommandCallback(void (*func)(uint32_t , uint32_t)){
	cmd_cb = func;
}
uint8_t BBB::refresh(){
	if(!serial_type){
		while(((HardwareSerial *)serial_port)->available()){
			uint8_t incoming_byte = ((HardwareSerial *)serial_port)->read();
			return packet_put(incoming_byte);
			//Serial.print(incoming_byte,HEX);
		}
	}
	else{
		while(((SoftwareSerial *)serial_port)->available()){
			uint8_t incoming_byte = ((SoftwareSerial *)serial_port)->read();
			return packet_put(incoming_byte);
			//Serial.print(incoming_byte,HEX);
		}
	}
}
uint8_t BBB::packet_put(uint8_t data){
	if(packet_buffer_tail >= packet_buffer_size){
		//flush packet buffer
		packet_buffer_tail = 0;
		return 0;
	}
	//add data to buffer
	//Serial.print(data,HEX);
	packet_buffer[packet_buffer_tail] = data;
	packet_buffer_tail++;
	//check to see if a valid packet has been received
	if(packet_buffer_tail >= 12){
		//uint32_t * tmp_check = (uint32_t *)((uint8_t *)packet_buffer + packet_buffer_tail-4);
		uint32_t tmp_check = ((uint32_t)packet_buffer[packet_buffer_tail-4] << 24);
		tmp_check |= ((uint32_t)packet_buffer[packet_buffer_tail-4] << 16);
		tmp_check |= ((uint32_t)packet_buffer[packet_buffer_tail-4] << 8);
		tmp_check |= ((uint32_t)packet_buffer[packet_buffer_tail-4]);
		
		//Serial.println(tmp_check,HEX);
		
		//Serial.println(*tmp_check,HEX);
		if(tmp_check == PAYLOAD_CMD_SUFFIX){
			uint32_t command_id = *(uint32_t *)((uint8_t *)packet_buffer + packet_buffer_tail-12);
			uint32_t command_option = *(uint32_t *)((uint8_t *)packet_buffer + packet_buffer_tail-8);
			//Serial.print("RX command: ");
			//Serial.println(command_id,HEX);
			//data can now be accessed
			packet_buffer_tail = 0;
			if(command_id == SEND_BBB_ID){
				//Serial.println("GOT BBB ID");
				BBB_id = command_option;
				return BBB_ID_RECEIVED;
			}
			else{
				//default_handler(command_id,command_option);
				cmd_cb(command_id,command_option);
				return CMD_RECEIVED;
			}
		}
		else if(tmp_check == PAYLOAD_IMU_SUFFIX){
			//Serial.println("got IMU data");
			for(uint32_t i = 0; i < sizeof(IMUData); i++){
				*(uint8_t *)(((uint8_t *)&imu_packet_data) + i) = packet_buffer[i+packet_buffer_tail-sizeof(IMUData)];
				//Serial.print(*(uint8_t *)(((uint8_t *)&imu_packet_data) + i),HEX);
				//Serial.print(*(float *)(((uint8_t *)&imu_packet_data) + i));
			}
			//Serial.print(*(float *)(((uint8_t *)packet_buffer + packet_buffer_tail - 8)));
			
			packet_buffer_tail = 0;
			return IMU_DATA_RECEIVED;
		}
		//Serial.println("test4");
	}
	
	return 0;
}
void BBB::default_handler(uint32_t command_id, uint32_t command_option){
	Serial.println("Received Ground Command!");
	Serial.print("Command ID: ");
	Serial.println(command_id,HEX);
	Serial.print("Command OPT: ");
	Serial.println(command_option,HEX);
	Serial.println("No packet handler defined!");
	
}
