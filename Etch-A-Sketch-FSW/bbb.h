#ifndef __BBB_H__
#define __BBB_H__

#include <Arduino.h>
#include <SoftwareSerial.h>
//#include "packet_handler.h"

#define PAYLOAD_RF_PREAMBLE 0xDFDFDFDF
#define PAYLOAD_RF_SUFFIX 0xDEADBEEF
//to payload
#define PAYLOAD_CMD_PREAMBLE 0x1F1F1F1F
#define PAYLOAD_CMD_SUFFIX 0x1E1E1E1E
#define PAYLOAD_IMU_SUFFIX 0x2E2E2E2E

#define PAYLOAD_SD_PREAMBLE (0xDEDEDEDE)
#define PAYLOAD_SD_SUFFIX 	(0xBEEFDEAD)

#define CMD_PACKET_PREAMBLE 0xCECECECE
#define CMD_PACKET_SUFFIX 0xCFCFCFCF

#define LITTLE_ENDIAN
//#define BIG_ENDIAN

/* List of BBB Commands */
#define BBB_CMD_BASE (0x100)
#define SEND_BBB_ID (BBB_CMD_BASE+6)
#define SEND_IMU_DATA (BBB_CMD_BASE+7)

#define PACKET_BUFFER_SIZE (50)

typedef enum{
  NO_STATUS = 0,
  BBB_ID_RECEIVED,
  CMD_RECEIVED,
  IMU_DATA_RECEIVED,
} packet_type;

typedef struct{
	uint32_t preamble;
	uint32_t time;
	float accel_x_mss;
	float accel_y_mss;
	float accel_z_mss;
	float gryo_x_rs;
	float gryo_y_rs;
	float gryo_z_rs;
	float mag_x_t;
	float mag_y_t;
	float mag_z_t;
	uint32_t suffix;
}IMUData;

typedef struct{
	uint32_t preamble;
	uint32_t BBB_id;
	uint32_t command;
	uint32_t command_option;
	uint32_t suffix;
}CMDPacket;




class BBB{
	private:
	uint8_t serial_type; //0 - hardware, 1- software
	//uint32_t BBB_id;
	//IMUData imu_packet_data;
	uint8_t rf_packet[256];
	uint8_t rf_packet_tail;
	//HardwareSerial * serial_port;
	void * serial_port;
	uint8_t save_pin;
	uint8_t low_bat_pin;
	uint8_t launch_pin;
	uint8_t deploy_pin;
	uint8_t imu_wake_pin;
	uint8_t i2c_hijack_pin;
	uint8_t i2c_state;
	const uint32_t packet_buffer_size = 50;
	uint32_t packet_buffer_tail = 0;
	uint8_t packet_buffer[PACKET_BUFFER_SIZE];
	
	void (*cmd_cb)(uint32_t , uint32_t);
	
	public:
	uint32_t BBB_id;
	IMUData imu_packet_data;
	
	uint8_t init(HardwareSerial * bbb_port, int baud_rate);
	uint8_t init(SoftwareSerial * bbb_port, int baud_rate);
	void setSavePin(uint8_t pin);
	void setLowBatPin(uint8_t pin);
	void setLaunchPin(uint8_t pin);
	void setDepPin(uint8_t pin);
	void setImu_wakePin(uint8_t pin);
	void setI2cHijPin(uint8_t pin);
	
	void startRFPacket();
	void addRFData(uint8_t data);
	void addRFData(uint16_t data);
	void addRFData(uint32_t data);
	void addRFData(int32_t data);
	void addRFData(uint64_t data);
	void addRFData(float data);
	void addRFData(double data);
	uint8_t sendRFPacket();
	uint8_t sendRFPacket(uint8_t * packet, size_t packet_length);
	uint8_t sendRFPacketWrap(uint8_t * data, size_t data_length);

	void sendBBBCommand(uint32_t command, uint32_t option);
	void saveDataToBBB(uint8_t * data, uint8_t length);

	uint32_t requestBBBId();
	void requestBBBIMUData();
	void enableI2COverride();
	void disableI2COverride();
	void getIMUDataDirect(IMUData * imu_data);
	
	uint8_t setHeater1SetPoint(float set_point_temp);
	uint8_t setHeater2SetPoint(float set_point_temp);
	/* 
		Request the BBB to turn off the payload for X number of seconds.
	*/
	uint8_t setPayloadTurnOffTime(uint32_t seconds);

	void setCommandCallback(void (*func)(uint32_t , uint32_t));
	
	uint8_t refresh();
	//uint8_t refresh(uint8_t byte);
	uint8_t packet_put(uint8_t data);
	void default_handler(uint32_t command_id, uint32_t command_option);
	uint8_t waitForCMDResponse(CMDPacket * packet);
};
#endif
