/******************************************************************************
SparkFunESP8266WiFi.h
ESP8266 WiFi Shield Library Main Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 20, 2015
http://github.com/sparkfun/SparkFun_ESP8266_AT_Arduino_Library

!!! Description Here !!!

Development environment specifics:
	IDE: Arduino 1.6.5
	Hardware Platform: Arduino Uno
	ESP8266 WiFi Shield Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef _SPARKFUNESP8266_H_
#define _SPARKFUNESP8266_H_

#include <Arduino.h>
#include <IPAddress.h>
#include "ESP8266_defines.h"

enum ESPState {ESP_IDLE = 0, ESP_CMD, ESP_COMM};

class ESP8266Class : public Stream
{
protected:
    Stream* _serial;
    
    int16_t tcpRecvCount = 0; //for tcpReceive(); must be signed to accommodate error codes
    esp8266_rec_state recState = ESP8266_REC_CLOSED;
    
    ////////////////////////
    // Buffer Definitions //
    ////////////////////////
    char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];
    uint16_t bufferHead = 0; // Holds position of latest byte placed in buffer.
        
public:
	ESP8266Class();
	
    bool begin(Stream* ser);

	///////////////////////
	// Basic AT Commands //
	///////////////////////
	bool test();
	bool reset();
	int16_t getVersion(char * ATversion, char * SDKversion, char * compileTime);
	bool echo(bool enable);
	
	////////////////////
	// WiFi Functions //
	////////////////////
	int16_t getMode();
	int16_t setMode(esp8266_wifi_mode mode);
	int16_t setMode(int8_t mode);
	int16_t connect(const char * ssid, const char * pwd = "");
	int16_t getAP(char * ssid);
	int16_t localMAC(char * mac);
	int16_t disconnect();
	IPAddress localIP();
	
	/////////////////////
	// TCP/IP Commands //
	/////////////////////    
	int16_t status();
	int16_t updateStatus();
	int16_t tcpConnect(const char * destination, uint16_t port, uint16_t keepAlive = 0);
	int16_t tcpSend(const char *buf, size_t size);
    int16_t tcpReceive(char* buffer, uint16_t buffer_len, uint32_t timeout = TCP_RECEIVE_TIMEOUT);

    uint8_t connected(void);
	int16_t close(void);
	int16_t setTransferMode(uint8_t mode);
	int16_t setMux(uint8_t mux);
	int16_t configureTCPServer(uint16_t port, uint8_t create = 1);
	int16_t ping(IPAddress ip);
	int16_t ping(char * server);
		
	//////////////////////////
	// Custom GPIO Commands //
	//////////////////////////
	int16_t pinMode(uint8_t pin, uint8_t mode);
	int16_t digitalWrite(uint8_t pin, uint8_t state);
	int8_t digitalRead(uint8_t pin);
	
	///////////////////////////////////
	// Virtual Functions from Stream //
	///////////////////////////////////
	size_t write(uint8_t);
	int available();
	int read();
	int peek();
	void flush();
	
	friend class ESP8266Client;
	friend class ESP8266Server;

private:
	//////////////////////////
	// Command Send/Receive //
	//////////////////////////
	void sendCommand(const char * cmd, enum esp8266_command_type type = ESP8266_CMD_EXECUTE, const char * params = NULL);
	int16_t readForResponse(const char * rsp, unsigned int timeout);
	int16_t readForResponses(const char * pass, const char * fail, unsigned int timeout);
	
	//////////////////
	// Buffer Stuff // 
	//////////////////
	/// clearBuffer() - Reset buffer pointer, set all values to 0
	void clearBuffer();
	
	/// readByteToBuffer() - Read first byte from UART receive buffer
	/// and store it in rxBuffer.
    char readByteToBuffer(int8_t&);
    uint8_t readByteToBuffer();
    
	/// searchBuffer([test]) - Search buffer for string [test]
	/// Success: Returns pointer to beginning of string
	/// Fail: returns NULL
	//! TODO: Fix this function so it searches circularly
	char * searchBuffer(const char * test);
	
	esp8266_status _status;
};

extern ESP8266Class esp8266;

#endif
