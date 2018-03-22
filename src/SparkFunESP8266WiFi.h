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
#include "SparkFunESP8266Client.h"
#include "SparkFunESP8266Server.h"


///////////////////////////////
// Command Response Timeouts //
///////////////////////////////
#define COMMAND_RESPONSE_TIMEOUT 1000
#define COMMAND_PING_TIMEOUT 3000
#define WIFI_CONNECT_TIMEOUT 10000
#define COMMAND_RESET_TIMEOUT 5000
#define CLIENT_CONNECT_TIMEOUT 3000
#define TCP_RECEIVE_TIMEOUT 500

#define ESP8266_RX_BUFFER_LEN 128 // Number of bytes in the serial receive buffer

#define ESP8266_MAX_SOCK_NUM 5
#define ESP8266_SOCK_NOT_AVAIL 255

enum esp8266_cmd_rsp {
    ESP8266_BUF_OVF = -6,
    ESP8266_CMD_BAD = -5,
	ESP8266_RSP_MEMORY_ERR = -4,
	ESP8266_RSP_FAIL = -3,
	ESP8266_RSP_UNKNOWN = -2,
	ESP8266_RSP_TIMEOUT = -1,
	ESP8266_RSP_SUCCESS = 0
};

enum esp8266_wifi_mode {
	ESP8266_MODE_STA = 1,
	ESP8266_MODE_AP = 2,
	ESP8266_MODE_STAAP = 3
};

enum esp8266_command_type {
	ESP8266_CMD_QUERY,
	ESP8266_CMD_SETUP,
	ESP8266_CMD_EXECUTE
};

enum esp8266_encryption {
	ESP8266_ECN_OPEN,
	ESP8266_ECN_WPA_PSK,
	ESP8266_ECN_WPA2_PSK,
	ESP8266_ECN_WPA_WPA2_PSK
};

enum esp8266_connect_status {
	ESP8266_STATUS_GOTIP = 2,
	ESP8266_STATUS_CONNECTED = 3,
	ESP8266_STATUS_DISCONNECTED = 4,
	ESP8266_STATUS_NOWIFI = 5	
};

enum esp8266_serial_port {
	ESP8266_SOFTWARE_SERIAL,
	ESP8266_HARDWARE_SERIAL
};

enum esp8266_socket_state {
	AVAILABLE = 0,
	TAKEN = 1,
};

enum esp8266_connection_type {
	ESP8266_TCP,
	ESP8266_UDP,
	ESP8266_TYPE_UNDEFINED
};

enum esp8266_tetype {
	ESP8266_CLIENT,
	ESP8266_SERVER
};

enum esp8266_rec_state
{
    ESP8266_REC_CLOSED,
    ESP8266_REC_WAITING,
    ESP8266_REC_IPD,
    ESP8266_REC_RECEIVING
};

struct esp8266_ipstatus
{
	uint8_t linkID;
	esp8266_connection_type type;
	IPAddress remoteIP;
	uint16_t port;
	esp8266_tetype tetype;
};

struct esp8266_status
{
	esp8266_connect_status stat;
	esp8266_ipstatus ipstatus[ESP8266_MAX_SOCK_NUM];
};

class ESP8266Class : public Stream
{
protected:
    int16_t tcpRecvCount = 0; //for tcpReceive(); must be signed to accommodate error codes
    esp8266_rec_state recState = ESP8266_REC_CLOSED;
    
    ////////////////////////
    // Buffer Definitions //
    ////////////////////////
    char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];
    uint16_t bufferHead = 0; // Holds position of latest byte placed in buffer.
    
    int16_t _state[ESP8266_MAX_SOCK_NUM];
    
public:
	ESP8266Class();
	
    bool begin(unsigned long baudRate, Stream* ser);

	///////////////////////
	// Basic AT Commands //
	///////////////////////
	bool test();
	bool reset();
	int16_t getVersion(char * ATversion, char * SDKversion, char * compileTime);
	bool echo(bool enable);
	bool setBaud(unsigned long baud);
	
	////////////////////
	// WiFi Functions //
	////////////////////
	int16_t getMode();
	int16_t setMode(esp8266_wifi_mode mode);
	int16_t setMode(int8_t mode);
	int16_t connect(const char * ssid);
	int16_t connect(const char * ssid, const char * pwd);
	int16_t getAP(char * ssid);
	int16_t localMAC(char * mac);
	int16_t disconnect();
	IPAddress localIP();
	
	/////////////////////
	// TCP/IP Commands //
	/////////////////////
	int16_t status();
	int16_t updateStatus();
	int16_t tcpConnect(uint8_t linkID, const char * destination, uint16_t port, uint16_t keepAlive);
	int16_t tcpSend(uint8_t linkID, const uint8_t *buf, size_t size);
    int16_t tcpReceive(uint8_t linkID, char* buffer, uint16_t buffer_len, uint32_t timeout = TCP_RECEIVE_TIMEOUT);

	int16_t close(uint8_t linkID);
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

protected:
    Stream* _serial;
	unsigned long _baud;
    
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
	
	uint8_t sync();
};

extern ESP8266Class esp8266;

#endif
