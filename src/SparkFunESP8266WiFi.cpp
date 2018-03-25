/******************************************************************************
SparkFunESP8266WiFi.cpp
ESP8266 WiFi Shield Library Main Source File
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

#include <SparkFunESP8266WiFi.h>
#include <Arduino.h>
#include "util/ESP8266_AT.h"

////////////////////
// Initialization //
////////////////////

ESP8266Class::ESP8266Class()
{}

//up to the user to start the Serial of his or her choice
bool ESP8266Class::begin(Stream* ser)
{
    _serial = ser;
    
    if(!test())
    {
        print("+++"); //chance we're stuck in TCPSEND transparent
        delay(1000);
        if(!test()) return false;
    }
    if(!setMux(0)) return false;    //disable mux -- just one cxn
    if(!echo(false)) return false;  //disable echo
    if(!setTransferMode(true)) return false;     //set to transparent mode

    return true;
}

///////////////////////
// Basic AT Commands //
///////////////////////
bool ESP8266Class::test()
{
	sendCommand(ESP8266_TEST); // Send AT

	if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0)
		return true;
	
	return false;
}

bool ESP8266Class::reset()
{
	sendCommand(ESP8266_RESET); // Send AT+RST
	
	if (readForResponse(RESPONSE_READY, COMMAND_RESET_TIMEOUT) > 0)
		return true;
	
	return false;
}

bool ESP8266Class::echo(bool enable)
{
	if (enable)
		sendCommand(ESP8266_ECHO_ENABLE);
	else
		sendCommand(ESP8266_ECHO_DISABLE);
	
	if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0)
		return true;
	
	return false;
}

int16_t ESP8266Class::getVersion(char * ATversion, char * SDKversion, char * compileTime)
{
	sendCommand(ESP8266_VERSION); // Send AT+GMR
	// Example Response: AT version:0.30.0.0(Jul  3 2015 19:35:49)\r\n (43 chars)
	//                   SDK version:1.2.0\r\n (19 chars)
	//                   compile time:Jul  7 2015 18:34:26\r\n (36 chars)
	//                   OK\r\n
	// (~101 characters)
	// Look for "OK":
	int16_t rsp = (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0);
	if (rsp > 0)
	{
		char *p, *q;
		// Look for "AT version" in the rxBuffer
		p = strstr(esp8266RxBuffer, "AT version:");
		if (p == NULL) return ESP8266_RSP_UNKNOWN;
		p += strlen("AT version:");
		q = strchr(p, '\r'); // Look for \r
		if (q == NULL) return ESP8266_RSP_UNKNOWN;
		strncpy(ATversion, p, q-p);
		
		// Look for "SDK version:" in the rxBuffer
		p = strstr(esp8266RxBuffer, "SDK version:");
		if (p == NULL) return ESP8266_RSP_UNKNOWN;
		p += strlen("SDK version:");
		q = strchr(p, '\r'); // Look for \r
		if (q == NULL) return ESP8266_RSP_UNKNOWN;
		strncpy(SDKversion, p, q-p);
		
		// Look for "compile time:" in the rxBuffer
		p = strstr(esp8266RxBuffer, "compile time:");
		if (p == NULL) return ESP8266_RSP_UNKNOWN;
		p += strlen("compile time:");
		q = strchr(p, '\r'); // Look for \r
		if (q == NULL) return ESP8266_RSP_UNKNOWN;
		strncpy(compileTime, p, q-p);
	}
	
	return rsp;
}

////////////////////
// WiFi Functions //
////////////////////

// getMode()
// Input: None
// Output:
//    - Success: 1, 2, 3 (ESP8266_MODE_STA, ESP8266_MODE_AP, ESP8266_MODE_STAAP)
//    - Fail: <0 (esp8266_cmd_rsp)
int16_t ESP8266Class::getMode()
{
	sendCommand(ESP8266_WIFI_MODE, ESP8266_CMD_QUERY);
	
	// Example response: \r\nAT+CWMODE_CUR?\r+CWMODE_CUR:2\r\n\r\nOK\r\n
	// Look for the OK:
	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	if (rsp > 0)
	{
		// Then get the number after ':':
		char * p = strchr(esp8266RxBuffer, ':');
		if (p != NULL)
		{
			char mode = *(p+1);
			if ((mode >= '1') && (mode <= '3'))
				return (mode - 48); // Convert ASCII to decimal
		}
		
		return ESP8266_RSP_UNKNOWN;
	}
	
	return rsp;
}

// setMode()
// Input: 1, 2, 3 (ESP8266_MODE_STA, ESP8266_MODE_AP, ESP8266_MODE_STAAP)
// Output:
//    - Success: >0
//    - Fail: <0 (esp8266_cmd_rsp)
int16_t ESP8266Class::setMode(esp8266_wifi_mode mode)
{
	char modeChar[2] = {0, 0};
	sprintf(modeChar, "%d", mode);
	sendCommand(ESP8266_WIFI_MODE, ESP8266_CMD_SETUP, modeChar);
	
	return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

// connect()
// Input: ssid and pwd const char's
// Output:
//    - Success: >0
//    - Fail: <0 (esp8266_cmd_rsp)
int16_t ESP8266Class::connect(const char * ssid, const char * pwd)
{
	print("AT");
	print(ESP8266_CONNECT_AP);
	print("=\"");
	print(ssid);
	print("\"");
	if (pwd != NULL)
	{
		print(',');
		print("\"");
		print(pwd);
		print("\"");
	}
	print("\r\n");
	
	return readForResponses(RESPONSE_OK, RESPONSE_FAIL, WIFI_CONNECT_TIMEOUT);
}

int16_t ESP8266Class::getAP(char * ssid)
{
	sendCommand(ESP8266_CONNECT_AP, ESP8266_CMD_QUERY); // Send "AT+CWJAP?"
	
	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	// Example Responses: No AP\r\n\r\nOK\r\n
	// - or -
	// +CWJAP:"WiFiSSID","00:aa:bb:cc:dd:ee",6,-45\r\n\r\nOK\r\n
	if (rsp > 0)
	{
		// Look for "No AP"
		if (strstr(esp8266RxBuffer, "No AP") != NULL)
			return 0;
		
		// Look for "+CWJAP"
		char * p = strstr(esp8266RxBuffer, ESP8266_CONNECT_AP);
		if (p != NULL)
		{
			p += strlen(ESP8266_CONNECT_AP) + 2;
			char * q = strchr(p, '"');
			if (q == NULL) return ESP8266_RSP_UNKNOWN;
			strncpy(ssid, p, q-p); // Copy string to temp char array:
			return 1;
		}
	}
	
	return rsp;
}

int16_t ESP8266Class::disconnect()
{
	sendCommand(ESP8266_DISCONNECT); // Send AT+CWQAP
	// Example response: \r\n\r\nOK\r\nWIFI DISCONNECT\r\n
	// "WIFI DISCONNECT" comes up to 500ms _after_ OK. 
	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	if (rsp > 0)
	{
		rsp = readForResponse("WIFI DISCONNECT", COMMAND_RESPONSE_TIMEOUT);
		if (rsp > 0)
			return rsp;
		return 1;
	}
	
	return rsp;
}

// status()
// Input: none
// Output:
//    - Success: 2, 3, 4, or 5 (ESP8266_STATUS_GOTIP, ESP8266_STATUS_CONNECTED, ESP8266_STATUS_DISCONNECTED, ESP8266_STATUS_NOWIFI)
//    - Fail: <0 (esp8266_cmd_rsp)
int16_t ESP8266Class::status()
{
	int16_t statusRet = updateStatus();
	if (statusRet > 0)
	{
        return _status.stat;
	}
	return statusRet;
}

int16_t ESP8266Class::updateStatus()
{
	sendCommand(ESP8266_TCP_STATUS); // Send AT+CIPSTATUS\r\n
	// Example response: (connected as client)
	// STATUS:3\r\n
	// +CIPSTATUS:0,"TCP","93.184.216.34",80,0\r\n\r\nOK\r\n 
	// - or - (clients connected to ESP8266 server)
	// STATUS:3\r\n
	// +CIPSTATUS:0,"TCP","192.168.0.100",54723,1\r\n
	// +CIPSTATUS:1,"TCP","192.168.0.101",54724,1\r\n\r\nOK\r\n 
	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	if (rsp > 0)
	{
		char * p = searchBuffer("STATUS:");
		if (p == NULL)
			return ESP8266_RSP_UNKNOWN;
		
		p += strlen("STATUS:");
		_status.stat = (esp8266_connect_status)(*p - 48);
		
//        for (int i=0; i<ESP8266_MAX_SOCK_NUM; i++)
//        {
//            p = strstr(p, "+CIPSTATUS:");
//            if (p == NULL)
//            {
//                // Didn't find any IPSTATUS'. Set linkID to 255.
//                for (int j=i; j<ESP8266_MAX_SOCK_NUM; j++)
//                    _status.ipstatus[j].linkID = 255;
//                return rsp;
//            }
//            else
//            {
//                p += strlen("+CIPSTATUS:");
//                // Find linkID:
//                uint8_t linkId = *p - 48;
//                if (linkId >= ESP8266_MAX_SOCK_NUM)
//                    return rsp;
//                _status.ipstatus[linkId].linkID = linkId;
//                
//                // Find type (p pointing at linkID):
//                p += 3; // Move p to either "T" or "U"
//                if (*p == 'T')
//                    _status.ipstatus[linkId].type = ESP8266_TCP;
//                else if (*p == 'U')
//                    _status.ipstatus[linkId].type = ESP8266_UDP;
//                else
//                    _status.ipstatus[linkId].type = ESP8266_TYPE_UNDEFINED;
//                
//                // Find remoteIP (p pointing at first letter or type):
//                p += 6; // Move p to first digit of first octet.
//                for (uint8_t j = 0; j < 4; j++)
//                {
//                    char tempOctet[4];
//                    memset(tempOctet, 0, 4); // Clear tempOctet
//                    
//                    size_t octetLength = strspn(p, "0123456789"); // Find length of numerical string:
//                    
//                    strncpy(tempOctet, p, octetLength); // Copy string to temp char array:
//                    _status.ipstatus[linkId].remoteIP[j] = atoi(tempOctet); // Move the temp char into IP Address octet
//                    
//                    p += (octetLength + 1); // Increment p to next octet
//                }
//                
//                // Find port (p pointing at ',' between IP and port:
//                p += 1; // Move p to first digit of port
//                char tempPort[6];
//                memset(tempPort, 0, 6);
//                size_t portLen = strspn(p, "0123456789"); // Find length of numerical string:
//                strncpy(tempPort, p, portLen);
//                _status.ipstatus[linkId].port = atoi(tempPort);
//                p += portLen + 1;
//                
//                // Find tetype (p pointing at tetype)
//                if (*p == '0')
//                    _status.ipstatus[linkId].tetype = ESP8266_CLIENT;
//                else if (*p == '1')
//                    _status.ipstatus[linkId].tetype = ESP8266_SERVER;
//            }
//        }
	}
	
	return rsp;
}

// localIP()
// Input: none
// Output:
//    - Success: Device's local IPAddress
//    - Fail: 0
IPAddress ESP8266Class::localIP()
{
	sendCommand(ESP8266_GET_LOCAL_IP); // Send AT+CIFSR\r\n
	// Example Response: +CIFSR:STAIP,"192.168.0.114"\r\n
	//                   +CIFSR:STAMAC,"18:fe:34:9d:b7:d9"\r\n
	//                   \r\n
	//                   OK\r\n
	// Look for the OK:
	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	if (rsp > 0)
	{
		// Look for "STAIP" in the rxBuffer
		char * p = strstr(esp8266RxBuffer, "STAIP");
		if (p != NULL)
		{
			IPAddress returnIP;
			
			p += 7; // Move p seven places. (skip STAIP,")
			for (uint8_t i = 0; i < 4; i++)
			{
				char tempOctet[4];
				memset(tempOctet, 0, 4); // Clear tempOctet
				
				size_t octetLength = strspn(p, "0123456789"); // Find length of numerical string:
				if (octetLength >= 4) // If it's too big, return an error
					return ESP8266_RSP_UNKNOWN;
				
				strncpy(tempOctet, p, octetLength); // Copy string to temp char array:
				returnIP[i] = atoi(tempOctet); // Move the temp char into IP Address octet
				
				p += (octetLength + 1); // Increment p to next octet
			}
			
			return returnIP;
		}
	}
	
	return rsp;
}

int16_t ESP8266Class::localMAC(char * mac)
{
	sendCommand(ESP8266_GET_STA_MAC, ESP8266_CMD_QUERY); // Send "AT+CIPSTAMAC?"

	int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

	if (rsp > 0)
	{
		// Look for "+CIPSTAMAC"
		char * p = strstr(esp8266RxBuffer, ESP8266_GET_STA_MAC);
		if (p != NULL)
		{
			p += strlen(ESP8266_GET_STA_MAC) + 2;
			char * q = strchr(p, '"');
			if (q == NULL) return ESP8266_RSP_UNKNOWN;
			strncpy(mac, p, q - p); // Copy string to temp char array:
			return 1;
		}
	}

	return rsp;
}

/////////////////////
// TCP/IP Commands //
/////////////////////
int16_t ESP8266Class::tcpConnect(const char * destination, uint16_t port, uint16_t keepAlive)
{
	print("AT");
	print(ESP8266_TCP_CONNECT);
    print('=');
//    print(linkID);
//    print(',');
	print("\"TCP\",");
	print("\"");
	print(destination);
	print("\",");
	print(port);
	if (keepAlive > 0)
	{
		print(',');
		// keepAlive is in units of 500 milliseconds.
		// Max is 7200 * 500 = 3600000 ms = 60 minutes.
		print(keepAlive / 500);
	}
	print("\r\n");
	// Example good: CONNECT\r\n\r\nOK\r\n
	// Example bad: DNS Fail\r\n\r\nERROR\r\n
	// Example meh: ALREADY CONNECTED\r\n\r\nERROR\r\n
	int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, CLIENT_CONNECT_TIMEOUT);
	
	if (rsp < 0)
	{
		// We may see "ERROR", but be "ALREADY CONNECTED".
		// Search for "ALREADY", and return success if we see it.
		char * p = searchBuffer("ALREADY");
		if (p != NULL)
			return 2;
		// Otherwise the connection failed. Return the error code:
		return rsp;
	}
	// Return 1 on successful (new) connection
	return 1;
}

int16_t ESP8266Class::tcpSend(const char* buf, size_t size)
{
    //need to be put in a better spot...
    tcpRecvCount = 0;
    recState = ESP8266_REC_WAITING;
    
    if (size > 2048)
        return ESP8266_CMD_BAD;

    sendCommand(ESP8266_TCP_SEND);
    delay(50);
    print((const char *)buf);
    delay(50);
    print("+++");
    
    return 1;
}

/*
 * tcpReceive captures input line-by-line
 */
int16_t ESP8266Class::tcpReceive(char* buffer, uint16_t buffer_len, uint32_t timeout)
{
    clearBuffer();
    uint32_t lastRead = millis();
    
    while(millis() - lastRead < timeout)
    {
        if(_serial->available())
        {
            //read byte and check for overflow
            int8_t errorCode = 0;
            char c = readByteToBuffer(errorCode);
            if(errorCode == ESP8266_BUF_OVF)
            {
                return ESP8266_BUF_OVF;
            }

            lastRead = millis();

            if(c == '\n')
            {
                //return whatever we have, which could be as little as \r\n
                //if there is nothing else, it'll timeout next time through
                memset(buffer, '\0', buffer_len);
                memcpy(buffer, esp8266RxBuffer, strlen(esp8266RxBuffer)); //CAREFUL on length!!!
                
                return strlen(esp8266RxBuffer);
            }
        }
    }
    
    return ESP8266_RSP_TIMEOUT;
}

uint8_t ESP8266Class::connected(void) //tcp cxn, not wifi
{
    return (status() == ESP8266_STATUS_CONNECTED);
}

int16_t ESP8266Class::close(void)
{
	sendCommand(ESP8266_TCP_CLOSE);
	
	// Eh, client virtual function doesn't have a return value.
	// We'll wait for the OK or timeout anyway.
	return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

int16_t ESP8266Class::setTransferMode(uint8_t mode)
{
	char params[2] = {0, 0};
	params[0] = (mode > 0) ? '1' : '0';
	sendCommand(ESP8266_TRANSMISSION_MODE, ESP8266_CMD_SETUP, params);
	
	return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

int16_t ESP8266Class::setMux(uint8_t mux)
{
	char params[2] = {0, 0};
	params[0] = (mux > 0) ? '1' : '0';
	sendCommand(ESP8266_TCP_MULTIPLE, ESP8266_CMD_SETUP, params);
	
	return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

//////////////////////////////
// Stream Virtual Functions //
//////////////////////////////

size_t ESP8266Class::write(uint8_t c)
{
    //SerialUSB.print((char)c);
	return _serial->write(c);
}

int ESP8266Class::available()
{
	return _serial->available();
}

int ESP8266Class::read()
{
    int ch = _serial->read();
    //SerialUSB.print((char)ch);
    return ch;
}

int ESP8266Class::peek()
{
	return _serial->peek();
}

void ESP8266Class::flush()
{
	_serial->flush();
}

//////////////////////////////////////////////////
// Private, Low-Level, Ugly, Hardware Functions //
//////////////////////////////////////////////////

void ESP8266Class::sendCommand(const char * cmd, enum esp8266_command_type type, const char * params)
{
	print("AT");
	print(cmd);
	if (type == ESP8266_CMD_QUERY)
		print('?');
	else if (type == ESP8266_CMD_SETUP)
	{
		print("=");
		print(params);
	}
	print("\r\n");
}

int16_t ESP8266Class::readForResponse(const char * rsp, unsigned int timeout)
{
	unsigned long lastRead = millis();	// Timestamp coming into function
	unsigned int received = 0; // received keeps track of number of chars read
	
	clearBuffer();	// Clear the class receive buffer (esp8266RxBuffer)
	while (millis() - lastRead < timeout) // While we haven't timed out
	{
		if (_serial->available()) // If data is available on UART RX
		{
            lastRead = millis();
			received += readByteToBuffer();
			if (searchBuffer(rsp))	// Search the buffer for goodRsp
				return received;	// Return how number of chars read
		}
	}
	
	if (received > 0) // If we received any characters
    {
		return ESP8266_RSP_UNKNOWN; // Return unkown response error code
    }
	else // If we haven't received any characters
		return ESP8266_RSP_TIMEOUT; // Return the timeout error code
}

int16_t ESP8266Class::readForResponses(const char * pass, const char * fail, unsigned int timeout)
{
	unsigned long timeIn = millis();	// Timestamp coming into function
	unsigned int received = 0; // received keeps track of number of chars read
	
	clearBuffer();	// Clear the class receive buffer (esp8266RxBuffer)
	while (timeIn + timeout > millis()) // While we haven't timed out
	{
		if (_serial->available()) // If data is available on UART RX
		{
			received += readByteToBuffer();
            
            //this is painful: searching the buffer every freakin' character?
			if (searchBuffer(pass))	// Search the buffer for goodRsp
				return received;	// Return how number of chars read
			if (searchBuffer(fail))
				return ESP8266_RSP_FAIL;
		}
	}
	
	if (received > 0) // If we received any characters
		return ESP8266_RSP_UNKNOWN; // Return unkown response error code
	else // If we haven't received any characters
		return ESP8266_RSP_TIMEOUT; // Return the timeout error code
}

//////////////////
// Buffer Stuff //
//////////////////
void ESP8266Class::clearBuffer()
{
	memset(esp8266RxBuffer, '\0', ESP8266_RX_BUFFER_LEN);
	bufferHead = 0;
}	

//maybe try: char readByte(int8_t& error): return c; set error if needed
//would need to find all calss to readByte, as this breaks some of the calls

char ESP8266Class::readByteToBuffer(int8_t& errorCode) //add char*?
{
    // Read the data in
    char c = read();
    
    // Store the data in the buffer
    esp8266RxBuffer[bufferHead++] = c;
    
    if(bufferHead == ESP8266_RX_BUFFER_LEN - 1)
    {
        bufferHead--; //reached overflow; all new data will just overwrites at the end
        errorCode = ESP8266_BUF_OVF; //let the caller know
    }
    errorCode = 0;
    
    return c;
}

uint8_t ESP8266Class::readByteToBuffer(void) //add char*?
{
    // Read the data in
    char c = read();
    
    // Store the data in the buffer
    esp8266RxBuffer[bufferHead++] = c;
    
    if(bufferHead == ESP8266_RX_BUFFER_LEN - 1)
    {
        bufferHead--; //reached overflow; all new data will just overwrites at the end
    }
    
    return 1;
}

char * ESP8266Class::searchBuffer(const char * test)
{
    return strstr(esp8266RxBuffer, test); //gcl: removed (const char*) cast -- not sure why it was needed
}

ESP8266Class esp8266;
