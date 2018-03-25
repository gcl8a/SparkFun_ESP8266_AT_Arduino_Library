//
//  ESP8266_defines.h
//  
//
//  Created by Gregory C Lewin on 3/24/18.
//

#ifndef ESP8266_defines_h
#define ESP8266_defines_h


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
    uint8_t linkID = ESP8266_SOCK_NOT_AVAIL; //this is going to be socket#
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


#endif /* ESP8266_defines_h */
