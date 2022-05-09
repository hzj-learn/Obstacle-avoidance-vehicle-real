#ifndef __SUPERX_II_H__
#define __SUPERX_II_H__
#include <stdint.h>
// #include "com.h"
#include "driver/uart/uart.h"

#define SUPERX_II_API_VERSION 1.4

/*
 * version note:
 * v1.3 message (command 06) supports 512 bytes message
 * v1.4 update heartbeat on get/set data returned from FC
*/

#define get_module_no(module_type, module_index) (module_type << 0x03 | module_index)
#define VERSION(ver, sub, biuld) (((uint32_t)ver << 24)|((uint32_t)sub << 16)|(biuld & 0xFFFF))
#define MODULE_STATUS_OFFLINE 0
#define MODULE_STATUS_ONLINE  1

#ifdef __cplusplus
extern "C" {
#endif
extern int get_data_size_;
extern	int superx_fd_uart;
extern uint32_t fccom_bautrate;
typedef enum
{
    MODULE_TYPE_FC = 0x01,
    MODULE_TYPE_AHRS = 0x02,
    MODULE_TYPE_GPS = 0x03,
    MODULE_TYPE_OSD = 0x04,
    MODULE_TYPE_XLINK = 0x05,
    MODULE_TYPE_BLACKBOX = 0x07,
    MODULE_TYPE_DATALINK = 0x08,
    MODULE_TYPE_RC = 0x09,
    MODULE_TYPE_SPRAY = 0x0A,
    MODULE_TYPE_SONAR = 0x0B,
    MODULE_TYPE_BATTERY = 0x0C,
    MODULE_TYPE_TOF = 0x0D,
    MODULE_TYPE_CAMERA = 0x0E,
    MODULE_TYPE_P20LED = 0x10,
    MODULE_TYPE_NFC = 0x11,
    MODULE_TYPE_OA  = 0x15,
    MODULE_TYPE_DEBUGGER = 0x0F
} API_ModuleType_t;

typedef struct {
    uint8_t From;       //���0�4�0�9�0�1�0�3�0�4�0�3�0�4�0�8�0�2�0�8���0�2���0�5�0�1From�0�2�0�9�0�2�0�5�0�7�����0�8�0�3�0�2�0�5�0�1�0�7�0�7�0�6�0�8�0�5�0�3�0�0�0�6�0�4�0�5
    uint8_t To;         //�0�4�0�7�0�8�0�9�0�3�0�4�0�3�0�4�0�8�0�2�0�8���0�2���0�5�0�1To�0�2�0�9�0�2�0�5�0�7�����0�8�0�3�0�2
    uint16_t Length;
    uint8_t * Data;
    uint8_t version;
} API_Message_t;

typedef struct {
    uint32_t ModuleID[3];
    uint8_t ModuleType;
    uint8_t ModuleIndex;		    //�0�7�0�7API�0�7���0�0�0�1
    uint8_t ModuleNo;			    //�0�2�0�5�0�7�����0�8�0�9�0�4���0�4�0�5���0�6�0�7�0�1�0�7�0�8���0�7���0�0�0�1
    uint8_t Status;                 //MODULE_STATUS_ONLINE, MODULE_STATUS_OFFLINE
    uint32_t HardwareVersion;       //�0�7�0�5�0�3�0�6���0�3���0�0�0�4�0�3�0�3�0�4
    uint32_t SoftwareVersion;       //�0�6���0�3�0�6���0�3���0�0�0�4�0�3�0�3�0�4
    uint16_t DataSize;		        //�0�8�0�5�0�0�0�6�0�5�0�3�0�5���0�4�0�3
    uint16_t ModuleTimeout;	        //�0�7���0�0�0�1�0�2�0�5�0�7���0�6�0�1�0�8���0�8���0�3�0�1
    uint8_t  bautrate;              // 6--921600  5--256000 4--115200
    uint8_t  index;
} API_Config_t;


void api_init(API_Config_t * config);

void api_loop(uint32_t time);

void api_get_data(int32_t offset, int32_t len);
void api_set_data(int32_t offset, int32_t len, void * data, uint8_t no_ack);
void api_send_message(API_Message_t *msg);

void api_port_received(int32_t len, uint8_t * data);

void api_port_send(int32_t len, uint8_t * data); //weak
void api_on_data_set(uint8_t result); //weak, 0 - successful, 1 - failed
void api_on_data_returned(int32_t offset, int32_t len, uint8_t * data); //weak
void api_on_modulelist_received(uint8_t * data);
void api_on_message_received(API_Message_t message); //weak
void api_on_upgrade(void);
void api_get_modulelist(void);
void api_auto_load_data(uint16_t offset, uint16_t len, uint16_t enabled, uint16_t interval);
void api_on_configurate_baudrate(int fd, int baudrate);

#define CRC16_INIT_VALUE 0xFFFF
typedef struct {
    uint16_t current; //termporary value during crc16 calculation,original value is 0xffff
    uint16_t crc16; //the crc16 value
} crc16_t;

void crc16(uint8_t input, crc16_t * output);
uint16_t crc_ccitt(uint8_t * input, int len);

#ifdef __cplusplus
}
#endif

#endif  /* endif __SUPERX_II_H__ */

