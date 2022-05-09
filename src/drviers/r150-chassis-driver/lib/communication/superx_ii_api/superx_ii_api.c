#include "superx_ii_api/superx_ii_api.h"
// #include "uart.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// #include "config/config.h"
// #include "queue.h"

/* Compiler Related Definitions */
/*#ifdef __CC_ARM                         // ARM Compiler 
#include <stdarg.h>
#define ALIGN(n)                    __attribute__((aligned(n)))
#define WEAK			            __weak

#elif defined (__IAR_SYSTEMS_ICC__)     // for IAR Compiler 
#include <stdarg.h>
#define PRAGMA(x)                   _Pragma(#x)
#define ALIGN(n)                    PRAGMA(data_alignment=n)
#define WEAK                        __weak
#endif*/

//#define STATUS_INIT  0x01
//#define STATUS_REGISTER 0x02
//#define STATUS_TIMEOUT 0x03

// frame state
#define FRAME_SYNC     0
#define FRAME_HEADER   1
#define FRAME_DATA     2
extern int fd_uart;
int superx_fd_uart = 0;
uint32_t fccom_bautrate = 115200;
// int set_auto_load_imu = 0;
// queue_t write_que;
// extern int need_write_touart_len;

int get_data_size_;

enum Command
{
    CMD_NOMODULE = 0x00,
    CMD_REGISTER = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_GET_MODULE = 0x03,
    CMD_GET_DATA = 0x04,
    CMD_SET_DATA = 0x05,
    CMD_SEND_MSG = 0x06,
    CMD_AUTO_LOAD_DATA = 0x07,
    CMD_UPGRADE = 0xf0,
    CMD_UPGRADE_REQUEST = 0xf1,
    CMD_UPGRADE_PK = 0xf2,
    CMD_UPGRADE_FINISH = 0xf3,
};

#define FRAME_BUFFER_SIZE       (512+8+4) //len(2), cmd(1), to(1)

typedef struct
{
    uint8_t Start;
    uint8_t Checksum;
    uint16_t Length;
    uint8_t From;
    uint8_t To;
    uint8_t CMD;
} header_t;

typedef struct
{
    uint16_t Checksum;
    uint16_t Length;
    uint8_t From;
    uint8_t To;
    uint8_t CMD;
} header_v2_t;

typedef struct
{
    union
    {
        uint8_t RawData[FRAME_BUFFER_SIZE];
        header_t Header;
        header_v2_t HeaderV2;
    } DataFrame;
    uint16_t Count;
    uint8_t Status;
    uint8_t Version;
    uint16_t Checksum;
    crc16_t crc16;
} DataFrameBuffer_t;


static DataFrameBuffer_t Recv_Buffer;
static API_Config_t *api_config = NULL;

static uint8_t module_key = 0;
static uint32_t heartbeat = 0;
static uint32_t api_time = 0;

static uint8_t SendBuffer[FRAME_BUFFER_SIZE];

static void XorData(uint8_t *data, uint16_t length, uint8_t key);
static uint32_t check_sum(uint8_t *data, uint16_t len);

static void api_tx_init(uint8_t from, uint8_t to, uint8_t cmd, uint16_t data_len)
{
    SendBuffer[0] = 0x5B;   //START
    SendBuffer[1] = 0x5B;   //START
    SendBuffer[4] = data_len;   //Length
    SendBuffer[5] = data_len >> 8;
    SendBuffer[6] = from;
    SendBuffer[7] = to;
    SendBuffer[8] = cmd;  //Command
    
    SendBuffer[9 + data_len] = 0x66; //END
    SendBuffer[10 + data_len] = 0x66; //END
}

static void api_tx_checksum(uint16_t data_len)
{
    uint16_t checksum = crc_ccitt(&SendBuffer[6], data_len + 3);
    SendBuffer[2] = checksum;
    SendBuffer[3] = checksum >> 8;
}

static void module_register(void)
{
    uint16_t msg_len = 25;
    if (api_config == NULL) return;
    api_tx_init(api_config->ModuleNo, 0x09, CMD_REGISTER, msg_len);
    memcpy(&SendBuffer[9], &api_config->ModuleID, 12);
    memcpy(&SendBuffer[21], &api_config->DataSize, 2);
    memcpy(&SendBuffer[23], &api_config->HardwareVersion, 4);
    memcpy(&SendBuffer[27], &api_config->SoftwareVersion, 4);
    memcpy(&SendBuffer[31], &api_config->ModuleTimeout, 2);
    memcpy(&SendBuffer[33] , &api_config->bautrate,1);
    api_tx_checksum(msg_len);
    // initQueue(&write_que);
 //   api_on_configurate_baudrate(superx_fd_uart , 115200);
    api_port_send(msg_len + 9 + 2, SendBuffer);
    // UartWrite(superx_fd_uart,SendBuffer,msg_len + 9 + 2,1);
}

uint8_t module_heartbeat(void)
{
    uint16_t msg_len = 0;
    api_tx_init(api_config->ModuleNo, 0x09, CMD_HEARTBEAT, msg_len);
    api_tx_checksum(msg_len);
    api_port_send(SendBuffer[4] + 9 + 2, SendBuffer);
    return 0;
}

void api_get_data(int32_t offset, int32_t len)
{
    uint16_t msg_len = 4;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
    api_tx_init(api_config->ModuleNo, 0x09, CMD_GET_DATA, msg_len);
    memcpy(&SendBuffer[9], &offset, 2);
    memcpy(&SendBuffer[11], &len, 2);
    api_tx_checksum(msg_len);
    api_port_send(msg_len + 9 + 2, SendBuffer);
    
    // if(!set_auto_load_imu)
    // {
    //     set_auto_load_imu=1;
    //     api_auto_load_data(sizeof(XrvioToFCStatusData)+sizeof(XrvioToFCEstimatorData)+sizeof(XrvioToFCControlData), sizeof(FCToXrvioIMUData) + 1, 1, 8);
    // }
}

void api_get_modulelist(void)
{
    SendBuffer[0] = 0x5A;   //START
    SendBuffer[2] = 0x00;
    SendBuffer[3] = 0x00;
    SendBuffer[4] = api_config->ModuleNo;
    SendBuffer[5] = 0x01;
    SendBuffer[6] = CMD_GET_MODULE;
    SendBuffer[1] = check_sum(&SendBuffer[4], SendBuffer[2] + 3);
    api_port_send(SendBuffer[2] + 7, SendBuffer);
}

void api_auto_load_data(uint16_t offset, uint16_t len, uint16_t enabled, uint16_t interval)
{
    uint16_t msg_len = 8, t = enabled;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
    api_tx_init(api_config->ModuleNo, 0x09, CMD_AUTO_LOAD_DATA, msg_len);
    memcpy(&SendBuffer[9], &offset, 2);
    memcpy(&SendBuffer[11], &len, 2);
    memcpy(&SendBuffer[13], &enabled, 2);
    memcpy(&SendBuffer[15], &interval, 2);
    api_tx_checksum(msg_len);
    //api_port_send(msg_len + 9 + 2, SendBuffer);
    UartWrite(superx_fd_uart,SendBuffer,msg_len + 9 + 2,1);
}

void api_set_data(int32_t offset, int32_t len, void * data, uint8_t no_ack)
{
    uint16_t msg_len = len + 4;
    if (len == 0 || api_config->Status == MODULE_STATUS_OFFLINE) return;
    if(no_ack)
        api_tx_init(api_config->ModuleNo, 0x09, CMD_SET_DATA | 0x80, msg_len);
    else
        api_tx_init(api_config->ModuleNo, 0x09, CMD_SET_DATA, msg_len);
    memcpy(&SendBuffer[9], &offset, 2);
    memcpy(&SendBuffer[11], &len, 2);
    memcpy(&SendBuffer[13], data, len);
    XorData(&SendBuffer[13], len, module_key);
    api_tx_checksum(msg_len);
    api_port_send(msg_len + 9 + 2, SendBuffer);
}

void api_send_message(API_Message_t *msg)
{
    uint16_t msg_len = msg->Length + 2;
    if (api_config->Status == MODULE_STATUS_OFFLINE) return;
    if(msg->version == 1)
    {//compatiblee with v1
        SendBuffer[0] = 0x5A;   //START
        SendBuffer[2] = msg_len;   //Length
        SendBuffer[3] = (uint8_t)(msg_len >> 8);
        SendBuffer[4] = api_config->ModuleNo;
        SendBuffer[5] = msg->To;
        SendBuffer[6] = CMD_SEND_MSG;  //Command
        SendBuffer[7] = msg->Length;
        SendBuffer[8] = msg->Length >> 0x08;
        memcpy(&SendBuffer[9], msg->Data, msg_len);
        SendBuffer[1] = check_sum(&SendBuffer[4], msg_len + 3);
        api_port_send(msg_len + 7, SendBuffer);
    }
    else
    {//v2 as default
        api_tx_init(api_config->ModuleNo, msg->To, CMD_SEND_MSG, msg_len);
        SendBuffer[9] = msg->Length;
        SendBuffer[10] = msg->Length >> 0x08;
        memcpy(&SendBuffer[11], msg->Data, msg->Length);
        api_tx_checksum(msg_len);
        api_port_send(msg_len + 9 + 2, SendBuffer);
    }
}

void api_on_configurate_baudrate(int fd, int baudrate)
{
	if(ConfigUartCom(fd,baudrate,8,1,'n')<0)
	{
		// printf("configure uart failed!");
		close_uart(fd);
		return ;
	}

}


static void module_response(void)
{
    uint8_t cmd = Recv_Buffer.DataFrame.Header.CMD;
    uint16_t length = Recv_Buffer.DataFrame.Header.Length;
    uint8_t *data = &Recv_Buffer.DataFrame.RawData[7];
	// printf("recv cmd = %d\n",cmd);
    switch (cmd)
    {
    case CMD_NOMODULE:
        api_config->Status = MODULE_STATUS_OFFLINE;
        break;
    case CMD_REGISTER:
        //uint32_t *id = (uint32_t*)&data[0];
        if (length == 13)
        {
            api_config->Status = MODULE_STATUS_ONLINE;
            //api_auto_load_data(sizeof(XrvioToFCStatusData)+sizeof(XrvioToFCEstimatorData)+sizeof(XrvioToFCControlData), sizeof(FCToXrvioIMUData), 1, 4);
			if(api_config->bautrate == 8)
			{
				printf("configure baudrate uart fd = %d,current baudrate=%d\n",superx_fd_uart,GetPortSpeed(superx_fd_uart));
				api_on_configurate_baudrate(superx_fd_uart, fccom_bautrate);
                // 暂时容忍你
				usleep(10000);
			}
            //module_key = 0x00;
            module_key = data[12];
            api_config->ModuleNo = Recv_Buffer.DataFrame.Header.To;
            api_config->ModuleType = api_config->ModuleNo >> 3;
            api_config->ModuleIndex = api_config->ModuleNo & 0x07;
            heartbeat = api_time;
        }
        break;
    case CMD_HEARTBEAT:
        heartbeat = api_time;
        break;
    case CMD_GET_MODULE:
        heartbeat = api_time;
        api_on_modulelist_received(data);
        break;
    case CMD_GET_DATA:
        heartbeat = api_time;
        if (length > 4)
        {
            uint16_t offset = 0, len = 0;
            memcpy(&offset, &data[0], 2);
            memcpy(&len, &data[2], 2);
            XorData(&data[4], len, module_key);
            api_on_data_returned(offset, len, &data[4]);
        }
        break;
    case CMD_SET_DATA:
        if(data[0]==0) return;
        heartbeat = api_time;
        api_on_data_set(1);
        break;
    case CMD_SEND_MSG:
    {
        API_Message_t msg;
        heartbeat = api_time;
        msg.From = Recv_Buffer.DataFrame.Header.From;
        msg.To = Recv_Buffer.DataFrame.Header.To;
        msg.Length = Recv_Buffer.DataFrame.Header.Length - 2;
        msg.Data = &Recv_Buffer.DataFrame.RawData[9];
        msg.version = Recv_Buffer.Version;
        api_on_message_received(msg);
    }
        break;
    case CMD_UPGRADE:
        api_on_upgrade();
        break;
    }
}

void api_init(API_Config_t * config)
{
    api_config = config;
    api_config->Status = MODULE_STATUS_OFFLINE;
    api_config->ModuleNo = api_config->ModuleType << 3;
}

void api_loop(uint32_t time)
{
    static uint32_t last_tick = 0;
    static uint16_t heartbeat_interval = 1000;
    api_time = time;
    if (api_config == NULL)return;
    if (api_config->Status == MODULE_STATUS_OFFLINE)
    {
        if (last_tick + 3000 < time || last_tick == 0)
        {
            last_tick = time;
            heartbeat_interval = api_config->ModuleTimeout <= 1500 ? 500 : api_config->ModuleTimeout / 3;
            module_register();
        }
        return;
    }

    //if ((last_tick + api_config->ModuleTimeout - 1000) <= time)
    if (time - last_tick >= heartbeat_interval)
    {
        last_tick = time;
        module_heartbeat();
    }
    else if (time - heartbeat >= api_config->ModuleTimeout)
    {
        api_config->Status = MODULE_STATUS_OFFLINE;
        api_config->ModuleIndex = 0;
        api_config->ModuleNo &= 0xF8;
    }

}


static uint8_t module_parse_frame(uint8_t data)
{
    static uint8_t v2Counter = 0;
    switch (Recv_Buffer.Status)
    {
        case FRAME_SYNC:
            if (data == 0x5A)
            {
                Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
                Recv_Buffer.Version = 1;
                Recv_Buffer.Status = FRAME_HEADER;
                break;
            }
            if (data == 0x5B)
            {
                if (++v2Counter == 2)
                {
                    Recv_Buffer.Version = 2;
                    Recv_Buffer.Status = FRAME_HEADER;
                    v2Counter = 0;
                }
                break;
            }
            goto API_FRAME_RESET;

        case FRAME_HEADER:
            Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
            if (Recv_Buffer.Count == 4)//sizeof(header_t))
            {
                if (Recv_Buffer.DataFrame.Header.Length <= (FRAME_BUFFER_SIZE - 7))
                {
                    Recv_Buffer.Checksum = 0;
                    Recv_Buffer.crc16.current = CRC16_INIT_VALUE;
                    Recv_Buffer.crc16.crc16 = CRC16_INIT_VALUE;
                    Recv_Buffer.Status = FRAME_DATA;
                }
                else    //\810\F45\B3\A4\B6\C8\D2\834\A74
                {
                    goto API_FRAME_RESET;
                }
            }
            break;

        case FRAME_DATA: //from to cmd & data
            Recv_Buffer.DataFrame.RawData[Recv_Buffer.Count++] = data;
            if (Recv_Buffer.Version == 1) Recv_Buffer.Checksum += data;
            else crc16(data, &Recv_Buffer.crc16);
            if (Recv_Buffer.Count >= Recv_Buffer.DataFrame.Header.Length + 7)
            {
                if ((Recv_Buffer.Version == 1 && (Recv_Buffer.Checksum & 0xFF) == Recv_Buffer.DataFrame.Header.Checksum) ||
                    (Recv_Buffer.Version == 2 && Recv_Buffer.crc16.crc16 == Recv_Buffer.DataFrame.HeaderV2.Checksum))
                {//\810\F45\CA\FD\BE\DD\A7\B5\D1\E9\810\C70\B9\FD
                    //\B4\A6\C0\ED\810\8F0\BF\E9\810\F45\CA\FD\BE\DD
                    module_response();
                }
                else
                {
                    printf("CRC error: %d!\n", get_data_size_);
                    //\810\F45\CA\FD\BE\DD\A7\B5\D1\E9\B4\ED\CE\F3
                }
                //\BD\D3\CA\D5\810\DD5\810\F45\CA\FD\BE\DD\CD\EA\B3\C9
                goto API_FRAME_RESET;
            }
            break;

        default:
            goto API_FRAME_RESET;
    }

    return 0;
    
    API_FRAME_RESET:
    v2Counter = 0;
    Recv_Buffer.Count = 0;
    Recv_Buffer.Status = FRAME_SYNC;
    return 0;
}

void api_port_received(int32_t len, uint8_t * data)
{
    get_data_size_ = len;
    if(len == 44 || len == 56 || len == 100 || len == 112)
    {
        // printf("get pose data!\n");
    }
    while (len--)
    {
        module_parse_frame(*data++);
    }
}


void  api_on_data_set(uint8_t result)
{
  

}

/*void api_on_data_returned(int32_t offset, int32_t len, uint8_t * data)
{
  

}
 void api_on_message_received(API_Message_t message)
{

}*/

 void api_on_upgrade(void)
{

}

static void XorData(uint8_t *data, uint16_t length, uint8_t key)
{
    uint16_t index = 0;
    for (index = 0; index < length; index++)
    {
        data[index] = data[index] ^ key;
    }
}

static uint32_t check_sum(uint8_t *data, uint16_t len)
{
    uint32_t checksum = 0;
    while (len--)
    {
        checksum += *data++;
    }
    return checksum;
}


//#ifdef __CC_ARM                         /* ARM Compiler */

 //   static uint16_t ccitt_table[256] __attribute__((section(".ccm"))) = {
//#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    static uint16_t ccitt_table[256] = {
//#endif
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

void crc16(uint8_t input, crc16_t * output)
{
    output->current = ccitt_table[(output->current >> 8 ^ input) & 0xff] ^ (output->current << 8);
    output->crc16 = ~output->current;
}


// void api_port_send(int32_t len, uint8_t * data)
// {
// 	if(data == NULL)
// 		return;
// 	if(superx_fd_uart==-1) return;
	
// // 	if(len>0)
// // 	{
// // 		printf("UART write data len = %d, uart_fd = %d, uart_baudrate=%d\n",len,superx_fd_uart,GetPortSpeed(superx_fd_uart));
// // 		int i=0;
// // 		for(i=0;i<len;i++)
// // 		{
// // 			printf(" %x",data[i]);
// // 		}
// // 		printf("\n");
// // 	}
//     need_write_touart_len +=len;
// 	enQueue(&write_que, data, len,superx_fd_uart);
//   //  UartWrite(superx_fd_uart,data,len,1);
// }

uint16_t crc_ccitt(uint8_t * input, int len)
{
    uint16_t crc = CRC16_INIT_VALUE;
    while (len-- > 0)
    crc = ccitt_table[(crc >> 8 ^ * input++) & 0xff] ^ (crc << 8);
    return ~crc;
}

//#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */

/*#pragma weak api_port_send
#pragma weak api_on_data_set
#pragma weak api_on_data_returned
#pragma weak api_on_message_received
#pragma weak api_on_upgrade*/

//#endif

