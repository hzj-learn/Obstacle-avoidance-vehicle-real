#ifndef _SERIAL_H_
#define	_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

enum SeriaComStat {
    WAIT_DATA_READY = 1,
    WAIT_DATA_EXCEPT = -1,
    WAIT_DATA_ERROR = -2,
    WAIT_DATA_TIMEOUT = -3,

};

int OpenUartCom(const char *path);
int PortSpeed(int fd, int speed);
int PortSet(int fd, int bitnum, int stopbits, int parity);
int UartWrite(int fd, unsigned char *pdatabuf, int ndatalen, int wait_time);
int WaitUartComReady(int fd, int rw_opt, int nms_timeout);
int UartRead(int fd, unsigned char *pDdatabuf, int nDatalen, int wait_time);
void FlushComCacheBuf(int fd);
void FlushComInputCacheBuf(int fd);
void FlushComOutputCacheBuf(int fd);
int ConfigUartCom(int fd,  int baudrate, int bitnum, int stopbits, int parity);
int GetPortSpeed(int fd);
void close_uart(int fd);

#ifdef __cplusplus
}
#endif

#endif


