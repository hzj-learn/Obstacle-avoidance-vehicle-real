#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <termios.h>
#include "uart.h"

#define	TXRXWAITOUT	2000
#define SERIALCOM_READ_WAITTIME	200   //200ms

//#define DEBUG

#define TRUE	0
#define FALSE	-1

/**************************************************************************************
 * ���ƣ�PortSpeed
 * ���ܣ����ڲ���������
 * �βΣ�fd�������ļ�������
 				 speed��������
 * ���أ��ɹ�����1��ʧ�ܷ���0
 * ˵����
**************************************************************************************/
int PortSpeed(int fd, int speed)
{
    const int speed_arr[] = {B1152000,B1000000,B576000,B460800,B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,};
    const int name_arr[] = {1152000,1000000,576000, 460800,230400,115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300,};

    int i;
    int status = -1;
    struct termios term;

//	printf("{}  \n",__FUNCTION__);

    tcgetattr(fd, &term);

    for (i = 0; i < sizeof(speed_arr)/sizeof(speed_arr[0]); i++) {
        if(speed == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&term, speed_arr[i]);
            cfsetospeed(&term, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &term);
            if(status != 0) {
                perror("tcserattr fd\n");
                return FALSE;
            } else {
                //printf("port speed set success.\n");
                return TRUE;
            }
        }
        tcflush(fd, TCIOFLUSH);
    }

    return FALSE;

}

/**************************************************************************************
 * ���ƣ�GetPortSpeed
 * ���ܣ���ȡ���ڲ�����
 * �βΣ�fd�������ļ�������

 * ���أ��ɹ�����1��ʧ�ܷ���0
 * ˵����
**************************************************************************************/
int GetPortSpeed(int fd)
{
    const int speed_arr[] = {B1152000,B1000000,B576000,B460800,B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,};
    const int name_arr[] = {1152000,1000000,576000, 460800,230400,115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300,};

    int i;
    struct termios term;
    int ispeed, ospeed;

//	printf("{}  \n",__FUNCTION__);

    tcgetattr(fd, &term);


    ispeed = cfgetispeed(&term);
    ospeed = cfgetospeed(&term);

    if (ispeed == ospeed) {
        for (i=0; i<sizeof(speed_arr)/sizeof(speed_arr[0]); i++) {
            if (ispeed == speed_arr[i]) {
                return name_arr[i];
            }
        }
    } else {
        return FALSE;
    }

    return FALSE;

}

/**************************************************************************************
 * ���ƣ�PortSet
 * ���ܣ���������
 * �βΣ�fd�������ļ�������
 				 bitnum������λ
 				 stopbits��ֹͣλ
 				 parity��У��λ
 * ���أ��ɹ�����1��ʧ�ܷ���0
 * ˵����
**************************************************************************************/
int PortSet(int fd, int bitnum, int stopbits, int parity)
{
//	printf("{}  \n",__FUNCTION__);

    struct termios term;

    if (tcgetattr(fd, &term) != 0) {
        perror("tcgetattr\n");
        return FALSE;
    }

    term.c_cflag &= ~CSIZE;

    switch (bitnum) {
    case 6:
        term.c_cflag |= CS6;
        break;

    case 7:
        term.c_cflag |= CS7;
        break;

    case 8:
        term.c_cflag |= CS8;
        break;

    default:
        fprintf(stderr, "Unsupported data bits\n");
        return FALSE;

    }

    switch (parity) {
    case 'N':
    case 'n':
        term.c_cflag &= ~PARENB;   /* ����:������żλ������:��������żλ */
        term.c_iflag &= ~INPCK;      /* ʹ��żУ���������� */
        break;

    case 'O':
    case 'o':
        term.c_cflag |= (PARENB | PARODD); /* ���ò�����żλ�������ó���У�� */
        term.c_iflag |= INPCK;   /* ʹ��żУ�������� */
        break;

    case 'E':
    case 'e':
        term.c_cflag |= PARENB;	/* ���ò�����żλ*/
        term.c_cflag &= ~PARODD;	/* ��λ:��У�飬����:żУ�� */
        term.c_iflag |= INPCK;
        break;

    default:
        printf("Unsupported patity bits");

    }

    switch(stopbits) {
    case 1:
        term.c_cflag &= ~CSTOPB;
        break;

    case 2:
        term.c_cflag |= CSTOPB;
        break;

    default:
        printf("Unsupported stop bits");
        return FALSE;
    }

    term.c_iflag = 0;
    term.c_cflag |= (CLOCAL | CREAD);
    term.c_lflag &= ~(ICANON | ECHO | ECHOE |ISIG);
    term.c_oflag &= ~OPOST;

    tcflush(fd, TCIOFLUSH);

    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &term) != 0) {
        perror("  SetupUart err");
        return FALSE;
    }
    return TRUE;

}


/**************************************************************************************
 * ���ƣ�ConfigUartCom
 * ���ܣ�
 * �βΣ���
 * ���أ������ļ�������
 * ˵����
**************************************************************************************/
int ConfigUartCom(int fd,  int baudrate, int bitnum, int stopbits, int parity)
{
    if (PortSpeed(fd, baudrate) < 0) {
        printf(" Uart com set baudrate failed.\n");
        return FALSE;
    }

    if (PortSet(fd, bitnum, stopbits, parity) < 0) {
        printf(" Uart com set failed.\n");
        return FALSE;
    }
    return TRUE;
}

/**************************************************************************************
 * ���ƣ�OpenUartCom
 * ���ܣ��򿪴���
 * �βΣ���
 * ���أ������ļ�������
 * ˵����
**************************************************************************************/
int OpenUartCom(const char *path)
{
    int fd = 0;

    if (path == NULL) {
        return FALSE;
    }

    fd = open(path, O_RDWR);
    if(fd == -1) {
        printf("Open file:{} error!\n", path);
        return FALSE;
    }
    return fd;
}


/**************************************************************************************
 * ���ƣ�WaitUartComReady
 * ���ܣ��ȴ����ھ���
 * �βΣ�fd�������ļ�������
 				 rw_opt����дѡ��
 				 nms_timeout���ȴ���ʱʱ��
 * ���أ��ɹ�����1��ʧ�ܷ���-1
 * ˵����
**************************************************************************************/
int WaitUartComReady(int fd, int rw_opt, int nms_timeout)
{
    int iRet;
    fd_set fdset, fderr;
    struct timeval tv;

//	printf("{}  \n",__FUNCTION__);

    tv.tv_usec = nms_timeout % 1000 * 1000;
    tv.tv_sec = nms_timeout/1000;

    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    fderr = fdset;

    if(rw_opt) {
        iRet = select(fd + 1, &fdset, NULL, &fderr, &tv);
    } else {
        iRet = select(fd + 1, NULL, &fdset, &fderr, &tv);
    }

    if(iRet > 0) {
        iRet = (FD_ISSET(fd, &fdset)) ? WAIT_DATA_READY
               : FD_ISSET(fd, &fderr)? WAIT_DATA_EXCEPT
               : WAIT_DATA_ERROR;
    } else {
        iRet = WAIT_DATA_ERROR;
    }

    return iRet;
}



static int WriteData(int fd, unsigned char *pdatabuf, int ndatalen)
{
    int rc;
    int nByteWr = 0;
    int nTotalByteWr = 0;
    int nRetryTimes = 0;
#ifdef DEBUG
    int i;
    unsigned char *p = pdatabuf;
#endif

    while(ndatalen > 0) {
        nByteWr = write(fd, pdatabuf, ndatalen);

        if(nByteWr > 0) {
            ndatalen -= nByteWr;
            pdatabuf -= nByteWr;
            nTotalByteWr += nByteWr;
            if(ndatalen <= 0) {

#ifdef DEBUG

                printf("\nSend: ");
                for(i=0; i<nTotalByteWr; i++) {
                    printf(" {:x}",p[i]);
                }
                printf("\n");
#endif
                break;	//Send data OK
            }

            printf("<0\n");
            rc = WaitUartComReady(fd,0, TXRXWAITOUT);
            if(rc <= 0) {
                printf("Write time out!!\n");
                return WAIT_DATA_TIMEOUT;
            }
        } else {
            if(++nRetryTimes >= 3) {
                printf("Error! Retry times out!!!\n");
                break;
            }

        }

    }

    return nTotalByteWr;
}


/**************************************************************************************
 * ���ƣ�UartComWrite
 * ���ܣ�����д����
 * �βΣ�fd�������ļ�������
 				 pdatebuf,ָ����д���ݻ�����ָ��
 				 ndatalen,��д���ݳ���
 * ���أ��ɹ�������д�ֽ���
 * ˵����
**************************************************************************************/
int UartWrite(int fd, unsigned char *pdatabuf, int ndatalen, int wait_time)
{


    int rc;

//	printf("{}  \n",__FUNCTION__);

    rc = WaitUartComReady(fd, 0, wait_time);	  //wait to write data availabe
    if(rc <= 0) {
        printf("write time out!!\n");
        return WAIT_DATA_TIMEOUT;
    }

    return WriteData(fd, pdatabuf, ndatalen);
}


/**************************************************************************************
 * ���ƣ�UartComWrite
 * ���ܣ����ڶ�����
 * �βΣ�fd�������ļ�������
 				 pdatebuf,ָ���������ݻ�����ָ��
 				 ndatalen,�������ݳ���
 * ���أ��ɹ����������ֽ���
 * ˵����
**************************************************************************************/
int UartRead(int fd, unsigned char *pdatabuf, int ndatalen, int wait_time)
{

    int nbyterd = 0;
    int rc = 1;
#ifdef DEBUG
    int i;
    unsigned char *p = pdatabuf;
#endif
//	printf("{}  \n",__FUNCTION__);

    rc = WaitUartComReady(fd, 1, wait_time);	  //check if data available
    if(rc <= 0) {
     //   printf("read time out!!\n");
        return WAIT_DATA_TIMEOUT;
    }

    nbyterd = read(fd, pdatabuf, ndatalen);

#ifdef DEBUG

    if (nbyterd) {
        printf("\nRcv: ");
        for(i=0; i<nbyterd; i++) {
            printf(" {:x}",p[i]);
        }
        printf("\n");
    }
#endif


    return nbyterd;
}



/**************************************************************************************
 * ���ƣ�FlushComCacheBuf
 * ���ܣ�ˢ��������������
 * �βΣ�fd�������ļ�������
 * ���أ���
 * ˵����
**************************************************************************************/

void FlushComCacheBuf(int fd)
{
    tcflush(fd, TCIOFLUSH);
}

void close_uart(int fd)
{
    close(fd);
}



