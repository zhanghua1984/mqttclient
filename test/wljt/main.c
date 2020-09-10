/*
 * @Author: jiejie
 * @Github: https://github.com/jiejieTop
 * @Date: 2019-12-11 21:53:07
 * @LastEditTime: 2020-06-08 20:45:33
 * @Description: the code belongs to jiejie, please keep the author information and source code according to the license.
 */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>

#include "mqtt_config.h"
#include "mqtt_log.h"
#include "mqttclient.h"

#include <string.h>
#include "cJSON.h"
#include "termios.h"
#include "config.h"

int fd,maxfd;

extern config_t config;

/*
RS485-RTU
地址码		功能码		寄存器起始地址			寄存器长度		校验码低字节			校验码高字节
1字节		1字节		2字节				2字节			1字节				1字节


DIO 通信
TX:01 02 00 10 00 04 78 0C
RX:01 02 01 00 A1 88

水浸传感器
TX:01 03 00 00 00 01 84 0A
RX:01 03 02 00 00 B8 44
RX:01 03 02 00 01 79 84

断路器

读取基本信息
TX:68 A0 01 01 10 CS
RX:68 A0 81 03 10+型号+分合闸状态 CS
   68 A0 81 03 10 00 01 9D

读取断路器信息(已包含基本信息)
TX:68 A0 01 07 00*7 CS
RX:68 A0 81 le 00+型号+分合闸状态+电网信息 CS
   68 A0 81 34 00 00 01 34 09 01 00 78 00 00 1F 00 00 57 00 00 27 1C 00 00 00 00 00 90 23 00 00 00 00 00 00 00 1F 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1F 01 00 00 20

读取版本号
TX:68 A0 01 01 01 CS
RX:68 A0 81 15 01+版本号 CS

写断路器
TX:68 A0 02 03 20+地址+断路器状态（0分，1合）
RX:68 A0 82 01 20 CS

写断路器电量清零
TX:68 A0 02 03 00+地址+00 CS
RX:68 A0 82 01 00 CS


*/
#pragma pack(1)

typedef struct RTU
{
	unsigned char m_u8Addr;
	unsigned char m_u8Func;
	unsigned short m_u16RegAddr;
	unsigned short m_u16RegLen;
	unsigned short m_u16Crc;
}rtu_t;

typedef struct Breaker
{
	unsigned char m_u8FrameHead;
	unsigned char m_u8Addr;
	unsigned char m_u8ControlCode;
	unsigned char m_u8Len;
	unsigned char m_u8Content[7];
	unsigned char m_u8Sum;
}breaker_t;


void dumpData(unsigned char * buf, int len)
{
	for(int i=0;i<len;i++)
	{
		printf("%02X ",buf[i]);
	}
	printf("\r\n");
}

unsigned char CheckSum(unsigned char * m_u8buf, int len)
{
	unsigned char m_u8Sum=0;
	for(int i=0;i<len;i++)
	{
		m_u8Sum+=m_u8buf[i];
	}

	return m_u8Sum;
}

unsigned short
crc_16_tab[] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

unsigned short crc16(unsigned char *buf, unsigned int bsize)
{
  unsigned short crc = 0xffff;
  while (bsize--)
    crc = (unsigned short)(crc >> 8) ^ crc_16_tab[(crc ^ *buf++) & 0xff];
  return crc;
}


// #define TEST_USEING_TLS  
extern const char *test_ca_get();


unsigned int m_ungControlFlag = 0;

static void topic1_handler(void* client, message_data_t* msg)
{
    (void) client;
	/*
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
    MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name, (char*)msg->message->payload);
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
	*/
	printf("%s\r\n",(char*)msg->message->payload);
	//解析json，比对项目名称、设备名称，解析子设备动作
	m_ungControlFlag = 1;
	cJSON *root=cJSON_Parse((char*)msg->message->payload);
	
	if (!root)
	{
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		goto DEAL_FINISH ;
	}
	else
	{
		cJSON *item=cJSON_GetObjectItem(root,"Project");
		if(item!=NULL)
		{
			printf("cJSON_GetObjectItem: type=%d, key is %s, value is %s\n",item->type,item->string,item->valuestring);
			if(strcmp(item->valuestring,config.project)!=0)
			{
				printf("project name mismatch\r\n");
				goto DEAL_FINISH ; ;
			}
		}

		item=cJSON_GetObjectItem(root,"DevId");
		if(item!=NULL)
		{
			printf("cJSON_GetObjectItem: type=%d, key is %s, value is %s\n",item->type,item->string,item->valuestring);
			if(strcmp(item->valuestring,config.devid)!=0)
			{
				printf("devid mismatch\r\n");
				goto DEAL_FINISH ; ;
			}
		}

		item=cJSON_GetObjectItem(root,"SubDevName");
		if(item!=NULL)
		{
			printf("cJSON_GetObjectItem: type=%d, key is %s, value is %s\n",item->type,item->string,item->valuestring);

			if(strcmp(item->valuestring,"Breaker")==0)	//断路器控制
			{
				printf("Breaker control start \r\n");
				item=cJSON_GetObjectItem(root,"SubDevStatus");
				if(item!=NULL)
				{
					printf("cJSON_GetObjectItem: type=%d, key is %s, value is %d\n",item->type,item->string,item->valueint);
					//状态写入
					printf("breaker status write in \r\n");
					
					unsigned char  m_u8breaker[16];
					
					m_u8breaker[0] = 0x68;
					m_u8breaker[1] = 0xA0;
					m_u8breaker[2] = 0x02;
					m_u8breaker[3] = 0x03;
					m_u8breaker[4] = 0x20;
					m_u8breaker[5] = 0xA0;
					m_u8breaker[6] = item->valueint;
					m_u8breaker[7] = CheckSum(m_u8breaker, 7);

					int m_nWriteRc=write(fd,m_u8breaker,8); 
					if (m_nWriteRc<0) 
					{
						printf("RS485 ctrl write fail\r\n");
					}
				}
			}
		}

		
	}

DEAL_FINISH:
	cJSON_Delete(root);
	m_ungControlFlag = 0;
	
}

static void topic2_handler(void* client, message_data_t* msg)
{
    (void) client;
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
    MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name, (char*)msg->message->payload);
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
}

static void topic3_handler(void* client, message_data_t* msg)
{
    (void) client;
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
    MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name, (char*)msg->message->payload);
    MQTT_LOG_I("-----------------------------------------------------------------------------------");
}


/* Create a bunch of objects as demonstration. */
static int print_preallocated(char *buf,cJSON *root)
{
    /* declarations */
    char *out = NULL;
    //char *buf = NULL;
    char *buf_fail = NULL;
    size_t len = 0;
    size_t len_fail = 0;

    /* formatted print */
    out = cJSON_Print(root);

    /* create buffer to succeed */
    /* the extra 5 bytes are because of inaccuracies when reserving memory */
    len = strlen(out) + 5;
    //buf = (char*)malloc(len);
    if (buf == NULL)
    {
        printf("Failed to allocate memory.\n");
        exit(1);
    }

    /* create buffer to fail */
    len_fail = strlen(out);
    buf_fail = (char*)malloc(len_fail);
    if (buf_fail == NULL)
    {
        printf("Failed to allocate memory.\n");
        exit(1);
    }

    /* Print to buffer */
    if (!cJSON_PrintPreallocated(root, buf, (int)len, 1)) {
        printf("cJSON_PrintPreallocated failed!\n");
        if (strcmp(out, buf) != 0) {
            printf("cJSON_PrintPreallocated not the same as cJSON_Print!\n");
            printf("cJSON_Print result:\n%s\n", out);
            printf("cJSON_PrintPreallocated result:\n%s\n", buf);
        }
        free(out);
        free(buf_fail);
        //free(buf);
        return -1;
    }

    /* success */
    //printf("%s\n", buf);

    /* force it to fail */
    if (cJSON_PrintPreallocated(root, buf_fail, (int)len_fail, 1)) {
        printf("cJSON_PrintPreallocated failed to show error with insufficient memory!\n");
        printf("cJSON_Print result:\n%s\n", out);
        printf("cJSON_PrintPreallocated result:\n%s\n", buf_fail);
        free(out);
        free(buf_fail);
        //free(buf);
        return -1;
    }

    free(out);
    free(buf_fail);
    //free(buf);
    return 0;
}
int crcIsOk(unsigned char * buf,int len)
{
	//dumpData(buf, len);
	unsigned short crc = crc16(buf, len-2);
	
	unsigned short temp = buf[len-1]<<8|buf[len-2];
	//printf("crc = %04X temp=%04X \r\n",crc,temp);
	if(crc == temp)
	{
		return 1;
	}
	return 0;
}
int sumIsOk(unsigned char *buf,int len)
{
	unsigned char sum =0;
	for(int i=0;i<len-1;i++)
	{
		sum += buf[i];
	}
	if(sum==buf[len-1])
	{
		return 1;
	}
	
	printf("sum check is fail,sum=%02X buf=%02x \r\n",sum,buf[len-1]);	
	return 0;
}
int ParserBreaker(struct Breaker* breaker,unsigned char *buf, int len, unsigned char * rtbuf)
{
	/* declare device info. */
    cJSON *root = NULL;
    cJSON *fmt = NULL;
	struct timeval tv;
	gettimeofday(&tv,NULL);
	
	dumpData(buf,len);
	
	/* Our "Device" datatype: */
    root = cJSON_CreateObject();
	if(sumIsOk((unsigned char *)buf,len))
	{
		switch(breaker->m_u8FrameHead)
		{
			case 0x68:
			{
				cJSON_AddStringToObject(root, "Project", config.project);
				cJSON_AddStringToObject(root, "DevId", config.devid);
				cJSON_AddStringToObject(root, "SubDevName", "Breaker");
				cJSON_AddNumberToObject(root, "SubDevType", buf[5]);
				cJSON_AddNumberToObject(root, "SubDevStatus", buf[6]);
				cJSON_AddNumberToObject(root, "SubDevVV", buf[8]<<8|buf[7]);	//0.1v
				cJSON_AddNumberToObject(root, "SubDevAA", buf[12]<<8|buf[11]);  //0.001a
				cJSON_AddNumberToObject(root, "SubDevKK", buf[16]<<16|buf[15]<<8|buf[14]);	//0.0001kw
				cJSON_AddNumberToObject(root, "SubDevKvar", buf[19]<<16|buf[18]<<8|buf[17]);  //0.0001kw		

				
				cJSON_AddNumberToObject(root, "TimeStamp", tv.tv_sec*1000 + tv.tv_usec/1000);	//ms

			}
			break;
			default:
			{
				printf("Parser breaker dev addr error \r\n");
			}
		}
		
	}
	else
	{
		printf("crc fail\r\n");
		    /* Print to text */
	    if (print_preallocated(rtbuf,root) != 0) {
	        cJSON_Delete(root);
	        exit(EXIT_FAILURE);
	    }
	    cJSON_Delete(root);

		return -1;
	}

    /* Print to text */
    if (print_preallocated(rtbuf,root) != 0) {
        cJSON_Delete(root);
        exit(EXIT_FAILURE);
    }
    cJSON_Delete(root);
	printf("parser breaker ok\r\n");
	return 0;
}

void devRTUoffline(struct RTU* rtu,unsigned char * rtbuf)
{
	
	/* declare device info. */
    cJSON *root = NULL;
    cJSON *fmt = NULL;
	struct timeval tv;
	gettimeofday(&tv,NULL);

	/* Our "Device" datatype: */
    root = cJSON_CreateObject();

	cJSON_AddStringToObject(root, "Project", config.project);
	cJSON_AddStringToObject(root, "DevId", config.devid);	
	switch(rtu->m_u8Addr)
	{
		case 0x01:
		{
			cJSON_AddStringToObject(root, "SubDevName", "DIO");
			cJSON_AddStringToObject(root, "SubDevStatus", "OffLine");
			cJSON_AddNumberToObject(root, "TimeStamp", tv.tv_sec*1000 + tv.tv_usec/1000);	//ms

		}
		break;
		case 0xf0:
		{
			cJSON_AddStringToObject(root, "SubDevName", "WaterIn");
			cJSON_AddStringToObject(root, "SubDevStatus", "OffLine");
			cJSON_AddNumberToObject(root, "TimeStamp", tv.tv_sec*1000 + tv.tv_usec/1000);	//ms

		}
		break;
		default:
			{
		printf("ParserRTU dev addr error \r\n");
		}
	}


    /* Print to text */
    if (print_preallocated(rtbuf,root) != 0) {
        cJSON_Delete(root);
        exit(EXIT_FAILURE);
    }
    cJSON_Delete(root);
		
}

void ParserRTU(struct RTU* rtu,unsigned char *buf, int len, unsigned char * rtbuf)
{
	/* declare device info. */
    cJSON *root = NULL;
    cJSON *fmt = NULL;
	struct timeval tv;
	gettimeofday(&tv,NULL);

	/* Our "Device" datatype: */
    root = cJSON_CreateObject();
	if(crcIsOk((unsigned char *)buf,len))
	{
		switch(rtu->m_u8Addr)
		{
			case 0x01:
			{
				cJSON_AddStringToObject(root, "Project", config.project);
				cJSON_AddStringToObject(root, "DevId", config.devid);
				cJSON_AddStringToObject(root, "SubDevName", "DIO");
				cJSON_AddStringToObject(root, "SubDevStatus", "Online");
				cJSON_AddStringToObject(root, "SubDevFunc", "DI");
				cJSON_AddNumberToObject(root, "SubDevValue", buf[3]);
				cJSON_AddNumberToObject(root, "TimeStamp", tv.tv_sec*1000 + tv.tv_usec/1000);	//ms

			}
			break;
			case 0xf0:
			{
				cJSON_AddStringToObject(root, "Project", config.project);
				cJSON_AddStringToObject(root, "DevId", config.devid);
				cJSON_AddStringToObject(root, "SubDevName", "WaterIn");
				cJSON_AddStringToObject(root, "SubDevStatus", "Online");
				cJSON_AddStringToObject(root, "SubDevFunc", "Status");
				cJSON_AddNumberToObject(root, "SubDevValue", buf[4]);
				cJSON_AddNumberToObject(root, "TimeStamp", tv.tv_sec*1000 + tv.tv_usec/1000);	//ms

			}
			break;
			default:
				{
			printf("ParserRTU dev addr error \r\n");
			}
		}
		
	}
	else
	{
		printf("crc fail\r\n");
	}

    /* Print to text */
    if (print_preallocated(rtbuf,root) != 0) {
        cJSON_Delete(root);
        exit(EXIT_FAILURE);
    }
    cJSON_Delete(root);

	
}



void *mqtt_publish_thread(void *arg)
{
    mqtt_client_t *client = (mqtt_client_t *)arg;
	struct timeval tv;



    char buf[1024] = { 0 };
    mqtt_message_t msg;
    memset(&msg, 0, sizeof(msg));
    sprintf(buf, "welcome to mqttclient, this is a publish device info ...");

    sleep(2);

    mqtt_list_subscribe_topic(client);
	//1、打开串口
	

	//O_NOCTTY 标志 ，该程序不想成为此端口的“控制终端"。 
	//O_NDELAY标志 , 标志告诉Linux ,该程序并不关注DCD信叼线所处的状态, 即不管另外一端的设备是在运行还是被挂起。如果没有指定该标志，那么程序就会被设置睡 眠状态
	fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY); 
	if (fd == -1) 
	{ 
		/*无法打开串口*/ 
		perror("open_port : Unable to open /dev/ttyUSB0"); 
		exit(1);
	} 

	//2、设置串口
	struct termios options;
	tcgetattr(fd, &options);
	/*
	 * Set the baud rates to 9600
	 */
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	/*
	 * Enable the receiver and set local mode
	 */
	options.c_cflag |= (CLOCAL | CREAD);

	/*
	 * Select 8 data bits, 1 stop bit and no parity bit
	 */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	/*
	 * Disable hardware flow control
	 */
	options.c_cflag &= ~CRTSCTS;

	/*
	 * Choosing raw input
	 */
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/*
	 * Disable software flow control
	 */
	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	/*
	 * Choosing raw output
	 */
	options.c_oflag &= ~OPOST;

	/*
	 * Set read timeouts
	 */
	options.c_cc[VMIN] = 8;
	options.c_cc[VTIME] = 10;
	//options.c_cc[VMIN] = 0;
	//options.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &options);

	
	char buff[BUFSIZE];
	char TXbuf[32];
	struct timeval tvrd;
	fd_set rfds,watchset;

	//3、读写串口


	maxfd = fd + 1;
	int i=0;
	rtu_t m_rtu[RTU_DEV_NUM];
	//数字IO 采集器
	m_rtu[i].m_u8Addr = 0x01;
	m_rtu[i].m_u8Func = 0x02;
	m_rtu[i].m_u16RegAddr = 0x1000; 	//小端模式
	m_rtu[i].m_u16RegLen = 0x0400;
	m_rtu[i].m_u16Crc = crc16((unsigned char *)&m_rtu[i], sizeof(rtu_t)-2);

	i++;
	//水浸传感器
	m_rtu[i].m_u8Addr = 0xf0;	//地址设置为240=0xf0
	m_rtu[i].m_u8Func = 0x03;
	m_rtu[i].m_u16RegAddr = 0x0000; 	//小端模式
	m_rtu[i].m_u16RegLen = 0x0100;
	m_rtu[i].m_u16Crc = crc16((unsigned char *)&m_rtu[i], sizeof(rtu_t)-2);


	//智能断路器	 断路器不是RTU协议
	breaker_t m_breaker;
	m_breaker.m_u8FrameHead = 0x68;
	m_breaker.m_u8Addr = 0xA0;
	m_breaker.m_u8ControlCode = 0x01;
	m_breaker.m_u8Len = 0x07;
	for(int j=0;j<7;j++)
		m_breaker.m_u8Content[j] = 0x00;	
	
	m_breaker.m_u8Sum = CheckSum((unsigned char *) &m_breaker, sizeof(breaker_t)-1);
	
	int m_nWriteRc,m_nReadRc,m_nSelectRc;

    while(1) {

		while(m_ungControlFlag)		//等待远程控制
		{
			sleep(1);
		}
	
        //sprintf(buf, "welcome to mqttclient, this is a publish device info, a rand number: %d ...", random_number());
		// RTC device
		for(i=0;i<RTU_DEV_NUM;i++)
		{
			//初始化 
			FD_ZERO(&rfds); 
			FD_ZERO(&watchset); 
			FD_SET(fd,&watchset);
			tvrd.tv_sec=1;
			tvrd.tv_usec=0;
			
			//dumpData((unsigned char *)&m_rtu[i],sizeof(rtu_t));

			memcpy(TXbuf,(unsigned char *)&m_rtu[i],sizeof(rtu_t));
			
			m_nWriteRc=write(fd,TXbuf,sizeof(rtu_t)); 
			if (m_nWriteRc<0) 
			{
				printf("RS485 write fail \r\n");
				continue;
			}

			if(m_nWriteRc!=sizeof(rtu_t))
			{
				printf("RS485 write error, m_nWriteRc=%d size rtu = %d \r\n",m_nWriteRc,sizeof(rtu_t));
			}
		 
			rfds = watchset;
			//printf("maxfd=%d\n",maxfd);
			lseek(fd,0,SEEK_SET);
			bzero(buff,BUFSIZE);
			
			
			gettimeofday(&tv,NULL);

			if(m_nSelectRc=(select(maxfd,&rfds,NULL,NULL,&tvrd))>0)
			{
				if(FD_ISSET(fd,&rfds))
				{	
					usleep(400000); //等待设备发送完成
					m_nReadRc=read(fd, buff, BUFSIZE);
					//printf("readlength=%d\n\r",m_nReadRc);
					//解析数据
					ParserRTU(&m_rtu[i],(unsigned char *)buff, m_nReadRc, (unsigned char *) buf);

				}
			}  
			else 
			{	//no data
				printf("read rtu device data timeout m_nSelectRc = %d\n",m_nSelectRc);
				devRTUoffline(&m_rtu[i],(unsigned char *) buf);
			}

			msg.payload = (void *) buf;
	        msg.qos = 0;
			mqtt_publish(client, config.mqtt_publish_qos0, &msg);

			sleep(1);
		}

		// NOT RTU device
		//初始化 
		FD_ZERO(&rfds); 
		FD_ZERO(&watchset); 
		FD_SET(fd,&watchset);
		tvrd.tv_sec=1;
		tvrd.tv_usec=0;
		
		//dumpData((unsigned char *)&m_rtu[1],sizeof(rtu_t));
		//dumpData((unsigned char *)&m_breaker,sizeof(breaker_t));

		memcpy(TXbuf,(unsigned char *)&m_breaker,sizeof(breaker_t));
		
		m_nWriteRc=write(fd,TXbuf,sizeof(breaker_t)); 
		if (m_nWriteRc<0) 
		{
			printf("RS485 write fail\r\n");
		}

		if(m_nWriteRc!=sizeof(breaker_t))
		{
			printf("RS485 write error, m_nWriteRc=%d size breaker = %d \r\n",m_nWriteRc,sizeof(breaker_t));
		}
	 
		rfds = watchset;
		//printf("maxfd=%d\n",maxfd);
		lseek(fd,0,SEEK_SET);
		bzero(buff,BUFSIZE);

		if(m_nSelectRc=(select(maxfd,&rfds,NULL,NULL,&tvrd))>0)
		{
			if(FD_ISSET(fd,&rfds))
			{	
				usleep(400000); //等待发送完成
				m_nReadRc=read(fd, buff, BUFSIZE);
				int m_nRt = ParserBreaker((struct breaker_t*) &m_breaker,buff, m_nReadRc, (unsigned char *) buf);
				if(m_nRt<0)
				{
					//sum check fail
					printf("ParserBreadker failed \r\n");
					sleep(1);
					continue;
				}
			}
		}  
		else	//读取设备超时认为设备离线 
		{
			
			printf("m_nSelectRc = %d\n",m_nSelectRc);
		}
		
		msg.payload = (void *) buf;

        msg.qos = 0;
        mqtt_publish(client, config.mqtt_publish_qos0, &msg);

#if 0
        msg.qos = 1;
        mqtt_publish(client, "device/info1", &msg);

        msg.qos = 2;
        mqtt_publish(client, "device/info2", &msg);
#endif
        sleep(1);
    }
	close(fd);
	
}

int main(void)
{
    int res;
    pthread_t thread1;
    mqtt_client_t *client = NULL;
    
    printf("\nwelcome to mqttclient wljt...\n");

	//读取配置文件
	cfg_init();
	cfg_read_file(CONFIG_FILE);
	dump_config();

    mqtt_log_init();

    client = mqtt_lease();

#ifdef TEST_USEING_TLS
    mqtt_set_port(client, config.serverport);
    mqtt_set_ca(client, (char*)test_ca_get());
#else

    mqtt_set_port(client, config.serverport);

#endif

    mqtt_set_host(client, config.serveraddr);
    mqtt_set_client_id(client, random_string(10));
    mqtt_set_user_name(client, random_string(10));
    mqtt_set_password(client, random_string(10));
    mqtt_set_clean_session(client, 1);

    mqtt_connect(client);
    
    mqtt_subscribe(client, config.mqtt_subscribe_qos0, QOS0, topic1_handler);
    mqtt_subscribe(client, config.mqtt_subscribe_qos1, QOS1, topic2_handler);
    mqtt_subscribe(client, config.mqtt_subscribe_qos2, QOS2, topic3_handler);
    
    res = pthread_create(&thread1, NULL, mqtt_publish_thread, client);
    if(res != 0) {
        MQTT_LOG_E("create mqtt publish thread fail");
        exit(res);
    }

    while (1) {
        sleep(100);
    }
}
