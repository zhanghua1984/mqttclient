/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * cfg.h - configuration related procedures
 *
 * Copyright (c) 2002-2003, 2013, Victor Antonovich (v.antonovich@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: cfg.h,v 1.3 2015/02/25 10:33:57 kapyar Exp $
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#define STRINGBUFSIZE 64
#define BUFSIZE	256
#define RTU_DEV_NUM 2

#define CONFIG_FILE "wljt.conf"


#define DEFALUT_PROJECT "WLJTYJS"

#define DEFALUT_DEVICE_ID "WLJTYJS0001"

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_SPEED 9600
#define DEFAULT_MODE "8N1"

#define DEFAULT_SERVERADDR "192.168.10.134"
#define DEFAULT_SERVERPORT "30301"


#define DEFAULT_MAXCONN		10
#define DEFAULT_MAXTRY 		0xffffffff
#define DEFAULT_CONNTIMEOUT 1

#define DEFAULT_MQTT_SUB_QOS0 "dev/ctrl"
#define DEFAULT_MQTT_SUB_QOS1 "dev/ctrl1"
#define DEFAULT_MQTT_SUB_QOS2 "dev/ctrl2"

#define DEFAULT_MQTT_PUB_QOS0 "dev/info"


/* Global configuration storage structure */
typedef struct
{
	char project[STRINGBUFSIZE];
	
	char devid[STRINGBUFSIZE];
	/* tty port name */
	char ttyport[STRINGBUFSIZE];
	/* tty speed */
	int ttyspeed;
	/* tty mode */
	char ttymode[STRINGBUFSIZE];

	/* mqtt server address */
	char serveraddr[STRINGBUFSIZE];
	/* mqtt server port number */
	int serverport[STRINGBUFSIZE];

	/* maximum number of connections */
	int maxconn;
	/* number of tries of request in case timeout (0 - no tries attempted) */
	int maxtry;
	/* staled connection timeout (in sec) */
	int conntimeout;

	char mqtt_subscribe_qos0[STRINGBUFSIZE];
	char mqtt_subscribe_qos1[STRINGBUFSIZE];
	char mqtt_subscribe_qos2[STRINGBUFSIZE];

	char mqtt_publish_qos0[STRINGBUFSIZE];
	
} config_t;

/* Prototypes */
extern config_t config;
extern char cfg_err[];
void cfg_init(void);
int cfg_read_file(const char *filename);
void dump_config();

#endif /* _CFG_H */
