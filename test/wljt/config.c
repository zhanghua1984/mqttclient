/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * cfg.c - configuration related procedures
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
 * $Id: cfg.c,v 1.3 2015/02/25 10:33:57 kapyar Exp $
 */

#include "config.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "stdio.h"

#define CFG_MAX_LINE_LENGTH 200
#define CFG_MAX_BUFSIZE		1024
#define CFG_NAME_MATCH(n) strcmp(n, name) == 0
#define CFG_VALUE_MATCH(n) strcasecmp(n, value) == 0

/* Global configuration storage variable */
config_t config;

/* Configuration error message */
char cfg_err[CFG_MAX_BUFSIZE];

#define CFG_ERR(s, v) snprintf(cfg_err, CFG_MAX_BUFSIZE, s, v)

/*
 * Setting up config defaults
 */
void cfg_init(void)
{
	strncpy(config.project, DEFALUT_PROJECT, sizeof(config.project));

	strncpy(config.devid, DEFALUT_DEVICE_ID, sizeof(config.devid));

	strncpy(config.ttyport, DEFAULT_PORT, sizeof(config.ttyport));
	config.ttyspeed = DEFAULT_SPEED;
	strncpy(config.ttymode, DEFAULT_MODE, sizeof(config.ttymode));

	strncpy(config.serveraddr, DEFAULT_SERVERADDR, sizeof(config.serveraddr));
	strncpy(config.serverport, DEFAULT_SERVERPORT, sizeof(config.serverport));

	config.maxconn = DEFAULT_MAXCONN;
	config.maxtry = DEFAULT_MAXTRY;
	config.conntimeout = DEFAULT_CONNTIMEOUT;

	strncpy(config.mqtt_subscribe_qos0, DEFAULT_MQTT_SUB_QOS0, sizeof(config.mqtt_subscribe_qos0));
	strncpy(config.mqtt_subscribe_qos1, DEFAULT_MQTT_SUB_QOS1, sizeof(config.mqtt_subscribe_qos1));
	strncpy(config.mqtt_subscribe_qos2, DEFAULT_MQTT_SUB_QOS2, sizeof(config.mqtt_subscribe_qos2));

	strncpy(config.mqtt_publish_qos0, DEFAULT_MQTT_PUB_QOS0, sizeof(config.mqtt_publish_qos0));
}

static char *cfg_rtrim(char *s)
{
  char *p = s + strlen(s);
  while (p > s && isspace((unsigned char )(*--p)))
    *p = '\0';
  return s;
}

static char * cfg_ltrim(const char *s)
{
  while (*s && isspace((unsigned char )(*s)))
    s++;
  return (char *) s;
}

int cfg_handle_param(char *name, char *value)
{
  if (CFG_NAME_MATCH("device"))
  {
    strncpy(config.ttyport, value, sizeof(config.ttyport));
  }
  else if (CFG_NAME_MATCH("devid"))
  {
	  strncpy(config.devid, value, sizeof(config.devid));
  }
  else if (CFG_NAME_MATCH("subqos0"))
  {
	  strncpy(config.mqtt_subscribe_qos0, value, sizeof(config.mqtt_subscribe_qos0));
  }
  else if (CFG_NAME_MATCH("subqos1"))
  {
	  strncpy(config.mqtt_subscribe_qos1, value, sizeof(config.mqtt_subscribe_qos1));
  }
  else if (CFG_NAME_MATCH("subqos2"))
  {
	  strncpy(config.mqtt_subscribe_qos2, value, sizeof(config.mqtt_subscribe_qos2));
  }
  else if (CFG_NAME_MATCH("pubqos0"))
  {
	  strncpy(config.mqtt_publish_qos0, value, sizeof(config.mqtt_publish_qos0));
  }
  else if (CFG_NAME_MATCH("project"))
  {
	  strncpy(config.project, value, sizeof(config.project));
  }
  else if (CFG_NAME_MATCH("speed"))
  {
    char *end;
    config.ttyspeed = strtoul(value, &end, 0);
    if (!config.ttyspeed || value == end || '\0' != *end)
    {
      CFG_ERR("invalid serial port speed: %s", value);
      return 0;
    }
  }
  else if (CFG_NAME_MATCH("mode"))
  {
    int mode_invalid;
    if (strlen(value) != 3)
      mode_invalid = 1;
    else
    {
      char parity = toupper(value[1]);
      mode_invalid = value[0] != '8' || (value[2] != '1' && value[2] != '2') ||
          (parity != 'N' && parity != 'E' && parity != 'O');
    }
    if (mode_invalid)
    {
      CFG_ERR("invalid device mode: %s", value);
      return 0;
    }
    strncpy(config.ttymode, value, sizeof(config.ttymode));
  }
  else if (CFG_NAME_MATCH("address"))
  {
    strncpy(config.serveraddr, value, sizeof(config.serveraddr));
  }
  else if (CFG_NAME_MATCH("port"))
  {
	strncpy(config.serverport, value, strlen(value));
  }
  else if (CFG_NAME_MATCH("maxconn"))
  {
    config.maxconn = strtoul(value, NULL, 0);
    if (config.maxconn < 1 || config.maxconn > DEFAULT_MAXCONN)
    {
      CFG_ERR("invalid maxconn value: %s", value);
      return 0;
    }
  }
  else if (CFG_NAME_MATCH("retries"))
  {
    config.maxtry = strtoul(value, NULL, 0);
    if (config.maxtry > DEFAULT_MAXTRY)
    {
      CFG_ERR("invalid retries value: %s", value);
      return 0;
    }
  }
#if 0
  else if (CFG_NAME_MATCH("pause"))
  {
    config.rqstpause = strtoul(value, NULL, 0);
    if (config.rqstpause < 1 || config.rqstpause > MAX_RQSTPAUSE)
    {
      CFG_ERR("invalid pause value: %s", value);
      return 0;
    }
  }
  else if (CFG_NAME_MATCH("wait"))
  {
    config.respwait = strtoul(value, NULL, 0);
    if (config.respwait < 1 || cfg.respwait > MAX_RESPWAIT)
    {
      CFG_ERR("invalid wait value: %s", value);
      return 0;
    }
  }
  #endif
  
  else if (CFG_NAME_MATCH("timeout"))
  {
    config.conntimeout = strtoul(value, NULL, 0);
    if (config.conntimeout > DEFAULT_CONNTIMEOUT)
      return 0;

  }
  
 #ifdef TRXCTL 
  else if (CFG_NAME_MATCH("trx_control"))
  {
    if (CFG_VALUE_MATCH("addc"))
    {
      config.trxcntl = TRX_ADDC;
    }
    else if (CFG_VALUE_MATCH("rts"))
    {
      config.trxcntl = TRX_RTS;
    }
    else if (CFG_VALUE_MATCH("sysfs_0"))
    {
      config.trxcntl = TRX_SYSFS_0;
    }
    else if (CFG_VALUE_MATCH("sysfs_1"))
    {
      config.trxcntl = TRX_SYSFS_1;
    }
    else
    {
      /* Unknown TRX control mode */
      CFG_ERR("unknown trx control mode: %s", value);
      return 0;
    }
  }
  else if (CFG_NAME_MATCH("trx_sysfile"))
  {
    strncpy(config.trxcntl_file, value, CFG_MAX_BUFSIZE);

  }
#endif
#ifdef LOG 
  else if (CFG_NAME_MATCH("loglevel"))
  {
    config.dbglvl = (char)strtol(optarg, NULL, 0);
  }
 #endif 
  else 
  {
    /* Unknown parameter name */
    CFG_ERR("unknown parameter: %s", name);
    return 0;
  }
  return 1;
}

int cfg_parse_file(void *file)
{
  char *line;
  char *start;
  char *end;
  char *name;
  char *value;
  int lineno = 0;
  int error = 0;

  *cfg_err = '\0';

  line = (char *) malloc(CFG_MAX_LINE_LENGTH);
  if (!line)
  {
    return -1;
  }

  while (fgets(line, CFG_MAX_LINE_LENGTH, file) != NULL)
  {
    lineno++;

    start = cfg_ltrim(cfg_rtrim(line));

    if (*start == '#')
    {
      /* skip comment */
      continue;
    }
    else if (*start)
    {
      /* parse `name=value` pair */
      for (end = start; *end && *end != '='; end++);
      if (*end == '=')
      {
        *end = '\0';

        name = cfg_rtrim(start);
        value = cfg_ltrim(cfg_rtrim(end + 1));

        /* handle name/value pair */
        if (!cfg_handle_param(name, value))
        {
          error = lineno;
          break;
        }
      }
      else
      {
        /* no '=' found on config line */
        error = lineno;
        CFG_ERR("can't parse line: %s", start);
        break;
      }
    }
  }

  free(line);

  return error;
}

int cfg_read_file(const char *filename)
{
  FILE* file;
  int error;

  file = fopen(filename, "r");
  if (!file)
    return -1;
  error = cfg_parse_file(file);
  fclose(file);
  return error;
}

void dump_config()
{
	printf("config.project = %s \n",config.project);
	printf("config.devid = %s \n",config.devid);
	printf("config.ttyport = %s \n",config.ttyport);
	printf("config.ttyspeed =%d \n",config.ttyspeed);
	printf("config.ttymode = %s \n",config.ttymode);
	printf("config.serveraddr = %s \n",config.serveraddr);
	printf("config.serverport = %s \n",config.serverport);
	printf("config.maxconn = %d \n",config.maxconn);
	printf("config.maxtry = %d \n",config.maxtry);
	printf("config.timeout = %d \n",config.conntimeout);
	
	printf("config.mqtt_subscribe_qos0 = %s \n",config.mqtt_subscribe_qos0);
	printf("config.mqtt_subscribe_qos1 = %s \n",config.mqtt_subscribe_qos1);
	printf("config.mqtt_subscribe_qos2 = %s \n",config.mqtt_subscribe_qos2);
	printf("config.mqtt_publish_qos0 = %s \n",config.mqtt_publish_qos0);
}

