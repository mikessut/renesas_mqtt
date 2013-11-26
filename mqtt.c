#include <mqtt.h>
#include <string.h>
#include <stdio.h>

#include "system/platform.h"
#include <system/Switch.h>
#include <sensors\Potentiometer.h>
#include <drv\Glyph\lcd.h>

#include "AtCmdLib/AtCmdLib.h"
#include "led.h"

// values below can be found by logging into the m2m.io portal (http://app.m2m.io)
// Username:  your email address you used to create the account
// Password:  MD5 hash of your password (32 character string)
// Domain:  Randomly generated.  Can be found on Accounts tab in portal.
// Device Type:  Can remain "things".  If this device is a commonly supported
//               device on the m2m.io portal there may be a type to choose.
//               For example, Arduino.
// Device ID: A string uniquely identifying this device from your other
//            devices.  Only needs to be unique to you, not across all users.
//            Common examples include device's MAC address or serial number.
//            Device-1, Device-2 are fine too.

#define  M2MIO_USERNAME   	"demo@renesas.com"
#define  M2MIO_PASSWORD   	"4c7a34d25eff9121c49658dbceadf694"    // MD5 key of password
#define  M2MIO_DOMAIN     	"com.renesas"
#define  M2MIO_DEVICE_TYPE	"rl78"
#define M2MIO_BROKER_HOSTNAME   "q.m2m.io"
#define	M2MIO_BROKER_PORT	1883



extern int atoi(const char *_S);
void led_on(int n);
void led_off(int n);
uint8_t led_get(int n);


uint8_t cid;

#define PACKET_BUF_SIZE 120
uint8_t packet_buffer[PACKET_BUF_SIZE];   // This is used to contruct the packets set to MQTT broker

extern uint8_t G_received[APP_MAX_RECEIVED_DATA + 1];
extern unsigned int G_receivedCount;

void App_PrepareIncomingData(void);

extern int16_t *Accelerometer_Get(void);
extern uint16_t Temperature_Get(void);
extern int16_t	gAccData[3];

#define PUB_POT_MODE 1
#define PUB_ACCEL_MODE 0


// json helper routines
void initJsonMsg(char *buffer) {
   sprintf(buffer, "%s", "{");
}

void finishJsonMsg(char *buffer) {
   sprintf(buffer+strlen(buffer), "%s", "}");
}

void addStringValToMsg(const char *n, const char *v, char *buffer) {
   if (strlen(buffer) > 1) {
      sprintf(buffer+strlen(buffer), ",");
   }
   sprintf(buffer+strlen(buffer), "\"%s\":", n);
   sprintf(buffer+strlen(buffer), "\"%s\"", v);
}

void addIntValToMsg(const char *n, const int l, char *buffer) {
   if (strlen(buffer) > 1) {
      sprintf(buffer+strlen(buffer), ",");
   }
   sprintf(buffer+strlen(buffer), "\"%s\":", n);
   sprintf(buffer+strlen(buffer), "%d", l);
}

void addDoubleValToMsg(const char *n, const double d, char *buffer) {
   if (strlen(buffer) > 1) {
      sprintf(buffer+strlen(buffer), ",");
   }
   sprintf(buffer+strlen(buffer), "\"%s\":", n);
   sprintf(buffer+strlen(buffer), "%g", d);
}

int send_packet(void* socket_info, const void* buf, unsigned int count) {
    int err;
	uint8_t fd = *((uint8_t*)socket_info);
        err = AtLibGs_SendTCPData(fd,buf,count);
//	return send(fd, (char*)buf, count, 0);
	//GSsendPacket(buf,count, fd);  // This is one of the few spots to custimize for 
										 // different TCP/IP stack implementations.
	//GSreadEscO();
	return err;
}

int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port) {
//	int flag = 1;
	int keepalive = 300; // Seconds
	char buf[100];
        char ip[17];
        char *p;
	
	
	// create socket (get cid)
	sprintf(buf,"%d",port);
                
        AtLibGs_DNSLookup((char*)hostname);
        AtLibGs_ParseDNSLookupResponse(ip);
	
        p = strstr(ip,"\r\nOK");   // Parse doesn't remove "OK" at end of string??
        *p = '\0';
        AtLibGs_FlushRxBuffer();
        
        AtLibGs_TCPClientStart(ip, port,&cid);
		
	// MQTT stuffs
	mqtt_set_alive(broker, keepalive);
	broker->socket_info = (void*)&cid;
	broker->send = send_packet;

	return 0;
}

int close_socket(mqtt_broker_handle_t* broker) {
//	int fd = *((int*)broker->socket_info);
//	return close(fd);
    return 0;
}


ATLIBGS_TCPMessage rxm;


int read_packet(int timeout) {
  
  AtLibGs_WaitForTCPMessage(timeout);
  AtLibGs_ParseTCPData(G_received,G_receivedCount,&rxm);
  
  return rxm.numBytes;
}



int m2mtest() {
	int16_t packet_length;
	
	mqtt_broker_handle_t broker;
	
	char clientIDStr[100];
	char pubTopic[100];
        char pubMsgStr[50];
        uint8_t msg[25];
        uint16_t l;
	//uint8_t mode = PUB_ACCEL_MODE;
        
        uint8_t loop = 0;
        uint8_t button = 0;
        uint8_t buttonDelay = 0;
        uint8_t led_n;
        char tmpStr[25];

        uint16_t temperature;
        
	// now connect using user/password, publish sensor values on
	// appropriate topic <domain>/<device type>/<device id>

        AtLibGs_GetMAC((char *)msg);
        uint8_t i = 0;
        uint8_t j = 0;
        uint8_t myMac[13];
        for (i=0; i<sizeof(msg); i++) {
          if (msg[i] != 0x3A) {
            myMac[j] = msg[i];
            j++;
          }
        }
        myMac[j] = '\0';
        DisplayLCD(LCD_LINE6, myMac);

	
	sprintf(clientIDStr, "%s", myMac);
	
	mqtt_init(&broker, clientIDStr);
	mqtt_init_auth(&broker, M2MIO_USERNAME, M2MIO_PASSWORD);

        AtLibGs_FlushIncomingMessage();
	init_socket(&broker, M2MIO_BROKER_HOSTNAME, M2MIO_BROKER_PORT);
	
        AtLibGs_FlushIncomingMessage();
        App_PrepareIncomingData();
        
	mqtt_connect(&broker);
	
	// wait for CONNACK	
	packet_length = read_packet(6000);
	
		
	if(packet_length < 0) {
		//printf("Error reading packet.\n");
                DisplayLCD(LCD_LINE4, "packet error");
		return -1;
	}
	
	if(MQTTParseMessageType(rxm.message) != MQTT_MSG_CONNACK) { // MQTT_MSG_CONNACK       2<<4
            DisplayLCD(LCD_LINE4, "connack error");
            return -2;
	}
	
	if(rxm.message[3] != 0x00) {
		return -2;
	}
        
        App_PrepareIncomingData();
        AtLibGs_FlushIncomingMessage();

        // subscribe        
        // Temporarily use pubTopic variable for subscribe
        sprintf(pubTopic, "%s/%s/%s/%s", M2MIO_DOMAIN, M2MIO_DEVICE_TYPE, myMac, "cmd");
        mqtt_subscribe(&broker, pubTopic, NULL);
        
        sprintf(pubTopic, "%s/%s/%s", M2MIO_DOMAIN, M2MIO_DEVICE_TYPE, myMac);
	
	while (1) {
          
          // Look for incoming data
          AtLibGs_WaitForTCPMessage(250);
          if (G_receivedCount > 0) {
            AtLibGs_ParseTCPData(G_received,G_receivedCount,&rxm);
            
            if (MQTTParseMessageType(rxm.message) == MQTT_MSG_SUBACK) {
              DisplayLCD(LCD_LINE2, "sub success");  // don't expect this with QoS 0
            } else if (MQTTParseMessageType(rxm.message) == MQTT_MSG_PUBLISH) {
              l = mqtt_parse_publish_msg(rxm.message,msg);
              msg[l] = '\0';
              if (strncmp("{\"LEDS\":\"ON\"}", (char const *)msg, 13) == 0) {
                led_all_on();
              } else if (strncmp("{\"LEDS\":\"OFF\"}", (char const *)msg, 14) == 0) {
                led_all_off();
              } else if (strncmp("{\"LED", (char const *)msg, 5) == 0) {
                led_n = 10*(msg[8]-48)+msg[9]-48;
                sprintf(tmpStr, "led_n:%d", led_n);
                DisplayLCD(LCD_LINE3, (uint8_t *)tmpStr);
                if (led_get(led_n) == 0) {
                  led_on(led_n);
                } else {
                  led_off(led_n);
                }
              } else {
                DisplayLCD(LCD_LINE3, msg);
              }
            }
               
            App_PrepareIncomingData();
          }
          
          if (Switch1IsPressed()) {
            button = 1;
          }
          
          if (button == 1 && buttonDelay == 0) {
            button = 0;
            buttonDelay = 8;
            // build publish message payload string
            initJsonMsg(pubMsgStr);
            addStringValToMsg("b1", "pressed", pubMsgStr);
            finishJsonMsg(pubMsgStr);
              
            // publish message            
            mqtt_publish(&broker, pubTopic, pubMsgStr, 0);
          }
          if (buttonDelay > 0) {
            buttonDelay--;
          }

          // publish sensor every 4th loop (roughly 1 second)
          if (loop == 0) {
            // build publish message payload string
            initJsonMsg(pubMsgStr);

            //addIntValToMsg("t", 12, pubMsgStr);
            Accelerometer_Get();
            
            temperature = Temperature_Get();
            
            addIntValToMsg("x",gAccData[0], pubMsgStr);
            addIntValToMsg("y",gAccData[1], pubMsgStr);
            addIntValToMsg("z",gAccData[2], pubMsgStr);
            addIntValToMsg("t",temperature>>3, pubMsgStr);
            addIntValToMsg("p",Potentiometer_Get(), pubMsgStr);            
            finishJsonMsg(pubMsgStr);
            
            // publish message            
            mqtt_publish(&broker, pubTopic, pubMsgStr, 0);
          }
          loop = loop >= 4 ? 0 : loop+1;
	}
}


// mqtt specific helper functions


#define MQTT_DUP_FLAG     1<<3
#define MQTT_QOS0_FLAG    0<<1
#define MQTT_QOS1_FLAG    1<<1
#define MQTT_QOS2_FLAG    2<<1

#define MQTT_RETAIN_FLAG  1

#define MQTT_CLEAN_SESSION  1<<1
#define MQTT_WILL_FLAG      1<<2
#define MQTT_WILL_RETAIN    1<<5
#define MQTT_USERNAME_FLAG  1<<7
#define MQTT_PASSWORD_FLAG  1<<6



uint8_t mqtt_num_rem_len_bytes(const uint8_t* buf) {
	uint8_t num_bytes = 1;
	
	//printf("mqtt_num_rem_len_bytes\n");
	
	if ((buf[1] & 0x80) == 0x80) {
		num_bytes++;
		if ((buf[2] & 0x80) == 0x80) {
			num_bytes ++;
			if ((buf[3] & 0x80) == 0x80) {
				num_bytes ++;
			}
		}
	}
	return num_bytes;
}

uint16_t mqtt_parse_rem_len(const uint8_t* buf) {
	uint16_t multiplier = 1;
	uint16_t value = 0;
	uint8_t digit;
	
	//printf("mqtt_parse_rem_len\n");
	
	buf++;	// skip "flags" byte in fixed header

	do {
		digit = *buf;
		value += (digit & 127) * multiplier;
		multiplier *= 128;
		buf++;
	} while ((digit & 128) != 0);

	return value;
}

uint8_t mqtt_parse_msg_id(const uint8_t* buf) {
	uint8_t type = MQTTParseMessageType(buf);
	uint8_t qos = MQTTParseMessageQos(buf);
	uint8_t id = 0;
	
	//printf("mqtt_parse_msg_id\n");
	
	if(type >= MQTT_MSG_PUBLISH && type <= MQTT_MSG_UNSUBACK) {
		if(type == MQTT_MSG_PUBLISH) {
			if(qos != 0) {
				// fixed header length + Topic (UTF encoded)
				// = 1 for "flags" byte + rlb for length bytes + topic size
				uint8_t rlb = mqtt_num_rem_len_bytes(buf);
				uint8_t offset = *(buf+1+rlb)<<8;	// topic UTF MSB
				offset |= *(buf+1+rlb+1);			// topic UTF LSB
				offset += (1+rlb+2);					// fixed header + topic size
				id = *(buf+offset)<<8;				// id MSB
				id |= *(buf+offset+1);				// id LSB
			}
		} else {
			// fixed header length
			// 1 for "flags" byte + rlb for length bytes
			uint8_t rlb = mqtt_num_rem_len_bytes(buf);
			id = *(buf+1+rlb)<<8;	// id MSB
			id |= *(buf+1+rlb+1);	// id LSB
		}
	}
	return id;
}

uint16_t mqtt_parse_pub_topic(const uint8_t* buf, uint8_t* topic) {
	const uint8_t* ptr;
	uint16_t topic_len = mqtt_parse_pub_topic_ptr(buf, &ptr);
	
	//printf("mqtt_parse_pub_topic\n");
	
	if(topic_len != 0 && ptr != NULL) {
		memcpy(topic, ptr, topic_len);
	}
	
	return topic_len;
}

uint16_t mqtt_parse_pub_topic_ptr(const uint8_t* buf, const uint8_t **topic_ptr) {
	uint16_t len = 0;
	
	//printf("mqtt_parse_pub_topic_ptr\n");

	if(MQTTParseMessageType(buf) == MQTT_MSG_PUBLISH) {
		// fixed header length = 1 for "flags" byte + rlb for length bytes
		uint8_t rlb = mqtt_num_rem_len_bytes(buf);
		len = *(buf+1+rlb)<<8;	// MSB of topic UTF
		len |= *(buf+1+rlb+1);	// LSB of topic UTF
		// start of topic = add 1 for "flags", rlb for remaining length, 2 for UTF
		*topic_ptr = (buf + (1+rlb+2));
	} else {
		*topic_ptr = NULL;
	}
	return len;
}

uint16_t mqtt_parse_publish_msg(const uint8_t* buf, uint8_t* msg) {
	const uint8_t* ptr;
	
	//printf("mqtt_parse_publish_msg\n");
	
	uint16_t msg_len = mqtt_parse_pub_msg_ptr(buf, &ptr);
	
	if(msg_len != 0 && ptr != NULL) {
		memcpy(msg, ptr, msg_len);
	}
	
	return msg_len;
}

uint16_t mqtt_parse_pub_msg_ptr(const uint8_t* buf, const uint8_t **msg_ptr) {
	uint16_t len = 0;
	
	//printf("mqtt_parse_pub_msg_ptr\n");
	
	if(MQTTParseMessageType(buf) == MQTT_MSG_PUBLISH) {
		// message starts at
		// fixed header length + Topic (UTF encoded) + msg id (if QoS>0)
		uint8_t rlb = mqtt_num_rem_len_bytes(buf);
		uint8_t offset = (*(buf+1+rlb))<<8;	// topic UTF MSB
		offset |= *(buf+1+rlb+1);			// topic UTF LSB
		offset += (1+rlb+2);				// fixed header + topic size

		if(MQTTParseMessageQos(buf)) {
			offset += 2;					// add two bytes of msg id
		}

		*msg_ptr = (buf + offset);
				
		// offset is now pointing to start of message
		// length of the message is remaining length - variable header
		// variable header is offset - fixed header
		// fixed header is 1 + rlb
		// so, lom = remlen - (offset - (1+rlb))
      	len = mqtt_parse_rem_len(buf) - (offset-(rlb+1));
	} else {
		*msg_ptr = NULL;
	}
	return len;
}

void mqtt_init(mqtt_broker_handle_t* broker, const char* clientid) {
	// Connection options
	broker->alive = 300; // 300 seconds = 5 minutes
	broker->seq = 1; // Sequency for message indetifiers
	// Client options
	memset(broker->clientid, 0, sizeof(broker->clientid));
	memset(broker->username, 0, sizeof(broker->username));
	memset(broker->password, 0, sizeof(broker->password));
	if(clientid) {
		strncpy(broker->clientid, clientid, sizeof(broker->clientid));
	} else {
		strcpy(broker->clientid, "emqtt");
	}
	// Will topic
	broker->clean_session = 1;
}

void mqtt_init_auth(mqtt_broker_handle_t* broker, const char* username, const char* password) {
	if(username && username[0] != '\0')
		strncpy(broker->username, username, sizeof(broker->username)-1);
	if(password && password[0] != '\0')
		strncpy(broker->password, password, sizeof(broker->password)-1);
}

void mqtt_set_alive(mqtt_broker_handle_t* broker, uint16_t alive) {
	broker->alive = alive;
}

int mqtt_connect(mqtt_broker_handle_t* broker)
{
	uint8_t flags = 0x00;

	uint16_t clientidlen = strlen(broker->clientid);
	uint16_t usernamelen = strlen(broker->username);
	uint16_t passwordlen = strlen(broker->password);
	uint16_t payload_len = clientidlen + 2;

	// Variable header
        uint8_t var_header[] = {
                0x00,0x06,0x4d,0x51,0x49,0x73,0x64,0x70, // Protocol name: MQIsdp
                0x03, // Protocol version
                flags, // Connect flags
                broker->alive>>8, broker->alive&0xFF, // Keep alive
        };

        uint8_t fixedHeaderSize = 2;
        uint8_t remainLen;
        uint8_t fixed_header[3];

        uint16_t offset;

        uint16_t packetLen;

	// Preparing the flags
	if(usernamelen) {
		payload_len += usernamelen + 2;
		flags |= MQTT_USERNAME_FLAG;
	}
	if(passwordlen) {
		payload_len += passwordlen + 2;
		flags |= MQTT_PASSWORD_FLAG;
	}
	if(broker->clean_session) {
		flags |= MQTT_CLEAN_SESSION;
	}

	var_header[9] = flags;



    remainLen = sizeof(var_header)+payload_len;
    if (remainLen > 127) {
        fixedHeaderSize++;          // add an additional byte for Remaining Length
    }
    // Message Type
    fixed_header[0] = MQTT_MSG_CONNECT;

    // Remaining Length
    if (remainLen <= 127) {
        fixed_header[1] = remainLen;
    } else {
        // first byte is remainder (mod) of 128, then set the MSB to indicate more bytes
        fixed_header[1] = remainLen % 128;
        fixed_header[1] = fixed_header[1] | 0x80;
        // second byte is number of 128s
        fixed_header[2] = remainLen / 128;
    }

    //return 0; // making it to this point
	offset = 0;
	packetLen = fixedHeaderSize + sizeof(var_header) + payload_len;
	// Make big enough for most applications without have to do malloc type operations.
	//uint8_t packet[3 + sizeof(var_header) + 100];     //[sizeof(fixed_header)+sizeof(var_header)+payload_len];
	memset(packet_buffer, 0, sizeof(packet_buffer));
	//return 0;  // making it here
	memcpy(packet_buffer, fixed_header, fixedHeaderSize);
	offset += fixedHeaderSize; // sizeof(fixed_header);
	memcpy(packet_buffer+offset, var_header, sizeof(var_header));
	offset += sizeof(var_header);
	// Client ID - UTF encoded
	packet_buffer[offset++] = clientidlen>>8;
	packet_buffer[offset++] = clientidlen&0xFF;
	memcpy(packet_buffer+offset, broker->clientid, clientidlen);
	offset += clientidlen;

	if(usernamelen) {
		// Username - UTF encoded
		packet_buffer[offset++] = usernamelen>>8;
		packet_buffer[offset++] = usernamelen&0xFF;
		memcpy(packet_buffer+offset, broker->username, usernamelen);
		offset += usernamelen;
	}

	if(passwordlen) {
		// Password - UTF encoded
		packet_buffer[offset++] = passwordlen>>8;
		packet_buffer[offset++] = passwordlen&0xFF;
		memcpy(packet_buffer+offset, broker->password, passwordlen);
		offset += passwordlen;
	}

	if(broker->send(broker->socket_info, packet_buffer, packetLen) < packetLen) {
		return -1;
	}

	return 1;
}

int mqtt_disconnect(mqtt_broker_handle_t* broker) {
	uint8_t packet[] = {
		MQTT_MSG_DISCONNECT, // Message Type, DUP flag, QoS level, Retain
		0x00 // Remaining length
	};

	// Send the packet
	if(broker->send(broker->socket_info, packet, sizeof(packet)) < sizeof(packet)) {
		return -1;
	}

	return 1;
}

int mqtt_ping(mqtt_broker_handle_t* broker) {
	uint8_t packet[] = {
		MQTT_MSG_PINGREQ, // Message Type, DUP flag, QoS level, Retain
		0x00 // Remaining length
	};

	// Send the packet
	if(broker->send(broker->socket_info, packet, sizeof(packet)) < sizeof(packet)) {
		return -1;
	}

	return 1;
}

int mqtt_publish(mqtt_broker_handle_t* broker, const char* topic, const char* msg, uint8_t retain) {
	return mqtt_publish_with_qos(broker, topic, msg, retain, 0, NULL);
}

int mqtt_publish_with_qos(mqtt_broker_handle_t* broker, const char* topic, const char* msg, uint8_t retain, uint8_t qos, uint16_t* message_id) {
	uint16_t topiclen = strlen(topic);
	uint16_t msglen = strlen(msg);

	uint8_t qos_flag = MQTT_QOS0_FLAG;
	uint8_t qos_size = 0; // No QoS included
	if(qos == 1) {
		qos_size = 2; // 2 bytes for QoS
		qos_flag = MQTT_QOS1_FLAG;
	}
	else if(qos == 2) {
		qos_size = 2; // 2 bytes for QoS
		qos_flag = MQTT_QOS2_FLAG;
	}

	// Variable header
	// Make big enough without having to do dynamic memory allocation.
	//uint8_t var_header[100];  //[topiclen+2+qos_size]; // Topic size (2 bytes), utf-encoded topic
	// Rework this for memory saving purposes.  Write directly to packet_buffer

	// packet_buffer ends up being:
	// fixed header (2 or 3 bytes)
	// var header
	// msg
	uint16_t varHeaderSize = topiclen+2+qos_size;
	//memset(var_header, 0, sizeof(var_header));
	memset(packet_buffer,0,PACKET_BUF_SIZE);

	uint8_t fixedHeaderSize = 2;    // Default size = one byte Message Type + one byte Remaining Length
	uint16_t remainLen = varHeaderSize+msglen;
	if (remainLen > 127) {
		fixedHeaderSize++;          // add an additional byte for Remaining Length
	}

	// Write fixed header
	// Fixed header
	// the remaining length is one byte for messages up to 127 bytes, then two bytes after that
	// actually, it can be up to 4 bytes but I'm making the assumption the embedded device will only
	// need up to two bytes of length (handles up to 16,383 (almost 16k) sized message)
	// Message Type, DUP flag, QoS level, Retain
	packet_buffer[0] = MQTT_MSG_PUBLISH | qos_flag;
	if(retain) {
		packet_buffer[0] |= MQTT_RETAIN_FLAG;
	}
	// Remaining Length
	uint16_t offset;
	if (remainLen <= 127) {
		packet_buffer[1] = remainLen;
		offset = 2;
	} else {
	   // first byte is remainder (mod) of 128, then set the MSB to indicate more bytes
		packet_buffer[1] = remainLen % 128;
		packet_buffer[1] = packet_buffer[1] | 0x80;
	   // second byte is number of 128s
		packet_buffer[2] = remainLen / 128;
		offset = 3;
	}

	// write var header
	packet_buffer[offset++] = topiclen>>8;
	packet_buffer[offset++] = topiclen&0xFF;
	//memcpy(var_header+2, topic, topiclen);
	memcpy(packet_buffer+offset,topic,topiclen);
	offset += topiclen;
	if(qos_size) {
		packet_buffer[offset++] = broker->seq>>8;
		packet_buffer[offset++] = broker->seq&0xFF;
		if(message_id) { // Returning message id
			*message_id = broker->seq;
		}
		broker->seq++;
	}

	// Write msg
	memcpy(packet_buffer+offset,msg,msglen);

	//uint8_t packet[200];  //[sizeof(fixed_header)+sizeof(var_header)+msglen];
	uint16_t packetSize = fixedHeaderSize + varHeaderSize + msglen;
	//memset(packet, 0, sizeof(packet));
	//memcpy(packet, fixed_header, fixedHeaderSize);
	//memcpy(packet+fixedHeaderSize, var_header, varHeaderSize);
	//memcpy(packet+fixedHeaderSize+varHeaderSize, msg, msglen);

	// Send the packet
	if(broker->send(broker->socket_info, packet_buffer, packetSize) < packetSize) {
		return -1;
	}

	return 1;
}

int mqtt_pubrel(mqtt_broker_handle_t* broker, uint16_t message_id) {
	uint8_t packet[] = {
		MQTT_MSG_PUBREL | MQTT_QOS1_FLAG, // Message Type, DUP flag, QoS level, Retain
		0x02, // Remaining length
		message_id>>8,
		message_id&0xFF
	};

	// Send the packet
	if(broker->send(broker->socket_info, packet, sizeof(packet)) < sizeof(packet)) {
		return -1;
	}

	return 1;
}

int mqtt_subscribe(mqtt_broker_handle_t* broker, const char* topic, uint16_t* message_id) {
	uint16_t topiclen = strlen(topic);

	// Variable header
	uint8_t var_header[2]; // Message ID
	var_header[0] = broker->seq>>8;
	var_header[1] = broker->seq&0xFF;
	if(message_id) { // Returning message id
		*message_id = broker->seq;
	}
	broker->seq++;

	// utf topic
	uint8_t utf_topic[100];    //[topiclen+3]; // Topic size (2 bytes), utf-encoded topic, QoS byte
	uint16_t topicSize = topiclen+3;
	memset(utf_topic, 0, sizeof(utf_topic));
	utf_topic[0] = topiclen>>8;
	utf_topic[1] = topiclen&0xFF;
	memcpy(utf_topic+2, topic, topiclen);

	// Fixed header
	uint8_t fixed_header[] = {
		MQTT_MSG_SUBSCRIBE | MQTT_QOS1_FLAG, // Message Type, DUP flag, QoS level, Retain
		sizeof(var_header)+topicSize
	};

	uint8_t packet[2 + 2 + 100];    //[sizeof(var_header)+sizeof(fixed_header)+sizeof(utf_topic)];
	uint16_t packetSize = sizeof(var_header)+sizeof(fixed_header)+topicSize;
	memset(packet, 0, sizeof(packet));
	memcpy(packet, fixed_header, sizeof(fixed_header));
	memcpy(packet+sizeof(fixed_header), var_header, sizeof(var_header));
	memcpy(packet+sizeof(fixed_header)+sizeof(var_header), utf_topic, topicSize);

	// Send the packet
	if(broker->send(broker->socket_info, packet, packetSize) < packetSize) {
		return -1;
	}

	return 1;
}

int mqtt_unsubscribe(mqtt_broker_handle_t* broker, const char* topic, uint16_t* message_id) {
	uint16_t topiclen = strlen(topic);

	// Variable header
	uint8_t var_header[2]; // Message ID
	var_header[0] = broker->seq>>8;
	var_header[1] = broker->seq&0xFF;
	if(message_id) { // Returning message id
		*message_id = broker->seq;
	}
	broker->seq++;

	// utf topic
	uint8_t utf_topic[100];  //[topiclen+2]; // Topic size (2 bytes), utf-encoded topic
	uint16_t topicSize = topiclen+2;
	memset(utf_topic, 0, sizeof(utf_topic));
	utf_topic[0] = topiclen>>8;
	utf_topic[1] = topiclen&0xFF;
	memcpy(utf_topic+2, topic, topiclen);

	// Fixed header
	uint8_t fixed_header[] = {
		MQTT_MSG_UNSUBSCRIBE | MQTT_QOS1_FLAG, // Message Type, DUP flag, QoS level, Retain
		sizeof(var_header)+sizeof(utf_topic)
	};

	uint8_t packet[2 + 2 + 100];  //[sizeof(var_header)+sizeof(fixed_header)+sizeof(utf_topic)];
	uint16_t packetSize = sizeof(var_header)+sizeof(fixed_header)+topicSize;
	memset(packet, 0, sizeof(packet));
	memcpy(packet, fixed_header, sizeof(fixed_header));
	memcpy(packet+sizeof(fixed_header), var_header, sizeof(var_header));
	memcpy(packet+sizeof(fixed_header)+sizeof(var_header), utf_topic, topicSize);

	// Send the packet
	if(broker->send(broker->socket_info, packet, packetSize) < packetSize) {
		return -1;
	}

	return 1;
}
