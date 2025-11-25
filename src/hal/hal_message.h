#ifndef MYMESSAGE
#define MYMESSAGE
#include <Arduino.h>
#include <stdarg.h> 
#include <array>
#include "type_defs.h"

#ifndef ESP_H
#include <avr/dtostrf.h>
#endif



enum MSG_ID {
  MSG_NONE = 0,
  MSG_AHRS_DATA,
  MSG_DEBUG_STR = 0xFE,
};

enum OutputMode{
  OUT_MODE_MAVLINK, 
  OUT_MODE_ML_ODOM, 
  OUT_MODE_ML_VISO, 
  OUT_MODE_ML_GNSS,
  OUT_MODE_NMEA,
  OUT_MODE_BIN,
  OUT_MODE_XBUS,
  OUT_MODE_STR,
  OUT_MODE_CONFIG,
  OUT_MODE_GNSS,
  OUT_MODE_NONE
};


const uint8_t HEADER[2] = {0xFA, 0xFF};
const byte HEADER_SIZE = 2;

uint8_t cal_xor_checksum(uint8_t *msg, int num_msg);
bool xor_checksum(uint8_t *msg, int num_msg, uint8_t checksum);

void convert2Sign_8B(double *value, uint8_t *buf);
void convert2Sign_4B(uint32_t* value, uint8_t* buf);
void convert2Sign_2B(uint16_t* value, uint8_t* buf);
void debug_print(Print& output, const char* format, ...);

void checkSTRCommand(OutputMode &output_mode, Stream &port=Serial);
uint8_t readMessage(Stream &port, uint8_t *buffer);
bool hal_msg_getAHRS_Data(AHRS_Data &data);

enum READING_DATA_STATE{
  EXPECTING_HEADER,
  EXPECTING_MID,
  EXPECTING_LEN,
  EXPECTING_PAYLOAD,
  EXPECTING_CHECKSUM,
  EXPECTING_TRAILER
};

#endif