# include "hal_message.h"

#define MAX_PRINTF_BUFFER_SIZE 64


// num_msg "not" include the checksum
uint8_t cal_xor_checksum(uint8_t *msg, int num_msg){
    uint8_t xor_result = 0;
    for (int i=0;i<num_msg;i++){
        xor_result ^= msg[i];
    }
    return xor_result;
}

// num_msg "not" include the checksum
bool xor_checksum(uint8_t *msg, int num_msg, uint8_t checksum){
    return cal_xor_checksum(msg, num_msg) == checksum;
}

void convert2Sign_8B(double *value, uint8_t *buf) {
    uint8_t temp[8];
    for (int i = 0; i < 8; i++) {
        temp[i] = buf[7 - i];  // 反轉順序
    }
    memcpy(value, temp, 8);  // 直接將 8 Byte 複製到 double
}

void convert2Sign_4B(uint32_t* value, uint8_t* buf){
    *value = *buf<<24 | *(buf+1)<<16 | *(buf+2)<<8 | *(buf+3);
}
  

void convert2Sign_2B(uint16_t* value, uint8_t* buf){
    *value = *buf<<8 | *(buf+1);
}

void debug_print(Print& output, const char* format, ...) {
  char buffer[MAX_PRINTF_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, MAX_PRINTF_BUFFER_SIZE, format, args);
  va_end(args);
  output.println(buffer);
}



void checkSTRCommand(OutputMode &output_mode, Stream &port){
    static const uint8_t MAX_COMMAND_LENGTH = 10;
    static char command_buffer[MAX_COMMAND_LENGTH + 1];
    static uint8_t buffer_index = 0;

    while (port.available()) {
        char c = port.read();
        if (c == '\r' || c == '\n'){continue;}      
    
        if (buffer_index < MAX_COMMAND_LENGTH){
            command_buffer[buffer_index++] = c;
            command_buffer[buffer_index] = '\0';
        } else {
            buffer_index = 0;
            command_buffer[0] = '\0';
        }
    
        if (buffer_index >= 3){
            if (strcmp(command_buffer, "STR") == 0) {
                port.println("Output mode: STR");
                output_mode = OUT_MODE_STR;
            } 
            else if (strcmp(command_buffer, "BIN") == 0) {
                port.println("Output mode: BIN");
                output_mode = OUT_MODE_BIN;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "XBUS") == 0) {
                port.println("Output mode: XBUS");
                output_mode = OUT_MODE_XBUS;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "GNSS") == 0) {
                port.println("Output mode: GNSS");
                output_mode = OUT_MODE_GNSS;
            } 
            else if (buffer_index >= 4 && strcmp(command_buffer, "NONE") == 0) {
                port.println("Output mode: None");
                output_mode = OUT_MODE_NONE;
            } 
            else if (buffer_index >= 6 && strcmp(command_buffer, "CONFIG") == 0) {
                port.println("Output mode: CONFIG");
                output_mode = OUT_MODE_CONFIG;
            } 
            else if (buffer_index >= 7 && strcmp(command_buffer, "MAVLINK") == 0) {
                port.println("Output mode: MAVLINK");
                output_mode = OUT_MODE_MAVLINK;
            } 
        }
        return;
    }
    
    if (buffer_index > 0){
        buffer_index = 0;
        command_buffer[0] = '\0';
    }    
}
  
uint8_t readMessage(Stream &port, uint8_t *input_buffer){
    static uint8_t idx = 0;
    static uint8_t buffer[64];

    static enum{
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_ID,
        WAITING_LEN,
        WAITING_PAYLOAD,
        WAITING_CHECKSUM
    } state = WAITING_HEADER1;


    while (port.available() && idx < 128) {
        uint8_t byte = port.read();

        switch (state)
        {
        case WAITING_HEADER1:{
            if (byte == HEADER[0]) {
                state = WAITING_HEADER2;
            }
            break;
        }
        case WAITING_HEADER2:{
            if (byte == HEADER[1]) {
                state = WAITING_ID;
            } else {
                state = WAITING_HEADER1;
            }
            break;
        }
        case WAITING_ID:{
            buffer[idx++] = byte;
            state = WAITING_LEN;
            break;
        }
        case WAITING_LEN:{
            buffer[idx++] = byte;
            state = WAITING_PAYLOAD;
            break;
        }
        case WAITING_PAYLOAD:{
            buffer[idx++] = byte;
            if (idx >= 4 + buffer[3]) { // 4 = HEADER(2) + ID(1) + LEN(1)
                state = WAITING_CHECKSUM;
            }
            break;
        }
        case WAITING_CHECKSUM:{
            uint8_t checksum = 0;
            for (uint8_t i=0;i<buffer[3];i++) {
                checksum &= buffer[4 + i];
            }
            if (checksum == byte) {
                memcpy(input_buffer, buffer + 4, buffer[3]);
                idx = 0;
                state = WAITING_HEADER1;
                return buffer[2]; // return MSG_ID
            } else {
                idx = 0;
                state = WAITING_HEADER1;
            }
            break;
        }
        }
    }

    return MSG_NONE;
}


bool hal_msg_getAHRS_Data(AHRS_Data &data) {
  static uint32_t last_time = 0;
  uint32_t now = millis();
  // uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  uint8_t buffer[64];
  if (readMessage(UART_CTL, (uint8_t*)&data) == MSG_AHRS_DATA) {
    memcpy(&data.time_sec, buffer, 4);
    memcpy(data.gyro_dps, buffer + 4, 12);
    memcpy(data.acc_mps2, buffer + 16, 12);
    memcpy(data.euler_deg, buffer + 28, 12);
    last_time = now;
    return true;
  }

//   if (now - last_time > 3000) {
//     sys_state = CONFIGURING;
//   } else if ( now == last_time) {
//     sys_state = IMU_MEASURING;
//   }
  return false;
}

