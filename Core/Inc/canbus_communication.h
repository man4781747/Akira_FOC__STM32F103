#ifndef CANBUS_COMM_H
#define CANBUS_COMM_H

#include <stdint.h>

typedef enum {
    STOP_MOTOR,
    SET_SPEED,
    SET_POSITION,
    SET_IQ,
    SET_ANG_SHIFT
} Commands;

typedef enum {
    Set_TargetVale,
    Set_PID_P,
    Set_PID_I,
    Set_PID_D
} CommandType;

typedef struct __attribute__((packed)) {
    Commands command;
    CommandType type;
    uint8_t value_bytes[4];
    uint8_t no_use;
    uint8_t no_use2;
} CommandDatas;




#endif