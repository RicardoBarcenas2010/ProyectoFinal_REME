#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <stdint.h>

typedef enum
{
    CONTROL_MODE_MASTER = 0,
    CONTROL_MODE_SETPOINT = 1
} control_mode_t;

typedef struct
{
    float master_angle;
    float follower_angle;
    float setpoint_angle;
    float pwm_left;
    float pwm_right;

} telemetry_packet_t;


/* Comandos enviados por la pantalla */
typedef struct
{
    uint8_t control_mode;
    float manual_setpoint;

} screen_command_t;

#endif