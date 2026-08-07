#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

typedef struct
{
    float master_angle;

    float follower_angle;

    float setpoint_angle;

    float pwm_left;

    float pwm_right;

} telemetry_packet_t;

#endif