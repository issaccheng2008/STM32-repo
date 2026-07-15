#ifndef FK723_HARDWARE_H
#define FK723_HARDWARE_H

#include "jetson_protocol.h"
#include <stdbool.h>
#include <stdint.h>

void Hardware_GetJointPositionRad(float output[PROTOCOL_NUM_JOINTS]);
void Hardware_GetJointVelocityRadS(float output[PROTOCOL_NUM_JOINTS]);
void Hardware_GetAccelerationMS2(float output[3]);
void Hardware_GetGyroRadS(float output[3]);
bool Hardware_ImuDataValid(void);
bool Hardware_EncodersDataValid(void);
bool Hardware_MotorsHaveFault(void);
bool Hardware_MotorsAreEnabled(void);
void Hardware_MotorsSetPositionTargets(
    const float q_des[PROTOCOL_NUM_JOINTS],
    float kp_scale,
    float kd_scale
);
void Hardware_MotorsDisable(void);
uint32_t Hardware_Micros32(void);

#endif
