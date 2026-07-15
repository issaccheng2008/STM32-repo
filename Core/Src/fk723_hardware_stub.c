/*
 * Safe communications-test implementation: no real motor output.
 * Replace these functions with your CAN/motor/IMU drivers before deployment.
 */

#include "fk723_hardware.h"

#include <string.h>

static const float demo_pose[PROTOCOL_NUM_JOINTS] = {
     0.15f, 0.0f, 0.0f,  0.30f, -0.15f, 0.0f,
    -0.15f, 0.0f, 0.0f, -0.30f,  0.15f, 0.0f
};

void Hardware_GetJointPositionRad(float output[PROTOCOL_NUM_JOINTS])
{
    memcpy(output, demo_pose, sizeof(demo_pose));
}

void Hardware_GetJointVelocityRadS(float output[PROTOCOL_NUM_JOINTS])
{
    memset(output, 0, sizeof(float) * PROTOCOL_NUM_JOINTS);
}

void Hardware_GetAccelerationMS2(float output[3])
{
    output[0] = 0.0f;
    output[1] = 0.0f;
    output[2] = 9.81f;
}

void Hardware_GetGyroRadS(float output[3])
{
    output[0] = 0.0f;
    output[1] = 0.0f;
    output[2] = 0.0f;
}

bool Hardware_ImuDataValid(void) { return true; }
bool Hardware_EncodersDataValid(void) { return true; }
bool Hardware_MotorsHaveFault(void) { return false; }
bool Hardware_MotorsAreEnabled(void) { return false; }

void Hardware_MotorsSetPositionTargets(
    const float q_des[PROTOCOL_NUM_JOINTS],
    float kp_scale,
    float kd_scale
)
{
    (void)q_des;
    (void)kp_scale;
    (void)kd_scale;
    /* Intentionally no motor output. */
}

void Hardware_MotorsDisable(void)
{
    /* Intentionally no motor output. */
}

uint32_t Hardware_Micros32(void)
{
    return HAL_GetTick() * 1000U;
}
