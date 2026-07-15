/* FK723M1-ZGT6 / STM32H723ZGT6 board-specific application glue. */

#include "fk723_robot_app.h"

#include "fk723_hardware.h"
#include "gpio.h"
#include "jetson_protocol.h"
#include "jetson_usb_cdc.h"
#include "tim.h"

#include <stdbool.h>
#include <string.h>

#define JETSON_WATCHDOG_MS 100U
#define STATE_DIVIDER       5U   /* 1 kHz timer / 5 = 200 Hz state packets. */

static RobotCommandPayload active_command;
static volatile bool state_transmit_due;
static volatile bool command_fresh;
static uint8_t state_divider;

static void RobotApp_Control1kHz(void)
{
    bool fresh = Protocol_GetFreshCommand(HAL_GetTick(), JETSON_WATCHDOG_MS, &active_command);
    bool enabled = fresh && ((active_command.command_flags & COMMAND_ENABLE) != 0U);
    bool estop = !fresh || ((active_command.command_flags & COMMAND_ESTOP) != 0U);

    command_fresh = fresh;
    if (estop || !enabled || Hardware_MotorsHaveFault()) {
        Hardware_MotorsDisable();
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);  /* Active-low LED off. */
        return;
    }

    Hardware_MotorsSetPositionTargets(
        active_command.joint_target,
        active_command.kp_scale,
        active_command.kd_scale
    );
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);    /* Active-low LED on. */
}

static void RobotApp_SendState(void)
{
    RobotStatePayload state;
    memset(&state, 0, sizeof(state));
    state.timestamp_us = Hardware_Micros32();
    Hardware_GetJointPositionRad(state.joint_position);
    Hardware_GetJointVelocityRadS(state.joint_velocity);
    Hardware_GetAccelerationMS2(state.accel_m_s2);
    Hardware_GetGyroRadS(state.gyro_rad_s);

    if (Hardware_MotorsAreEnabled()) state.status_flags |= STATE_MOTORS_ENABLED;
    if (Hardware_MotorsHaveFault()) state.status_flags |= STATE_FAULT;
    if (Hardware_ImuDataValid()) state.status_flags |= STATE_IMU_VALID;
    if (Hardware_EncodersDataValid()) state.status_flags |= STATE_ENCODERS_VALID;
    if (command_fresh) state.status_flags |= STATE_COMMAND_FRESH;

    (void)JetsonUsbCdc_SendState(&state);
}

void RobotApp_Init(void)
{
    Protocol_Init();
    memset(&active_command, 0, sizeof(active_command));
    state_transmit_due = false;
    command_fresh = false;
    state_divider = 0U;
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    (void)HAL_TIM_Base_Start_IT(&htim6);
}

void RobotApp_MainLoop(void)
{
    if (state_transmit_due) {
        state_transmit_due = false;
        RobotApp_SendState();
    }
}

void RobotApp_OnUsbReceive(uint8_t *data, uint32_t length)
{
    JetsonUsbCdc_OnReceive(data, length, HAL_GetTick());
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        RobotApp_Control1kHz();
        if (++state_divider >= STATE_DIVIDER) {
            state_divider = 0U;
            state_transmit_due = true;
        }
    }
}
