#ifndef FK723_ROBOT_APP_H
#define FK723_ROBOT_APP_H

#include <stdint.h>

void RobotApp_Init(void);
void RobotApp_MainLoop(void);
void RobotApp_OnUsbReceive(uint8_t *data, uint32_t length);

#endif
