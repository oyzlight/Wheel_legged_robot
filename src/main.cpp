#include <Arduino.h>
#include "serial.h"
#include "can.h"
#include "imu.h"
#include "motor.h"
#include "ble.h"
#include "legs.h"
#include "ctrl.h"
#include "adc.h"
void setup()
{
//优先级1
Serial_Init();
//优先级4
CAN_Init();
IMU_Init();
Motor_InitAll();
// 优先级2
Legs_Init();
//状态更新优先级3    姿态控制优先级为1 
Ctrl_Init(); 
// //蓝牙遥控器捏
BLE_Init();
BLE_Test();

}

void loop()
{
}