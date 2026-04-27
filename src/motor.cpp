#include "motor.h"
#include "can.h"
#include <esp_task_wdt.h>
#include <math.h>
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象
float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
/******* 电机模块 *******/

//初始化一个电机对象
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed))
{
	motor->speed = motor->angle = motor->voltage = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->calcRevVolt = calcRevVolt;
}

//只测量正转 后面有处理
float Motor_CalcRevVolt4310(float speed)
{
	return 0.000002f * speed * speed * speed - 0.0002f * speed * speed + 0.1521f * speed;
}

//2804电机反电动势计算函数(输入速度，输出反电动势)，测量方法同上,轮电机多次实验发现不补偿效果可能更好一些
float Motor_CalcRevVolt2805(float speed)
{
	//return 0.0000003f * speed * speed * speed - 0.00007f * speed * speed + 0.0275f * speed;
	return 0;
}


//由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
//补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
void Motor_UpdateVoltage(Motor *motor)
{
	float voltage = motor->torque / motor->torqueRatio * motorOutRatio;
	if (motor->speed >= 0)
		voltage += motor->calcRevVolt(motor->speed);
	else if (motor->speed < 0)
		voltage -= motor->calcRevVolt(-motor->speed);	//这段没问题 方向最后加上 先按照标量算

	//限幅
	if (voltage > motor->maxVoltage)
		voltage = motor->maxVoltage;
	else if (voltage < -motor->maxVoltage)
		voltage = -motor->maxVoltage;
	motor->voltage = voltage  * motor->dir;
}



//电机指令发送任务
void Motor_SendTask(void *arg)
{
	uint8_t data[8] = {0};
	Motor* motorList[] = {&leftJoint[0], &leftJoint[1], &leftWheel, &rightJoint[0], &rightJoint[1], &rightWheel};
	while (1)
	{
		for (int i = 0; i < 6; i++)
			Motor_UpdateVoltage(motorList[i]); //计算补偿后的电机电压
		
		*(int16_t *)&data[0] = ((int16_t)(leftJoint[0].voltage* 1000));
		*(int16_t *)&data[2] = ((int16_t)(leftJoint[1].voltage * 1000));
		*(int16_t *)&data[4] = ((int16_t)(leftWheel.voltage * 1000));
		CAN_SendFrame(0x100, data);
		*(int16_t *)&data[0] = ((int16_t)(rightJoint[0].voltage * 1000));
		*(int16_t *)&data[2] = ((int16_t)(rightJoint[1].voltage * 1000));
		*(int16_t *)&data[4] = ((int16_t)(rightWheel.voltage * 1000));
		CAN_SendFrame(0x200, data);
		vTaskDelay(2);
	}
}


//初始化所有电机对象
//各个参数需要通过实际测量或拟合得到
void Motor_InitAll(void)
{
    Motor_Init(&leftJoint[0],-4.342f, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&leftJoint[1],-4.242f, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&leftWheel, 0, 4.5f, 0.01f, -1, Motor_CalcRevVolt2805);
    Motor_Init(&rightJoint[0],-1.986f, 9.69f, 0.0333f, -1, Motor_CalcRevVolt4310);
    Motor_Init(&rightJoint[1],1.261f, 9.69f, 0.0333f, -1, Motor_CalcRevVolt4310);
    Motor_Init(&rightWheel, 0, 4.5f, 0.01f, -1, Motor_CalcRevVolt2805);
    xTaskCreate(Motor_SendTask, "Motor_SendTask", 2048, NULL, 4, NULL);
}
//1。343

//从CAN总线接收到的数据中解析出电机角度和速度
void Motor_Update(Motor *motor, uint8_t *data)
{
	motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
	// motor->angle = (*(int32_t *)&data[0] / 1000.0f ) * motor->dir;
	motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, float torque)
{
	motor->torque = torque;
}

