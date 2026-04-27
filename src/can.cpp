#include "driver/twai.h"
#include <esp_task_wdt.h>
#include "can.h"
#include <string.h>
#include "motor.h"
void CAN_RecvCallback(uint32_t id, uint8_t *data)
{
	switch (id) //根据CAN ID更新各电机数据
	{
	case 0x101:
		Motor_Update(&leftJoint[0], data);
		break;
	case 0x102:
		Motor_Update(&leftJoint[1], data);
		break;
	case 0x103:
		Motor_Update(&leftWheel, data);
		break;
	case 0x105:
		Motor_Update(&rightJoint[0], data);
		break;
	case 0x106:
		Motor_Update(&rightJoint[1], data);
		break;
	case 0x107:
		Motor_Update(&rightWheel, data);
		break;
	}
}


void CAN_TestTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t data[8] = {0};
	while(1)
	{
		*(int16_t *)&data[0] = ((int16_t)(1 * 0000));
		*(int16_t *)&data[2] = ((int16_t)(1 * 0000));
		*(int16_t *)&data[4] = ((int16_t)(1 * 1000));
		CAN_SendFrame(0x100,data); //0x100为左侧电机 0x200为右侧电机
		*(int16_t *)&data[0] = ((int16_t)(1 * 0000));
		*(int16_t *)&data[2] = ((int16_t)(1 * 0000));
		*(int16_t *)&data[4] = ((int16_t)(1 * 0000));
		CAN_SendFrame(0x200,data); //0x100为左侧电机 0x200为右侧电机
		vTaskDelayUntil(&xLastWakeTime, 2); //2ms轮询一次
	}
}

//CAN数据帧轮询接收任务
void CAN_RecvTask(void *arg)
{
	twai_message_t msg;
	twai_status_info_t status;
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		twai_get_status_info(&status);
		for(uint8_t i = 0; i < status.msgs_to_rx; i++)
		{
			if(twai_receive(&msg, 0) == ESP_OK)
				CAN_RecvCallback(msg.identifier, msg.data);
		}
		vTaskDelayUntil(&xLastWakeTime, 2); //2ms轮询一次
	}
}


//CAN通信外设初始化
void CAN_Init(void)
{
	twai_general_config_t twai_conf = {
		.mode = TWAI_MODE_NORMAL,
		.tx_io = GPIO_NUM_33, //使用33和32
		.rx_io = GPIO_NUM_32,
		.clkout_io = TWAI_IO_UNUSED,
		.bus_off_io = TWAI_IO_UNUSED,
		.tx_queue_len = 5,
		.rx_queue_len = 10,
		.alerts_enabled = TWAI_ALERT_NONE,
		.clkout_divider = 0,
		.intr_flags = ESP_INTR_FLAG_LEVEL1};

	twai_timing_config_t twai_timing = TWAI_TIMING_CONFIG_1MBITS();

	twai_filter_config_t twai_filter = {
		.acceptance_code = 0x00000000,
		.acceptance_mask = 0xFFFFFFFF,
		.single_filter = true};

	twai_driver_install(&twai_conf, &twai_timing, &twai_filter);
	twai_start();
	// xTaskCreate(CAN_TestTask, "CAN_TestTask", 2048, NULL, 5, NULL);           //测试CAN通信用
	xTaskCreate(CAN_RecvTask, "CAN_RecvTask", 2048, NULL, 4, NULL);   //正常任务
}


//发送一帧CAN数据(data为8字节数据)
void CAN_SendFrame(uint32_t id, uint8_t *data)
{
	twai_message_t msg;
	msg.flags = 0;
	msg.identifier = id;
	msg.data_length_code = 8;
	memcpy(msg.data, data, 8);
	twai_transmit(&msg, 100);
}
