#include "imu.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_task_wdt.h>

MPU6050 mpu;
IMUData imuData;

// YAW漂移补偿（每秒减去的角度，单位：度）
#define YAW_DRIFT_PER_SEC 0.005f  // 每秒减去0.2度
float yawDriftCompensation = 0.0f;

//陀螺仪数据获取任务
void IMU_Task(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint8_t fifoBuffer[64]; //dmp数据接收区
	int16_t yawRound = 0; //统计yaw转过的整圈数
	float lastYaw = 0;
	
	while (1)
	{
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
		{
			//获取陀螺仪角速度
			int16_t gyroData[3];
			mpu.getRotation(&gyroData[0], &gyroData[1], &gyroData[2]);

			imuData.rollSpd = gyroData[0] / 16.4f * M_PI / 180.0f;	
			imuData.pitchSpd = gyroData[1] / 16.4f * M_PI / 180.0f;		
			imuData.yawSpd = gyroData[2] / 16.4f * M_PI / 180.0f;       //转换为弧度制 rad/s 根据安装调整
			
			//获取陀螺仪欧拉角
			float ypr[3];
			Quaternion q;
			VectorFloat gravity;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);//ZYX
			
			float yaw = -ypr[0];
			imuData.pitch = -ypr[1];
			imuData.roll = ypr[2];


			if (yaw - lastYaw > M_PI)
				yawRound--;
			else if (yaw - lastYaw < -M_PI)
				yawRound++;
			lastYaw = yaw;
			// 累加漂移补偿（每5ms一次，每秒累加YAW_DRIFT_PER_SEC度）
			yawDriftCompensation += YAW_DRIFT_PER_SEC * 5.0f / 1000.0f * M_PI / 180.0f;
			imuData.yaw = yaw + yawRound * 2 * M_PI ; //imuData.yaw为累计转角

			//获取陀螺仪Z轴加速度
			VectorInt16 rawAccel;
			mpu.dmpGetAccel(&rawAccel, fifoBuffer);
			VectorInt16 accel;
			mpu.dmpGetLinearAccel(&accel, &rawAccel, &gravity);
			imuData.zAccel = accel.z / 8192.0f * 9.8f;      //dmp方法得到的精度就这么大
		}
		vTaskDelayUntil(&xLastWakeTime, 5); //5ms轮询一次
	}
}

//陀螺仪模块初始化
void IMU_Init()
{
	//初始化IIC
	Wire.begin(21, 22);//SDA SCL
	Wire.setClock(400000);

	//初始化陀螺仪
	mpu.initialize();
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	Serial.println("Address: 0x" + String(mpu.getDeviceID(), HEX));
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);       //设置加速度和陀螺仪为最大精度
	while(mpu.dmpInitialize() != 0);
	mpu.setXAccelOffset(-3529);
	mpu.setYAccelOffset(1181);
	mpu.setZAccelOffset(3605);
	mpu.setXGyroOffset(-86);
	mpu.setYGyroOffset(15);
	mpu.setZGyroOffset(-112);
	mpu.CalibrateAccel(6); //测量偏移数据
	mpu.CalibrateGyro(6);
	mpu.PrintActiveOffsets();
	mpu.setDMPEnabled(true);

	//开启陀螺仪数据获取任务
	xTaskCreate(IMU_Task, "IMU_Task", 2048, NULL, 4, NULL);
}