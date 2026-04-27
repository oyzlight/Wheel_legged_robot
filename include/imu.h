#ifndef IMU_H
#define IMU_H

typedef struct 
{
	float yaw, pitch, roll;			 // rad     角度
	float yawSpd, pitchSpd, rollSpd; // rad/s   角速度
	float zAccel;                    // m/s^2   z轴加速度
} IMUData;


extern IMUData imuData;

void IMU_Init(void);

#endif