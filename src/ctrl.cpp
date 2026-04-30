#include <Arduino.h>
#include "pid.h"
#include "legs.h"
#include "motor.h"
#include "../include/matlab_code/leg_vmc_conv.h"
#include "../include/matlab_code/lqr_k.h"
#include "ctrl.h"
#include "imu.h"
#include <esp_task_wdt.h>
#include <math.h>

CascadePID legAnglePID, legLengthPID; //腿部角度和长度控制PID
CascadePID yawPID, rollPID; //机身yaw和roll控制PID

CascadePID leftlegLengthPID;
CascadePID rightlegLengthPID;
#define MAX_LEG_LENGTH 0.17f
uint8_t cnt=0;
Target target = {0, 0, 0, 0, 0, 0, 0.07f};
StateVar stateVar;
FallDetector falldetector ={0,0};
StandupState standupState = StandupState_Standup;
GroundDetector groundDetector = {10, 10, true, false};	//离地检测器 默认状态为触地
//测试用
#define SIN_FREQUENCY_MS 5000 // 周期，单位毫秒
#define SIN_AMPLITUDE 0.04      // 幅度
#define SIN_OFFSET 0.1         // 偏移量

//倒地检测
#define FALL_PITCH_THRES 0.2f
#define FALL_ROLL_THRES 0.2f

bool check_Fallground(){
	if(fabs(imuData.pitch)>FALL_PITCH_THRES||fabs(imuData.roll)>FALL_ROLL_THRES)
	{
		return true;
	}
	else{
		return false;
	}
}
void DisableAllMotors(){
	Motor_SetTorque(&leftWheel, 0);
    Motor_SetTorque(&rightWheel, 0);
    Motor_SetTorque(&leftJoint[0], 0);
    Motor_SetTorque(&leftJoint[1], 0);
    Motor_SetTorque(&rightJoint[0], 0);
    Motor_SetTorque(&rightJoint[1], 0);
}

void vSinGeneratorTask(void *arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SIN_FREQUENCY_MS);

    // 获取当前系统时间
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 计算当前时刻的正弦函数值
        float sin_value = SIN_AMPLITUDE * sin((2 * M_PI * xTaskGetTickCount() / xFrequency))+SIN_OFFSET;
		target.legLength = sin_value; // 假设虚拟腿0的腿部长度为正弦函数值
        // 在这里可以将 sin_value 用于其他操作，比如输出到外设

        // 任务挂起，直到下一个周期
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

#define STEP_INTERVAL_MS 1000 // 跃变间隔，单位毫秒
#define STEP_AMPLITUDE 5      // 幅度
// 阶跃函数生成任务
void vStepGeneratorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(STEP_INTERVAL_MS);
    int step_value = 0;

    // 获取当前系统时间
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 每隔一定时间改变阶跃函数值
        step_value += STEP_AMPLITUDE;

        // 在这里可以将 step_value 用于其他操作，比如输出到外设

        // 任务挂起，直到下一个跃变间隔
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void VMC_TestTask(void *arg)
{
	float targetLegLength = 0.09f; //m，目标虚拟腿腿部长度 
    float targetLegAngle = 1.57f; //rad，目标虚拟腿腿部角度 1.57 = pai/2  居中

	TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
{
	// float targetLegLength = target.legLength;
	
	// //测试左腿
	// float legLength = leftLegPos.length;
	// float dLegLength = leftLegPos.dLength;

	// PID_CascadeCalc(&legLengthPID,targetLegLength,legLength, dLegLength);
	// PID_CascadeCalc(&legAnglePID,targetLegAngle, leftLegPos.angle, leftLegPos.dAngle);
		
	// float leftJointTorque[2]={0};
	// float leftForce = legLengthPID.output;
	// float leftTp =  - legAnglePID.output;
	// leg_vmc_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
	// Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
	// Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);

	// //测试右腿
	// float legLength = rightLegPos.length;
	// float dLegLength = rightLegPos.dLength;

	// PID_CascadeCalc(&legLengthPID,targetLegLength,legLength, dLegLength);
	// PID_CascadeCalc(&legAnglePID,targetLegAngle, rightLegPos.angle, rightLegPos.dAngle);
		
	// float rightJointTorque[2]={0};
	// float rightForce = legLengthPID.output;
	// float rightTp =   -legAnglePID.output;
	// leg_vmc_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
	// Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
	// Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);

	//测试两条腿单独控制
	float leftLegLength = leftLegPos.length;
	float rightLegLength = rightLegPos.length;

	PID_CascadeCalc(&leftlegLengthPID, target.leftlegLength, leftLegLength, leftLegPos.dLength);
	PID_CascadeCalc(&rightlegLengthPID, target.rightlegLength, rightLegLength, rightLegPos.dLength);
	PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
		 
	float leftJointTorque[2]={0};
	float leftForce = leftlegLengthPID.output;
	float leftTp = - legAnglePID.output;
	leg_vmc_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);

	float rightJointTorque[2]={0};
	float rightForce = rightlegLengthPID.output;
	float rightTp = + legAnglePID.output;
	leg_vmc_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);

	Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
	Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
	Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
	Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);
	// Serial.println("l0: " + String(leftJoint[0].angle) + ", l1: " + String(leftJoint[1].angle) + ", r0: " + String(rightJoint[0].angle) + ", r1: " + String(rightJoint[1].angle));

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
}
}

//目标量更新任务(根据蓝牙收到的目标量计算实际控制算法的给定量)
void Ctrl_TargetUpdateTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.003f;
	while (1)
	{
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		speedSlopeStep = -(legLength - 0.07f) * 0.02f + 0.002f;

		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}

		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;

		//限制速度目标在当前速度的±0.3m/s内
		if(target.speed - stateVar.dx > 0.3f)
			target.speed = stateVar.dx + 0.3f;
		else if(target.speed - stateVar.dx < -0.3f)
			target.speed = stateVar.dx - 0.3f;

		//计算yaw方位角目标
		target.yawAngle += target.yawSpeedCmd * 0.004f;
		
		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}
void Ctrl_StandupPrepareTask(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    standupState = StandupState_Prepare;

    int phase = 0;
    int phaseTimer = 0;
    
    // 蓄力与弹射参数
    float squatLength = 0.09f;   // 蓄力时的极短腿长
    float thrustLength = 0.12f;  // 蹬地瞬间的目标腿长
    float targetAngle = 1.57f;   // 始终保持垂直 90度
    float burstTorque = 2.0f;    // 轮子冲刺的爆发力矩 (需根据你的电机发力调整)

    float curTargetLen = squatLength;
    float wheelOut = 0.0f;

    while (1)
    {
        // ================= 状态机控制 =================
        if (phase == 0) 
        {
            // 阶段 0：趴在地上收腿蓄力 (持续 1 秒)
            curTargetLen = squatLength;
            wheelOut = 0.0f; // 轮子不动
            
            phaseTimer += 4;
            if (phaseTimer > 1000) {
                phase = 1; // 蓄力完毕，进入弹射
            }
        }
        else if (phase == 1)
        {
            // 阶段 1：腿部瞬间弹射，轮子全力冲刺钻入重心下方
            curTargetLen = thrustLength; 
            
            // 判断当前是往前趴还是往后趴，决定轮子冲刺方向
            // 注意：这里的正负号必须和你的 imuData.pitch 及电机极性匹配
            if (imuData.pitch > 0) {
                wheelOut = burstTorque;  // 假设 pitch>0 是往前趴，轮子就往前猛冲
            } else {
                wheelOut = -burstTorque; // 往后趴，轮子往后猛冲
            }

            // 阶段 1 退出条件：当机身被甩到接近直立时 (例如 pitch 小于 8 度/0.14rad)
            if (fabs(imuData.pitch) < 0.14f) {
                phase = 2; // 进入接管阶段
            }
        }
        else if (phase == 2)
        {
            // 阶段 2：交接给 LQR
            // 此时机身已经靠惯性立起来了，立刻同步目标值并退出任务
            target.legLength = 0.12f; // 恢复你正常站立的腿长
            target.position = (leftWheel.angle + rightWheel.angle) / 2 * 0.0325f; // 重置里程计
            
            standupState = StandupState_Standup;
            vTaskDelete(NULL); 
        }

        // ================= VMC 底层执行 =================
        // 1. 腿长控制 (弹射瞬间需要极强的 P)
        // PID_CascadeCalc(&legLengthPID, curTargetLen, (leftLegPos.length + rightLegPos.length)/2, 
        //                                              (leftLegPos.dLength + rightLegPos.dLength)/2);
		PID_CascadeCalc(&leftlegLengthPID, curTargetLen, leftLegPos.length ,leftLegPos.dLength);
		PID_CascadeCalc(&rightlegLengthPID, curTargetLen, rightLegPos.length ,rightLegPos.dLength);

        // float force = legLengthPID.output;
		float left_force =leftlegLengthPID.output;
		float right_force =rightlegLengthPID.output;


        // 2. 角度控制 (强制保持向中间夹紧)
        float kp_angle = 20.0f; // 角度刚度要非常硬
        float kd_angle = 0.5f;
        float tpL = kp_angle * (targetAngle - leftLegPos.angle) - kd_angle * leftLegPos.dAngle;
        float tpR = kp_angle * (targetAngle - rightLegPos.angle) - kd_angle * rightLegPos.dAngle;
		
        // 3. 轮子爆发输出
        // 注意这里的负号，必须保证 burstTorque 能让轮子往机身倾倒的方向开
        Motor_SetTorque(&leftWheel, wheelOut); 
        Motor_SetTorque(&rightWheel,wheelOut);

        // 4. VMC 转换与电机写入
        float tauL[2], tauR[2];
        leg_vmc_conv(left_force, tpL, leftJoint[1].angle, leftJoint[0].angle, tauL);
        leg_vmc_conv(right_force, tpR, rightJoint[1].angle, rightJoint[0].angle, tauR);

        Motor_SetTorque(&leftJoint[0], -tauL[0]);
        Motor_SetTorque(&leftJoint[1], -tauL[1]);
        Motor_SetTorque(&rightJoint[0], -tauR[0]);
        Motor_SetTorque(&rightJoint[1], -tauR[1]);

        vTaskDelayUntil(&xLastWakeTime, 4);
    }
}
//没有起立
void CtrlBasic_Task(void *arg)
{
	const float wheelRadius = 0.0325f; 	//m，车轮半径
	const float legMass = 0.052f; 		//kg，腿部质量

	TickType_t xLastWakeTime = xTaskGetTickCount();

	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果 {0.5f, 0.4f, 0.8f, 0.5f, 0.75f, 0.6f},
	//                     theta dTheta  x,   dx,   phi, dPhi
	//0.5f,  0.0f,  0.8f, 0.5f, 0.75f, 0.6f 轮子  {0.4f,  0.1f,  0.8f, 0.5f, 0.75f, 0.6f}加积分
	float kRatio[2][6] = {
		{0.4f,  0.1f,  1.0f, 0.7f, 0.75f, 0.75f},	//lqrOutT		轮子
		{1.0f,1.0f,	0.8f,0.5f,	0.0f,0.0f}	//lqrOutTp		髋关节
		};			
	float lqrTRatio = 1.0f,				    //轮子
		  lqrTpRatio = 1.0f;					//髋关节

	target.rollAngle = 0.0f;
	target.legLength = 0.08f;
	target.leftlegLength = 0.08f;
	target.rightlegLength = 0.08f;

	/*跳跃状态初始化*/
	target.jump_flag = 0;
	target.jump_status = 0;
	target.jump_time = 0;
	
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	// Serial.printf("%.3f\n",target.position);	

	static float integral_x_error = 0;
	float dt = 0.004f;  // 控制周期4ms
	float integral_gain = 0.015f;  // 需要调节
	float kp_gain=0.3f;
	float kd_gain = 0.1f;  // 微分增益，可调
	while (1)
	{
		if(falldetector.isFallen==1)
		{
			DisableAllMotors();
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}
		//计算状态变量
		stateVar.phi = imuData.pitch;
		stateVar.dPhi = imuData.pitchSpd;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imuData.pitch;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;

		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;	

		float leftlegLength = leftLegPos.length;
		float dleftlegLength = leftLegPos.dLength;

		float rightlegLength = rightLegPos.length;
		float drightlegLength = rightLegPos.dLength;
		
		//如果正在站立准备状态，则不进行后续控制
		if(standupState == StandupState_Prepare)
		{
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}
		if(standupState == StandupState_jump) 
		{ 
		
			if(target.jump_flag == 0)
			{
				target.jump_flag = 1;
				target.jump_status = 0;
				target.jump_time = 0;
			}
			if(target.jump_flag == 1)
			{
				/*锁定yaw目标+清零yawPID输出*/
				target.yawAngle = imuData.yaw;
				yawPID.output = 0;
				target.position = stateVar.x;
				target.speed = 0.0f;
				switch (target.jump_status)
				{
					/*下压状态*/
					case 0:
					{
						target.leftlegLength=0.05f;
						target.rightlegLength=0.05f;
						falldetector.isFallen = 0;  
						if(leftLegPos.length<0.07&&rightLegPos.length<0.07)
						{
							target.jump_time++;
						}
						if(target.jump_time>=100)
						{
							target.jump_time=0;
							target.jump_status=1;
						}
					}
					break;
					/*弹起*/
					case 1:
					{
						
						target.leftlegLength=0.25f;
						target.rightlegLength=0.25f;
						falldetector.isFallen = 0;  
						// Motor_SetTorque(&leftWheel, 0);
						// Motor_SetTorque(&rightWheel, 0);
						if(leftLegPos.length>0.10&&rightLegPos.length>0.10)
						{
							target.jump_time++;
						}
						if(target.jump_time>=3)
						{
							target.jump_time=0;
							target.jump_status=3;
						}
					}
					break;

					case 2:
					{
						target.leftlegLength=0.07f;
						target.rightlegLength=0.07f;
						falldetector.isFallen = 0;  
						if(leftLegPos.length<0.10&&rightLegPos.length<0.10)
						{
							target.jump_time++;
						}
						if (target.jump_time>=1)
						{
							target.jump_time=0;
							target.jump_status=3;
						}
						
					}
					break;
					case 3:
					{
						standupState=StandupState_Standup;
						target.jump_flag=0;
						target.position = stateVar.x;
						target.yawAngle = imuData.yaw;

						  // 2. 强制解除倒地保护（最关键！）
						falldetector.isFallen = 0;

						// 3. 重置目标，让LQR重新接管平衡
						target.position = stateVar.x;       // 位置同步
						target.speed = 0.0f;                // 速度清零
						target.yawAngle = imuData.yaw;      // 方向清零
						
						// 4. 清零LQR积分项（防止轮子猛冲）
						integral_x_error = 0;
						target.leftlegLength=0.05f;
						target.rightlegLength=0.05f;

						
					}
					break;
				}
			}
			if(target.jump_flag == 2)
			{
				// 已经结束跳跃，直接跳过剩余跳跃逻辑
				vTaskDelayUntil(&xLastWakeTime, 4);
				continue;
			}

		}

		if(check_Fallground())
		{
			falldetector.isFallen=1;
			DisableAllMotors();
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}
		// ================= 新增：腾空期间切断 LQR 干扰 =================
        // if(standupState == StandupState_jump && (target.jump_status == 1 || target.jump_status == 2))
        // {
        //     lqrTRatio = 0.0f;  // 空中轮子不转
        //     lqrTpRatio = 0.0f; // 空中髋关节不参与平衡，防止乱踹影响收腿
        // }
        // else
        // {
        //     lqrTRatio = 1.0f;  // 落地后恢复
        //     lqrTpRatio = 1.0f; 
        // }
        // ==============================================================

		//计算LQR反馈矩阵
		float kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);	
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 2; j++)
				k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
		}

		//准备状态变量
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//与给定量作差
		x[2] -= target.position;
		x[3] -= target.speed;

		//矩阵相乘，计算LQR输出
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
		
		float x_error = target.position-stateVar.x ;
		static float last_x_error = 0;
		float dx_error = (x_error - last_x_error) / dt;
		last_x_error = x_error;
		integral_x_error += x_error * dt;
		// 可选：积分限幅
		if (integral_x_error > 0.1f) integral_x_error = 0.1f;
		if (integral_x_error < -0.1f) integral_x_error = -0.1f;

		// 将积分项叠加到LQR输出上
		
		lqrOutT = lqrOutT + integral_gain * integral_x_error+ x_error* kp_gain;

		PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);

		Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
		Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);

		// PID_CascadeCalc(&legLengthPID,target.legLength, legLength, dLegLength);
		PID_CascadeCalc(&leftlegLengthPID,target.leftlegLength, leftlegLength, dleftlegLength);
		PID_CascadeCalc(&rightlegLengthPID,target.rightlegLength, rightlegLength, drightlegLength);

		//计算左右腿角度差PID输出
		PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
		PID_CascadeCalc(&rollPID, target.rollAngle, imuData.roll, imuData.rollSpd);

		float leftForce = leftlegLengthPID.output-rollPID.output ;
		// float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output;
		float leftTp = -lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
		
		float leftJointTorque[2]={0};
		leg_vmc_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);

		float rightForce = rightlegLengthPID.output+rollPID.output ;	
		// float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output;
		float rightTp = -lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
		// Serial.printf("lqr:%.3f,pid:%.3f\n",lqrOutTp,legAnglePID.output);
	
		float rightJointTorque[2]={0};
		leg_vmc_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);

		//设定关节电机输出扭矩

			Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
			Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
			Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
			Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);
		
	
		// Serial.printf("l1:%.3f,l2:%.3f,r1:%.3f,r2:%.3f\r\n",leftJointTorque[0],leftJointTorque[1],rightJointTorque[0],rightJointTorque[1]);
		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
	
}

void Ctrl_Init(void)
{
	PID_Init(&rollPID.inner, 1, 0.1, 2.5, 1, 5);
	PID_Init(&rollPID.outer, 10, 0, 0, 0, 3);
	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);

	PID_Init(&yawPID.inner, 0.01, 0.001, 0.03, 0.2, 0.5);
	PID_Init(&yawPID.outer, 7, 0, 6, 0, 2);

	PID_Init(&legLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
	PID_Init(&legLengthPID.outer, 5.0f, 0.0f, 0.0f, 0.0f, 2.0f);
	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);

	// 在 Ctrl_Init 函数中添加：
	PID_Init(&leftlegLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f); // 起立需要极大的刚度
	PID_Init(&leftlegLengthPID.outer, 6.0f, 0.0f, 0.0f, 0.0f, 3.0f);

	PID_Init(&rightlegLengthPID.inner,10.0f, 1, 30.0f, 2.0f, 10.0f);
	PID_Init(&rightlegLengthPID.outer, 6.0f, 0.0f, 0.0f, 0.0f, 3.0f);

	PID_Init(&legAnglePID.inner, 0.04, 0, 0, 0, 1);
	PID_Init(&legAnglePID.outer, 10, 0, 0, 0, 20);
	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);

	xTaskCreate(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL);
	vTaskDelay(2);
	// xTaskCreate(Ctrl_StandupPrepareTask, "Ctrl_StandupPrepareTask", 4096, NULL, 1, NULL);
	//xTaskCreate(vSinGeneratorTask, "vSinGeneratorTask", 4096, NULL, 1, NULL);
	// xTaskCreate(VMC_TestTask, "VMC_TestTask", 4096, NULL, 1, NULL);//测试
	xTaskCreate(CtrlBasic_Task, "CtrlBasic_Task", 4096, NULL, 1, NULL);//主任务

}