#ifndef CTRL_H
#define CTRL_H

#include "stdio.h"
#include "pid.h"
typedef struct 
{
   	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
	float leftlegLength; // m
	float rightlegLength; // m
	float pitchAngle;

	uint8_t jump_flag;
	uint8_t jump_status;
	uint8_t jump_time;
}Target;


typedef struct 
{
	float theta, dTheta;
	float x, dx;
	float phi, dPhi;
} StateVar;

//站立过程状态枚举量
typedef enum  {
	StandupState_None,
	StandupState_Prepare,
	StandupState_Standup,
	StandupState_jump,
	StandupState_left_up,
	StandupState_right_up,
} StandupState;

typedef struct GroundDetector
{
	float leftSupportForce, rightSupportForce;
	bool isTouchingGround, isCuchioning;
} GroundDetector;

typedef struct{
	bool isFallen;
	bool motorDisable;
}FallDetector;

extern Target target;
extern CascadePID leftlegLengthPID;
extern CascadePID rightlegLengthPID;
extern StateVar stateVar;
extern StandupState standupState;
extern FallDetector falldetector;

void Ctrl_Init(void);
void StandUp(void);


#endif