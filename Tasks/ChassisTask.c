/**
 * Author: Rian
 * Date: 2026/5/12.
 */

#include "cmsis_os2.h"
#include "config.h"
#include "math.h"
#include "Driver_Gyroscope.h"
#include "Driver_Vofa.h"
#include "leg_conv.h"
#include "leg_pos.h"
#include "leg_spd.h"
#include "lqr_k.h"
#include "DM_Driver.h"
#include "Driver_PID.h"
#include "DBUS.h"

//lf 1 lb 2 rf 3 rb 4 dl 5 dr 6
#define Motor_Legf_Bias  (-PI/4)
#define Motor_LegB_Bias  (PI/4)
#define Get_Motor_leg_Bias (i / 2 % 2 ? Motor_LegB_Bias : Motor_Legf_Bias)

#define K_roll 1
#define mass 3.2
#define  DRIVE_WHEEL_RADIUS 0.042f

 struct LegPos_t{
  float length;
  float angle;
  float dLength;
  float dAngle;

}leftLegPos, rightLegPos;

struct stateVar_t{
  float theta,dTheta;
  float x,dx;
  float phi,dPhi;
}stateVar;

struct Target
{
	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
} target = {0, 0, 0, 0, 0, 0, 0.12f};

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;
extern osEventFlagsId_t motorInitHandle;

extern volatile VofaData_type *VofaData;

DM_Motor_Type motors[6]={0};

float DW_Left_Bias;
float DW_Right_Bias;

static setSendData(DM_Motor_Type motors[6], float T[6]){
  motors[0].sendData.torque = -T[0];
  motors[1].sendData.torque = -T[1];
  motors[2].sendData.torque = T[2];
  motors[3].sendData.torque = T[3];
  motors[4].sendData.torque = -T[4];
  motors[5].sendData.torque = -T[5];
}

static receive2model(DM_Motor_Type motors[6]){
  uint8_t id;
  for(int i = 0; i < 2; ++i) {
    id = i;
    motors[i].modelData.p_des = - (motors[i].receiveData.p_des - DM_MIT_P_MAX) + Get_Motor_leg_Bias;
    motors[i].modelData.v_des = - motors[i].receiveData.v_des;
    motors[i].modelData.torque = - motors[i].receiveData.torque;
  }
  for(int i = 2; i < 4; ++i) {
    motors[i].modelData.p_des = (motors[i].receiveData.p_des - DM_MIT_P_MAX) + Get_Motor_leg_Bias;
    motors[i].modelData.v_des = motors[i].receiveData.v_des;
    motors[i].modelData.torque = motors[i].receiveData.torque;
  }
    motors[4].modelData.p_des = - (motors[4].receiveData.p_des - DW_Left_Bias);
    motors[4].modelData.v_des = - motors[4].receiveData.v_des;
    motors[4].modelData.torque = - motors[4].receiveData.torque;
    motors[5].modelData.p_des = (motors[5].receiveData.p_des - DW_Right_Bias);
    motors[5].modelData.v_des = motors[5].receiveData.v_des;
    motors[5].modelData.torque = motors[5].receiveData.torque;
}

void ChassisTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
  osEventFlagsWait(motorInitHandle, motor_init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

  DM_Motor_Type * Motor_LegJoint_Lf = &motors[0];
  DM_Motor_Type * Motor_LegJoint_LB =&motors[1];
  DM_Motor_Type * Motor_LegJoint_RF =&motors[2];
  DM_Motor_Type * Motor_LegJoint_RB =&motors[3];
  DM_Motor_Type * DW_Left =&motors[4];
  DM_Motor_Type * DW_Right =&motors[5];

  DW_Left_Bias = DW_Left->receiveData.p_des;
  DW_Right_Bias = DW_Right->receiveData.p_des;

  float legPos[2], legSpd[2];

  float xd[6] = {0};
  float x[6] = {0};
  float K[12] = {0};
  float u[2] = {0};
  float T_leg_L[2] = {0};
  float T_leg_R[2] = {0};
  float thetaL = 0;
  float thetaR = 0;
  float DW_T_L = 0;
  float DW_T_R = 0;
  float Tp_L = 0;
  float Tp_R = 0;
  float F_L = 0;
  float F_R = 0;

  PID_Type DW_PID;
  PID_Type Joint_T_PID;
  PID_Type Joint_F_L_PID;
  PID_Type Joint_F_R_PID;

  PID_Init(&DW_PID, 1, 0, 0.1, 10, 0);
  PID_Init(&Joint_F_L_PID, 1, 0.01, 0.1, 10, 0);
  PID_Init(&Joint_F_R_PID, 1, 0.01, 0.1, 10, 0);
  PID_Init(&Joint_T_PID, 1, 0, 0.1, 10, 0);

	target.rollAngle = 0.0f;
	target.legLength = 0.07f;
	target.speed = 0.0f;
	target.position = DW_Left->receiveData.p_des* DRIVE_WHEEL_RADIUS;

  while (1)
  {

    /* Usr code */
    target.speed = remoteHandle.ly / 660.0f * 6.0f;

    receive2model(motors);
    /* update x */

    leg_pos(Motor_LegJoint_Lf->modelData.p_des, Motor_LegJoint_LB->modelData.p_des, legPos);
    leftLegPos.length=legPos[0];
    leftLegPos.angle=legPos[1];
    leg_pos(Motor_LegJoint_RF->modelData.p_des, Motor_LegJoint_RB->modelData.p_des, legPos);
    rightLegPos.length=legPos[0];
    rightLegPos.angle=legPos[1];
    leg_spd(Motor_LegJoint_Lf->modelData.v_des,Motor_LegJoint_LB->modelData.v_des ,Motor_LegJoint_Lf->modelData.p_des ,Motor_LegJoint_LB->modelData.p_des ,legSpd);
    leftLegPos.dLength=legSpd[0];
    leftLegPos.dAngle=legSpd[1];
    leg_spd(Motor_LegJoint_RF->modelData.v_des,Motor_LegJoint_RF->modelData.v_des ,Motor_LegJoint_RF->modelData.p_des ,Motor_LegJoint_RF->modelData.p_des ,legSpd);
    rightLegPos.dLength=legSpd[0];
    rightLegPos.dAngle=legSpd[1];

    stateVar.theta= - PI/2.0f + (leftLegPos.angle + rightLegPos.angle)/2.0f - Gyroscope_EulerData.pitch * PI / 180.0f;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle)/2.0f  - Gyroscope_EulerData.pitchSpeed* PI / 180.0f;
		stateVar.phi = Gyroscope_EulerData.pitch* PI / 180.0f;
		stateVar.dPhi = Gyroscope_EulerData.pitchSpeed* PI / 180.0f;
		stateVar.x = DW_Right->modelData.p_des  * DRIVE_WHEEL_RADIUS;
		stateVar.dx = DW_Right->modelData.v_des* DRIVE_WHEEL_RADIUS;

    float x[6] = {- stateVar.theta,  - stateVar.dTheta,  - stateVar.x,  - stateVar.dx, - stateVar.phi, - stateVar.dPhi};
    /* update x end */

		x[3] += target.position;
    x[4] += target.speed;

    lqr_k((leftLegPos.length + rightLegPos.length)/2.0f, K);
    for (uint8_t i = 0; i < 2; ++i)
    {
      u[i] = K[i*6 + 0]*(xd[0] - x[0])
      + K[i*6 + 1]*(xd[1] - x[1])
      + K[i*6 + 2]*(xd[2] - x[2])
      + K[i*6 + 3]*(xd[3] - x[3])
      + K[i*6 + 4]*(xd[4] - x[4])
      + K[i*6 + 5]*(xd[5] - x[5]);
    }

    PID_Calculate(&DW_PID, 0, stateVar.phi);
    DW_T_L = u[0] - DW_PID.output;
    DW_T_R = u[0] + DW_PID.output;

    thetaL = - PI/2.0f + leftLegPos.angle - Gyroscope_EulerData.pitch * PI / 180.0f;
    thetaR = - PI/2.0f + rightLegPos.angle - Gyroscope_EulerData.pitch * PI / 180.0f;
    PID_Calculate(&Joint_T_PID, thetaR, thetaL);
    Tp_L = u[1] - Joint_T_PID.output;
    Tp_R = u[1] + Joint_T_PID.output;

    PID_Calculate(&Joint_F_L_PID, target.legLength, leftLegPos.length);
    PID_Calculate(&Joint_F_R_PID, target.legLength, rightLegPos.length);
    F_L = Joint_F_L_PID.output + 9.18f * mass + K_roll * (target.rollAngle - Gyroscope_EulerData.roll * PI / 180.0f);
    F_R = Joint_F_R_PID.output + 9.18f * mass - K_roll * (target.rollAngle - Gyroscope_EulerData.roll * PI / 180.0f);

    leg_conv(F_L, Tp_L, Motor_LegJoint_Lf->modelData.p_des, Motor_LegJoint_LB->modelData.p_des, T_leg_L);
    leg_conv(F_R, Tp_R, Motor_LegJoint_RF->modelData.p_des, Motor_LegJoint_RB->modelData.p_des, T_leg_R);

    float Output_T[6] = {T_leg_L[0], T_leg_L[1], T_leg_R[0], T_leg_R[1], DW_T_L, DW_T_R};
    setSendData(motors, Output_T);
    /* End */
    osSemaphoreRelease(motorUpdateSemHandle);
    osDelayUntil(50);
  }
}