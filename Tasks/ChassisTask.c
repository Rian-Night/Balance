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

//lf 1 lb 2 rf 3 rb 4 dl 5 dr 6
#define Motor_LegLf_Bios  0
#define Motor_LegLB_Bios  0
#define Motor_LegRB_Bios  0
#define Motor_LegRF_Bios  0

 struct LegPos_t{
  float length;
  float angle;
  float dLength;
  float dAngle;

}leftLegPos;

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
} target = {0, 0, 0, 0, 0, 0, 0.07f};

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;

extern volatile VofaData_type *VofaData;

DM_Motor_Type motors[6];

void ChassisTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

  DM_Motor_Type * Motor_LegJoint_Lf = &motors[0];
  DM_Motor_Type * Motor_LegJoint_LB =&motors[1];
  DM_Motor_Type * Motor_LegJoint_RB =&motors[2];
  DM_Motor_Type * Motor_LegJoint_RF =&motors[3];
  DM_Motor_Type * DW_Left =&motors[4];
  DM_Motor_Type * DW_Right =&motors[5];

  float legPos[2], legSpd[2];

  float xd[6] = {0};
  float x[6] = {0};
  float K[12] = {0};
  float u[2] = {0};
  float T[2] = {0};
  float phi1 = 0;
  float phi4 = 0;
  float dphi1 =0;
  float dphi4=0;

	target.rollAngle = 0.0f;
	target.legLength = 0.07f;
	target.speed = 0.0f;
	target.position = DW_Left->receiveData.p_des* DRIVE_WHEEL_RADIUS;

  while (1)
  {
    /* Usr code */

    /* update x */
    phi1 = Motor_LegJoint_RF->receiveData.p_des - Motor_LegLf_Bios;
    phi4 = Motor_LegJoint_RB->receiveData.p_des - Motor_LegLB_Bios;

    leg_pos(phi1, phi4, legPos);
    leftLegPos.length=legPos[0];
    leftLegPos.angle=legPos[1];

    leg_spd(dphi1,dphi4,phi1,phi4,legSpd);
    leftLegPos.dLength=legSpd[0];
    leftLegPos.dAngle=legSpd[1];

    stateVar.theta=leftLegPos.angle - M_PI_2 - Gyroscope_EulerData.pitch;
		stateVar.dTheta = leftLegPos.dAngle  - Gyroscope_EulerData.pitchSpeed;
		stateVar.phi = Gyroscope_EulerData.pitch;
		stateVar.dPhi = Gyroscope_EulerData.pitchSpeed;
		stateVar.x = DW_Left->receiveData.p_des  * DRIVE_WHEEL_RADIUS;
		stateVar.dx = DW_Left->receiveData.v_des* DRIVE_WHEEL_RADIUS;

    float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
    /* update x end */

		x[2] -= target.position;
		x[3] -= target.speed;

    lqr_k(leftLegPos.length, K);
    for (uint8_t i = 0; i < 2; ++i)
    {
      u[i] = K[i*6 + 0]*(xd[0] - x[0])
      + K[i*6 + 1]*(xd[1] - x[1])
      + K[i*6 + 2]*(xd[2] - x[2])
      + K[i*6 + 3]*(xd[3] - x[3])
      + K[i*6 + 4]*(xd[4] - x[4])
      + K[i*6 + 5]*(xd[5] - x[5]);
    }
    leg_conv(u[0], u[1], phi1, phi4, T);




    /* End */
    osSemaphoreRelease(motorUpdateSemHandle);
    osDelayUntil(50);
  }
}