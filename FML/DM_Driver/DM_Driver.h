#ifndef DM_DRIVER_H
#define DM_DRIVER_H
#include <stdint.h>


#define DM_PARAM_TX_CAN_ID ((uint16_t) 0x7FF)
#define DM_MIT_P_MIN (-12.5f)
#define DM_MIT_P_MAX (12.5f)
#define DM_MIT_V_MIN (-12.5f)
#define DM_MIT_V_MAX (12.5f)
#define DM_MIT_T_MIN (-100.0f)
#define DM_MIT_T_MAX (100.0f)
#define DM_MIT_KP_MIN (0.0f)
#define DM_MIT_KP_MAX (500.0f)
#define DM_MIT_KD_MIN (0.0f)
#define DM_MIT_KD_MAX (5.0f)


enum DM_Motor_Command { Motor_Enble, Motor_Disable };
enum DM_Motor_Mode { MODE_MIT, MODE_POS_VEL, MODE_SPEED };
enum DM_Motor_Param_Command { DM_PARAM_CMD_READ = 0x33, DM_PARAM_CMD_WRITE = 0x55, DM_PARAM_CMD_SAVE = 0xAA };

typedef struct {
    uint16_t id;
    uint16_t  mode;
    struct
    {
        float kp;
        float kd;
        float p_des;
        float v_des;
        float torque;
    } sendData;

    struct
    {
        float p_des;
        float v_des;
        float torque;
    }receiveData;

    uint8_t inputEnabled;

} DM_Motor_Type;

int float_to_uint(float X_float, float X_min, float X_max, int bits);

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id,uint8_t inputEnabled);

void DM_Motor_Command(DM_Motor_Type *motor, uint8_t command);

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd);

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque);

void DM_Motor_Control(DM_Motor_Type *motor);

void DM_Motor_Read_Param(DM_Motor_Type *motor, uint8_t rid);
void DM_Motor_Write_Param(DM_Motor_Type *motor, uint8_t rid, const uint8_t write_param[4]);
void DM_Motor_Write_Param_U32(DM_Motor_Type *motor, uint8_t rid, uint32_t value);
void DM_Motor_Write_Param_Float(DM_Motor_Type *motor, uint8_t rid, float value);
void DM_Motor_Save_Param(DM_Motor_Type *motor);

uint32_t DM_Motor_Param_U32_From_Rx(const uint8_t rx_data[8]);
float    DM_Motor_Param_Float_From_Rx(const uint8_t rx_data[8]);

#endif
