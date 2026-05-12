#ifndef DM_DRIVER_H
#define DM_DRIVER_H
#include <stdint.h>


#define DM_PARAM_TX_CAN_ID ((uint16_t) 0x7FF)
#define DM_MIT_P_MIN (id / 4 ? -12.5f : -12.5f)
#define DM_MIT_P_MAX (id / 4 ? 12.5f : 12.5f)
#define DM_MIT_V_MIN (id / 4 ? -45.0f : -30.0f)
#define DM_MIT_V_MAX (id / 4 ? 45.0f : 30.0f)
#define DM_MIT_T_MIN (id / 4 ? -10.0f : -10.0f)
#define DM_MIT_T_MAX (id / 4 ? 10.0f : 10.0f)
#define DM_MIT_KP_MIN (id / 4 ? 0.0f : 0.0f)
#define DM_MIT_KP_MAX (id / 4 ? 500.0f : 500.0f)
#define DM_MIT_KD_MIN (id / 4 ? 0.0f : 0.0f)
#define DM_MIT_KD_MAX (id / 4 ? 5.0f : 5.0f)

enum DM_Motor_Command { Motor_Enble, Motor_Disable };
enum DM_Motor_Mode { MODE_MIT, MODE_POS_VEL, MODE_SPEED };
enum DM_Motor_Param_Command { DM_PARAM_CMD_READ = 0x33, DM_PARAM_CMD_WRITE = 0x55, DM_PARAM_CMD_SAVE = 0xAA };

typedef struct {
    uint16_t id;
    uint16_t  mode;
    uint8_t status;
    uint8_t firstTime;
    int32_t cnt;
    float bias;
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

        struct
    {
        float p_des;
        float v_des;
        float torque;
    }modelData;

    uint8_t inputEnabled;
    uint8_t MultiCycleEnabled;

} DM_Motor_Type;

int float_to_uint(float X_float, float X_min, float X_max, int bits);

float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits);

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id,uint8_t inputEnabled , uint8_t MultiCycleEnabled);

void DM_Motor_Command(DM_Motor_Type *motor, uint8_t command);

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd);

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque);

void DM_Motor_Control(DM_Motor_Type *motor);

void DM_Motor_Parse_Feedback(DM_Motor_Type *motor, const uint8_t data[8]);

void DM_Motor_Set_Bias(DM_Motor_Type *motor, const float Bias);

void DM_Motor_Read_Param(DM_Motor_Type *motor, uint8_t rid);
void DM_Motor_Write_Param(DM_Motor_Type *motor, uint8_t rid, const uint8_t write_param[4]);
void DM_Motor_Write_Param_U32(DM_Motor_Type *motor, uint8_t rid, uint32_t value);
void DM_Mootr_Write_Param_Float(DM_Motor_Type *motor, uint8_t rid, float value);
void DM_Motor_Save_Param(DM_Motor_Type *motor);

uint32_t DM_Motor_Param_U32_From_Rx(const uint8_t rx_data[8]);
float    DM_Motor_Param_Float_From_Rx(const uint8_t rx_data[8]);

#endif
