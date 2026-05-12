#include "DM_Driver.h"
#include "can.h"

static void DM_Motor_Param_Frame_Send(uint16_t motor_id, uint8_t command, uint8_t rid, const uint8_t payload[4]) {
    uint8_t sendbuff[8];

    sendbuff[0] = (uint8_t) (motor_id & 0xFF);
    sendbuff[1] = (uint8_t) ((motor_id >> 8) & 0xFF);
    sendbuff[2] = command;
    sendbuff[3] = rid;
    sendbuff[4] = payload[0];
    sendbuff[5] = payload[1];
    sendbuff[6] = payload[2];
    sendbuff[7] = payload[3];

    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.DLC = 8;
    txHeader.StdId = DM_PARAM_TX_CAN_ID;

    // Can_Send_Msg(CAN1, DM_PARAM_TX_CAN_ID, sendbuff, 8);
    HAL_CAN_AddTxMessage(&hcan2, &txHeader, sendbuff, NULL);
}

int float_to_uint(float X_float, float X_min, float X_max, int bits) {
    float span   = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id, uint8_t inputEnabled) {
    motor->mode         = mode;
    motor->id           = id;
    motor->inputEnabled = inputEnabled;

    if (motor->inputEnabled) {
        DM_Motor_Command(motor, Motor_Enble);

        DM_Motor_Write_Param_U32(motor, 0x0A, 0x01);
    }
}

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd) {
    motor->kp = kp;
    motor->kd = kd;
}

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque) {
    motor->p_des  = p_des;
    motor->v_des  = v_des;
    motor->torque = torque;
}

void DM_Motor_Command(DM_Motor_Type *motor, uint8_t command) {
    uint8_t sendbuff[8];
    for (int i = 0; i < 7; i++)
        sendbuff[i] = 0xff;

    switch (command) {
    case Motor_Enble:
        sendbuff[7] = 0xfc;
        break;
    case Motor_Disable:
        sendbuff[7] = 0xfd;
        break;

    default:
        break;
    }
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.DLC = 8;
    txHeader.StdId = motor->id;
    // Can_Send_Msg(CAN1, motor->id, sendbuff, 8);
    HAL_CAN_AddTxMessage(&hcan2, &txHeader, sendbuff, NULL);
}

void DM_Motor_Read_Param(DM_Motor_Type *motor, uint8_t rid) {
    const uint8_t zero_payload[4] = {0, 0, 0, 0};
    DM_Motor_Param_Frame_Send(motor->id, DM_PARAM_CMD_READ, rid, zero_payload);
}

void DM_Motor_Write_Param(DM_Motor_Type *motor, uint8_t rid, const uint8_t write_param[4]) {
    DM_Motor_Param_Frame_Send(motor->id, DM_PARAM_CMD_WRITE, rid, write_param);
}

void DM_Motor_Write_Param_U32(DM_Motor_Type *motor, uint8_t rid, uint32_t value) {
    uint8_t payload[4];

    payload[0] = (uint8_t) (value & 0xFF);
    payload[1] = (uint8_t) ((value >> 8) & 0xFF);
    payload[2] = (uint8_t) ((value >> 16) & 0xFF);
    payload[3] = (uint8_t) ((value >> 24) & 0xFF);

    DM_Motor_Write_Param(motor, rid, payload);
}

void DM_Motor_Write_Param_Float(DM_Motor_Type *motor, uint8_t rid, float value) {
    union {
        float    f;
        uint32_t u32;
    } converter;

    converter.f = value;
    DM_Motor_Write_Param_U32(motor, rid, converter.u32);
}

void DM_Motor_Save_Param(DM_Motor_Type *motor) {
    const uint8_t save_payload[4] = {0, 0, 0, 0};
    DM_Motor_Param_Frame_Send(motor->id, DM_PARAM_CMD_SAVE, 1, save_payload);
}

uint32_t DM_Motor_Param_U32_From_Rx(const uint8_t rx_data[8]) {
    return ((uint32_t) rx_data[4]) | ((uint32_t) rx_data[5] << 8) | ((uint32_t) rx_data[6] << 16) | ((uint32_t) rx_data[7] << 24);
}

float DM_Motor_Param_Float_From_Rx(const uint8_t rx_data[8]) {
    union {
        uint32_t u32;
        float    f;
    } converter;

    converter.u32 = DM_Motor_Param_U32_From_Rx(rx_data);
    return converter.f;
}

void DM_Motor_Control(DM_Motor_Type *motor) {
    uint8_t sendbuff[8];

    if (motor->mode == MODE_MIT) {
        uint16_t Position_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

        Position_Tmp = float_to_uint(motor->p_des, DM_MIT_P_MIN, DM_MIT_P_MAX, 16);
        Velocity_Tmp = float_to_uint(motor->v_des, DM_MIT_V_MIN, DM_MIT_V_MAX, 12);
        Torque_Tmp   = float_to_uint(motor->torque, DM_MIT_T_MIN, DM_MIT_T_MAX, 12);
        KP_Tmp       = float_to_uint(motor->kp, DM_MIT_KP_MIN, DM_MIT_KP_MAX, 12);
        KD_Tmp       = float_to_uint(motor->kd, DM_MIT_KD_MIN, DM_MIT_KD_MAX, 12);

        sendbuff[0] = (uint8_t) (Position_Tmp >> 8);
        sendbuff[1] = (uint8_t) (Position_Tmp);
        sendbuff[2] = (uint8_t) (Velocity_Tmp >> 4);
        sendbuff[3] = (uint8_t) ((Velocity_Tmp & 0x0F) << 4) | (KP_Tmp >> 8);
        sendbuff[4] = (uint8_t) (KP_Tmp);
        sendbuff[5] = (uint8_t) (KD_Tmp >> 4);
        sendbuff[6] = (uint8_t) ((KD_Tmp & 0x0F) << 4) | (Torque_Tmp >> 8);
        sendbuff[7] = (uint8_t) (Torque_Tmp);
    }
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.DLC = 8;
    txHeader.StdId = motor->id;
    if (motor->inputEnabled) HAL_CAN_AddTxMessage(&hcan2, &txHeader, sendbuff, NULL);
}
