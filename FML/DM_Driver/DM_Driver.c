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

float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    uint32_t max_int = (1UL << bits) - 1UL;
    return ((float)x) * span / (float)max_int + x_min;
}

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id, uint8_t inputEnabled, uint8_t MultiCycleEnabled) {
    motor->mode         = mode;
    motor->id           = id;
    motor->inputEnabled = inputEnabled;
    motor->MultiCycleEnabled = MultiCycleEnabled;

    if (motor->inputEnabled) {
        DM_Motor_Command(motor, Motor_Enble);

        DM_Motor_Write_Param_U32(motor, 0x07, id|=0x10);
        DM_Motor_Save_Param(motor);
    }
}

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd) {
    motor->sendData.kp = kp;
    motor->sendData.kd = kd;
}

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque) {
    motor->sendData.p_des  = p_des;
    motor->sendData.v_des  = v_des;
    motor->sendData.torque = torque;
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
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.ExtId = 0;
    uint32_t tick = HAL_GetTick();
    uint32_t mailbox = 0;
    // Can_Send_Msg(CAN1, motor->id, sendbuff, 8);
    while(HAL_CAN_AddTxMessage(&hcan2, &txHeader, sendbuff, &mailbox) != HAL_OK){
        if (HAL_GetTick() - tick > 500)
        {
            /* code */
            break;
        }
    }
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
    uint32_t id = motor->id;

    if (motor->mode == MODE_MIT) {
        uint16_t Position_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

        Position_Tmp = float_to_uint(motor->sendData.p_des, DM_MIT_P_MIN, DM_MIT_P_MAX, 16);
        Velocity_Tmp = float_to_uint(motor->sendData.v_des, DM_MIT_V_MIN, DM_MIT_V_MAX, 12);
        Torque_Tmp   = float_to_uint(motor->sendData.torque, DM_MIT_T_MIN, DM_MIT_T_MAX, 12);
        KP_Tmp       = float_to_uint(motor->sendData.kp, DM_MIT_KP_MIN, DM_MIT_KP_MAX, 12);
        KD_Tmp       = float_to_uint(motor->sendData.kd, DM_MIT_KD_MIN, DM_MIT_KD_MAX, 12);

        sendbuff[0] = (uint8_t) (Position_Tmp >> 8);
        sendbuff[1] = (uint8_t) (Position_Tmp);
        sendbuff[2] = (uint8_t) (Velocity_Tmp >> 4);
        sendbuff[3] = (uint8_t) ((Velocity_Tmp & 0x0F) << 4) | (KP_Tmp >> 8);
        sendbuff[4] = (uint8_t) (KP_Tmp);
        sendbuff[5] = (uint8_t) (KD_Tmp >> 4);
        sendbuff[6] = (uint8_t) ((KD_Tmp & 0x0F) << 4) | (Torque_Tmp >> 8);
        sendbuff[7] = (uint8_t) (Torque_Tmp);
    }
    uint32_t tick = HAL_GetTick();
    uint32_t mailbox = 0;
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.DLC = 8;
    txHeader.StdId = motor->id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.ExtId = 0;
    if (motor->inputEnabled) {
        while(HAL_CAN_AddTxMessage(&hcan2, &txHeader, sendbuff, &mailbox) != HAL_OK){
            if (HAL_GetTick() - tick > 500)
            {
                /* code */
                break;
            }

        }
    }
}


void DM_Motor_Parse_Feedback(DM_Motor_Type *motor, const uint8_t data[8]){

    uint32_t id = motor->id;

    motor->status = data[0] >> 4;

    uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t t_int = (((uint16_t)data[4] & 0x0FU) << 8) | data[5];

    float tmp = uint_to_float(p_int, DM_MIT_P_MIN, DM_MIT_P_MAX, 16) + DM_MIT_P_MAX;
    if(motor->MultiCycleEnabled && motor->firstTime){
        if(tmp - motor->receiveData.p_des <= DM_MIT_P_MIN){
            motor->cnt++;
        }else if(tmp - motor->receiveData.p_des > DM_MIT_KD_MAX){
            motor->cnt--;
        }
        motor->receiveData.p_des = tmp + motor->cnt * 2 * DM_MIT_P_MAX;
    }else{
        motor->receiveData.p_des = tmp;
        motor->firstTime = 1;
    }
    motor->receiveData.v_des = uint_to_float(v_int, DM_MIT_V_MIN, DM_MIT_V_MAX, 12);
    motor->receiveData.torque = uint_to_float(t_int, DM_MIT_T_MIN, DM_MIT_T_MAX, 12);
}

void DM_Motor_Set_Bias(DM_Motor_Type *motor, const float Bias){
    motor->bias = Bias;
    motor->receiveData.p_des -= Bias;
}