/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <inttypes.h>
#include <string.h>
#include "config.h"
#include "math_ops.h"
#include "transmit.h"

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01

typedef struct
{
  uint16_t motor_id;
  uint8_t INS_code;   
  uint8_t motor_fbd;  
} MotorCommFbd;

typedef struct
{
  uint16_t angle_actual_int;
  uint16_t angle_desired_int;
  int16_t speed_actual_int;
  int16_t speed_desired_int;
  int16_t current_actual_int;
  int16_t current_desired_int;
  float speed_actual_rad;
  float speed_desired_rad;
  float angle_actual_rad;
  float angle_desired_rad;
  uint16_t motor_id;
  uint8_t temperature;
  uint8_t error;
  float angle_actual_float;
  float speed_actual_float;
  float current_actual_float;
  float angle_desired_float;
  float speed_desired_float;
  float current_desired_float;
  float power;
  uint16_t acceleration;
  uint16_t linkage_KP;
  uint16_t speed_KI;
  uint16_t feedback_KP;
  uint16_t feedback_KD;
} OD_Motor_Msg;

typedef struct
{
  float angle_float[3];
  float gyro_float[3];
  float accel_float[3];
  float mag_float[3];
  float quat_float[4];
} IMU_Msg;

extern OD_Motor_Msg rv_motor_msg[6];
extern IMU_Msg imu_msg;
extern uint16_t motor_id_check;

void MotorIDReset(EtherCAT_Msg* TxMessage);
void MotorIDSetting(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint16_t motor_id_new);
void MotorSetting(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint8_t cmd);
void MotorCommModeReading(EtherCAT_Msg* TxMessage, uint16_t motor_id);
void MotorIDReading(EtherCAT_Msg* TxMessage);

void send_motor_ctrl_cmd(EtherCAT_Msg* TxMessage, uint8_t channel, uint16_t motor_id, float kp, float kd, float pos,
                         float spd, float cur);
void set_motor_position(EtherCAT_Msg* TxMessage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur,
                        uint8_t ack_status);
void set_motor_speed(EtherCAT_Msg* TxMessage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
void set_motor_cur_tor(EtherCAT_Msg* TxMessage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status,
                       uint8_t ack_status);
void set_motor_acceleration(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint16_t acc, uint8_t ack_status);
void set_motor_linkage_speedKI(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint16_t linkage, uint16_t speedKI,
                               uint8_t ack_status);
void set_motor_feedbackKP(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint16_t fdbKP, uint8_t ack_status);
void get_motor_parameter(EtherCAT_Msg* TxMessage, uint16_t motor_id, uint8_t param_cmd);

void RV_can_data_repack(EtherCAT_Msg* RxMessage, uint8_t comm_mode, uint8_t slave_id);
void RV_can_imu_data_repack(EtherCAT_Msg* RxMessage);

int float_to_uint(float x, float x_min, float x_max, int bits);

#endif
