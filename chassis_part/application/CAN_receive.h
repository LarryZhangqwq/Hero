/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"


/* CAN send and receive ID */
#define CAN_Communication hcan2
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
		CAN_CAP = 0x211,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

		CAN_FRIC_LIFT_MOTOR_ID = 0x201,
		CAN_FRIC_RIGHT_MOTOR_ID = 0x202,
		
		
} can_msg_id_e;

typedef enum
{
		CAP_RUN = 0x600,
		CAP_INPUT_POWER = 0x601,
		CAP_OUTPUT_VOLTAGE = 0x602,
		CAP_INPUT_REQ = 0x611,
		CAP_OUTPUT_REQ = 0x612,
	
} can_cap_id_e;

/*Addtional Define*/
//can be added to enum
//#define gimbal_board_Id 0x210
//#define chassis_board_Id 0x211
#define chassis_motor_cmd_id 0x301
#define ui_info_id 0x302
#define heat_data_id 0x401
#define shoot_data_id 0x402
#define state_data_id 0x403

#define transform_key 1000

typedef struct
{		
		fp32 speed_set[3];
		uint8_t chassis_mode;
		uint8_t shoot_mode;  // ���ģʽ: 0-Ħ���֡������ֶ�����; 1-Ħ���ֶ������ֲ���; 2-����
		uint8_t swing_flag;  // 1��С���� 0����
		fp32 pitch_angel_degree;
		int16_t fric_rpm;			//Ħ����ת��
		fp32 yaw_relative_angel;  // ��Ϊ body_angel
		uint8_t scope_state;		//1-scope_on 0-scope_off
		uint8_t selected_key;
}gimbal_data_t;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct 
{
	 fp32 InputVot;
	 fp32 CapVot;
	 fp32 TestCurrent;
	 fp32 Target_Power;
	 fp32 cap_percent;
	 fp32 OutputVolt;
} cap_measure_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
/**
  * @brief          return the fric 3508 motor data point
  * @param[in]      i: motor number,range [1,2]
  * @retval         motor data point
  */
/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[1,2]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_fric_motor_measure_point(uint8_t i);

/**
  * @brief          send fric current
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����Ħ���ֵ������
  * @param[in]      none
  * @retval         none
  */
 extern void CAN_CMD_FRIC(int16_t motor1, int16_t motor2);
void CAN_CMD_CAP(float power, float buffer);

void CAN_CAP_REQ(uint16_t request_id);
void CAN_CAP_start(uint16_t volt);

void CAN_heat_data_send(uint16_t shooter_heat, uint16_t shoot_heat_limit, uint8_t power_state);

void CAN_shoot_data_send(uint8_t	bullet_type, uint8_t	bullet_freq, float	bullet_speed);

void get_cap_proportion(fp32 *cap_proportion);

gimbal_data_t *get_gimbal_data(void);
/*Addtional Functions*/

#endif
