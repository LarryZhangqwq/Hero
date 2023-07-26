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

#include "CAN_receive.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern fp32 intermedia_chassis_speed[4];
extern chassis_move_t chassis_move;
//motor data read
#define CAN_Commucation hcan2

//motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

#define get_cap_measure(ptr, data)                               \
  {                                                                \
		(ptr)->InputVot = ((fp32) ((data)[1] << 8 | (data)[0]))/100;      \
		(ptr)->CapVot = ((fp32) ((data)[3] <<8 | (data) [2]))/100; \
		(ptr) -> TestCurrent = ((fp32) ((data)[5] <<8 | (data) [4]))/100;   \
		(ptr) -> Target_Power = ((fp32) ((data)[7] <<8 | (data) [6]))/100;   \
  }

#define get_anhe_cap_measure(ptr, data)                               \
  {                                                                \
		(ptr)->InputVot = ((fp32) ((data)[2] << 8 | (data)[3]))/100;      \
		(ptr) -> TestCurrent = ((fp32) ((data)[4] <<8 | (data) [5]))/100;   \
		(ptr) -> Target_Power = ((fp32) ((data)[0] <<8 | (data) [1]))/100;   \
  }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006��� 7:��Ħ���ֵ�� 3508 8����Ħ���ֵ�� 3508*/
motor_measure_t motor_chassis[9];
cap_measure_t cap_measure;
gimbal_data_t gimbal_trans={{0,0,0},0,0,0};
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef fric_tx_message;
static uint8_t fric_can_send_data[8];

static uint8_t cap_can_send_data[8];
static CAN_TxHeaderTypeDef cap_tx_measure;
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[9];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
		
	/*Add*/		
		case chassis_motor_cmd_id:
		{
				//get yaw,pitch
				int16_t temp[4];
				temp[0]=(int16_t)(rx_data[0]<<8|rx_data[1]);
				temp[1]=(int16_t)(rx_data[2]<<8|rx_data[3]);
				temp[2]=(int16_t)(rx_data[4]<<8|rx_data[5]);
				temp[3]=(int16_t)(rx_data[6]<<8|rx_data[7]);
				for(int i=0;i<3;i++)
						gimbal_trans.speed_set[i]=((fp32)temp[i])/transform_key;
				gimbal_trans.chassis_mode=temp[3];

				break;
		}
		case ui_info_id:
		{
				int16_t temp[4];
				
				temp[1]=(int16_t)(rx_data[2]<<8|rx_data[3]);
				temp[2]=(int16_t)(rx_data[4]<<8|rx_data[5]);
				temp[3]=(int16_t)(rx_data[6]<<8|rx_data[7]);
				gimbal_trans.shoot_mode=rx_data[0]&0x1;
				gimbal_trans.selected_key = (rx_data[0]&0xc0)|(rx_data[0]&0x1f)>>1;	//contains 7 keys:a,w,s,d,shift,q,e
				gimbal_trans.swing_flag=rx_data[1]&0x01;
				gimbal_trans.scope_state=rx_data[1]&0x02;
				gimbal_trans.pitch_angel_degree=(fp32)temp[1]/100;
				gimbal_trans.fric_rpm=temp[2];
				gimbal_trans.yaw_relative_angel=(fp32)temp[3]/100;
		}

    default:
    {
      break;
    }
    }
  }
	
  if (hcan == &hcan1)
  {
    switch (rx_header.StdId)
    {
			
		case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      static uint8_t i = 0;
      //get motor id
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
		case CAN_CAP:
		{
			get_cap_measure(&cap_measure, rx_data);
  		detect_hook(CAP_TOE);
			break;
		}
		case CAP_INPUT_REQ:
		{
			get_anhe_cap_measure(&cap_measure,rx_data);
			detect_hook(CAP_TOE);
      break;
		}
		case CAP_OUTPUT_REQ:
		{
			cap_measure.CapVot = (rx_data[2]<<8|rx_data[3]) * 0.01;
			detect_hook(CAP_TOE);
		}
    default:
    {
      break;
    }
    }
  }
	
}

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
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
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
void CAN_CMD_FRIC(int16_t motor1, int16_t motor2)
{
  uint32_t send_mail_box;
  fric_tx_message.StdId = 0x200;
  fric_tx_message.IDE = CAN_ID_STD;
  fric_tx_message.RTR = CAN_RTR_DATA;
  fric_tx_message.DLC = 0x08;
  fric_can_send_data[0] = (motor1 >> 8);
  fric_can_send_data[1] = motor1;
  fric_can_send_data[2] = (motor2 >> 8);
  fric_can_send_data[3] = motor2;
  HAL_CAN_AddTxMessage(&hcan2, &fric_tx_message, fric_can_send_data, &send_mail_box);
}

void CAN_CMD_CAP(float power, float buffer)
{
	uint16_t powertx;
	powertx = power * 100;
	uint16_t save_command = 0;
  uint32_t send_mail_box;
  cap_tx_measure.StdId = CAP_INPUT_POWER;
  cap_tx_measure.IDE = CAN_ID_STD;
  cap_tx_measure.RTR = CAN_RTR_DATA;
  cap_tx_measure.DLC = 0x08;
	cap_can_send_data[0] = powertx>>8;
	cap_can_send_data[1] = powertx;
	cap_can_send_data[2] = save_command>>8;
	cap_can_send_data[3] = save_command;
  HAL_CAN_AddTxMessage(&hcan1, &cap_tx_measure, cap_can_send_data, &send_mail_box);
}

void CAN_CAP_REQ(uint16_t request_id)
{
  uint32_t send_mail_box;
  cap_tx_measure.StdId = request_id;
  cap_tx_measure.IDE = CAN_ID_STD;
  cap_tx_measure.RTR = CAN_RTR_REMOTE;
  cap_tx_measure.DLC = 0x06;
  HAL_CAN_AddTxMessage(&hcan1, &cap_tx_measure, cap_can_send_data, &send_mail_box);
}

void CAN_CAP_start(uint16_t volt)
{
	
	uint16_t save_command = 1;
  uint32_t send_mail_box;
  cap_tx_measure.StdId = CAP_RUN;
  cap_tx_measure.IDE = CAN_ID_STD;
  cap_tx_measure.RTR = CAN_RTR_DATA;
  cap_tx_measure.DLC = 0x08;
	cap_can_send_data[0] = 0;
	cap_can_send_data[1] = 2;
	cap_can_send_data[2] = save_command>>8;
	cap_can_send_data[3] = save_command;
  HAL_CAN_AddTxMessage(&hcan1, &cap_tx_measure, cap_can_send_data, &send_mail_box);
	cap_tx_measure.StdId = CAP_OUTPUT_VOLTAGE;
  cap_tx_measure.IDE = CAN_ID_STD;
  cap_tx_measure.RTR = CAN_RTR_DATA;
  cap_tx_measure.DLC = 0x08;
	cap_can_send_data[0] = volt>>8;
	cap_can_send_data[1] = volt;
	cap_can_send_data[2] = save_command>>8;
	cap_can_send_data[3] = save_command;
  HAL_CAN_AddTxMessage(&hcan1, &cap_tx_measure, cap_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


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
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

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
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

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
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

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
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
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
const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
  return &motor_chassis[i + 6];
}

/*Additional Functions*/

static CAN_TxHeaderTypeDef  gimbal_board_heat_message;
static uint8_t              gimbal_board_heat_send_data[8]={0};

void CAN_heat_data_send(uint16_t shooter_heat, uint16_t shoot_heat_limit, uint8_t power_state){
		uint32_t send_mail_box;
		
    gimbal_board_heat_message.StdId = heat_data_id;
    gimbal_board_heat_message.IDE = CAN_ID_STD;
    gimbal_board_heat_message.RTR = CAN_RTR_DATA;
    gimbal_board_heat_message.DLC = 0x08;
    gimbal_board_heat_send_data[0]=shooter_heat>>8;
		gimbal_board_heat_send_data[1]=shooter_heat;
		gimbal_board_heat_send_data[2]=shoot_heat_limit>>8;
		gimbal_board_heat_send_data[3]=shoot_heat_limit;
		gimbal_board_heat_send_data[4]=power_state>>8;
		gimbal_board_heat_send_data[5]=power_state;
		
    HAL_CAN_AddTxMessage(&CAN_Commucation, &gimbal_board_heat_message, gimbal_board_heat_send_data, &send_mail_box);
}

static CAN_TxHeaderTypeDef  gimbal_board_shoot_message;
static uint8_t              gimbal_board_shoot_data[8]={0};

void CAN_shoot_data_send(uint8_t	bullet_type, uint8_t	bullet_freq, float	bullet_speed){
		uint32_t send_mail_box;
		uint16_t bullet_s=(uint16_t)(bullet_speed*100);
    gimbal_board_shoot_message.StdId = shoot_data_id;
    gimbal_board_shoot_message.IDE = CAN_ID_STD;
    gimbal_board_shoot_message.RTR = CAN_RTR_DATA;
    gimbal_board_shoot_message.DLC = 0x08;
    gimbal_board_shoot_data[0]=bullet_type;
		gimbal_board_shoot_data[1]=bullet_freq;
		gimbal_board_shoot_data[2]=bullet_s>>8;
		gimbal_board_shoot_data[3]=bullet_s;
		
    HAL_CAN_AddTxMessage(&CAN_Commucation, &gimbal_board_shoot_message, gimbal_board_shoot_data, &send_mail_box);
}

void get_cap_proportion(fp32 *cap_proportion){
		*cap_proportion = (cap_measure.CapVot-17)/(24.0f-17.0f);
}

gimbal_data_t *get_gimbal_data(void){
		return &gimbal_trans;
}

