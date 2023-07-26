/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include <stdlib.h>
#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
#include "calculate.h"
extern ext_game_robot_state_t robot_state;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern chassis_data_t chassis_data_receive;
#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
shoot_control_t shoot_control; //�������
fp32 trigger_speed = 0;
static const uint16_t tolerant = FASTER_TRIGGER_SPEED/6*100+1200;		//trigger_speed * 100 / 6 + 1200
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    
		static const fp32 Trigger_ecd_reverse_pid[3] = {TRIIGER_ECD_REVERSE_PID_KP, TRIIGER_ECD_REVERSE_PID_KI, TRIIGER_ECD_REVERSE_PID_KD};
		static const fp32 Bullet_speed_adpt_pid[3] = {40, 3, 0};
		static const fp32 Fric_speed_pid0[3] = {35, 0.35, 0};
		static const fp32 Fric_speed_pid1[3] = {36, 0.35, 0};
    shoot_control.shoot_mode = SHOOT_ZERO_FORCE;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    shoot_control.shoot_state = get_robot_status_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric_motor_measure[0] = get_fric_motor_measure_point(1);
    shoot_control.fric_motor_measure[1] = get_fric_motor_measure_point(2);
    //��ʼ��PID
		static const fp32 Trigger_speed_pid[3] = {TRIGGER_FAST_SPEED_PID_KP, TRIGGER_FAST_SPEED_PID_KI, TRIGGER_FAST_SPEED_PID_KD};
		trigger_speed = FASTER_TRIGGER_SPEED;
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		
		PID_init(&shoot_control.trigger_motor_ecd_pid, PID_POSITION, Trigger_ecd_reverse_pid, 16000, 3000);
    PID_init(&shoot_control.fric_motor_pid[0], PID_POSITION, Fric_speed_pid0, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_pid[1], PID_POSITION, Fric_speed_pid1, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		shoot_control.fric_motor_pid[0].proportion_output_filter_coefficient = exp(-600*1E-3);
		shoot_control.fric_motor_pid[1].proportion_output_filter_coefficient = exp(-550*1E-3);
//		shoot_control.trigger_motor_pid.derivative_output_filter_coefficient = exp(-0.05*1E-3);
		shoot_control.trigger_motor_pid.proportion_output_filter_coefficient = exp(-1000*1E-3);
		
		shoot_control.ecd_count=0;
    //��������
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_16, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_16, FRIC_OFF);
    shoot_control.fric_can1 = FRIC_OFF;
    shoot_control.fric_can2 = FRIC_OFF;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
		shoot_control.sum_ecd_set=shoot_control.sum_ecd;
		shoot_control.fric_error_count = 0;
		shoot_control.last_bullet_speed = 0;
		shoot_control.bullet_speed = 0;
		shoot_control.adp_flag = 0;
		shoot_control.adp_tolerant = 3;
		PID_init(&shoot_control.bullet_speed_pid, PID_POSITION,Bullet_speed_adpt_pid, 70, 30);
}

/**
  * @brief          ���ѭ��q2
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
		
		//trigger set
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
				
						PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
						shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
				
    }
		else if(shoot_control.shoot_mode == SHOOT_ZERO_FORCE)
					shoot_control.given_current = 0;
		else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    
		//fric set
    if (shoot_control.shoot_mode == SHOOT_STOP||shoot_control.shoot_mode == SHOOT_ZERO_FORCE)
    {
				shoot_laser_off();
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        shoot_control.fric1_ramp.out = 0;
        shoot_control.fric2_ramp.out = 0;
    }
    else
    {
        shoot_laser_on(); //���⿪��
        //���㲦���ֵ��PID
				if(shoot_control.reverse_time==0){
						PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
						shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
				}
				else{
						shoot_control.given_current = shoot_control.trigger_motor_ecd_pid.out;
				}
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
    }

    shoot_control.fric_can1 = (int16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_can2 = (int16_t)(shoot_control.fric2_ramp.out);
    PID_calc(&shoot_control.fric_motor_pid[0], shoot_control.fric_motor_measure[0]->speed_rpm, shoot_control.fric_can1);
    PID_calc(&shoot_control.fric_motor_pid[1], shoot_control.fric_motor_measure[1]->speed_rpm, -shoot_control.fric_can2);
		CAN_CMD_FRIC((int16_t)shoot_control.fric_motor_pid[0].out, (int16_t)shoot_control.fric_motor_pid[1].out, gimbal_control.gimbal_scope_motor.current_set);
//		CAN_CMD_FRIC(0,0,gimbal_control.gimbal_scope_motor.current_set);
		
		
			return shoot_control.given_current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
static int8_t last_s = RC_SW_UP;
    static uint8_t fric_state = 0;
    static uint16_t press_time = 0;
    //if GIMBAL_ZERO_FORCE, set ammo booster to ZERO_FORCE, where 0 is sent to trigger and fric stop
		if(gimbal_behaviour == GIMBAL_ZERO_FORCE)	
				shoot_control.shoot_mode = SHOOT_ZERO_FORCE;
		else if(shoot_control.shoot_mode == SHOOT_ZERO_FORCE)
				shoot_control.shoot_mode = SHOOT_STOP;
		//���˲��Ͽ���Ħ���֣��ٴβ��Ϲر�
		if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
		{
				shoot_control.shoot_mode = SHOOT_READY;
		}
		else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP && shoot_control.shoot_mode != SHOOT_ZERO_FORCE))
		{
				shoot_control.shoot_mode = SHOOT_STOP;
		}

		//���˲��а�X����Ħ���֣���Z�ر�
		if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && (shoot_control.shoot_mode == SHOOT_STOP || shoot_control.shoot_mode == SHOOT_ZERO_FORCE))//try to remove SHOOT_ZERO_FORCE
		{
				shoot_control.shoot_mode = SHOOT_READY;
		}
		else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP && shoot_control.shoot_mode != SHOOT_ZERO_FORCE)
		{
				shoot_control.shoot_mode = SHOOT_STOP;
		}
		//����Ħ���ֺ󣬲��˲��£��̶������3s�����䵯��
		if (shoot_control.shoot_mode == SHOOT_READY)
		{
				if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
				{
						static int count=0;
						count++;
						if(count>3000)
						{
							shoot_control.shoot_mode = SHOOT_BULLET;
							count=0;
						}
				}
		}
		//����Ħ���ֺ󣬲��˲��У���һ��������һ�ε���
		if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_mode == SHOOT_READY||shoot_control.shoot_mode == SHOOT_BULLET))
		{

				if ((shoot_control.press_l && shoot_control.last_press_l == 0) )
				{
						shoot_control.shoot_mode = SHOOT_BULLET;
				}
		}
		//��������
		get_shoot_heat1_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
		if (!toe_is_error(REFEREE_TOE))
		{		//Ӣ�۴�����������100
				if ((shoot_control.heat + 100 > shoot_control.heat_limit) && !(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G) && !shoot_control.move_flag && shoot_control.shoot_mode==SHOOT_BULLET)
				{
							shoot_control.shoot_mode = SHOOT_READY;
				}
		}
		//Ħ�������ߣ�ammo booster �ϵ磩������������
		if(shoot_control.fric_error_count == 150)
				shoot_control.shoot_mode = SHOOT_ZERO_FORCE;
    //��̨����������������
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_ZERO_FORCE;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

		shoot_control.firc_speed[0]=shoot_control.fric_motor_measure[0]->speed_rpm;
		shoot_control.firc_speed [1]=-shoot_control.fric_motor_measure[1]->speed_rpm;
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

//    //���׵�ͨ�˲�
//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = speed_fliter_3;
//    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
//    shoot_control.speed = speed_fliter_3;
		
		
    speed_fliter_3 = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    shoot_control.speed = speed_fliter_3;
    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 20Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
		
		shoot_control.sum_ecd= shoot_control.shoot_motor_measure->ecd_count*8191+ shoot_control.shoot_motor_measure->ecd;
    //��갴��
		shoot_control.last_press_l = shoot_control.press_l;
		shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
		if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }//try to remove this part
		
		if(shoot_control.shoot_mode==SHOOT_ZERO_FORCE)
		{
				shoot_control.sum_ecd_set=shoot_control.sum_ecd;
		}
		
		//limit Iout of trigger_pid
		if(shoot_control.trigger_motor_pid.Iout>6000)
				shoot_control.trigger_motor_pid.Iout = 6000;
		
		//bullet_speed_adapt();
		
		//fric offline count
		if(toe_is_error(FRIC_LEFT_MOTOR_TOE)||toe_is_error(FRIC_RIGHT_MOTOR_TOE))
				shoot_control.fric_error_count++;
		else
				shoot_control.fric_error_count=0;
		
}

static void trigger_motor_turn_back(void)
{
		
		if (shoot_control.block_time < BLOCK_TIME)//set speed
				shoot_control.speed_set = trigger_speed;
		
    if(shoot_control.speed < BLOCK_TRIGGER_SPEED && shoot_control.speed > -0.5 && shoot_control.block_time < BLOCK_TIME)//probably blocked
    {
        shoot_control.block_time++;
				shoot_control.sum_ecd_reverse = shoot_control.sum_ecd - REVERSE_ECD;
    }
    else if (shoot_control.block_time >= BLOCK_TIME)//block confirmed
    {
        shoot_control.reverse_time++;
    }
		else if(shoot_control.speed >=BLOCK_TRIGGER_SPEED){
				shoot_control.block_time=0;
				shoot_control.reverse_time = 0;
		}
		
		if(shoot_control.reverse_time)//reverse
		{		
				PID_calc(&shoot_control.trigger_motor_ecd_pid, shoot_control.sum_ecd, shoot_control.sum_ecd_reverse);
				if(abs(shoot_control.sum_ecd-shoot_control.sum_ecd_reverse)<629)//reverse completed
				{
						shoot_control.reverse_time = 0;
						shoot_control.block_time = 0;
						PID_clear(&shoot_control.trigger_motor_ecd_pid);
				}
				if(shoot_control.reverse_time>REVERSE_TIME )//abandon reverse, continue pushing trigger even blocked
				{
						
						shoot_control.block_time = 0;
						shoot_control.speed_set = trigger_speed;
						//PID_clear(&shoot_control.trigger_motor_ecd_pid);
				}
		}
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //ÿ�β��� 2/5PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.sum_ecd_set=shoot_control.sum_ecd_set+31458;
				//8191*3591/187/5==31458.6963
        shoot_control.move_flag = 1;
    }
    //����Ƕ��ж�
    if (shoot_control.sum_ecd_set - shoot_control.sum_ecd > tolerant)
    {
        //û����һֱ������ת�ٶ�
        shoot_control.speed_set = trigger_speed;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY;
    }
}
shoot_control_t *get_shoot_point(void)
{
    return &shoot_control;
}
uint8_t get_shoot_mode(void){
		return shoot_control.shoot_mode;
}

void bullet_speed_adapt(void){
//		if(shoot_control.bullet_speed !=chassis_data_receive.bullet_speed && chassis_data_receive.bullet_speed > 14.5 && chassis_data_receive.bullet_speed < 17.0){		//new bullet speed comes
//				//bullet speed incorrect
//				if(shoot_control.last_bullet_speed !=0){
//						if(shoot_control.bullet_speed < TARGET_BULLET_SPEED && shoot_control.last_bullet_speed < TARGET_BULLET_SPEED && chassis_data_receive.bullet_speed < TARGET_BULLET_SPEED - 0.35){
//								shoot_control.adp_flag = 1;
//						}
//						else if(shoot_control.bullet_speed > TARGET_BULLET_SPEED && shoot_control.last_bullet_speed > TARGET_BULLET_SPEED && chassis_data_receive.bullet_speed > TARGET_BULLET_SPEED + 0.35){
//								shoot_control.adp_flag = 1;
//						}
//						//TODO: time factor
//				}
//				//update bullet speed record
//				if(shoot_control.adp_flag){
//						fp32 adpt_value = -(shoot_control.last_bullet_speed + shoot_control.bullet_speed + chassis_data_receive.bullet_speed) / 3 + TARGET_BULLET_SPEED;
//						adpt_value *= 40;
//						shoot_control.fric1_ramp.max_value += adpt_value;
//						shoot_control.fric2_ramp.max_value += adpt_value;
//				}
//				shoot_control.last_bullet_speed = shoot_control.bullet_speed;
//				shoot_control.bullet_speed = chassis_data_receive.bullet_speed;
//				shoot_control.adp_flag = 0;
//		}
		if(shoot_control.bullet_speed !=chassis_data_receive.bullet_speed)
				PID_calc(&shoot_control.bullet_speed_pid, chassis_data_receive.bullet_speed,TARGET_BULLET_SPEED);
		if(shoot_control.bullet_speed !=chassis_data_receive.bullet_speed && chassis_data_receive.bullet_speed < TARGET_BULLET_SPEED - 0.35)
				shoot_control.adp_tolerant--;
		else if(shoot_control.bullet_speed !=chassis_data_receive.bullet_speed && chassis_data_receive.bullet_speed > TARGET_BULLET_SPEED + 0.35)
				shoot_control.adp_tolerant++;
		if(shoot_control.adp_tolerant == 0){
				shoot_control.fric1_ramp.max_value += shoot_control.bullet_speed_pid.out;
				shoot_control.fric2_ramp.max_value += shoot_control.bullet_speed_pid.out;
				shoot_control.adp_tolerant += 2;
		}
		else if(shoot_control.adp_tolerant == 5){
				shoot_control.fric1_ramp.max_value += shoot_control.bullet_speed_pid.out;
				shoot_control.fric2_ramp.max_value += shoot_control.bullet_speed_pid.out;
				shoot_control.adp_tolerant -= 2;
		}
		shoot_control.bullet_speed = chassis_data_receive.bullet_speed;
}

int16_t get_mean_fric_rpm(void){
		return (shoot_control.firc_speed[0] - shoot_control.firc_speed[1])/2;
}
