/**
  ****************************�����Ƽ���ѧCOD****************************
  * @file       user_task.c/h-���䳤��
  * @brief      һ����ͨ�����������ƶԴ���лл��ң�д��demo��δ���ü���װ���պϿ���
  ==============================================================================
  @endverbatim
  ****************************�����Ƽ���ѧCOD****************************
  */

#include "User_Task.h"
#include "referee.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdint.h"
#include "RM_Cilent_UI.h"
#include "UI_ProgressBar.h"
#include "UI_car.h"
#include "UI_label.h"

extern vision_control_t vision_control;

#include "vision.h"
#include "CAN_receive.h" //����Ħ���ַ���ջ���ʵת��

#define PI 3.1415936
#define cbc car.basic_config

#define TIME_COUNT_START_ASCII 30  // '0'
#define TIME_COUNT_STOP_ASCII 39   // '9'

#define set_name(d, s1, s2, s3) do{\
                                    (d)[0] = (uint8_t) (s1); \
                                    (d)[1] = (uint8_t) (s2); \
                                    (d)[2] = (uint8_t) (s3); \
                                } while(0)


void UI_send_init();

void UI_car_init();

void UI_car_change();

extern uint16_t Robot_number;
extern uint16_t Robot_cline_number;
extern UI_show_t ui;
extern uint8_t ui_armors_state[];


car_handle car;

id_data_t id_data;

Graph_Data line_1;
Graph_Data line_2;
Graph_Data line_3;
Graph_Data line_4;
String_Data time_data;

void robot_id_data_init(void);

void robot_id_select(void);

void UI_aimline();

void UI_car_static(void);

void UserTask(void const *pvParameters) {
    static uint16_t time = 0;
    static char time_count = TIME_COUNT_START_ASCII;

    memset(&car, 0, sizeof(car));

    UI_send_init();
    UI_car_init();
    for (int i = 0; i < 3; i++) {  // ˢ������
        UI_label_static();
        UI_car_init();
        UI_car_static();
        UI_aimline();
    }
    for (int i = 0; i < 3; i++) {  // ˢ������
        UI_label_static();
        UI_car_static();
        UI_aimline();

        // time change shower init
        Char_Draw(&time_data, "987", UI_Graph_ADD,
                  2, UI_Color_Yellow, 14, 1, 2,
                  910, 100, &time_count);
        Char_ReFresh(time_data);

        //aimline init
        memset(&line_1, 0, sizeof(line_1));
        Line_Draw(&line_1, "901", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 930, 712, 990, 712);

        memset(&line_2, 0, sizeof(line_2));
        Line_Draw(&line_2, "902", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 900, 696, 1020, 696);

        memset(&line_3, 0, sizeof(line_3));
        Line_Draw(&line_3, "903", UI_Graph_ADD, 1, UI_Color_Pink, 2, 870, 655, 1050, 655);

        memset(&line_4, 0, sizeof(line_4));
        Line_Draw(&line_4, "904", UI_Graph_ADD, 1, UI_Color_Green, 0, 0, 0, 0, 0);
        UI_ReFresh(2, line_1, line_2);
        UI_ReFresh(2, line_3, line_4);
    }
    while (1) {
        // ˢ��
        time = (time + 1) % 128;
        if (time == 0) {
            UI_label_static();  // ���¼������ݱ��
            UI_car_static();
        } else if (time % 50 == 0) {
            // ��֤�Ȳ�Σ�����ѡ��ID
            robot_id_select();
            // ���»���
            UI_label_cache_reset();
        } else if (time % 10 == 0) {
            UI_label_cache_reset();  // ���������ݵĻ���
            UI_aimline();  // ���»�����׼��
            Char_Draw(&time_data, "987", UI_Graph_ADD,
                      2, UI_Color_Yellow, 14, 1, 2,
                      910, 100, &time_count);
            Char_ReFresh(time_data);
            //aimline init
            memset(&line_1, 0, sizeof(line_1));
            Line_Draw(&line_1, "901", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 930, 712, 990, 712);

            memset(&line_2, 0, sizeof(line_2));
            Line_Draw(&line_2, "902", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 900, 696, 1020, 696);

            memset(&line_3, 0, sizeof(line_3));
            Line_Draw(&line_3, "903", UI_Graph_ADD, 1, UI_Color_Pink, 2, 870, 655, 1050, 655);

            memset(&line_4, 0, sizeof(line_4));
            Line_Draw(&line_4, "904", UI_Graph_ADD, 1, UI_Color_Green, 0, 0, 0, 0, 0);
            UI_ReFresh(2, line_1, line_2);
            UI_ReFresh(2, line_3, line_4);
        }
        if (time % 32 == 0) {
            // ��������
            UI_label_change();
            // ����ˢ�¼�ʱ��
            // ��ʱ������
            if (time_count < TIME_COUNT_STOP_ASCII) {
                time_count += 1;
            } else {  // time_count == TIME_COUNT_STOP_ASCII
                time_count = TIME_COUNT_START_ASCII;
            }
            Char_Draw(&time_data, "987", UI_Graph_Change,
                      2, UI_Color_Yellow, 14, 1, 2,
                      910, 100, &time_count);
            Char_ReFresh(time_data);
        }
        UI_car_change();
        vTaskDelay(1);
    }
}

void UI_car_init() {
    /*** С�� ***/
/*С����λ*/
    cbc.central_x = 1700;
    cbc.central_y = 700;
/*����*/
    cbc.full_radius = 110;
    cbc.body_half_length = 70;
    cbc.body_half_width = 55;
    cbc.rear_half_width = 40;
    cbc.head_radius = 35;

    cbc.drawing_width = 2;
    cbc.normal_colour_code = UI_Color_Yellow;
    cbc.attacked_colour_code = UI_Color_Orange;
    cbc.head_layer = 1;
    cbc.body_layer = 2;

    set_name(cbc.body_name_front, '3', '0', '0');
    set_name(cbc.body_name_back, '3', '0', '1');
    set_name(cbc.body_name_left, '3', '0', '2');
    set_name(cbc.body_name_right, '3', '0', '3');
    set_name(cbc.head_name_line, '3', '0', '4');
    set_name(cbc.head_name_circle, '3', '0', '5');
    set_name(cbc.rear_name_back, '3', '0', '6');
    set_name(cbc.rear_name_left, '3', '0', '7');
    set_name(cbc.rear_name_right, '3', '0', '8');

    car.body_degree = (uint16_t) (ui.ui_gimbal_data->yaw_relative_angel + 180);
    car.head_degree = 180;
    car.front_armor_showing_attacked = 0;  // ��ʼ״̬�ǲ�������
    car.back_armor_showing_attacked = 0;
    car.left_armor_showing_attacked = 0;
    car.right_armor_showing_attacked = 0;
}

void UI_car_static() {
    car_init_by_handle(&car);
}

void UI_car_change() {
    car_rotate_body(&car, (uint16_t) (ui.ui_gimbal_data->yaw_relative_angel + 180));
    // ���Բ�������:
    //     ǰװ�װ���ʾ����
    //     ��װ�װ���ʾ����
    //     ��װ�װ�����װ�װ�
    //     ��װ�װ�����װ�װ�
    car_front_armor_showing_attacked(&car, ui_armors_state[0]);
    car_back_armor_showing_attacked(&car, ui_armors_state[2]);
    car_left_armor_showing_attacked(&car, ui_armors_state[1]);
    car_right_armor_showing_attacked(&car, ui_armors_state[3]);
}

void UI_send_init() {

/*** ���ݸ�ֵ ***/
    robot_id_data_init();
    robot_id_select();
}


//��׼������ Ӣ�۸����� ֻ��һ�ֵ��ٵ���� �ֿ����͹ؾ�״̬
void UI_aimline() {
    uint8_t ui_scope_state = ui.ui_gimbal_data->scope_state;

    if (ui_scope_state == 2) {        //����

        memset(&line_1, 0, sizeof(line_1));
        Line_Draw(&line_1, "901", UI_Graph_Change, 1, UI_Color_Green, 2, 900, 761, 1020, 761);

        memset(&line_2, 0, sizeof(line_2));
        Line_Draw(&line_2, "902", UI_Graph_Change, 1, UI_Color_Green, 2, 900, 692, 1020, 692);

        memset(&line_3, 0, sizeof(line_3));
        Line_Draw(&line_3, "903", UI_Graph_Change, 1, UI_Color_Green, 2, 900, 422, 1020, 422);

        memset(&line_4, 0, sizeof(line_4));
        Line_Draw(&line_4, "904", UI_Graph_Change, 1, UI_Color_Green, 2, 900, 354, 1020, 354);
        UI_ReFresh(2, line_1, line_2);
        UI_ReFresh(2, line_3, line_4);
    } else {  //����֮������
        memset(&line_1, 0, sizeof(line_1));
        Line_Draw(&line_1, "901", UI_Graph_Change, 1, UI_Color_Yellow, 2, 930, 712, 990, 712);

        memset(&line_2, 0, sizeof(line_2));
        Line_Draw(&line_2, "902", UI_Graph_Change, 1, UI_Color_Yellow, 2, 900, 696, 1020, 696);

        memset(&line_3, 0, sizeof(line_3));
        Line_Draw(&line_3, "903", UI_Graph_Change, 1, UI_Color_Pink, 2, 870, 655, 1050, 655);

        memset(&line_4, 0, sizeof(line_4));
        Line_Draw(&line_4, "904", UI_Graph_Change, 1, UI_Color_Green, 0, 0, 0, 0, 0);
        UI_ReFresh(2, line_1, line_2);
        UI_ReFresh(2, line_3, line_4);
    }
}


void robot_id_data_init(void) {  //�޿��С��ڱ����״�վ��id
    id_data.ID[0] = 1;
    id_data.ID[1] = 2;
    id_data.ID[2] = 3;
    id_data.ID[3] = 4;
    id_data.ID[4] = 5;


    id_data.ID[5] = 101;
    id_data.ID[6] = 102;
    id_data.ID[7] = 103;
    id_data.ID[8] = 104;
    id_data.ID[9] = 105;


    id_data.sender_ID[0] = 1;
    id_data.sender_ID[1] = 2;
    id_data.sender_ID[2] = 3;
    id_data.sender_ID[3] = 4;
    id_data.sender_ID[4] = 5;

    id_data.sender_ID[5] = 101;
    id_data.sender_ID[6] = 102;
    id_data.sender_ID[7] = 103;
    id_data.sender_ID[8] = 104;
    id_data.sender_ID[9] = 105;

    id_data.receiver_ID[0] = 0x101;
    id_data.receiver_ID[1] = 0x102;
    id_data.receiver_ID[2] = 0x103;
    id_data.receiver_ID[3] = 0x104;
    id_data.receiver_ID[4] = 0x105;

    id_data.receiver_ID[5] = 0x165;
    id_data.receiver_ID[6] = 0x166;
    id_data.receiver_ID[7] = 0x167;
    id_data.receiver_ID[8] = 0x168;
    id_data.receiver_ID[9] = 0x169;
}


void robot_id_select(void) {
    Uint8_t i = 0;
    Robot_number = get_robot_id();
    for (i = 0; i <= 9; i++) {
        if (Robot_number == id_data.ID[i]) {
            Robot_cline_number = id_data.receiver_ID[i];
        }
    }
}

