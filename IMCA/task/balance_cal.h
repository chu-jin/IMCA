#ifndef BALANCE_CAL_H
#define BALANCE_CAL_H
#include "main.h"
#include "pid.h"
/*
//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//               ���汣��         ����BUG
//
*/
/*because This task is hard ,so I had to use my power of Buddha*/


/**** �Ƿ���� ****/
#define BALANCE_PITCH_ANGLE -3.0
#define iq_const 8  //

extern pid_t BALANCE_ANGLE[1];//�Ƕ�
extern pid_t BALANCE_GYRO[1];//���ٶ�
extern pid_t OUT_INTERAL[1];//������ֻ�
extern pid_t BALANCE_SPEED[1];//�ٶȻ�
extern pid_t BALANCE_LOCATION[1];//�ٶȻ�
extern pid_t derivative[1];//����΢����
float ANGLE_Limit(float input, int32_t max_angle , int32_t min_angle );
void PARAM_Pid_init(void);
void chassis_balance_static(void);//���̾�̬ģʽ
#endif
