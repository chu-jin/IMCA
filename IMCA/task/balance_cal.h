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
//               佛祖保佑         永无BUG
//
*/
/*because This task is hard ,so I had to use my power of Buddha*/


/**** 是否编译 ****/
#define BALANCE_PITCH_ANGLE -3.0
#define iq_const 8  //

extern pid_t BALANCE_ANGLE[1];//角度
extern pid_t BALANCE_GYRO[1];//角速度
extern pid_t OUT_INTERAL[1];//输出积分环
extern pid_t BALANCE_SPEED[1];//速度环
extern pid_t BALANCE_LOCATION[1];//速度环
extern pid_t derivative[1];//测试微分项
float ANGLE_Limit(float input, int32_t max_angle , int32_t min_angle );
void PARAM_Pid_init(void);
void chassis_balance_static(void);//底盘静态模式
#endif
