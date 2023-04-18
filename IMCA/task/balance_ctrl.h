#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H
#include "main.h"
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
/*because This task is hard ,so i had to use my power of god*/

#define PITCH_MID_MINUS  0.0f //pitch轴自平衡角度
#define PITCH_MID_MAX_MINI -12.0f //pitch轴最小角度 
#define PITCH_MID_MAX_POST 12.0f //pitch轴最大角度

//struct{
//	  float x_speed; //前进速度  
//    float x_potion; //前进位置    
//}chassis_state;
extern int16_t Iq_Left;
extern int16_t Iq_Right;
extern int16_t Iq_Left_gm;
extern int16_t Iq_Right_gm;
void BALANCE_Contrl(void);

#endif
