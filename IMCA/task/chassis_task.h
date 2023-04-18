#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"

extern uint8_t Chassis_Ctrl;

typedef struct{
	float speed_move;
	float location_move;
}chassis_status_t ;

extern chassis_status_t chassis_status;
void chassis_task(void);
#endif
