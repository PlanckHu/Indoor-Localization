#ifndef __LOCATING_H
#define __LOCATING_H

#include <stdlib.h>
#include "sys.h"

typedef struct origin_data 
{
	int error;
	unsigned int mr_offset;
	unsigned long A0;
	unsigned long A1;
	unsigned long A2;
	unsigned long A3;
	struct origin_data* next;
}Origin_data;

typedef struct kalman_para
{
	double kalman_p;
	double kalman_q;
	double kalman_r;
	double result;
}Kalman_para;

/**************************/
void Kalman_initialize(void);
Kalman_para Kp_initialize(Kalman_para Kp);
void Process(u8* data, unsigned Data_num,unsigned int process_cnt);//调用这个函数就ok,参数是接收的数据数组和这个数据的大小
Origin_data* Fill_node(Origin_data* pt, unsigned long* side);//ok
Origin_data* Data_transform(u8* data, Origin_data* head);//ok
Origin_data* Delete_all_nodes(Origin_data* head);//OK
Origin_data* Delete_single_node(Origin_data* former,Origin_data* present);//return former//OK
Kalman_para Kalman(double origin, Kalman_para KP);//OK
Origin_data* Filter(Origin_data* head);//OK
void Axis_Calculating(unsigned int process_cnt);//OK
Origin_data* Get_mr_offset(u8* data, unsigned int Data_num);//ok
void Display_positioning(void);//显示定位结果

void Display_test(Origin_data* head);
void Display_origin(Origin_data* head);
/************************/
#endif
