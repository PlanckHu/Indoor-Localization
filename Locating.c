#include "Locating.h"
#include "usart.h"
/****************************
�����Ƿ��˸��ֿ������˲������ĸ��ֽṹ��
�õ�ʱ��ȡKp_X.result,Kp_Y.result,Kp_Z.result
��ʾX,Y,Z������
****************************/
Kalman_para Kp_A0;
Kalman_para Kp_A1;
Kalman_para Kp_A2;
Kalman_para Kp_A3;
Kalman_para Kp_X;
Kalman_para Kp_Y;
Kalman_para Kp_Z;
//�������ڴ�������
const double A0_A1 = 268;
const double A0_A2 = 268;
const double A0_A3 = 50;
double X;
double Y;
double Z;
////////////////////////////////
/*kalman�˲��Ĳ�����ʼ��*/
void Kalman_initialize(void)
{
	Kp_A0 = Kp_initialize(Kp_A0);
	Kp_A1 = Kp_initialize(Kp_A1);
	Kp_A2 = Kp_initialize(Kp_A2);
	Kp_A3 = Kp_initialize(Kp_A3);
	Kp_X = Kp_initialize(Kp_X);
	Kp_Y = Kp_initialize(Kp_Y);
	Kp_Z = Kp_initialize(Kp_Z);
}
Kalman_para Kp_initialize(Kalman_para Kp)
{
	Kp.kalman_p = 1;
	Kp.kalman_q = 0.000002;
	Kp.kalman_r = 0.00025;
	Kp.result = 0;
	return Kp;
}
/*���������ܹ���*/
void Process(u8* data, unsigned Data_num,unsigned int process_cnt)
{
	Origin_data* head = Get_mr_offset(data, Data_num);
	printf("Data_num=%d\r\n",Data_num);
	head = Data_transform(data, head);
	head = Filter(head);
	Display_origin(head);
	Axis_Calculating(process_cnt);
	Delete_all_nodes(head);
	Display_positioning();
}
/*������������*/
Origin_data* Fill_node(Origin_data* pt, unsigned long* side)
{
	pt->A0 = *(side + 0);
	pt->A1 = *(side + 1);
	pt->A2 = *(side + 2);
	pt->A3 = *(side + 3);
	return pt;
}
//�Ӷ�λϵͳ���յ��ַ���ת�������õ�����
Origin_data*  Data_transform(u8* data, Origin_data* head)
{
	unsigned long single_side_data[4] = { 0,0,0,0 };
	unsigned int side_cnt;
	unsigned long int trans;
	unsigned int single;
	unsigned int bit_cnt;
	Origin_data* present = head;
	Origin_data* former = head;
	while (present != NULL)
	{
		trans = 0;
		single = 0;
		for (side_cnt = 0; side_cnt < 4; side_cnt++)
		{
			for (bit_cnt = 0; bit_cnt < 8; bit_cnt++)
			{
				single = *(data + 6 + present->mr_offset  + 9 * side_cnt + bit_cnt);
				if (single >= 'a' && single <= 'f') single = single - 'a' + 10;
				else if (single >= '0' && single <= '9') single = single - '0';
				else { present->error = 1; break; }
				trans = trans * 16 + single;
			}
			single_side_data[side_cnt] = trans;
		}
		if (present->error == 1)
		{
			/*����������������д��������*/
			if (present == head)
			{
				free(present);
				present = present->next;
			}
			else
			{
				former = Delete_single_node(former, present);
				present = former->next;
			}
			/*****************************/
			continue;
		}
		else
		{
			present = Fill_node(present, single_side_data);
			former = present;
			present = former->next;
		}
	}
	return head;//���ش��ַ����з��������ԭʼ����,����Origin_data������
	//�����������˲�,Ȼ��ɾ����������
}
//ɾ������
Origin_data* Delete_all_nodes(Origin_data* head)
{	
	Origin_data* p1 = head;
	while (head != NULL)
	{
		p1 = head->next;
		free(head);
		head = p1;
	}
	return head;
	//head = NULL;
}
//ɾ���ýڵ�,������ǰһ���ڵ�
Origin_data* Delete_single_node(Origin_data* former, Origin_data* present)
{
	Origin_data* p_temp;
	if (present->next == NULL)
	{
		free(present);
		present = NULL;
		former->next = NULL;
	}
	else
	{
		p_temp = present->next;
		free(present);
		present = NULL;
		former->next = p_temp;
	}
	return former;
}
//Kalman�˲���,ÿ����վ����Kalman_para�ṹ
Kalman_para Kalman(double origin, Kalman_para KP)
{
	double gain;
	KP.kalman_p = KP.kalman_p + KP.kalman_q;
	gain = KP.kalman_p / (KP.kalman_p + KP.kalman_r);
	KP.result = KP.result + (gain*(origin - KP.result));
	KP.kalman_p = (1 - gain)*KP.kalman_p;
	return KP;
}
//Ӧ�ÿ������˲���
//�˲����tag��A0 A1 A2 A3�Ĳ������ݷ���Kp_A0...Kp_A3
Origin_data* Filter(Origin_data* head)
{
	Origin_data* p1 = head;
	while (p1 != NULL)
	{
		Kp_A0 = Kalman((double)p1->A0, Kp_A0);
		Kp_A1 = Kalman((double)p1->A1, Kp_A1);
		Kp_A2 = Kalman((double)p1->A2, Kp_A2);
		Kp_A3 = Kalman((double)p1->A3, Kp_A3);
		p1 = p1->next;
	}
	return head;
}
//����XYZ��������
//�����ô��˿������˲��������±궨һ��,���������
void Axis_Calculating(unsigned int process_cnt)
{
	static double X0=0;
	static double Y0=0;
	static double Z0=0;
	unsigned int cnt=process_cnt+1;
	double a = Kp_A0.result;
	double b = Kp_A1.result;
	double c = Kp_A2.result;
	double d = Kp_A3.result;
	double e = A0_A1;
	double f = A0_A2;
	double g = A0_A3;
	double x;
	double y;
	double z;
	a = (2 * 0.001f*a*0.001f*a) + 0.0796f*a - 23.837f;
	b = (2 * 0.0001f*b*0.001f*b + 0.0926f*b - 11.628f);
	c = (4 * 0.001f*c*0.001f*c + 0.0723f*c - 6.3944f);
	d = (2 * 0.001f*d*0.0001f*d + 0.091f*d - 10.176f);
	x = (a*a - b * b + e * e) / (2 * e);
	y = (a*a - c * c + f * f) / (2 * f);
	z = (a*a - d * d + g * g) / (2 * g);
	Kp_X = Kalman(x, Kp_X);
	Kp_Y = Kalman(y, Kp_Y);
	Kp_Z = Kalman(z, Kp_Z);
	if(process_cnt<100)
	{
		X0=Kp_X.result/cnt+(double)(cnt-1)/cnt*X0;
		Y0=Kp_Y.result/cnt+(double)(cnt-1)/cnt*Y0;
		Z0=Kp_Z.result/cnt+(double)(cnt-1)/cnt*Z0;
		X=X0;
		Y=Y0;
		Z=Z0;
	}
	else{
		printf("x0 = %f\r\n",X0);
		printf("y0 = %f\r\n",Y0);
		printf("z0 = %f\r\n",Z0);
		X=Kp_X.result-X0;
		Y=Kp_Y.result-Y0;
		Z=Kp_Z.result-Z0;
	}

}
/*����ԭʼ����,��mr��ƫ�����Ž�������*/
Origin_data* Get_mr_offset(u8* data, unsigned int Data_num)
{
	unsigned int dat_cnt = 0;
	unsigned int off_cnt = 0;
	Origin_data* head = (Origin_data*)malloc(sizeof(Origin_data));
	Origin_data* p1 = head;
	Origin_data* p2 = p1;
	while (dat_cnt < Data_num)
	{
		if (*(data + dat_cnt) == 'm' && *(data + dat_cnt + 1) == 'r')
		{
			p1 = p2;
			p1->mr_offset = dat_cnt;
			p1->error = 0;
			p2 = (Origin_data*)malloc(sizeof(Origin_data));
			p1->next = p2;
			off_cnt++;
		}
		dat_cnt++;
	}
	p1->next = NULL;
	free(p2);
	return head;
}
void Display_positioning(void)
{
	printf("A0=%f\r\n",Kp_A0.result);
	printf("A1=%f\r\n",Kp_A1.result);
	printf("A2=%f\r\n",Kp_A2.result);
	printf("A3=%f\r\n",Kp_A3.result);
	printf("X=%f\r\n",X);
	printf("Y=%f\r\n",Y);
	printf("Z=%f\r\n",Z);
	printf("ENDS\r\n");
}
void Display_test(Origin_data* head)
{
	while(head!=NULL)
	{
		printf("mr=%d, ",head->mr_offset);

		head=head->next;
	}
	printf("\r\n");
}
void Display_origin(Origin_data* head)
{
	while(head!=NULL)
	{
		printf("mr=%d, a0=%ld, a1=%ld, a2=%ld, a3=%ld\r\n",head->mr_offset,head->A0,
			head->A1,head->A2,head->A3);
		head=head->next;
	}
	printf("\r\n");
}