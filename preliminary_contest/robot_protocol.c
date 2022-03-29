/************************************************************************
  Title     : Robot Body Protocol Source File
  File name : robot_protocol.c    

  Author    : adc inc. (oxyang@adc.co.kr)
  History
		+ v0.0  2007/2/14
		+ v1.0  2008/8/6
************************************************************************/
#include <stdio.h>
#include <string.h>
#include "robot_protocol.h"
#include "uart_api.h"
//////////////////////////////////////////////////// Protocol Test

void DelayLoop(double delay_time)
{
	while(delay_time)
		delay_time--;
}

void Send_Command(unsigned char command)
{
	int i = 0;
	unsigned char Command_Buffer[1] = {0,};

	Command_Buffer[0] = command;	// Command Byte
	 //	for(i=0; i<1; i++)	printf("0x%x ",Command_Buffer[i]);

	uart1_buffer_write(Command_Buffer, 1);
}

void Receive_Command(void)
{
	int i= 0;
	unsigned char Command_Buffer[1] = { 0, };

	uart1_buffer_read(Command_Buffer, 1);

	//for (i = 0; i < 1; i++)	printf("%d\n ", Command_Buffer[i]);	
}

void Receive_Command_For_Walk(int cnt)
{
	int i = 0;
	unsigned char Command_Buffer[1] = { 0, };
	while (1) {
		uart1_buffer_read(Command_Buffer, 1);
		//for (i = 0; i < 1; i++)	printf("walk1 : %d   buffer val : %d \n ", cnt, Command_Buffer[i]);
		cnt--;	
		if (cnt == 0) {
			uart1_buffer_write(Command_Buffer, 1);
			uart1_buffer_read(Command_Buffer, 1);
			//for (i = 0; i < 1; i++)	printf("STOP : %d   buffer val : %d \n ", cnt, Command_Buffer[i]);
			break;
		}
	}
}


#define ERROR 0
#define OK	1

#define THREE 1000000

/* Command Function */
void GO_GREEN(int cnt)				//�ʷϻ� �ٸ��� ����
{
	Send_Command(61);		
	////printf("\nGO_GREEN!\n");
	Receive_Command_For_Walk(cnt);
}

void GO_NORM(int cnt)				//��Һ���
{
	Send_Command(62);
	//printf("\nGO_NORM!\n");
	Receive_Command_For_Walk(cnt);
}

void GO_RIGHT(void)				//���� ���̵�
{
	Send_Command(63);
	//printf("\nGO_RIGHT!\n");
	Receive_Command();
}

void GO_LEFT(void)				//���� ���̵�
{
	Send_Command(64);
	//printf("\nGO_LEFT!\n");
	Receive_Command();
}

void TUCK(void)				//���ִ� �� ������
{
	Send_Command(65);
	//printf("\nTUCK!\n");
	Receive_Command();
}


void UP_STAIR(void)				//��ܿ�����
{
	Send_Command(67);		
	//printf("\nUP_STAIR!\n");
	Receive_Command();
}

void KICK_BALL(void)				//������
{
	Send_Command(68);
	//printf("\nKICK_BALL!\n");
	Receive_Command();
}

void TURN_RIGHT(void)				//������ ��
{
	Send_Command(69);
	//printf("\nTURN_RIGHT!\n");
	Receive_Command();
}

void HUDDLE(void)				//��ֹ� �ѱ�
{
	Send_Command(70);
	//printf("\nHUDDLE!\n");
	Receive_Command();
}


void BIG_TURN_LEFT(void)				//ũ�� ���� ��
{
	Send_Command(72);
	//printf("\nBIG_TURN_LEFT!\n");
	Receive_Command();
}

void JJWALKING(void)				//���� ����
{
	Send_Command(73);
	//printf("\nJJWALKING!\n");
	Receive_Command();
}

void BIG_TURN_RIGHT(void)				//ũ�� ������ ��
{
	Send_Command(74);
	//printf("\nBIG_TURN_RIGHT!\n");
	Receive_Command();
}

void TURN_LEFT(void)				//���� ��
{
	Send_Command(75);
	//printf("\nTURN_LEFT!\n");
	Receive_Command();
}

void G_TURN_LEFT(void)				//���� ��
{
	Send_Command(78);
	//printf("\nTURN_LEFT!\n");
	Receive_Command();
}
void G_TURN_RIGHT(void)				//���� ��
{
	Send_Command(80);
	//printf("\nTURN_LEFT!\n");
	Receive_Command();
}
void GO_JONGJONG(int cnt)				//���Ŀ� ����
{
	Send_Command(76);
	//printf("\nGO_JONGJONG!\n");
	Receive_Command_For_Walk(cnt);
}

void DOWN_STAIR(void)				//��� ��������
{
	Send_Command(77);
	//printf("\nDOWN_STAIR!\n");
	Receive_Command();
}

void HEAD_4RED(void)				//�Ӹ� ������
{
	Send_Command(83);
	//printf("\nHEAD_DOWN!\n");
	Receive_Command();
}

void NORMAL_FORM(void)				//�⺻�ڼ�
{
	Send_Command(79);
	//printf("\nNORMAL_FORM!\n");
	Receive_Command();
}

void HEAD_4GREEN(void)				//�Ӹ� �ø���
{
	Send_Command(84);
	//printf("\nHEAD_UP!\n");
	Receive_Command();
}
void HEAD_4BLACK(void)				//�Ӹ� �ø���
{
	Send_Command(50);
	//printf("\nHEAD_UP!\n");
	Receive_Command();
}
void YELLO_TUCK(void)				//��� ����
{
	Send_Command(82);
	//printf("\nYELLO_TUCK!\n");
	Receive_Command();
}

void ARM_NORM(void)				//�� �⺻
{
	Send_Command(83);
	//printf("\nARM_NORM!\n");
	Receive_Command();
}

void ARM_BACK(int mode)				//�� �ڷ�
{
	if (mode == LP_HEAD_LEFT) {
		Send_Command(54);
		//printf("\n ARM_BACK LEFT! \n");
		Receive_Command();
	}
	else if (mode == LP_HEAD_RIGHT) {
		Send_Command(55);
		//printf("\n ARM_BACK RIGHT! \n");
		Receive_Command();
	}
	else if(mode == LP_HEAD_FRONT){	// ���� �߰�?

	}
	
}
void GO_RIGHT_5(void)
{
	Send_Command(56);
	//printf("\nGO_RIGHT_5mm!\n");
	Receive_Command();
}
void GO_LEFT_5(void)
{
	Send_Command(57);
	//printf("\nGO_LEFT_5mm!\n");
	Receive_Command();
}
void GO_RIGHT_17(void)
{
	Send_Command(58);
	//printf("\nGO_RIGHT_17mm!\n");
	Receive_Command();
}
void GO_LEFT_17(void)
{
	Send_Command(59);
	//printf("\nGO_LEFT_17mm!\n");
	Receive_Command();
}

