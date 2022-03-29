#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include "amazon2_sdk.h"
#include "graphic_api.h"
#include "hajima.h"

//////////////// robot /////////////////
#include <getopt.h>
#include "uart_api.h"
#include "robot_protocol.h"
#include <termios.h>

static struct termios inittio, newtio;
void init_console(void)
{
	tcgetattr(0, &inittio);
	newtio = inittio;
	newtio.c_lflag &= ~ICANON;
	newtio.c_lflag &= ~ECHO;
	newtio.c_lflag &= ~ISIG;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	cfsetispeed(&newtio, B115200);
	tcsetattr(0, TCSANOW, &newtio);
}
//////////////////////////////////////////

//ȭ�� ��� ���
#define B_GRAY_MODE 1
#define B_COLOR_MODE 3

#define C_YELLO 1
#define C_GATE 1
#define C_RED 2
#define C_BALL 3
#define C_GOAL 4
#define C_GREEN 5
#define C_BLK 6
#define C_BLUE 7

/////////// �� ���� HSV �� ////////////
//����
#define RED_HL 0   
#define RED_HH 50
#define RED_SL 0  
#define RED_SH 255      
#define RED_VL 0
#define RED_VH 255

//��Ȳ - ������
#define BALL_HL 0   
#define BALL_HH 50
#define BALL_SL 120
#define BALL_SH 255
#define BALL_VL 0
#define BALL_VH 255

/*
#define GOAL_HL   
#define GOAL_HH 70
#define GOAL_SL 50
#define GOAL_SH 255
#define GOAL_VL 40
#define GOAL_VH 255
*/
/*
//�� ����
#define GOAL_HL 0
#define GOAL_HH 255
#define GOAL_SL 0
#define GOAL_SH 80
#define GOAL_VL 0
#define GOAL_VH 150
*/

//�Ͼ�� ����
#define GOAL_HL 0
#define GOAL_HH 255
#define GOAL_SL 0
#define GOAL_SH 50
#define GOAL_VL 150
#define GOAL_VH 255


/*
#define GOAL_HL 0
#define GOAL_HH 50
#define GOAL_SL 120
#define GOAL_SH 255
#define GOAL_VL 0
#define GOAL_VH 255
*/
//���� ��



//���
#define YELLO_HL 50   
#define YELLO_HH 100
#define YELLO_SL 0 
#define YELLO_SH 255
#define YELLO_VL 50 
#define YELLO_VH 255

//��� - GATE
#define GATE_HL 50   
#define GATE_HH 100
#define GATE_SL 60  
#define GATE_SH 255
#define GATE_VL 50
#define GATE_VH 255

//�ʷ�	112
#define GREEN_HL 70      
#define GREEN_HH 150
#define GREEN_SL 50
#define GREEN_SH 255      
#define GREEN_VL 70		
#define GREEN_VH 255 

//�Ķ�
#define BLUE_HL 150
#define BLUE_HH 255
#define BLUE_SL 100
#define BLUE_SH 255     
#define BLUE_VL 0   
#define BLUE_VH 255
/*
//���� 5��
#define BLK_HL 0	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 170	//70
#define BLK_VL 0	//10
#define BLK_VH 60	
*/

//����	112�ٽ�
#define BLK_HL 10	
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 100	
#define BLK_VL 0	
#define BLK_VH 90	

/*
//5�� 2
#define BLK_HL 10	
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 140	
#define BLK_VL 0	
#define BLK_VH 90	
*/
//��
#define WHT_HL 0
#define WHT_HH 255
#define WHT_SL 0
#define WHT_SH 50 
#define WHT_VL 180
#define WHT_VH 255

///////////// �̼� ///////////////////
typedef enum {
	END = 0, M1, M2, M3, M4, M5, M6, M7
} MISSION;

void MISSION_1(int pix_s);
void MISSION_2(void);
void MISSION_4(void);
void MISSION_5R(int kick_mod);
void MISSION_6(void);
void MISSION_GATE(void);

//�ʿ��� ����� ������ ����Ͽ� ����� �������� �����ϴ� �Լ�
#define LP_HEAD_RIGHT 2
#define LP_HEAD_FRONT 3
#define LP_COLOR_BLACK 4
#define LP_COLOR_WHITE 5
#define JAL 10
#define DAECHUNG 20

//������ ���
#define KICK_LEFT 1
#define KICK_RIGHT 2


void COLOR_LINE_UP(int head_mode, int color_set, int level);

//�ʿ��� ���� �������� �߽����� �ȼ� ���� ��� �Լ�
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

void INFO_GATE_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, int* out_MX, int* out_MY	//OUPUT
	);

void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

//�߼������� ���� ������ ���� ã�� �Լ�
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode 	//OUPUT
	);

void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	);

int main(int argc, char **argv)
{
	////////////////// Robot Init /////////////////////
	init_console();
	int ret = uart_open();
	if (ret < 0) return EXIT_FAILURE;
	uart_config(UART1, 4800, 8, UART_PARNONE, 1);
	////////////////////////////////////////////////////

	BOOL b_loop = TRUE;

	if (open_graphic() < 0) return -1;
	if (direct_camera_display_stat() > 0)	direct_camera_display_off();

	clear_screen();
	flip();
	clear_screen();

	int JJ_NUM = 0;			//JJWALKING ������
	int flag_state = M1;	//�����ϴ� �̼� ����
	double cnt = 0;			//delay

	//while(1) COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

	//while(1)	MISSION_GATE();

	while (b_loop) {
		switch (flag_state) {

		case 1:		//�̼�1. ����-�ٸ�����Ʈ	- 30

			MISSION_1(40);	//����Ʈ�� ���� ��� �������� ����, ������ Ȯ�α���

			GO_NORM(8);

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			//MISSION_GATE();

			GO_NORM(8);

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, DAECHUNG);
			//COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_NORM(10);

			GO_JONGJONG(10);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M2;
			break;

		case 2:		//�̼�2. ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���

			MISSION_2();	//������ ���鼭 �¿� ���ı���

			JJ_NUM = 3;
			while (JJ_NUM--) { JJWALKING(); }

			TUCK();		//�ٸ������� ����

			HEAD_4NORM();

			GO_JONGJONG(10);

			JJ_NUM = 8;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M3;
			break;

		case 3:		//�̼�3. ��� ��ֹ�

			HEAD_4NORM();
			NORMAL_FORM();

			HUDDLE();			//��� ����	

			HEAD_4NORM();

			GO_BACK(4);

			BIG_TURN_LEFT();	//90�� ��		
			G_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);		//���������� �߽� ����

			GO_NORM(5);

			//MISSION_GATE();
			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_NORM(16);

			GO_JONGJONG(7);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M4;
			break;

		case 4:			//�̼�4. �ʷϻ� �ٸ��ǳʱ�

			MISSION_4();	//���������� �������� ���� �˻����

			DOWN_STAIR();

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_RIGHT();

			GO_NORM(11);

			flag_state = M5;
			break;

		case 5:			//�̼�5. ������ ����

			////////////////////// ������ //////////////////////////
			/*
			MISSION_5R(KICK_RIGHT); 	//��Ȳ�� �� �밭 ���� ====> ��� �� ��ġ Ž��

			HEAD_4NORM();
			NORMAL_FORM();

			KICK_BALL_R();	//������

			GO_NORM(4);		//5; @1012-16:03

			// ������
			BIG_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();

			*/
			////////////////////////////////////////////////

			/////////////////////  �޹� ///////////////////////////

			//while (1) {

			MISSION_5R(KICK_LEFT); 	//��Ȳ�� �� �밭 ���� ====> ��� �� ��ġ Ž��

			HEAD_4NORM();
			NORMAL_FORM();

			KICK_BALL_L();

			//}

			GO_NORM(4);		//5; @1012-16:03

			BIG_TURN_LEFT();
			BIG_TURN_LEFT();

			/////////////////////////////////////////////////

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_NORM(10);

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			//MISSION_GATE();
			//GO_NORM(10);
			//COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);	

			GO_NORM(15);
			
			GO_JONGJONG(20);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M6;
			break;

		case 6:			//�̼�6. ����� ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���

			MISSION_6();	//���� ���� ��ġ���� �̵�

			HEAD_4NORM();
			NORMAL_FORM();

			YELLO_TUCK();	//����

			HEAD_4NORM();

			flag_state = M7;
			break;

		case 7:		//�̼�7. ȸ��-�ٸ�����Ʈ

			//GO_BACK(15);
			//cnt = 400000;
			//while(cnt--){}
			//GO_NORM(8);	

			//G_TURN_RIGHT();
			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_NORM(12);

			//MISSION_GATE();

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, DAECHUNG);

			GO_NORM(8);

			MISSION_1(25);	//����� �ν� �� �� ���� ��ٸ��� - ī�޶� ��ĵ ����� �ν� - ī�޶� ����

			GO_NORM(20);

			flag_state = END;
			break;
		
		case 0:
			b_loop = FALSE;
			break;

		}//switch

	}//while

	close_graphic();
	uart_close();	//robot

	return 0;
}


//111111111111111111111111111111111111111111111111111111111111111111111111111111111
void MISSION_1(int pix_s)		//�̼�1. ����-�ٸ�����Ʈ, //�̼�7. ȸ��-�ٸ�����Ʈ						
{
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;
	int Y_base = pix_s;												

	double cnt = 400000;
	while(cnt--){}

	//����� ���� ã�� - ����Ʈ�� ���� ��� ��ٸ�
	while (Y_pix < Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GATE_HL, GATE_HH, GATE_SL, GATE_SH, GATE_VL, GATE_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	}//while

	cnt = 400000;
	while (cnt--) {}

	 //����� ���� ã�� - ����Ʈ�� ������� ��ٸ�
	while (Y_pix >= Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GATE_HL, GATE_HH, GATE_SL, GATE_SH, GATE_VL, GATE_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	}//while

}

//222222222222222222222222222222222222222222222222222222222222222222222222
void MISSION_2(void)		//�̼�2. ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���					
{
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int R_cx = 0, R_cy = 0, R_pix = 0;

	HEAD_4RED();
	double cnt = 200000;
	while (cnt--) {}

	while (1) {

		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,

			imgThresholded, labelOUT, MAXlabelOUT,
			RED_HL, RED_HH, RED_SL, RED_SH, RED_VL, RED_VH, //INPUT
			&R_cx, &R_cy, &R_pix	//OUPUT
			);

		int d = 1;
		int G_RD = 23;
		int G_LD = 22;

		//������ �߽� ���߱�
		//������
		if (R_cx > G_RD + 1 * d)	{	GO_RIGHT();		JJWALKING();		}
		else if (R_cx > G_RD)		{	GO_RIGHT_17();	JJWALKING();		}

		//����
		else if (R_cx < G_LD - 1 * d)	{	GO_LEFT();		JJWALKING();		}
		else if (R_cx < G_LD)			{	GO_LEFT_17();	JJWALKING();		}
		else { break; }

	}//while

}

//444444444444444444444444444444444444444444444444444444444444444444444444444444444
void MISSION_4(void)	//�̼�4. �ʷϻ� �ٸ��ǳʱ�
{
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int G_cx = 0, G_cy = 0, G_pix = 0;
	unsigned int tmp = 0;
	
	//�ʷϻ� ���� ã�� - ������ �� ��� ����
	
	HEAD_4GREEN();
	double cnt = 200000;
	while (cnt--) {}
	
	while (1) {

		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix	//OUPUT
			);


		//�ʷϻ� �߽� ���߱�
		int d = 1;
		int G_RD = 23;
		int G_LD = 22;

		//�ʷϻ� �߽� ���߱�
		if (G_cx > G_RD + 1 * d)	{	GO_RIGHT();		JJWALKING();	JJWALKING();
		}
		else if (G_cx > G_RD)		{	GO_RIGHT_17();	JJWALKING();	JJWALKING();
		}

		else if (G_cx < G_LD - 1 * d)	{	GO_LEFT();		JJWALKING();	JJWALKING();
		}
		else if (G_cx < G_LD)			{	GO_LEFT_17();	JJWALKING();	JJWALKING();
		}

		else { break; }

	}//while

	tmp = 4;
	while (tmp--) { JJWALKING(); }

	UP_STAIR();		//��� ������

	GO_GREEN(3);

	HEAD_4GREEN();
	cnt = 200000;
	while (cnt--) {}

	int num_run = 2;	//�ּ� �ʷϻ� �ٸ� ���� ��

	//�ʷϻ� ���� ã�� - �ʷϻ� �ٸ� ������ �߽� ã���� ����
	while (1) {

		cnt = 300000;
		while (cnt--) {}

		float mean = 0, var = 0;

		INFO_MEAN_VAR(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix, &mean, &var	//OUPUT
			);

		int cent = 24;
		if (num_run <= 0) {
			if (G_pix > 20) {

				if (var > 0.4) { //ȸ��
					if (mean < cent - 1.5)		{	TURN_LEFT();	}
					else if (mean > cent + 1.5) {	TURN_RIGHT();	}
					else	GO_GREEN(5);
				}
				else {	//��� ���߱�
					if (mean < cent - 1)	{	GO_LEFT_17();	}
					else if (mean>cent)		{	GO_RIGHT_17();	}
					else	{	GO_GREEN(5);	}
				}

			}	//G_pix
			else break; 
			
		}
		else {	
			if (var > 0.4) {	//ȸ��
				if (mean < cent - 1.5)		{	TURN_LEFT();	}
				else if (mean > cent + 1.5) {	TURN_RIGHT();	}
				else {
					num_run--;
					GO_GREEN(5);
				}
			}
			else {	//��� ���߱�
				if (mean < cent - 1)	{	GO_LEFT_17();	}
				else if (mean>cent)		{	GO_RIGHT_17();	}
				else {
					num_run--;
					GO_GREEN(5);
				}
			}

		}

	}//while

	//printf("\n##############GREEN END =========> BLACK ################\n");
	float G = 0;
	int B_cx = 0, B_cy = 0, B_pix = 0;
	float M = 0;

	HEAD_4BLACK();
	cnt = 200000;
	while (cnt--) {}

	//������ �� ã��
	while (1) {
		cnt = 120000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BLK_HL, BLK_HH, BLK_SL, BLK_SH, BLK_VL, BLK_VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, LP_HEAD_FRONT	//OUPUT
			);

		M = atan2((float)G, 1) * 180 / 3.14159265;

		if (M > 2)  TURN_LEFT();
		else if (M < -2)	TURN_RIGHT();
		else {
			if (B_cy < 7)		{	GO_GREEN(6);	break;	}
			else if (B_cy < 13) {	GO_GREEN(5);	break;	}
			else	{	GO_GREEN(4);	break;	}
		}

	}//while

	cnt = 600000;
	while (cnt--) {}

}

//555555555555555555555555555555555555555555555555555555555555555555555555555555555
void MISSION_5R(int kick_mode)	//�̼�5. ������ ����
{
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int BALL_cx = 0, BALL_cy = 0, BALL_pix = 0;
	int GOAL_cx = 0, GOAL_cy = 0, GOAL_pix = 0;
	int flag = 0;

	//int no_find = 0;

	HEAD_4GREEN();
	double cnt = 400000;
	while (cnt--) {}

	//������ �ν��ؼ� ������ ����
	while (1) {	

		cnt = 400000;
		while (cnt--) {}
		
		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);
	
		//�Ӹ� ���� ����

		if (flag == 1 && BALL_cy == 0 && BALL_cx == 0) {	break;	}

		if (BALL_cy == 0 && BALL_cx == 0 && flag == 0) {	GO_NORM(2);		flag = 1;	}
		//else if (BALL_cy < 16) {	GO_GREEN(3);	}
		else if (BALL_cy < 20) {	GO_GREEN(2);	}
		else if (BALL_cy < 24) {	GO_GREEN(1);	}
		//else {

			if (BALL_cx > 26)		{	GO_RIGHT();		}
			else if (BALL_cx > 24)	{	GO_RIGHT_17();	}

			else if (BALL_cx < 18)	{	GO_LEFT();	}
			else if (BALL_cx < 20) { GO_LEFT_17(); }
			else if(BALL_cy >= 24 && (BALL_cx <=24 && BALL_cx >= 20)){	break;	}

		//}

	}//while

	 printf("=================> MORE \n");

	 HEAD_4BLACK();
	 cnt = 400000;
	 while (cnt--) {}

	 //////////////////////////// RIGHT ///////////////////////////////////


	while (1) {	//���� �� ������ �Ӹ� �� ������

		HEAD_4BLACK();
		cnt = 400000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);

		printf("BALL ::: cx : %d, cy : %d, pixel : %d \n", BALL_cx, BALL_cy, BALL_pix);

		if (BALL_cx == 0 && BALL_cy == 0 && BALL_pix == 0)	break;

		HEAD_4GREEN();
		cnt = 400000;
		while (cnt--) {}

		INFO_GOAL_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GOAL_HL, GOAL_HH, GOAL_SL, GOAL_SH, GOAL_VL, GOAL_VH, //INPUT
			&GOAL_cx, &GOAL_cy, &GOAL_pix	//OUPUT
			);

		printf("GOAL ::: cx : %d, cy : %d, pixel : %d \n", GOAL_cx, GOAL_cy, GOAL_pix);

		int R = 29;
		int L = 29;
		int goal = 14;
		int goal_d = 2;

		if (kick_mode == KICK_LEFT) {
			R = 19;	//17
			L = 19;
			goal_d = -3;
		}

		if (GOAL_cx + goal_d <= BALL_cx + 1 && GOAL_cx + goal_d >= BALL_cx - 1)	//������ ����
			if (BALL_cy < goal && BALL_cx <= R && BALL_cx >= L) { break; }

		//ȸ��
		if ((GOAL_cx + goal_d > BALL_cx + 1) && (GOAL_pix != 0)) {
			if (GOAL_cx + goal_d > BALL_cx + 10)	TURN_RIGHT();
			if (GOAL_cx + goal_d> BALL_cx + 5)		TURN_RIGHT();
			if (GOAL_cx + goal_d > BALL_cx + 3)		TURN_RIGHT();
			TURN_RIGHT();


			if (BALL_cx > 25) {	GO_RIGHT();	}
			else if (BALL_cx <15) {	GO_LEFT();}

		}
		else if ((GOAL_cx + goal_d < BALL_cx - 1) && (GOAL_pix != 0)) {
			if (GOAL_cx + goal_d < BALL_cx - 10)	TURN_LEFT();
			if (GOAL_cx + goal_d < BALL_cx - 5)		TURN_LEFT();
			if (GOAL_cx + goal_d < BALL_cx - 3)		TURN_LEFT();
			TURN_LEFT();


			if (BALL_cx > 25) { GO_RIGHT();	}
			else if (BALL_cx <15) { GO_LEFT();	}
		}
		else {

			if ((BALL_cy >= goal || BALL_cx > 32 || BALL_cx < 12) && (GOAL_pix != 0)) {

				int d = 1;

				if (BALL_cx > R) {
					if (BALL_cx > R + 3 * d)	GO_RIGHT();
					if (BALL_cx > R + 1 * d) 	GO_RIGHT_17();
					GO_RIGHT_17();

				}

				else if (BALL_cx < L) {
					if (BALL_cx < L - 3 * d)	GO_LEFT();
					if (BALL_cx < L - 1 * d)	GO_LEFT_17();
					GO_LEFT_17();

				}
				else break;
			}
			else {		//������ ����
				int d = 1;
				if (BALL_cy < goal - d * 2)	JJWALKING();
				if (BALL_cy < goal - d * 3)	JJWALKING();
				if (BALL_cy < goal - d * 4)	JJWALKING();
				if (BALL_cy < goal - d * 5)	JJWALKING();
				JJWALKING();
			}
		}
	}//while

	
}

//6666666666666666666666666666666666666666666666666666666666666666666666666666666666
void MISSION_6(void)	//�̼�6. ����� ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���
{
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;
	unsigned int tmp = 0;

	HEAD_4RED();
	double cnt = 200000;
	while (cnt--) {}

	//�ö� ���¿��� ���� ������ ��ġ�� �̵�
	while (1) {
		
		INFO_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	
		int d = 1;
		int G_RD = 22;
		int G_LD = 21;

		if (Y_cx > G_RD + 1 * d)	{	GO_RIGHT();		JJWALKING();		}
		else if (Y_cx > G_RD)		{	GO_RIGHT_17();	JJWALKING();		}

		else if (Y_cx < G_LD - 1 * d)	{	GO_LEFT();		JJWALKING();		}
		else if (Y_cx < G_LD)			{	GO_LEFT_17();	JJWALKING();		}

		else	{	break;	}	//���� ����

	}//while
	
	tmp = 4;
	while (tmp--) { JJWALKING(); }

	UP_STAIR();

	GO_GREEN(3);

	float G = 0;
	float M = 0;
	
	HEAD_4BLACK();
	cnt = 200000;
	while (cnt--) {}


	//���������� ã��
	while (1) {

		cnt = 500000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix, &G, LP_HEAD_FRONT	//OUPUT
			);

		M = atan2((float)G, 1) * 180 / 3.14159265;

		if (M > 4)		{	TURN_LEFT();	}
		else if (M< -4) {	TURN_RIGHT();	}
		else {

			if (Y_cx < 22)		{	GO_LEFT_17();	}
			else if (Y_cx > 23) {	GO_RIGHT_17();	}
			else {

				int d = 1;
				if (Y_cy >= 13) { break; }	
				else {
					JJWALKING();
					if (Y_cy < 13 - 1 * d)	JJWALKING();
					if (Y_cy < 13 - 2 * d)	JJWALKING();
					if (Y_cy < 13 - 3 * d)	JJWALKING();
				}

			}

		}//ȸ��
	
	}//while
	
}

//BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB
void MISSION_GATE(void) {

	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int BG_cx = 0, BG_cy = 0, BG_pix=0, BG_MX = 0, BG_MY=0;

	HEAD_4BLUE();
	double cnt = 400000;
	while (cnt--) {}

	while (1) {

		cnt = 200000;
		while (cnt--) {}

		INFO_GATE_CHECK(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BLUE_HL, BLUE_HH, BLUE_SL, BLUE_SH, BLUE_VL, BLUE_VH, //INPUT
			&BG_cx, &BG_cy, &BG_pix, &BG_MX, &BG_MY	//OUPUT
			);
		
		if(BG_MY == 0||(BG_cx==0&& BG_cy==0&& BG_pix == 0) )	break;
		if (BG_MY > 27) break;
		else if (BG_MY < 15)		{	GO_GREEN(3);	}

		else if (BG_MY < 19){	

			if (BG_MX < 19) {	GO_LEFT();		}
			else if (BG_MX > 29) {	GO_RIGHT(); }
			else GO_GREEN(2);	

		}
		else {

			if (BG_MX < 19) {	GO_LEFT();	}
			else if (BG_MX > 29) {	GO_RIGHT();	}
			else {	break;	}

		}

	}//while

	HEAD_4NORM();

}

//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void COLOR_LINE_UP(int mode, int color_set, int level) {	//������ ���� ���� ���� �Լ�
	U16* fpga_src = (U16*)malloc(180 * 120 * 2); //FPGA Original image
	U16* fpga_dst = (U16*)malloc(180 * 120 * 2);
	U8* Filt = (U8*)malloc(180 * 120);

	U8* imgBGR = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgVSH = (U8*)malloc(WIDTH * HEIGHT * 3);
	U8* imgThresholded_H = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_S = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded_V = (U8*)malloc(WIDTH * HEIGHT);
	U8* imgThresholded = (U8*)malloc(WIDTH * HEIGHT);
	U8* labelOUT = (U8*)malloc(HEIGHT * WIDTH);
	U8* MAXlabelOUT = (U8*)malloc(HEIGHT * WIDTH);

	int B_cx = 0, B_cy = 0, B_pix = 0;
	float G = 0;
	int  HL = 0, HH = 0, SL = 0, SH = 0, VL = 0, VH = 0;

	HL = BLK_HL; HH = BLK_HH;  SL = BLK_SL; SH = BLK_SH; VL = BLK_VL; VH = BLK_VH;
	
	double cnt = 0;
	float M = 0;

	ARM_BACK(mode);
	NORMAL_FORM();

	while (1) {

		cnt = 100000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			HL, HH, SL, SH, VL, VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, mode //OUPUT
			);

		M = atan2((float)G, 1) * 180 / 3.14159265;

		float T_RD = -12.2;
		float T_LD = -8.6;

		if (level == DAECHUNG){

			T_RD = -12;
			T_LD = -8;

			int Ld = 6;
			int Rd = 7;
			if (M < T_RD) {	//ȸ�� ��
				if (M < T_RD - 1 * Rd) {
					TURN_RIGHT(); cnt = 100000;
					while (cnt--) {}
				}
				else if (M < T_RD) {	TURN_RIGHT();
					cnt = 100000;
					while (cnt--) {}
				}
			}
			else if (M > T_LD) {
				if (M > T_LD + 1 * Ld) {
					TURN_LEFT(); 
					cnt = 100000;
					while (cnt--) {}
				}
				else if (M > T_LD) {	TURN_LEFT();
					cnt = 100000;
					while (cnt--) {}
				}
			}
			else {	break;	}

		}
		else if (level == JAL){

			T_RD = -12;
			T_LD = -8;

			int G_LD = 16;//22;	
			int G_RD = 13;//19;	
			int Ld = 6;
			int Rd = 7;
			printf("M = %f \n", M);

			draw_rectfill(B_cx, B_cy, 5, 5, MAKE_COLORREF(255, 0, 0));

			if (M < T_RD) {	//ȸ�� ��
				if (M < T_RD - 1 * Rd) {
					TURN_RIGHT(); cnt = 100000;
					while (cnt--) {}
				}
				else if (M < T_RD) {
					TURN_RIGHT();
					cnt = 100000;
					while (cnt--) {}
				}
			}
			else if (M > T_LD) {
				if (M > T_LD + 1 * Ld) {
					TURN_LEFT();
					cnt = 100000;
					while (cnt--) {}
				}
				else if (M > T_LD) {
					TURN_LEFT();
					cnt = 100000;
					while (cnt--) {}
				}
			}
			else{
				if (B_cy > G_LD) {
					if (B_cy > G_LD + 1)	{	GO_LEFT();		}
					else if (B_cy > G_LD)	{	GO_LEFT_17();	}
				}
				else if (B_cy < G_RD){
					if (B_cy < G_RD - 1)	{	GO_RIGHT();		}
					else if (B_cy < G_RD)	{	GO_RIGHT_17();	}
				}
				else break;
			}
		}

	}//while

	HEAD_4NORM();

}


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

//�ʿ��� ���� �������� �߽����� �ȼ� ���� ���  �Լ�
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	read_fpga_video_data(fpga_src);
	draw_fpga_video_data_full(fpga_src);

	RGB565_to_BGR888(fpga_src, imgBGR, 180, 120);	//printf("RGB565_to_BGR888\n");
	BGR888_to_VSH888_30(imgBGR, imgVSH, WIDTH, HEIGHT);		//printf("BGR888_to_VSH888_30\n");
	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, WIDTH, HEIGHT, HL, HH);	
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, WIDTH, HEIGHT, SL, SH);	
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, WIDTH, HEIGHT, VL, VH);	//printf("VSH888_to_H8_thresh\n");
	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);	//printf("HSV_to_Binary8\n");

	int C_mode = 0;
	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {
		C_mode = C_YELLO;	//yello
	}
	else if (HL == GATE_HL && HH == GATE_HH && SL == GATE_SL && SH == GATE_SH && VL == GATE_VL && VH == GATE_VH) {
		C_mode = C_GATE;	//GATE
	}
	else if (HL == RED_HL && HH == RED_HH && SL == RED_SL && SH == RED_SH && VL == RED_VL && VH == RED_VH) {
		C_mode = C_RED;		//red
	}
	else if (HL == BALL_HL && HH == BALL_HH && SL == BALL_SL && SH == BALL_SH && VL == BALL_VL && VH == BALL_VH) {
		C_mode = C_BALL;	//BALL
	}
	else if (HL == GOAL_HL && HH == GOAL_HH && SL == GOAL_SL && SH == GOAL_SH && VL == GOAL_VL && VH == GOAL_VH) {
		C_mode = C_GOAL;	//GOAL
	}
	else if (HL == GREEN_HL && HH == GREEN_HH && SL == GREEN_SL && SH == GREEN_SH && VL == GREEN_VL && VH == GREEN_VH) {
		C_mode = C_GREEN;	//GREEN
	}
	else if (HL == BLK_HL && HH == BLK_HH && SL == BLK_SL && SH == BLK_SH && VL == BLK_VL && VH == BLK_VH) {
		C_mode = C_BLK;		//BLACK
	}
	else if (HL == BLUE_HL && HH == BLUE_HH && SL == BLUE_SL && SH == BLUE_SH && VL == BLUE_VL && VH == BLUE_VH) {
		C_mode = C_BLUE;	//BLUE
	}

	//printf("draw_rectfill2\n");
	//draw_rectfill2(imgThresholded, Filt, C_mode);	//180*120 -> 30*20

	int i, j;
	//�ٿ���� ����ó��
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//���� �׵θ�
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//���� �׵θ�

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < HEIGHT - 1; j++) {
		for (i = 1; i < WIDTH - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + WIDTH * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�

						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //�ȼ��� ������ ����, � ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {
					label++;
					imgThresholded[i + WIDTH * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < HEIGHT; j++)
		for (i = 0; i< WIDTH; i++) {
			labelOUT[i + WIDTH * j] = 0;
			MAXlabelOUT[i + WIDTH * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (imgThresholded[i + WIDTH * j] != 0) {
				idx = imgThresholded[i + WIDTH * j];
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]�� 255�̻��̸� 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
	}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j] != 0) {
				LABEL[labelOUT[i + WIDTH * j]].pix++;
				LABEL[labelOUT[i + WIDTH * j]].sumX += i;
				LABEL[labelOUT[i + WIDTH * j]].sumY += j;

				if (i > LABEL[labelOUT[i + WIDTH * j]].maxX) LABEL[labelOUT[i + WIDTH * j]].maxX = i;
				if (i < LABEL[labelOUT[i + WIDTH * j]].minX) LABEL[labelOUT[i + WIDTH * j]].minX = i;
				if (j > LABEL[labelOUT[i + WIDTH * j]].maxY) LABEL[labelOUT[i + WIDTH * j]].maxY = j;
				if (j < LABEL[labelOUT[i + WIDTH * j]].minY) LABEL[labelOUT[i + WIDTH * j]].minY = j;
			}
		}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j]>0 && LABEL[labelOUT[i + WIDTH * j]].pix > 0) {
				LABEL[labelOUT[i + WIDTH * j]].cX = LABEL[labelOUT[i + WIDTH * j]].sumX / LABEL[labelOUT[i + WIDTH * j]].pix;
				LABEL[labelOUT[i + WIDTH * j]].cY = LABEL[labelOUT[i + WIDTH * j]].sumY / LABEL[labelOUT[i + WIDTH * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 6 && LABEL[i].pix > MAXLABEL.pix) {		//LABEL[i].pix > 100
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}
	//printf("MAXLABELOUT\n");
	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc)		{	MAXlabelOUT[i + WIDTH * j] = 0;		}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0)	{	MAXlabelOUT[i + WIDTH * j] = 255;	}
		}

	//printf("draw_rectfill2\n");
	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}


//�ʿ��� ���� �������� �߽����� �ȼ� ���� ���  �Լ�
void INFO_GATE_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, int* out_MX, int* out_MY	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	read_fpga_video_data(fpga_src);
	draw_fpga_video_data_full(fpga_src);

	RGB565_to_BGR888(fpga_src, imgBGR, 180, 120);		//printf("RGB565_to_BGR888\n");
	BGR888_to_VSH888_30(imgBGR, imgVSH, WIDTH, HEIGHT);	//printf("BGR888_to_VSH888_30\n");
	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, WIDTH, HEIGHT, HL, HH);	
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, WIDTH, HEIGHT, SL, SH);	
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, WIDTH, HEIGHT, VL, VH);	//printf("VSH888_to_H8_thresh\n");
	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);	//printf("HSV_to_Binary8\n");

	int C_mode = 0;
	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {
		C_mode = C_YELLO;	//yello
	}
	else if (HL == GATE_HL && HH == GATE_HH && SL == GATE_SL && SH == GATE_SH && VL == GATE_VL && VH == GATE_VH) {
		C_mode = C_GATE;	//GATE
	}
	else if (HL == RED_HL && HH == RED_HH && SL == RED_SL && SH == RED_SH && VL == RED_VL && VH == RED_VH) {
		C_mode = C_RED;		//red
	}
	else if (HL == BALL_HL && HH == BALL_HH && SL == BALL_SL && SH == BALL_SH && VL == BALL_VL && VH == BALL_VH) {
		C_mode = C_BALL;	//BALL
	}
	else if (HL == GOAL_HL && HH == GOAL_HH && SL == GOAL_SL && SH == GOAL_SH && VL == GOAL_VL && VH == GOAL_VH) {
		C_mode = C_GOAL;	//GOAL
	}
	else if (HL == GREEN_HL && HH == GREEN_HH && SL == GREEN_SL && SH == GREEN_SH && VL == GREEN_VL && VH == GREEN_VH) {
		C_mode = C_GREEN;	//GREEN
	}
	else if (HL == BLK_HL && HH == BLK_HH && SL == BLK_SL && SH == BLK_SH && VL == BLK_VL && VH == BLK_VH) {
		C_mode = C_BLK;		//BLACK
	}
	else if (HL == BLUE_HL && HH == BLUE_HH && SL == BLUE_SL && SH == BLUE_SH && VL == BLUE_VL && VH == BLUE_VH) {
		C_mode = C_BLUE;	//BLUE
	}

	//printf("draw_rectfill2\n");
	//draw_rectfill2(imgThresholded, Filt, C_mode);	//180*120 -> 30*20

	int i, j;	//�ٿ���� ����ó��
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//���� �׵θ�
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; 	}	//���� �׵θ�

	//for (j = 0; j < 15; j++)
		//for (i = 0; i < WIDTH; i++) { imgThresholded[i + WIDTH * j] = 0; }	//���� �׵θ�



	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < HEIGHT - 1; j++) {
		for (i = 1; i < WIDTH - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + WIDTH * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //�ȼ��� ������ ����, � ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {
					label++;
					imgThresholded[i + WIDTH * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < HEIGHT; j++)
		for (i = 0; i< WIDTH; i++) {
			labelOUT[i + WIDTH * j] = 0;
			MAXlabelOUT[i + WIDTH * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (imgThresholded[i + WIDTH * j] != 0) {
				idx = imgThresholded[i + WIDTH * j];
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1]; 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
		LABEL[m].maxX = 0;
		LABEL[m].maxY= 0;
	}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j] != 0) {
				LABEL[labelOUT[i + WIDTH * j]].pix++;
				LABEL[labelOUT[i + WIDTH * j]].sumX += i;
				LABEL[labelOUT[i + WIDTH * j]].sumY += j;

				if (i > LABEL[labelOUT[i + WIDTH * j]].maxX) LABEL[labelOUT[i + WIDTH * j]].maxX = i;
				if (i < LABEL[labelOUT[i + WIDTH * j]].minX) LABEL[labelOUT[i + WIDTH * j]].minX = i;
				if (j > LABEL[labelOUT[i + WIDTH * j]].maxY) LABEL[labelOUT[i + WIDTH * j]].maxY = j;
				if (j < LABEL[labelOUT[i + WIDTH * j]].minY) LABEL[labelOUT[i + WIDTH * j]].minY = j;
			}
		}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j]>0 && LABEL[labelOUT[i + WIDTH * j]].pix > 0) {
				LABEL[labelOUT[i + WIDTH * j]].cX = LABEL[labelOUT[i + WIDTH * j]].sumX / LABEL[labelOUT[i + WIDTH * j]].pix;
				LABEL[labelOUT[i + WIDTH * j]].cY = LABEL[labelOUT[i + WIDTH * j]].sumY / LABEL[labelOUT[i + WIDTH * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 6 && LABEL[i].pix > MAXLABEL.pix) {	
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXLABEL.maxX = LABEL[i].maxX;
			MAXLABEL.maxY = LABEL[i].maxY;
			MAXlabelc = i;
		}
	}
	//printf("MAXLABELOUT\n");
	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc) {
				MAXlabelOUT[i + WIDTH * j] = 0;
			}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + WIDTH * j] = 255;
			}
		}


	//printf("draw_rectfill2\n");
	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;
	*out_MX = MAXLABEL.maxX;
	*out_MY = MAXLABEL.maxY;

	printf("cx : %d , cy : %d, pix : %d, MX : %d, MY : %d \n", *out_cX, *out_cY, *out_pix, *out_MX, *out_MY);
}

void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	read_fpga_video_data(fpga_src);
	draw_fpga_video_data_full(fpga_src);

	RGB565_to_BGR888(fpga_src, imgBGR, 180, 120);	//565 -> 888
	BGR888_to_VSH888_30(imgBGR, imgVSH, WIDTH, HEIGHT);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, WIDTH, HEIGHT, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, WIDTH, HEIGHT, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, WIDTH, HEIGHT, VL, VH);	//V

	//HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);
	//HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);

	HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);


	int C_mode = 0;
	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {
		C_mode = C_YELLO;	//yello
	}
	else if (HL == GATE_HL && HH == GATE_HH && SL == GATE_SL && SH == GATE_SH && VL == GATE_VL && VH == GATE_VH) {
		C_mode = C_GATE;	//GATE
	}
	else if (HL == RED_HL && HH == RED_HH && SL == RED_SL && SH == RED_SH && VL == RED_VL && VH == RED_VH) {
		C_mode = C_RED;	//red
	}
	else if (HL == BALL_HL && HH == BALL_HH && SL == BALL_SL && SH == BALL_SH && VL == BALL_VL && VH == BALL_VH) {
		C_mode = C_BALL;	//BALL
	}
	else if (HL == GOAL_HL && HH == GOAL_HH && SL == GOAL_SL && SH == GOAL_SH && VL == GOAL_VL && VH == GOAL_VH) {
		C_mode = C_GOAL;	//GOAL
	}
	else if (HL == GREEN_HL && HH == GREEN_HH && SL == GREEN_SL && SH == GREEN_SH && VL == GREEN_VL && VH == GREEN_VH) {
		C_mode = C_GREEN;	//GREEN
	}
	else if (HL == BLK_HL && HH == BLK_HH && SL == BLK_SL && SH == BLK_SH && VL == BLK_VL && VH == BLK_VH) {
		C_mode = C_BLK;	//BLACK
	}
	else if (HL == BLUE_HL && HH == BLUE_HH && SL == BLUE_SL && SH == BLUE_SH && VL == BLUE_VL && VH == BLUE_VH) {
		C_mode = C_BLUE;	//BLUE
	}

	//draw_rectfill2(imgThresholded, Filt, C_mode);	//180*120 -> 30*20

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0; }	//���� �׵θ�
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//���� �׵θ�

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < HEIGHT - 1; j++) {
		for (i = 1; i < WIDTH - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��	
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + WIDTH * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //�ȼ��� ������ ����, � ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {
					label++;
					imgThresholded[i + WIDTH * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < HEIGHT; j++)
		for (i = 0; i< WIDTH; i++) {
			labelOUT[i + WIDTH * j] = 0;
			MAXlabelOUT[i + WIDTH * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (imgThresholded[i + WIDTH * j] != 0) {
				idx = imgThresholded[i + WIDTH * j];
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1]; 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
	}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j] != 0) {
				LABEL[labelOUT[i + WIDTH * j]].pix++;
				LABEL[labelOUT[i + WIDTH * j]].sumX += i;
				LABEL[labelOUT[i + WIDTH * j]].sumY += j;

				if (i > LABEL[labelOUT[i + WIDTH * j]].maxX) LABEL[labelOUT[i + WIDTH * j]].maxX = i;
				if (i < LABEL[labelOUT[i + WIDTH * j]].minX) LABEL[labelOUT[i + WIDTH * j]].minX = i;
				if (j > LABEL[labelOUT[i + WIDTH * j]].maxY) LABEL[labelOUT[i + WIDTH * j]].maxY = j;
				if (j < LABEL[labelOUT[i + WIDTH * j]].minY) LABEL[labelOUT[i + WIDTH * j]].minY = j;
			}
		}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j]>0 && LABEL[labelOUT[i + WIDTH * j]].pix > 0) {
				LABEL[labelOUT[i + WIDTH * j]].cX = LABEL[labelOUT[i + WIDTH * j]].sumX / LABEL[labelOUT[i + WIDTH * j]].pix;
				LABEL[labelOUT[i + WIDTH * j]].cY = LABEL[labelOUT[i + WIDTH * j]].sumY / LABEL[labelOUT[i + WIDTH * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {	//70
		if (LABEL[i].cY < 18 && LABEL[i].pix > 3 && LABEL[i].pix < 35 && LABEL[i].cY> MAXLABEL.cY) {	 
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}

	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc) {
				MAXlabelOUT[i + WIDTH * j] = 0;
			}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + WIDTH * j] = 255;
			}
		}

	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	//printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}

//�߼������� ���� ������ ���� ã�� �Լ�
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode  //OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	read_fpga_video_data(fpga_src);
	draw_fpga_video_data_full(fpga_src);

	RGB565_to_BGR888(fpga_src, imgBGR, 180, 120);	//565 -> 888
	BGR888_to_VSH888_30(imgBGR, imgVSH, WIDTH, HEIGHT);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, WIDTH, HEIGHT, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, WIDTH, HEIGHT, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, WIDTH, HEIGHT, VL, VH);	//V

	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH)
		HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);
	else
		HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);


	int C_mode = 0;
	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {
		C_mode = C_YELLO;	//yello
	}
	else if (HL == GATE_HL && HH == GATE_HH && SL == GATE_SL && SH == GATE_SH && VL == GATE_VL && VH == GATE_VH) {
		C_mode = C_GATE;	//GATE
	}
	else if (HL == RED_HL && HH == RED_HH && SL == RED_SL && SH == RED_SH && VL == RED_VL && VH == RED_VH) {
		C_mode = C_RED;	//red
	}
	else if (HL == BALL_HL && HH == BALL_HH && SL == BALL_SL && SH == BALL_SH && VL == BALL_VL && VH == BALL_VH) {
		C_mode = C_BALL;	//BALL
	}
	else if (HL == GOAL_HL && HH == GOAL_HH && SL == GOAL_SL && SH == GOAL_SH && VL == GOAL_VL && VH == GOAL_VH) {
		C_mode = C_GOAL;	//GOAL
	}
	else if (HL == GREEN_HL && HH == GREEN_HH && SL == GREEN_SL && SH == GREEN_SH && VL == GREEN_VL && VH == GREEN_VH) {
		C_mode = C_GREEN;	//GREEN
	}
	else if (HL == BLK_HL && HH == BLK_HH && SL == BLK_SL && SH == BLK_SH && VL == BLK_VL && VH == BLK_VH) {
		C_mode = C_BLK;	//BLACK
	}
	else if (HL == BLUE_HL && HH == BLUE_HH && SL == BLUE_SL && SH == BLUE_SH && VL == BLUE_VL && VH == BLUE_VH) {
		C_mode = C_BLUE;	//BLUE
	}

	//draw_rectfill2(imgThresholded, Filt, C_mode);	//180*120 -> 30*20

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//���� �׵θ�
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0;	}	//���� �׵θ�

	//�� or �츦 �� ��� �� �ڸ���
	if (mode == LP_HEAD_RIGHT) {
		for (j = 0; j < HEIGHT; j++) 
			for (i = 22; i < WIDTH; i++) { imgThresholded[i + WIDTH * j] = 0; }
	}
	else if (mode == LP_HEAD_FRONT) {
		for (j = HEIGHT-10; j < HEIGHT; j++) {	//�� �ڸ���
			for (i = 0; i < WIDTH; i++)	imgThresholded[i + WIDTH * j] = 0;
		}
		for (j = 0; j < HEIGHT; j++) {
			for (i = 0; i < 4; i++)	imgThresholded[i + WIDTH * j] = 0;
			for (i = WIDTH - 4; i < WIDTH; i++)	imgThresholded[i + WIDTH * j] = 0;
		}
	}

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < HEIGHT - 1; j++) {
		for (i = 1; i < WIDTH - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + WIDTH * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //�ȼ��� ������ ����, � ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {
					label++;
					imgThresholded[i + WIDTH * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < HEIGHT; j++)
		for (i = 0; i< WIDTH; i++) {
			labelOUT[i + WIDTH * j] = 0;
			MAXlabelOUT[i + WIDTH * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (imgThresholded[i + WIDTH * j] != 0) {
				idx = imgThresholded[i + WIDTH * j];
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1]; 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
		LABEL[m].minX = 255;
		LABEL[m].maxX = 0;
		LABEL[m].minY = 255;
		LABEL[m].maxY = 0;
	}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j] != 0) {
				LABEL[labelOUT[i + WIDTH * j]].pix++;
				LABEL[labelOUT[i + WIDTH * j]].sumX += i;
				LABEL[labelOUT[i + WIDTH * j]].sumY += j;

				if (i > LABEL[labelOUT[i + WIDTH * j]].maxX) LABEL[labelOUT[i + WIDTH * j]].maxX = i;
				if (i < LABEL[labelOUT[i + WIDTH * j]].minX) LABEL[labelOUT[i + WIDTH * j]].minX = i;
				if (j > LABEL[labelOUT[i + WIDTH * j]].maxY) LABEL[labelOUT[i + WIDTH * j]].maxY = j;
				if (j < LABEL[labelOUT[i + WIDTH * j]].minY) LABEL[labelOUT[i + WIDTH * j]].minY = j;
			}
		}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j]>0 && LABEL[labelOUT[i + WIDTH * j]].pix > 0) {
				LABEL[labelOUT[i + WIDTH * j]].cX = LABEL[labelOUT[i + WIDTH * j]].sumX / LABEL[labelOUT[i + WIDTH * j]].pix;
				LABEL[labelOUT[i + WIDTH * j]].cY = LABEL[labelOUT[i + WIDTH * j]].sumY / LABEL[labelOUT[i + WIDTH * j]].pix;
			}
		}

	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {		///////////////////////////////// �̼� 6
		for (i = 1; i < m_label + 1; i++) {
			if (LABEL[i].pix > 10 && LABEL[i].pix > MAXLABEL.pix) {	
				MAXLABEL.pix = LABEL[i].pix;
				MAXLABEL.cX = LABEL[i].cX;
				MAXLABEL.cY = LABEL[i].cY;
				MAXlabelc = i;
				MAXLABEL.maxX = LABEL[i].maxX;
				MAXLABEL.minX = LABEL[i].minX;
				MAXLABEL.maxY = LABEL[i].maxY;
				MAXLABEL.minY = LABEL[i].minY;
			}
		}
	}
	else {
		for (i = 1; i < m_label + 1; i++) {																				///////////////////////////////// ������
			if (LABEL[i].pix > 10 && LABEL[i].cY > MAXLABEL.cY) {
				MAXLABEL.pix = LABEL[i].pix;
				MAXLABEL.cX = LABEL[i].cX;
				MAXLABEL.cY = LABEL[i].cY;
				MAXlabelc = i;
				MAXLABEL.maxX = LABEL[i].maxX;
				MAXLABEL.minX = LABEL[i].minX;
				MAXLABEL.maxY = LABEL[i].maxY;
				MAXLABEL.minY = LABEL[i].minY;
			}
		}
	}

	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc)		{	MAXlabelOUT[i + WIDTH * j] = 0;	}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0)	{	MAXlabelOUT[i + WIDTH * j] = 255;	}
		}

	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20

	//�߼��� �׸��� 6��
	if (HH == BLK_HH && HL == BLK_HL  && SH == BLK_SH && SL == BLK_SL  && VH == BLK_VH && VL == BLK_VL) {	/////////////////////////////////////////// �������� ���
																									//�߼��� ���� ���ϱ�
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + WIDTH*j] == 255 && cnt < 1&& j>((MAXLABEL.maxY + MAXLABEL.minY)/2)) {
					e = e + i*i;
					r = r + i;
					sX = sX + i;
					q = q + i * (120 - j);
					numb++;
					sY = sY + (120 - j);
					cnt++;
				}
			}
		}

		r = r * r;
		e = e * numb;
		q = q * numb;
		w = sY * sX;
		*out_cX = sX / numb;
		*out_cY = (120 - sY / numb);
		*out_pix = MAXLABEL.pix;
		*G = (q - w) / (e - r);
	}

	else if (HH == YELLO_HH && HL == YELLO_HL  && SH == YELLO_SH && SL == YELLO_SL  && VH == YELLO_VH && VL == YELLO_VL) {		//////////////////////// �̼� 6
		//�߼��� ���� ���ϱ�
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + WIDTH*j] == 255 && cnt < 6) {
					e = e + i*i;
					r = r + i;
					sX = sX + i;
					q = q + i * (120 - j);
					numb++;
					sY = sY + (120 - j);
					cnt++;
				}
			}
		}

		r = r * r;
		e = e * numb;
		q = q * numb;
		w = sY * sX;
		*out_cX = sX / numb;
		*out_cY = (120 - sY / numb);
		*out_pix = MAXLABEL.pix;
		*G = (q - w) / (e - r);
	}

	float M = atan2((float)*G, 1) * 180 / 3.14159265;
	float gi = *G, b = 0;
	int cx = *out_cX, cy = *out_cY, y = 0;

	b = gi * cx + cy;

	for (i = 0; i < 180; i++) {
	y = -1 * gi * i + b;
	if (y > 0 && y < 120)
		draw_rectfill(160 -2.66*y , 2.66*i , 2, 2, MAKE_COLORREF(0, 0, 255));
	}
	
	printf("cx : %d, cy : %d , pix : %d, G : %f, ATAN : %f \n", *out_cX ,*out_cY, *out_pix , *G , M);

	flip();

}

//��հ��� ǥ������ ���� ã�� �Լ�
void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	read_fpga_video_data(fpga_src);
	draw_fpga_video_data_full(fpga_src);

	RGB565_to_BGR888(fpga_src, imgBGR, 180, 120);	//565 -> 888
	BGR888_to_VSH888_30(imgBGR, imgVSH, WIDTH, HEIGHT);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, WIDTH, HEIGHT, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, WIDTH, HEIGHT, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, WIDTH, HEIGHT, VL, VH);	//V
	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, WIDTH, HEIGHT);

	int C_mode = 0;
	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {
		C_mode = C_YELLO;	//yello
	}
	else if (HL == GATE_HL && HH == GATE_HH && SL == GATE_SL && SH == GATE_SH && VL == GATE_VL && VH == GATE_VH) {
		C_mode = C_GATE;	//GATE
	}
	else if (HL == RED_HL && HH == RED_HH && SL == RED_SL && SH == RED_SH && VL == RED_VL && VH == RED_VH) {
		C_mode = C_RED;	//red
	}
	else if (HL == BALL_HL && HH == BALL_HH && SL == BALL_SL && SH == BALL_SH && VL == BALL_VL && VH == BALL_VH) {
		C_mode = C_BALL;	//BALL
	}
	else if (HL == GOAL_HL && HH == GOAL_HH && SL == GOAL_SL && SH == GOAL_SH && VL == GOAL_VL && VH == GOAL_VH) {
		C_mode = C_GOAL;	//GOAL
	}
	else if (HL == GREEN_HL && HH == GREEN_HH && SL == GREEN_SL && SH == GREEN_SH && VL == GREEN_VL && VH == GREEN_VH) {
		C_mode = C_GREEN;	//GREEN
	}
	else if (HL == BLK_HL && HH == BLK_HH && SL == BLK_SL && SH == BLK_SH && VL == BLK_VL && VH == BLK_VH) {
		C_mode = C_BLK;	//BLACK
	}
	else if (HL == BLUE_HL && HH == BLUE_HH && SL == BLUE_SL && SH == BLUE_SH && VL == BLUE_VL && VH == BLUE_VH) {
		C_mode = C_BLUE;	//BLUE
	}

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//���� �׵θ�
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//���� �׵θ�

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < HEIGHT - 1; j++) {
		for (i = 1; i < WIDTH - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + WIDTH * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //�ȼ��� ������ ����, � ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {
					label++;
					imgThresholded[i + WIDTH * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < HEIGHT; j++)
		for (i = 0; i< WIDTH; i++) {
			labelOUT[i + WIDTH * j] = 0;
			MAXlabelOUT[i + WIDTH * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (imgThresholded[i + WIDTH * j] != 0) {
				idx = imgThresholded[i + WIDTH * j];
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1];  
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
		LABEL[m].minX = 255;
		LABEL[m].maxX = 0;
		LABEL[m].minY = 255;
		LABEL[m].maxY = 0;
	}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j] != 0) {
				LABEL[labelOUT[i + WIDTH * j]].pix++;
				LABEL[labelOUT[i + WIDTH * j]].sumX += i;
				LABEL[labelOUT[i + WIDTH * j]].sumY += j;

				if (i > LABEL[labelOUT[i + WIDTH * j]].maxX) LABEL[labelOUT[i + WIDTH * j]].maxX = i;
				if (i < LABEL[labelOUT[i + WIDTH * j]].minX) LABEL[labelOUT[i + WIDTH * j]].minX = i;
				if (j > LABEL[labelOUT[i + WIDTH * j]].maxY) LABEL[labelOUT[i + WIDTH * j]].maxY = j;
				if (j < LABEL[labelOUT[i + WIDTH * j]].minY) LABEL[labelOUT[i + WIDTH * j]].minY = j;
			}
		}

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (labelOUT[i + WIDTH * j]>0 && LABEL[labelOUT[i + WIDTH * j]].pix > 0) {
				LABEL[labelOUT[i + WIDTH * j]].cX = LABEL[labelOUT[i + WIDTH * j]].sumX / LABEL[labelOUT[i + WIDTH * j]].pix;
				LABEL[labelOUT[i + WIDTH * j]].cY = LABEL[labelOUT[i + WIDTH * j]].sumY / LABEL[labelOUT[i + WIDTH * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 0 && LABEL[i].pix > MAXLABEL.pix) {
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
			MAXLABEL.maxX = LABEL[i].maxX;
			MAXLABEL.minX = LABEL[i].minX;
			MAXLABEL.maxY = LABEL[i].maxY;
			MAXLABEL.minY = LABEL[i].minY;
		}
	}

	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc)		{	MAXlabelOUT[i + WIDTH * j] = 0;	}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0)	{	MAXlabelOUT[i + WIDTH * j] = 255;	}
		}

	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20
	printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);

	//��� ǥ������ ���ϱ�
	int tp_minY = MAXLABEL.minY; 
	int tp_maxY = MAXLABEL.maxY;

	int MidPoint[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MidPoint[i] = 0;
	int MPcnt[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MPcnt[i] = 0;

	int tp = 0;
	for (j = tp_minY; j <= tp_maxY; j++) {
		tp = j - tp_minY;
		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			if (MAXlabelOUT[i + WIDTH*j] == 255) {
				MidPoint[tp] = MidPoint[tp] + i;
				MPcnt[tp]++;
			}
		}
		MidPoint[tp] = MidPoint[tp] / MPcnt[tp];
	}

	int n = sizeof(MidPoint) / sizeof(int);
	float sum = 0, var = 0;
	for (i = 0; i<n; i = i + 1)
		sum += MidPoint[i];
	*mean = sum / n;		//���

	sum = 0;
	for (i = 0; i < n; i = i + 1)	{	sum += (MidPoint[i] - *mean)*(MidPoint[i] - *mean);		}
	var = sum / (n - 1);
	*std_dev = var;		//�л�

	printf("MEAN = %8.3f	VAR = %8.3f	STAN DEV = %8.3f\n", *mean,var, *std_dev);

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

}