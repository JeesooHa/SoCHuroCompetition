////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
/////MDF �� �ٽ� ���߱�, �ָ������� �׸��� ���ݸ� ������ �Ⱥ���
/////line find����


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
//ȭ��ũ��
#define m_nWidth 180
#define m_nHeight 120

//ȭ�� ��� ���
#define B_GRAY_MODE 1
#define B_COLOR_MODE 3

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

//�� ����
#define GOAL_HL 0   
#define GOAL_HH 70
#define GOAL_SL 50
#define GOAL_SH 255
#define GOAL_VL 40
#define GOAL_VH 255

//���
#define YELLO_HL 50   
#define YELLO_HH 100
#define YELLO_SL 0//50  //0  
#define YELLO_SH 255
#define YELLO_VL 50//100 //120   
#define YELLO_VH 255

//��� - GATE
#define GATE_HL 50   
#define GATE_HH 100
#define GATE_SL 60  //0  
#define GATE_SH 255
#define GATE_VL 50//100 //120   
#define GATE_VH 255

/*//�ʷ�	5��
#define GREEN_HL 60      
#define GREEN_HH 150
#define GREEN_SL 0//0  
#define GREEN_SH 255      
#define GREEN_VL 70		//100		// 0
#define GREEN_VH 255 
*/
//�ʷ�	112
#define GREEN_HL 70      
#define GREEN_HH 150
#define GREEN_SL 50//0  
#define GREEN_SH 255      
#define GREEN_VL 70		//100		// 0
#define GREEN_VH 255 

//�Ķ�
#define BLUE_HL 150
#define BLUE_HH 255
#define BLUE_SL 100
#define BLUE_SH 255     
#define BLUE_VL 0   
#define BLUE_VH 255

/*//����	5��
#define BLK_HL 0	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 170	//70
#define BLK_VL 0	//10
#define BLK_VH 60	//90	//70
*/
/*
//����	112���� �˾Ҵµ� �ʷϻ�����
#define BLK_HL 10	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 150	//70
#define BLK_VL 0	//10
#define BLK_VH 100	//90	//70
*/
//����	112�ٽ�
#define BLK_HL 10	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 100	//70
#define BLK_VL 0	//10
#define BLK_VH 90	//90	//70


//��
#define WHT_HL 0
#define WHT_HH 255
#define WHT_SL 0
#define WHT_SH 50 //70
#define WHT_VL 180 //100
#define WHT_VH 255

///////////// �̼� ///////////////////
typedef enum {
	END = 0, M1, M2, M3, M4, M5, M6, M7,
	M8,	//�Ķ�����Ʈ 1
	M9  //�Ķ�����Ʈ 2
} MISSION;


void MISSION_1(void);
void MISSION_2(void);
void MISSION_4(void);
void MISSION_5R(void);
void MISSION_6(void);

//�ʿ��� ����� ������ ����Ͽ� ����� �������� �����ϴ� �Լ�
#define LP_HEAD_RIGHT 2
#define LP_HEAD_FRONT 3
#define LP_COLOR_BLACK 4
#define LP_COLOR_WHITE 5
#define JAL 10
#define DAECHUNG 20

void COLOR_LINE_UP(int head_mode, int color_set, int level);
void COLOR_LINE_UP2(int head_mode, int color_set, int level);

//�ʿ��� ���� �������� �߽����� �ȼ� ���� ��� �Լ�
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

//�߼������� ���� ������ ���� ã�� �Լ�
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode 	//OUPUT
	);

void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	);

int main(int argc, char **argv)
{
	////////////////// Robot Init /////////////////////
	int ret;
	init_console();
	ret = uart_open();
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
	double cnt;
	while (b_loop) {
		switch (flag_state) {
	
		case 1:												//�̼�1. ����-�ٸ�����Ʈ

			MISSION_1(); //����Ʈ�� ���� ��� �������� ����, ������ Ȯ�α���
			
			GO_GREEN(11);	//GO_GREEN(10);
			GO_JONGJONG(4);
	
			JJ_NUM = 10;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M2;
			break;

		case 2:				//�̼�2. ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���

			MISSION_2();	//������ ���鼭 �¿� ���ı���
			
			JJ_NUM = 3;
			while (JJ_NUM--) { JJWALKING(); }

			TUCK();	//�ٸ������� ����

			GO_GREEN(3);
			GO_JONGJONG(4);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M3;
			break;

		case 3:								//�̼�3. ��� ��ֹ�

			NORMAL_FORM();
		//	cnt = 500000;
		// while (cnt--) {}

			HUDDLE();			//��� ����		

			GO_GREEN(4);
								
			BIG_TURN_LEFT();	//90�� ��		
			BIG_TURN_LEFT();
			//BIG_TURN_LEFT();

			GO_GREEN(5);

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);		//���������� �߽� ����

			GO_GREEN(13);	//GO_GREEN(16);
			GO_JONGJONG(11);	//GO_JONGJONG(8);

			flag_state = M4;
			break;

		case 4:				//�̼�4. �ʷϻ� �ٸ��ǳʱ�
			MISSION_4();	//���������� �������� ���� �˻����

			GO_GREEN(4);	//������ ��ġ�� �̵�
			//GO_JONGJONG(1);
			//JJWALKING(); //JJWALKING(); JJWALKING();
			//G_TURN_RIGHT();
			DOWN_STAIR(); 

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);

			GO_GREEN(7);

			flag_state = M5;
			break;

		case 5:			//�̼�5. ������ ����

		    MISSION_5R(); 	//��Ȳ�� �� �밭 ���� ====> ��� �� ��ġ Ž�� ====> ű����

			BIG_TURN_LEFT();	//90��
			BIG_TURN_LEFT();
			//BIG_TURN_LEFT();
		
			COLOR_LINE_UP2(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);
			
			GO_GREEN(7);	

			COLOR_LINE_UP2(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);

			GO_GREEN(14);
			
			GO_JONGJONG(11);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M6;
			break;

		case 6:			//�̼�6. ����� ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���

			MISSION_6();	//���� ���� ��ġ���� �̵�, �⺻�ڼ�����

			YELLO_TUCK();	//����

			//COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,DAECHUNG);

			flag_state = M7;
			break;

		case 7:	//�̼�7. ȸ��-�ٸ�����Ʈ
			GO_GREEN(6);

			MISSION_1();	//����� �ν� �� �� ���� ��ٸ��� - ī�޶� ��ĵ ����� �ν� - ī�޶� ����
		
			GO_GREEN(10);

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
void MISSION_1(void)		//�̼�1. ����-�ٸ�����Ʈ, //�̼�7. ȸ��-�ٸ�����Ʈ						
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;
	int Y_base = 500;									//���谪

	//����� ���� ã�� - ����Ʈ�� ���� ��� ��ٸ�
	while (Y_pix < Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GATE_HL, GATE_HH, GATE_SL, GATE_SH, GATE_VL, GATE_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	}//while

	//����� ���� ã�� - ����Ʈ�� ������� ��ٸ�
	while (Y_pix >= Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
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
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int R_cx = 0, R_cy = 0, R_pix = 0;

	while (1) {

		HEAD_4RED();
		double cnt = 200000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			RED_HL, RED_HH, RED_SL, RED_SH, RED_VL, RED_VH, //INPUT
			&R_cx, &R_cy, &R_pix	//OUPUT
			);	

		int d = 2;		
		int G_RD = 93;
		int G_LD = 87;

		//������ �߽� ���߱�
		if (R_cx > G_RD) {			
						
			if (R_cx > G_RD + 9 * d) GO_RIGHT();
			if (R_cx > G_RD + 8 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 6 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 5 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 4 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 2 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING(); }
			GO_RIGHT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else if (R_cx < G_LD) {
			
			if (R_cx < G_LD - 9 * d) GO_LEFT();
			if (R_cx < G_LD - 8 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 6 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 5 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 4 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 2 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING(); }
			GO_LEFT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else { break;}
	}//while
	
}

//444444444444444444444444444444444444444444444444444444444444444444444444444444444
void MISSION_4(void)	//�̼�4. �ʷϻ� �ٸ��ǳʱ�
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int G_cx = 0, G_cy = 0, G_pix = 0;
	int tmp = 0;
	
	//�ʷϻ� ���� ã�� - ������ �� ��� ����
	while (1) {

		HEAD_4GREEN();
		double cnt = 250000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix	//OUPUT
			);
	
		//�ʷϻ� �߽� ���߱�
		int d = 2;
		int G_RD = 93;
		int G_LD = 87;

		//�ʷϻ� �߽� ���߱�
		if (G_cx > G_RD) {

			if (G_cx > G_RD + 9 * d) GO_RIGHT();
			if (G_cx > G_RD + 8 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 6 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 5 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 4 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 2 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING(); }
			GO_RIGHT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else if (G_cx < G_LD) {

			if (G_cx < G_LD - 9 * d) GO_LEFT();
			if (G_cx < G_LD - 8 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 6 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 5 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 4 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 2 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING(); }
			GO_LEFT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else { break; }
		
		/*
		int d = 3;
		int G_RD = 96;
		int G_LD = 86;
		
		if (G_cx > G_RD) {
			if (G_cx > G_RD + 12 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 11 * d) { GO_RIGHT(); JJWALKING(); JJWALKING(); }
			if (G_cx > G_RD + 10 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 9 * d) { GO_RIGHT();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 8 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 7 * d) { GO_RIGHT();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 6 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 5 * d) { GO_RIGHT_17();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 4 * d) { GO_RIGHT_17(); }
			if (G_cx > G_RD + 3 * d) { GO_RIGHT_17();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 2 * d) { GO_RIGHT_17(); }
			if (G_cx > G_RD + d) { GO_RIGHT_5();	JJWALKING(); JJWALKING(); }
			GO_RIGHT_5();
			tmp = 4;
			while (tmp--) { JJWALKING(); }
		}
		else if (G_cx < G_LD) {
			if (G_cx < G_LD - 12 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 11 * d) { GO_LEFT(); JJWALKING(); JJWALKING(); }
			if (G_cx < G_LD - 10 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 9 * d) { GO_LEFT(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 8 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 6 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 5 * d) { GO_LEFT_17(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 4 * d) { GO_LEFT_17(); }
			if (G_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 2 * d) { GO_LEFT_17();  }
			if (G_cx < G_LD - 1 * d) { GO_LEFT_5();  JJWALKING(); JJWALKING(); }
			GO_LEFT_5();
			tmp = 4;
			while (tmp--) { JJWALKING(); }
		}
		else {	break;	}
		*/
	}//while


	UP_STAIR();	//��� ������
	GO_GREEN(3);
	

	int num_run = 2;	//�ּ� �ʷϻ� �ٸ� ���� ��

	//�ʷϻ� ���� ã�� - �ʷϻ� �ٸ� ������ �߽� ã���� ����
	while (1) {	

		HEAD_4GREEN();
		double cnt = 250000;
		while (cnt--) {}

		float mean = 0, std_dev = 0;

		INFO_MEAN_VAR(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix, &mean, &std_dev	//OUPUT
			);
	
		/////104 ~ 94 ~ 84
		////89-4	   82-4
		float d = 7;// 4;// 5;
		if (num_run <= 0){
			if (G_pix > 500) {
				if(std_dev > 2){ //4
					if (mean < 89 - 1.5){
						G_TURN_LEFT();
						if (mean < 89 - 15) G_TURN_LEFT();
					}
					else if (mean > 89 + 1.5){
						G_TURN_RIGHT(); 
						if (mean > 89 + 15) G_TURN_RIGHT(); 
					}	
				}
		
				if (mean < 89 - 4) {
					GO_LEFT_17();
					if (mean < 89 - 5 - 1 * d) GO_LEFT_17();
					if (mean < 89 - 5 - 2 * (d + 1)) GO_LEFT_17();
				}
				else if (mean>89 + 4) {
					GO_RIGHT_17();
					if (mean > 89 + 5 + 1 * d) GO_RIGHT_17();
					if (mean > 89 + 5 + 2 * (d + 1)) GO_RIGHT_17();
				}
				else {
					if (G_cy > 90)	GO_GREEN(4);
					else	GO_GREEN(5);
				}
			}
			else break;
		}
		else {
			if (std_dev > 2) {
				if (mean < 89 - 1.5) {
					G_TURN_LEFT();
					if (mean < 89 - 15) G_TURN_LEFT();
				}
				else if (mean > 89 + 1.5) {
					G_TURN_RIGHT();
					if (mean > 89 + 15) G_TURN_RIGHT();
				}
				else JJWALKING();
			}
			
			
			if (mean < 89 - 4) {
				GO_LEFT_17();
				if (mean < 89 - 5 - 1 * d) GO_LEFT_17();
				if (mean < 89 - 5 - 2 * (d + 1)) GO_LEFT_17();
			}
			else if (mean>89 + 4) {
				GO_RIGHT_17();
				if (mean > 89 + 5 + 1 * d) GO_RIGHT_17();
				if (mean > 89 + 5 + 2 * (d + 1)) GO_RIGHT_17();
			}
			else {
				num_run--;
				GO_GREEN(5);
			}
			
		}


		/*
		if (num_run <= 0){
			if (G_pix > 500) {
				if(std_dev > 2){ //4
					if (mean < 89 - 1.5){
						G_TURN_LEFT();
						if (mean < 89 - 15) G_TURN_LEFT();
					}
					else if (mean > 89 + 1.5){
						G_TURN_RIGHT(); 
						if (mean > 89 + 15) G_TURN_RIGHT(); 
					}	
				}
				else {
					if (mean < 89 - 4.5) {
						GO_LEFT_17();
						if (mean < 84.5 - 1 * d) GO_LEFT_17();
						if (mean < 84.5 - 2 * d) GO_LEFT_17();
					}
					else if (mean>89 + 4.5) {
						GO_RIGHT_17();
						if (mean > 93.5 + 1 * d) GO_RIGHT_17();
						if (mean > 93.5 + 2 * d) GO_RIGHT_17();
					}
					else {
						if (G_cy > 90)	GO_GREEN(4);
						else	GO_GREEN(5);
					}
				}
			
			}else break;
		}
		else {
			if (std_dev > 2) {	
				if (mean < 89 - 1.5){
					G_TURN_LEFT();
					if (mean < 89 - 15) G_TURN_LEFT(); 
				}
				else if (mean > 89 + 1.5){
					G_TURN_RIGHT(); 
					if (mean > 89 + 15) G_TURN_RIGHT(); 
				}
				else JJWALKING();
			}
			else {
				if (mean < 89 - 4.5) {
					GO_LEFT_17();
					if(mean < 84.5 - 1 * d ) GO_LEFT_17();
					if(mean < 84.5 - 2 * d) GO_LEFT_17();
				}
				else if (mean>89 + 4.5){
					GO_RIGHT_17();
					if (mean > 93.5 + 1 * d) GO_RIGHT_17();
					if (mean > 93.5 + 2 * d) GO_RIGHT_17();
				}
				else {
					num_run--;
					GO_GREEN(5);
				}
			}
		}*/

	}//while


	//printf("\n##############GREEN END =========> BLACK ################\n");
	float G = 0;
	int B_cx = 0, B_cy = 0, B_pix = 0;

	//������ �� ã��
	while (1) {	

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BLK_HL, BLK_HH, BLK_SL, BLK_SH, BLK_VL, BLK_VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, LP_HEAD_FRONT	//OUPUT
			);
	
		double M = atan2((double)G, 1) * 180 / 3.14159265;
			
		if (M > 2)  G_TURN_LEFT(); 
		else if (M < -2)	G_TURN_RIGHT(); 
		else break;
	}//while

}

//555555555555555555555555555555555555555555555555555555555555555555555555555555555
void MISSION_5R(void)	//�̼�5. ������ ����
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int BALL_cx = 0, BALL_cy = 0, BALL_pix = 0;
	int GOAL_cx = 0, GOAL_cy = 0, GOAL_pix = 0;
	int flag = 0;

	//������ �ν��ؼ� ������ ����
	while (1) {	//�̼� ó�� �� ���� ã��

		HEAD_4GREEN();
		double cnt = 200000;
		while (cnt--) {}
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);

		//�Ӹ� ���� ����

		/*
		if (BALL_cy == 0 && BALL_cx == 0 && flag == 0) {
			GO_GREEN(1);
			flag = 1;
		}
		else if (BALL_cy < 50) { GO_GREEN(5); }
		else if (BALL_cy < 60) { GO_GREEN(4); }
		else if (BALL_cy < 70) { GO_GREEN(3); }
		else if (BALL_cy < 80) { GO_GREEN(2); }
		else if (BALL_cy < 90) { GO_GREEN(1); }
		else {
			if (BALL_cx > 90 + 15) {
				if (BALL_cx > 135 )	GO_RIGHT();
				if (BALL_cx > 125 )	GO_RIGHT();
				if (BALL_cx > 115 )	GO_RIGHT_17();
				GO_RIGHT_17();
			}
			else if (BALL_cx < 90-15) {
				
				if (BALL_cx > 75 - 30 )	GO_LEFT();
				if (BALL_cx > 75 - 20 )	GO_LEFT();
				if (BALL_cx > 75 - 10 )	GO_LEFT_17();
				GO_LEFT_17();
			}
			else {
				break;
			}
		}
		*/
		if (flag == 1 && BALL_cy == 0 && BALL_cx == 0 ) { break; }
		
		if (BALL_cy == 0 && BALL_cx == 0 && flag == 0) {
			GO_GREEN(1);
			flag = 1;
		}
		else if (BALL_cy < 50) { GO_GREEN(5); }
		else if (BALL_cy < 60) { GO_GREEN(4); }
		else if (BALL_cy < 70) { GO_GREEN(3); }
		else if (BALL_cy < 80) { GO_GREEN(2); }
		else if (BALL_cy < 90) { GO_GREEN(1); }
		else {
			if (BALL_cx > 80) {
				GO_RIGHT_17();
				if (BALL_cx > 90 - 5)	GO_RIGHT_17();
				if (BALL_cx > 100 - 5)	GO_RIGHT();
				if (BALL_cx > 110 - 5)	GO_RIGHT();
			}
			else if (BALL_cx < 60) {
				GO_LEFT_17();
				if (BALL_cx > 50 + 5)	GO_LEFT_17();
				if (BALL_cx > 40 + 5)	GO_LEFT();
				if (BALL_cx > 30 + 5)	GO_LEFT();
			}
			else {
				break;
			}
		}


	}//while


	//printf("=================> MORE \n");

	while (1) {	//���� �� ������//�Ӹ� �� ������

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		//printf("<<<<<<<<<<<<<<<<<<< BALL >>>>>>>>>>>>>>>>>>");
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);

		printf("BALL ::: cx : %d, cy : %d, pixel : %d \n", BALL_cx, BALL_cy, BALL_pix);

		if (BALL_cx == 0 && BALL_cy == 0 && BALL_pix == 0)	break;

		HEAD_4GREEN();
		cnt = 200000;
		while (cnt--) {}

		//printf("<<<<<<<<<<<<<<<<<<< GOAL >>>>>>>>>>>>>>>>>>");

		INFO_GOAL_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GOAL_HL, GOAL_HH, GOAL_SL, GOAL_SH, GOAL_VL, GOAL_VH, //INPUT
			&GOAL_cx, &GOAL_cy, &GOAL_pix	//OUPUT
			);

		printf("GOAL ::: cx : %d, cy : %d, pixel : %d \n", GOAL_cx, GOAL_cy, GOAL_pix);

		int d, R, L;
		R = 68 + 5;	//73
		L = 64 + 5;	//69
		d = 5;

		if(GOAL_cx <= BALL_cx + 6  && GOAL_cx >= BALL_cx - 6)
			if (BALL_cy < 50   &&   BALL_cx <= R && BALL_cx >= L) { break; }

		//ȸ��
		if (GOAL_cx > BALL_cx + 6){				//6
			TURN_RIGHT();
			if (GOAL_cx > BALL_cx + 10)	G_TURN_RIGHT();	// 10 ���� ����!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}	
		else if (GOAL_cx < BALL_cx - 6){		//6  	
			TURN_LEFT();
			if (GOAL_cx < BALL_cx - 10) G_TURN_LEFT();
		}

		//�¿�� �̵�
		if (BALL_cy >= 50 || BALL_cx > 130 || BALL_cx < 50) {
			if (BALL_cx > R) {
					
				if(BALL_cx > R + 2 * d) 	GO_RIGHT_17();
				if (BALL_cx > R + 3 * d)	GO_RIGHT_17();
				if (BALL_cx > R + 4 * d)	GO_RIGHT_17();
				if (BALL_cx > R + 5 * d)	GO_RIGHT_17();
				GO_RIGHT_5();
			}
			else if (BALL_cx < L) {
				if (BALL_cx < L - 2 * d)	GO_LEFT_17();
				if (BALL_cx < L - 3 * d)	GO_LEFT_17();
				if (BALL_cx < L - 4 * d)	GO_LEFT_17();
				if (BALL_cx < L - 5 * d)	GO_LEFT_17();
				GO_LEFT_5();
			}
			else break;
		}
		else {
			int d = 4; //3;
		//������ ����
			if (BALL_cy < 55 - d * 2)	JJWALKING();
			if (BALL_cy < 55 - d * 3)	JJWALKING();
			if (BALL_cy < 55 - d * 4)	JJWALKING();
			if (BALL_cy < 55 - d * 5)	JJWALKING();
			JJWALKING();
		}
	}//while

	NORMAL_FORM();
	KICK_BALL();
}

//6666666666666666666666666666666666666666666666666666666666666666666666666666666666
void MISSION_6(void)	//�̼�6. ����� ���� �ٸ� �ǳʱ� - ������/������� �ٴڸ� ���� ���
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;

	//�ö� ���¿��� ���� ������ ��ġ�� �̵�
	while (1) {

		HEAD_4RED();
		double cnt = 200000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);

		int d = 2;		
		int G_RD = 93;
		int G_LD = 87;

		if (Y_cx > G_RD) {	
			if (Y_cx > G_RD + 9 * d) { GO_RIGHT(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 8 * d) { GO_RIGHT(); }
			if (Y_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 6 * d) { GO_RIGHT(); }
			if (Y_cx > G_RD + 5 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 4 * d) { GO_RIGHT_17(); }
			if (Y_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 2 * d) { GO_RIGHT_17(); }
			if (Y_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			GO_RIGHT_17();	
			JJWALKING();
		}
		else if (Y_cx < G_LD) {
			if (Y_cx < G_LD - 9 * d) { GO_LEFT(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 8 * d) { GO_LEFT(); }
			if (Y_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 6 * d) { GO_LEFT(); }
			if (Y_cx < G_LD - 5 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 4 * d) { GO_LEFT_17(); }
			if (Y_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 2 * d) { GO_LEFT_17(); }
			if (Y_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			GO_LEFT_17();
			JJWALKING();
		}
		else {	break;	}
	}

	int tmp = 4;
	while (tmp--) { JJWALKING(); }

	UP_STAIR();
	GO_GREEN(2);
	
	float G = 0;

	//���������� ã��
	while (1) {

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix, &G, LP_HEAD_FRONT	//OUPUT
			);
		double M = atan2((double)G, 1) * 180 / 3.14159265;

		int d = 3;

		if (M > 3) {
			G_TURN_LEFT();
			if(M>7)	G_TURN_LEFT();
		}
		else if (M< -3) {
			G_TURN_RIGHT();
			if(M<-7)	G_TURN_RIGHT();
		}
		else {
			d = 4;
			if (Y_cx < 89 - 3) {
				GO_LEFT_17(); 
				if (Y_cx < 89 - 3 - 1 * d) GO_LEFT_17();
				if (Y_cx < 89 - 3 - 2 * d) GO_LEFT_17();
			}
			else if (Y_cx>89 + 3) {
				GO_RIGHT_17(); 
				if (Y_cx > 89 + 3 + 1 * d) GO_RIGHT_17();
				if (Y_cx > 89 + 3 + 2 * d) GO_RIGHT_17();
			}
			else {
				d = 4;

				if (Y_cy >= 65) {	break;	}
				else { 			
					JJWALKING();
					if (Y_cy < 65 - 1 * d)	JJWALKING();
					if (Y_cy < 65 - 2 * d)	JJWALKING();
					if (Y_cy < 65 - 3 * d)	JJWALKING();
					if (Y_cy < 65 - 4 * d)	JJWALKING();
					if (Y_cy < 65 - 5 * d)	JJWALKING();
				}
			}
		}

	}//while

	//JJWALKING();
	NORMAL_FORM();
}

//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void COLOR_LINE_UP(int mode,int color_set, int level) {	//������ ���� ���� ���� �Լ�
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int B_cx = 0, B_cy = 0, B_pix = 0;
	float G = 0;
	int  HL = 0, HH = 0, SL = 0, SH = 0, VL = 0, VH = 0;
	int G_LD;
	int G_RD;
	if (color_set == LP_COLOR_WHITE) {
		HL = WHT_HL; HH = WHT_HH;  SL = WHT_SL; SH = WHT_SH; VL = WHT_VL; VH = WHT_VH;
	}
	else if (color_set == LP_COLOR_BLACK) {
		HL = BLK_HL; HH = BLK_HH;  SL = BLK_SL; SH = BLK_SH; VL = BLK_VL; VH = BLK_VH;
	}

	while (1) {

		ARM_BACK(mode);
		double cnt = 250000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			HL, HH, SL, SH, VL, VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, mode //OUPUT
			);
		if (mode == LP_HEAD_RIGHT) {			
			if(color_set == LP_COLOR_BLACK){

				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 4;
				int T_RD = -13;
				int T_LD = -7;
			
				if (M <T_RD) {	//ȸ�� ���� ��
					if (M <T_RD - 9 * d)	G_TURN_RIGHT();
					if (M <T_RD - 8 * d)	G_TURN_RIGHT();
					if (M <T_RD - 7 * d)	G_TURN_RIGHT();
					if (M <T_RD - 6 * d)	G_TURN_RIGHT();
					if (M <T_RD - 5 * d)	G_TURN_RIGHT();
					if (M <T_RD - 4 * d)	G_TURN_RIGHT();
					if (M <T_RD - 3 * d)	G_TURN_RIGHT();
					if (M <T_RD - 2 * d)	G_TURN_RIGHT();
					if (M <T_RD - 1 * d - 2)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > T_LD){
					if (M > T_LD + 9 * d)	G_TURN_LEFT();
					if (M > T_LD + 8 * d)	G_TURN_LEFT();
					if (M > T_LD + 7 * d)	G_TURN_LEFT();
					if (M > T_LD + 6 * d)	G_TURN_LEFT();
					if (M > T_LD + 5 * d)	G_TURN_LEFT();
					if (M > T_LD + 4 * d)	G_TURN_LEFT();
					if (M > T_LD + 3 * d)	G_TURN_LEFT();
					if (M > T_LD + 2 * d + 1)	G_TURN_LEFT();
					if (M > T_LD + 1 * d + 2)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					if (level == JAL) {
						G_LD = 95;//90;//97;
						G_RD = 81;//76;//83;
					}
					else if (level == DAECHUNG) {
						G_LD = 90 + 8;
						G_RD = 76 - 8;
					}
					d = 2;
					if (B_cy > G_LD) {		//�߽�ã��
						if (B_cy > G_LD + 9 * d) GO_LEFT();
						if (B_cy > G_LD + 8 * d) GO_LEFT();
						if (B_cy > G_LD + 7 * d) GO_LEFT();
						if (B_cy > G_LD + 6 * d) GO_LEFT();
						if (B_cy > G_LD + 5 * d) GO_LEFT();
						if (B_cy > G_LD + 4 * d) GO_LEFT();
						if (B_cy > G_LD + 3 * d) GO_LEFT_17();
						if (B_cy > G_LD + 2 * d) GO_LEFT_17();
						if (B_cy > G_LD + 1 * d) GO_LEFT_17();
						GO_LEFT_17();
					}
					else if (B_cy < G_RD){
						if (B_cy < G_RD - 9 * d)	GO_RIGHT();
						if (B_cy < G_RD - 8 * d)	GO_RIGHT();
						if (B_cy < G_RD - 7 * d)	GO_RIGHT();
						if (B_cy < G_RD - 6 * d)	GO_RIGHT();
						if (B_cy < G_RD - 5 * d)	GO_RIGHT();
						if (B_cy < G_RD - 4 * d)	GO_RIGHT();
						if (B_cy < G_RD - 3 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 2 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else {	break;	}
				}

			}//black color

			else if (color_set == LP_COLOR_WHITE) {
				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 6;
				if (M < -13){
					if (M < -13 - 9 * d)	G_TURN_RIGHT();
					if (M < -13 - 8 * d)	G_TURN_RIGHT();
					if (M < -13 - 7 * d)	G_TURN_RIGHT();
					if (M < -13 - 6 * d)	G_TURN_RIGHT();
					if (M < -13 - 5 * d)	G_TURN_RIGHT();
					if (M < -13 - 4 * d)	G_TURN_RIGHT();
					if (M < -13 - 3 * d)	G_TURN_RIGHT();
					if (M < -13 - 2 * d)	G_TURN_RIGHT();
					if (M < -13 - 1 * d)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > -4){
					if (M > -4 + 9 * d)	G_TURN_LEFT();
					if (M > -4 + 8 * d)	G_TURN_LEFT();
					if (M > -4 + 7 * d)	G_TURN_LEFT();
					if (M > -4 + 6 * d)	G_TURN_LEFT();
					if (M > -4 + 5 * d)	G_TURN_LEFT();
					if (M > -4 + 4 * d)	G_TURN_LEFT();
					if (M > -4 + 3 * d)	G_TURN_LEFT();
					if (M > -4 + 2 * d)	G_TURN_LEFT();
					if (M > -4 + 1 * d)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					d = 2;
					if (B_cy > 106 + 4){
						if (B_cy > 113 + 9 * d) GO_LEFT_17();
						if (B_cy > 113 + 8 * d) GO_LEFT_17();
						if (B_cy > 113 + 7 * d) GO_LEFT_17();
						if (B_cy > 113 + 6 * d) GO_LEFT_17();
						if (B_cy > 113 + 5 * d) GO_LEFT_17();
						if (B_cy > 113 + 4 * d) GO_LEFT_17();
						if (B_cy > 113 + 3 * d) GO_LEFT_17();
						if (B_cy > 113 + 2 * d) GO_LEFT_17();
						if (B_cy > 113 + 1 * d) GO_LEFT_17();
						GO_LEFT_17();	//�߽�ã��
					}
					else if (B_cy < 106 - 4){
						if (B_cy < 99 - 9 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 8 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 7 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 6 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 5 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 4 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 3 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 2 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}
			}//white color

		}//RIGHT

		else if(mode == LP_HEAD_FRONT) {		
			if (G * 100 > 7)	TURN_RIGHT();	//ȸ�� ���� ��
			else if (G * 100 < -5)	TURN_LEFT();
			else {
				if (B_cx > 55)	GO_LEFT();	//�߽�ã��
				else if (B_cx < 45)		GO_RIGHT();
				else {
					if (G * 100 > 7)	TURN_RIGHT();
					else if (G * 100 < -5)		TURN_LEFT();
					else {		break;	}
				}
			}
		}

	}//while

	NORMAL_FORM();

}


//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void COLOR_LINE_UP2(int mode, int color_set, int level) {	//������ ���� ���� ���� �Լ�
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int B_cx = 0, B_cy = 0, B_pix = 0;
	float G = 0;
	int  HL = 0, HH = 0, SL = 0, SH = 0, VL = 0, VH = 0;
	int G_LD;
	int G_RD;
	if (color_set == LP_COLOR_WHITE) {
		HL = WHT_HL; HH = WHT_HH;  SL = WHT_SL; SH = WHT_SH; VL = WHT_VL; VH = WHT_VH;
	}
	else if (color_set == LP_COLOR_BLACK) {
		HL = BLK_HL; HH = BLK_HH;  SL = BLK_SL; SH = BLK_SH; VL = BLK_VL; VH = BLK_VH;
	}

	while (1) {

		ARM_BACK(mode);
		double cnt = 250000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			HL, HH, SL, SH, VL, VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, mode //OUPUT
			);
		if (mode == LP_HEAD_RIGHT) {
			if (color_set == LP_COLOR_BLACK) {

				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 4;
				int T_RD = -13;
				int T_LD = -7;

				if (M <T_RD) {	//ȸ�� ���� ��
					if (M <T_RD - 9 * d)	G_TURN_RIGHT();
					if (M <T_RD - 8 * d)	G_TURN_RIGHT();
					if (M <T_RD - 7 * d)	G_TURN_RIGHT();
					if (M <T_RD - 6 * d)	G_TURN_RIGHT();
					if (M <T_RD - 5 * d)	G_TURN_RIGHT();
					if (M <T_RD - 4 * d)	G_TURN_RIGHT();
					if (M <T_RD - 3 * d)	G_TURN_RIGHT();
					if (M <T_RD - 2 * d)	G_TURN_RIGHT();
					if (M <T_RD - 1 * d - 2)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > T_LD) {
					if (M > T_LD + 9 * d)	G_TURN_LEFT();
					if (M > T_LD + 8 * d)	G_TURN_LEFT();
					if (M > T_LD + 7 * d)	G_TURN_LEFT();
					if (M > T_LD + 6 * d)	G_TURN_LEFT();
					if (M > T_LD + 5 * d)	G_TURN_LEFT();
					if (M > T_LD + 4 * d)	G_TURN_LEFT();
					if (M > T_LD + 3 * d)	G_TURN_LEFT();
					if (M > T_LD + 2 * d + 1)	G_TURN_LEFT();
					if (M > T_LD + 1 * d + 2)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					if (level == JAL) {
						G_LD = 97;
						G_RD = 83;
					}
					else if (level == DAECHUNG) {
						G_LD = 90 + 8;
						G_RD = 76 - 8;
					}
					d = 2;
					if (B_cy > G_LD) {		//�߽�ã��
						if (B_cy > G_LD + 9 * d) GO_LEFT();
						if (B_cy > G_LD + 8 * d) GO_LEFT();
						if (B_cy > G_LD + 7 * d) GO_LEFT();
						if (B_cy > G_LD + 6 * d) GO_LEFT();
						if (B_cy > G_LD + 5 * d) GO_LEFT();
						if (B_cy > G_LD + 4 * d) GO_LEFT();
						if (B_cy > G_LD + 3 * d) GO_LEFT_17();
						if (B_cy > G_LD + 2 * d) GO_LEFT_17();
						if (B_cy > G_LD + 1 * d) GO_LEFT_17();
						GO_LEFT_17();
					}
					else if (B_cy < G_RD) {
						if (B_cy < G_RD - 9 * d)	GO_RIGHT();
						if (B_cy < G_RD - 8 * d)	GO_RIGHT();
						if (B_cy < G_RD - 7 * d)	GO_RIGHT();
						if (B_cy < G_RD - 6 * d)	GO_RIGHT();
						if (B_cy < G_RD - 5 * d)	GO_RIGHT();
						if (B_cy < G_RD - 4 * d)	GO_RIGHT();
						if (B_cy < G_RD - 3 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 2 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}

			}//black color

			else if (color_set == LP_COLOR_WHITE) {
				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 6;
				if (M < -13) {
					if (M < -13 - 9 * d)	G_TURN_RIGHT();
					if (M < -13 - 8 * d)	G_TURN_RIGHT();
					if (M < -13 - 7 * d)	G_TURN_RIGHT();
					if (M < -13 - 6 * d)	G_TURN_RIGHT();
					if (M < -13 - 5 * d)	G_TURN_RIGHT();
					if (M < -13 - 4 * d)	G_TURN_RIGHT();
					if (M < -13 - 3 * d)	G_TURN_RIGHT();
					if (M < -13 - 2 * d)	G_TURN_RIGHT();
					if (M < -13 - 1 * d)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > -4) {
					if (M > -4 + 9 * d)	G_TURN_LEFT();
					if (M > -4 + 8 * d)	G_TURN_LEFT();
					if (M > -4 + 7 * d)	G_TURN_LEFT();
					if (M > -4 + 6 * d)	G_TURN_LEFT();
					if (M > -4 + 5 * d)	G_TURN_LEFT();
					if (M > -4 + 4 * d)	G_TURN_LEFT();
					if (M > -4 + 3 * d)	G_TURN_LEFT();
					if (M > -4 + 2 * d)	G_TURN_LEFT();
					if (M > -4 + 1 * d)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					d = 2;
					if (B_cy > 106 + 4) {
						if (B_cy > 113 + 9 * d) GO_LEFT_17();
						if (B_cy > 113 + 8 * d) GO_LEFT_17();
						if (B_cy > 113 + 7 * d) GO_LEFT_17();
						if (B_cy > 113 + 6 * d) GO_LEFT_17();
						if (B_cy > 113 + 5 * d) GO_LEFT_17();
						if (B_cy > 113 + 4 * d) GO_LEFT_17();
						if (B_cy > 113 + 3 * d) GO_LEFT_17();
						if (B_cy > 113 + 2 * d) GO_LEFT_17();
						if (B_cy > 113 + 1 * d) GO_LEFT_17();
						GO_LEFT_17();	//�߽�ã��
					}
					else if (B_cy < 106 - 4) {
						if (B_cy < 99 - 9 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 8 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 7 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 6 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 5 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 4 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 3 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 2 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}
			}//white color

		}//RIGHT

		else if (mode == LP_HEAD_FRONT) {
			if (G * 100 > 7)	TURN_RIGHT();	//ȸ�� ���� ��
			else if (G * 100 < -5)	TURN_LEFT();
			else {
				if (B_cx > 55)	GO_LEFT();	//�߽�ã��
				else if (B_cx < 45)		GO_RIGHT();
				else {
					if (G * 100 > 7)	TURN_RIGHT();
					else if (G * 100 < -5)		TURN_LEFT();
					else { break; }
				}
			}
		}

	}//while

	NORMAL_FORM();

}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

//�ʿ��� ���� �������� �߽����� �ȼ� ���� ���  �Լ�
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//���� �׵θ�
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//���� �׵θ�
		imgThresholded[i] = 0;

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //�ȼ��� ������ ����
																		  //� ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
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

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]�� 255�̻��̸� 
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

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}

	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	//printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}
void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//���� �׵θ�
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//���� �׵θ�
		imgThresholded[i] = 0;

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //�ȼ��� ������ ����
																		  //� ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
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

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]�� 255�̻��̸� 
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

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if ( LABEL[i].cY < 70 && LABEL[i].pix > 60  &&LABEL[i].pix < 190 && LABEL[i].cY> MAXLABEL.cY) {	//LABEL[i].pix > MAXLABEL.pix &&
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}

	//////////////////   MAXLABEL �� MAXlabelOUT�� �����    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);
																		
	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	//printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}

//�߼������� ���� ������ ���� ã�� �Լ�
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode  //OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);


	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	if (HL == YELLO_HL && HH == YELLO_HH)
		HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);
	else
		HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//���� �׵θ�
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//���� �׵θ�
		imgThresholded[i] = 0;

	//�� or �츦 �� ��� �� �ڸ���
	if (mode == LP_HEAD_RIGHT) {
		for (j = 0; j < m_nHeight; j++) {
			for (i = 91; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
		}
	}
	else if(mode == LP_HEAD_FRONT) {
		for (j = 75; j < m_nHeight; j++) {	//�� �ڸ���
			for (i = 0; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
		}
		for (j = 0; j < m_nHeight; j++) {
			for (i = 0; i < 20; i++)	imgThresholded[i + m_nWidth * j] = 0;
			for (i = m_nWidth-20; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
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

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //�ȼ��� ������ ����
																		  //� ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
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

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]�� 255�̻��̸� 
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

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	if (HL == YELLO_HL && HH == YELLO_HH){
		for (i = 1; i < m_label + 1; i++) {
			if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {	//if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
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
		for (i = 1; i < m_label + 1; i++) {
			if (LABEL[i].pix > 200 && LABEL[i].cY > MAXLABEL.cY) {	//if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
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
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}


	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	if (HH == WHT_HH && HL == WHT_HL  && SH == WHT_SH && SL == WHT_SL  && VH == WHT_VH && VL == WHT_VL) {	//����� ��� - �����

		float GI_55 = 0, GI_60 = 0, GI_65 = 0, GI_70 = 0, GI_75 = 0, GI_80 = 0;
		float G1 = 0, G2 = 0, G3 = 0, G4 = 0, G5 = 0;

		for (i = 55; i < 81; i = i + 5){
			for (j = 118; j >= 0; j--){
				if (MAXlabelOUT[i + j * m_nWidth] > 0){
					if (i == 55) GI_55++;
					if (i == 60) GI_60++;
					if (i == 65) GI_65++;
					if (i == 70) GI_70++;
					if (i == 75) GI_75++;
					if (i == 80) GI_80++;
				}
			}
		}

		G1 = (GI_60 - GI_55) / 5;
		G2 = (GI_65 - GI_60) / 5;
		G3 = (GI_70 - GI_65) / 5;
		G4 = (GI_75 - GI_70) / 5;
		G5 = (GI_80 - GI_75) / 5;

		*out_cX = MAXLABEL.cX;
		*out_cY = MAXLABEL.cY;
		*out_pix = MAXLABEL.pix;
		*G = (G1 + G2 + G3 + G4 + G5) / 5;
	}

	//�ؿ��� 6���� ������ �߼��� �׸���
	else if (HH == BLK_HH && HL == BLK_HL  && SH == BLK_SH && SL == BLK_SL  && VH == BLK_VH && VL == BLK_VL) {	//�������� ���

		//�߼��� ���� ���ϱ�
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX ; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + m_nWidth*j] == 255 && cnt < 6 ) {
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
	else if (HH == YELLO_HH && HL == YELLO_HL  && SH == YELLO_SH && SL == YELLO_SL  && VH == YELLO_VH && VL == YELLO_VL) {	
		//�߼��� ���� ���ϱ�
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + m_nWidth*j] == 255 && cnt < 6) {
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

	double M = atan2((double)*G, 1) * 180 / 3.14159265;
	/*
	float gi = *G, b = 0;
	int cx = *out_cX, cy = *out_cY, y = 0;

	b = gi * cx + cy;

	for (i = 0; i < 180; i++) {
		y = -1 * gi * i + b;
		if (y > 0 && y < 120)
			draw_rectfill(160 - 1.3*y, 240 + 1.3*i, 2, 2, MAKE_COLORREF(255, 0, 255));
	}
	*/
	
	//printf("cx : %d, cy : %d , pix : %d, G : %f, ATAN : %f \n", *out_cX ,*out_cY, *out_pix , *G , M);

	flip();

}


//��հ��� ǥ������ ���� ã�� �Լ�
void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	if(HL == YELLO_HL && HH == YELLO_HH )
		HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);
	else
		HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//�ٿ���� ����ó��
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//���� �׵θ�
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//���� �׵θ�
		imgThresholded[i] = 0;

	if (HL == YELLO_HL && HH == YELLO_HH){	//����� ����ó��
		for (j = 75; j < 120; j++)
			for (i = 0; i < m_nWidth; i++)		imgThresholded[i + m_nWidth * j] = 0;
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

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//��
			R = i + 1;	//��
			U = j - 1;	//��
			D = j + 1;	//�Ʒ�

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//��, �� ���̺��� ����
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //�ȼ��� ������ ����
																		  //� ���̺� ����
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// �ٷ� �� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//���� �ȼ����� ���̺��� ������ ���
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//������ ���̺��� ����. ���ο� ���̺� �ο�
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
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

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]�� 255�̻��̸� 
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

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
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
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	//��� ǥ������ ���ϱ�
	int tp_minY = MAXLABEL.minY + 10;
	int tp_maxY = MAXLABEL.maxY -10;

	int MidPoint[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MidPoint[i] = 0;
	int MPcnt[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MPcnt[i] = 0;

	int tp = 0;
	for (j = tp_minY; j <= tp_maxY; j++) {
		tp = j - tp_minY;
		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			if (MAXlabelOUT[i + m_nWidth*j] == 255){
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
	for (i = 0; i<n; i = i + 1)
		sum += (MidPoint[i] - *mean)*(MidPoint[i] - *mean);
	var = sum / (n - 1);
	*std_dev = sqrt(var);		//ǥ������

	//printf("MEAN = %8.3f	VAR = %8.3f	STAN DEV = %8.3f\n", *mean,var, *std_dev);

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	flip();

}