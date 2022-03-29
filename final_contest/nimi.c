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

//화면 출력 모드
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

/////////// 각 색깔별 HSV 값 ////////////
//빨강
#define RED_HL 0   
#define RED_HH 50
#define RED_SL 0  
#define RED_SH 255      
#define RED_VL 0
#define RED_VH 255

//주황 - 골프공
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
//골 나무
#define GOAL_HL 0
#define GOAL_HH 255
#define GOAL_SL 0
#define GOAL_SH 80
#define GOAL_VL 0
#define GOAL_VH 150
*/

//하얀색 반전
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
//민지 집



//노랑
#define YELLO_HL 50   
#define YELLO_HH 100
#define YELLO_SL 0 
#define YELLO_SH 255
#define YELLO_VL 50 
#define YELLO_VH 255

//노랑 - GATE
#define GATE_HL 50   
#define GATE_HH 100
#define GATE_SL 60  
#define GATE_SH 255
#define GATE_VL 50
#define GATE_VH 255

//초록	112
#define GREEN_HL 70      
#define GREEN_HH 150
#define GREEN_SL 50
#define GREEN_SH 255      
#define GREEN_VL 70		
#define GREEN_VH 255 

//파랑
#define BLUE_HL 150
#define BLUE_HH 255
#define BLUE_SL 100
#define BLUE_SH 255     
#define BLUE_VL 0   
#define BLUE_VH 255
/*
//검정 5공
#define BLK_HL 0	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 170	//70
#define BLK_VL 0	//10
#define BLK_VH 60	
*/

//검정	112다시
#define BLK_HL 10	
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 100	
#define BLK_VL 0	
#define BLK_VH 90	

/*
//5공 2
#define BLK_HL 10	
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 140	
#define BLK_VL 0	
#define BLK_VH 90	
*/
//흰
#define WHT_HL 0
#define WHT_HH 255
#define WHT_SL 0
#define WHT_SH 50 
#define WHT_VL 180
#define WHT_VH 255

///////////// 미션 ///////////////////
typedef enum {
	END = 0, M1, M2, M3, M4, M5, M6, M7
} MISSION;

void MISSION_1(int pix_s);
void MISSION_2(void);
void MISSION_4(void);
void MISSION_5R(int kick_mod);
void MISSION_6(void);
void MISSION_GATE(void);

//필요한 색깔과 방향을 사용하여 경기장 라인으로 정렬하는 함수
#define LP_HEAD_RIGHT 2
#define LP_HEAD_FRONT 3
#define LP_COLOR_BLACK 4
#define LP_COLOR_WHITE 5
#define JAL 10
#define DAECHUNG 20

//발차기 모드
#define KICK_LEFT 1
#define KICK_RIGHT 2


void COLOR_LINE_UP(int head_mode, int color_set, int level);

//필요한 색깔 영역에서 중심점과 픽셀 갯수 얻는 함수
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

//추세선으로 라인 정렬할 정보 찾는 함수
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

	int JJ_NUM = 0;			//JJWALKING 걸음수
	int flag_state = M1;	//시작하는 미션 설정
	double cnt = 0;			//delay

	//while(1) COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

	//while(1)	MISSION_GATE();

	while (b_loop) {
		switch (flag_state) {

		case 1:		//미션1. 직각-바리게이트	- 30

			MISSION_1(40);	//게이트가 없을 경우 지나가기 까지, 없어짐 확인까지

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

		case 2:		//미션2. 빨간 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용

			MISSION_2();	//빨간색 보면서 좌우 정렬까지

			JJ_NUM = 3;
			while (JJ_NUM--) { JJWALKING(); }

			TUCK();		//다리위에서 덤블링

			HEAD_4NORM();

			GO_JONGJONG(10);

			JJ_NUM = 8;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M3;
			break;

		case 3:		//미션3. 허들 장애물

			HEAD_4NORM();
			NORMAL_FORM();

			HUDDLE();			//허들 덤블링	

			HEAD_4NORM();

			GO_BACK(4);

			BIG_TURN_LEFT();	//90도 턴		
			G_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);		//검정색으로 중심 맞춤

			GO_NORM(5);

			//MISSION_GATE();
			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_NORM(16);

			GO_JONGJONG(7);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M4;
			break;

		case 4:			//미션4. 초록색 다리건너기

			MISSION_4();	//검정색으로 내려가는 조건 검사까지

			DOWN_STAIR();

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);

			GO_RIGHT();

			GO_NORM(11);

			flag_state = M5;
			break;

		case 5:			//미션5. 골프공 차기

			////////////////////// 오른발 //////////////////////////
			/*
			MISSION_5R(KICK_RIGHT); 	//주황색 공 대강 보기 ====> 골과 공 위치 탐색

			HEAD_4NORM();
			NORMAL_FORM();

			KICK_BALL_R();	//오른발

			GO_NORM(4);		//5; @1012-16:03

			// 오른발
			BIG_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();
			G_TURN_LEFT();

			*/
			////////////////////////////////////////////////

			/////////////////////  왼발 ///////////////////////////

			//while (1) {

			MISSION_5R(KICK_LEFT); 	//주황색 공 대강 보기 ====> 골과 공 위치 탐색

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

		case 6:			//미션6. 노란색 함정 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용

			MISSION_6();	//덤블링 가능 위치까지 이동

			HEAD_4NORM();
			NORMAL_FORM();

			YELLO_TUCK();	//덤블링

			HEAD_4NORM();

			flag_state = M7;
			break;

		case 7:		//미션7. 회전-바리게이트

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

			MISSION_1(25);	//노란색 인식 될 때 까지 기다리기 - 카메라 스캔 노란색 인식 - 카메라 멈춤

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
void MISSION_1(int pix_s)		//미션1. 직각-바리게이트, //미션7. 회전-바리게이트						
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

	//노란색 영역 찾기 - 게이트가 없을 경우 기다림
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

	 //노란색 영역 찾기 - 게이트가 있을경우 기다림
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
void MISSION_2(void)		//미션2. 빨간 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용					
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

		//빨간색 중심 맞추기
		//오른쪽
		if (R_cx > G_RD + 1 * d)	{	GO_RIGHT();		JJWALKING();		}
		else if (R_cx > G_RD)		{	GO_RIGHT_17();	JJWALKING();		}

		//왼쪽
		else if (R_cx < G_LD - 1 * d)	{	GO_LEFT();		JJWALKING();		}
		else if (R_cx < G_LD)			{	GO_LEFT_17();	JJWALKING();		}
		else { break; }

	}//while

}

//444444444444444444444444444444444444444444444444444444444444444444444444444444444
void MISSION_4(void)	//미션4. 초록색 다리건너기
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
	
	//초록색 영역 찾기 - 오르기 전 가운데 정렬
	
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


		//초록색 중심 맞추기
		int d = 1;
		int G_RD = 23;
		int G_LD = 22;

		//초록색 중심 맞추기
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

	UP_STAIR();		//계단 오르기

	GO_GREEN(3);

	HEAD_4GREEN();
	cnt = 200000;
	while (cnt--) {}

	int num_run = 2;	//최소 초록색 다리 가는 수

	//초록색 영역 찾기 - 초록색 다리 위에서 중심 찾으며 가기
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

				if (var > 0.4) { //회전
					if (mean < cent - 1.5)		{	TURN_LEFT();	}
					else if (mean > cent + 1.5) {	TURN_RIGHT();	}
					else	GO_GREEN(5);
				}
				else {	//가운데 맞추기
					if (mean < cent - 1)	{	GO_LEFT_17();	}
					else if (mean>cent)		{	GO_RIGHT_17();	}
					else	{	GO_GREEN(5);	}
				}

			}	//G_pix
			else break; 
			
		}
		else {	
			if (var > 0.4) {	//회전
				if (mean < cent - 1.5)		{	TURN_LEFT();	}
				else if (mean > cent + 1.5) {	TURN_RIGHT();	}
				else {
					num_run--;
					GO_GREEN(5);
				}
			}
			else {	//가운데 맞추기
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

	//검정색 선 찾기
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
void MISSION_5R(int kick_mode)	//미션5. 골프공 차기
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

	//골프공 인식해서 가까이 가기
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
	
		//머리 각도 조절

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


	while (1) {	//공에 더 가까이 머리 더 내리고

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

		if (GOAL_cx + goal_d <= BALL_cx + 1 && GOAL_cx + goal_d >= BALL_cx - 1)	//공차기 조건
			if (BALL_cy < goal && BALL_cx <= R && BALL_cx >= L) { break; }

		//회전
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
			else {		//앞으로 조금
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
void MISSION_6(void)	//미션6. 노란색 함정 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용
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

	//올라간 상태에서 덤블링 가능한 위치로 이동
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

		else	{	break;	}	//돌기 조건

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


	//검은색으로 찾기
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

		}//회전
	
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
void COLOR_LINE_UP(int mode, int color_set, int level) {	//검정색 기준 라인 정렬 함수
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
			if (M < T_RD) {	//회전 비교
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

			if (M < T_RD) {	//회전 비교
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

//필요한 색깔 영역에서 중심점과 픽셀 갯수 얻는  함수
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
	//바운더리 예외처리
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//왼쪽 테두리
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//위쪽 테두리

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
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + WIDTH * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여

						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //픽셀에 작은값 대입, 등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
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

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
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
				labelOUT[i + WIDTH * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]가 255이상이면 
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
	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
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


//필요한 색깔 영역에서 중심점과 픽셀 갯수 얻는  함수
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

	int i, j;	//바운더리 예외처리
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//왼쪽 테두리
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; 	}	//위쪽 테두리

	//for (j = 0; j < 15; j++)
		//for (i = 0; i < WIDTH; i++) { imgThresholded[i + WIDTH * j] = 0; }	//위쪽 테두리



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
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + WIDTH * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //픽셀에 작은값 대입, 등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
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

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
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
	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
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

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0; }	//왼쪽 테두리
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//위쪽 테두리

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
			L = i - 1;	//왼
			R = i + 1;	//오	
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + WIDTH * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //픽셀에 작은값 대입, 등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
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

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
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

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
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

//추세선으로 라인 정렬할 정보 찾는 함수
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

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//왼쪽 테두리
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0;	}	//위쪽 테두리

	//앞 or 우를 볼 경우 반 자르기
	if (mode == LP_HEAD_RIGHT) {
		for (j = 0; j < HEIGHT; j++) 
			for (i = 22; i < WIDTH; i++) { imgThresholded[i + WIDTH * j] = 0; }
	}
	else if (mode == LP_HEAD_FRONT) {
		for (j = HEIGHT-10; j < HEIGHT; j++) {	//앞 자르기
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
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + WIDTH * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //픽셀에 작은값 대입, 등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
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

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
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

	if (HL == YELLO_HL && HH == YELLO_HH && SL == YELLO_SL && SH == YELLO_SH && VL == YELLO_VL && VH == YELLO_VH) {		///////////////////////////////// 미션 6
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
		for (i = 1; i < m_label + 1; i++) {																				///////////////////////////////// 검정색
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

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////

	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc)		{	MAXlabelOUT[i + WIDTH * j] = 0;	}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0)	{	MAXlabelOUT[i + WIDTH * j] = 255;	}
		}

	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20

	//추세선 그리기 6개
	if (HH == BLK_HH && HL == BLK_HL  && SH == BLK_SH && SL == BLK_SL  && VH == BLK_VH && VL == BLK_VL) {	/////////////////////////////////////////// 검은색일 경우
																									//추세선 기울기 구하기
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

	else if (HH == YELLO_HH && HL == YELLO_HL  && SH == YELLO_SH && SL == YELLO_SL  && VH == YELLO_VH && VL == YELLO_VL) {		//////////////////////// 미션 6
		//추세선 기울기 구하기
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

//평균값과 표준편차 정보 찾는 함수
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

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < HEIGHT; j++)	{	imgThresholded[WIDTH * j] = 0;	}	//왼쪽 테두리
	for (i = 0; i < WIDTH; i++)		{	imgThresholded[i] = 0; }	//위쪽 테두리

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
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + WIDTH * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + WIDTH * j] != 0) && (imgThresholded[i + WIDTH * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + WIDTH * j] == imgThresholded[i + WIDTH * U])
						imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여
						maxl = MAX(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);
						minl = MIN(imgThresholded[L + WIDTH * j], imgThresholded[i + WIDTH * U]);

						imgThresholded[i + WIDTH * j] = minl;          //픽셀에 작은값 대입, 등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + WIDTH * U] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[i + WIDTH * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + WIDTH * j] != 0)	imgThresholded[i + WIDTH * j] = imgThresholded[L + WIDTH * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
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

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
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

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
	for (j = 1; j < HEIGHT - 1; j++)
		for (i = 1; i < WIDTH - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + WIDTH * j] != MAXlabelc)		{	MAXlabelOUT[i + WIDTH * j] = 0;	}
			else if (labelOUT[i + WIDTH * j] == MAXlabelc || m_label == 0)	{	MAXlabelOUT[i + WIDTH * j] = 255;	}
		}

	draw_rectfill2(MAXlabelOUT, Filt, C_mode);	//180*120 -> 30*20
	printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);

	//평균 표준편차 구하기
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
	*mean = sum / n;		//평균

	sum = 0;
	for (i = 0; i < n; i = i + 1)	{	sum += (MidPoint[i] - *mean)*(MidPoint[i] - *mean);		}
	var = sum / (n - 1);
	*std_dev = var;		//분산

	printf("MEAN = %8.3f	VAR = %8.3f	STAN DEV = %8.3f\n", *mean,var, *std_dev);

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

}